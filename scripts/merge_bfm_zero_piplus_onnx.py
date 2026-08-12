"""Merge the PiPlus BFM-Zero actor and backward encoder ONNX graphs.

The source actor consumes ``state | last_action | history_actor | z`` and the
source backward encoder consumes ``state | privileged_state``.  This exporter
keeps those semantic inputs separate for sim2real and performs the eight-frame
latent average plus the source ``norm_z`` projection inside one graph.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import onnx
from onnx import TensorProto, helper
from onnx.compose import add_prefix


def _shape(value_info: onnx.ValueInfoProto) -> list[int | str]:
    result: list[int | str] = []
    for dim in value_info.type.tensor_type.shape.dim:
        result.append(dim.dim_value if dim.HasField("dim_value") else dim.dim_param)
    return result


def _single_input(model: onnx.ModelProto, name: str) -> onnx.ValueInfoProto:
    matches = [value for value in model.graph.input if value.name == name]
    if len(matches) != 1:
        raise ValueError(
            f"Expected one input {name!r}, got {[value.name for value in model.graph.input]}"
        )
    return matches[0]


def _rename_input_nodes(model: onnx.ModelProto, old_name: str, new_name: str) -> None:
    for node in model.graph.node:
        for index, input_name in enumerate(node.input):
            if input_name == old_name:
                node.input[index] = new_name


def _make_concat_input(
    *,
    nodes: list[onnx.NodeProto],
    inputs: list[onnx.ValueInfoProto],
    output_name: str,
    input_names: list[str],
    dimensions: list[int],
    input_prefix: str,
) -> None:
    for name, dimension in zip(input_names, dimensions, strict=True):
        inputs.append(
            helper.make_tensor_value_info(
                name,
                TensorProto.FLOAT,
                ["batch" if input_prefix == "actor" else 8, dimension],
            )
        )
    nodes.append(
        helper.make_node(
            "Concat",
            input_names,
            [output_name],
            name=f"{input_prefix}_semantic_concat",
            axis=1,
        )
    )


def build(actor_path: Path, encoder_path: Path, output_path: Path) -> None:
    actor = add_prefix(onnx.load(actor_path, load_external_data=True), "actor/")
    encoder = add_prefix(onnx.load(encoder_path, load_external_data=True), "encoder/")

    actor_input = _single_input(actor, "actor/actor_obs")
    encoder_input = _single_input(encoder, "encoder/encoder_obs")
    if _shape(actor_input)[-1] != 616:
        raise ValueError(f"Unexpected actor input shape {_shape(actor_input)}")
    if _shape(encoder_input)[-1] != 438:
        raise ValueError(f"Unexpected encoder input shape {_shape(encoder_input)}")

    # The source exporter fixes batch=1.  All graph operations are batch-safe,
    # so make the window batch explicit for the eight target frames.
    encoder_input.type.tensor_type.shape.dim[0].dim_value = 8
    encoder_input.type.tensor_type.shape.dim[0].ClearField("dim_param")

    _rename_input_nodes(actor, "actor/actor_obs", "actor_obs_concat")
    _rename_input_nodes(encoder, "encoder/encoder_obs", "encoder_obs_concat")

    inputs: list[onnx.ValueInfoProto] = []
    prefix_nodes: list[onnx.NodeProto] = []
    _make_concat_input(
        nodes=prefix_nodes,
        inputs=inputs,
        output_name="actor_obs_concat",
        input_names=["actor_state", "last_action", "history_actor", "latent_z"],
        dimensions=[50, 22, 288, 256],
        input_prefix="actor",
    )
    # latent_z is internal, so remove it from the public inputs and connect the
    # projected weighted encoder output below.
    inputs.pop()
    prefix_nodes.pop()
    inputs.extend(
        [
            helper.make_tensor_value_info("encoder_state", TensorProto.FLOAT, [8, 50]),
            helper.make_tensor_value_info(
                "privileged_state", TensorProto.FLOAT, [8, 388]
            ),
            helper.make_tensor_value_info(
                "encoder_window_weight", TensorProto.FLOAT, [8, 1]
            ),
        ]
    )
    _make_concat_input(
        nodes=prefix_nodes,
        inputs=[],
        output_name="encoder_obs_concat",
        input_names=["encoder_state", "privileged_state"],
        dimensions=[50, 388],
        input_prefix="encoder",
    )

    initializers = list(actor.graph.initializer) + list(encoder.graph.initializer)
    nodes = list(prefix_nodes) + list(encoder.graph.node)

    # Average the eight normalized backward-map outputs and apply the same
    # sqrt(z_dim) * F.normalize projection used by the source model.
    nodes.extend(
        [
            helper.make_node(
                "Constant",
                [],
                ["axis_zero"],
                name="axis_zero",
                value=helper.make_tensor(
                    "axis_zero_value", TensorProto.INT64, [1], [0]
                ),
            ),
            helper.make_node(
                "Mul",
                ["encoder/z", "encoder_window_weight"],
                ["z_weighted"],
                name="latent_weight",
            ),
            helper.make_node(
                "ReduceSum",
                ["z_weighted", "axis_zero"],
                ["z_sum"],
                name="latent_sum",
                keepdims=1,
            ),
            helper.make_node(
                "ReduceSum",
                ["encoder_window_weight", "axis_zero"],
                ["weight_sum"],
                name="weight_sum",
                keepdims=1,
            ),
            helper.make_node(
                "Constant",
                [],
                ["weight_eps"],
                name="weight_eps",
                value=helper.make_tensor(
                    "weight_eps_value", TensorProto.FLOAT, [], [1.0e-6]
                ),
            ),
            helper.make_node(
                "Clip",
                ["weight_sum", "weight_eps"],
                ["weight_sum_clipped"],
                name="weight_clip",
            ),
            helper.make_node(
                "Div",
                ["z_sum", "weight_sum_clipped"],
                ["z_average"],
                name="latent_average",
            ),
            helper.make_node(
                "ReduceL2",
                ["z_average"],
                ["z_norm"],
                name="latent_norm",
                axes=[-1],
                keepdims=1,
            ),
            helper.make_node(
                "Constant",
                [],
                ["norm_eps"],
                name="norm_eps",
                value=helper.make_tensor(
                    "norm_eps_value", TensorProto.FLOAT, [], [1.0e-12]
                ),
            ),
            helper.make_node(
                "Clip", ["z_norm", "norm_eps"], ["z_norm_clipped"], name="norm_clip"
            ),
            helper.make_node(
                "Div", ["z_average", "z_norm_clipped"], ["z_unit"], name="latent_unit"
            ),
            helper.make_node(
                "Constant",
                [],
                ["z_scale"],
                name="z_scale",
                value=helper.make_tensor(
                    "z_scale_value", TensorProto.FLOAT, [], [16.0]
                ),
            ),
            helper.make_node(
                "Mul", ["z_unit", "z_scale"], ["latent_z"], name="latent_project"
            ),
        ]
    )
    nodes.append(
        helper.make_node(
            "Concat",
            ["actor_state", "last_action", "history_actor", "latent_z"],
            ["actor_obs_concat"],
            name="actor_semantic_concat",
            axis=1,
        )
    )
    nodes.extend(actor.graph.node)

    actor_output = actor.graph.output[0]
    actor_output_name = actor_output.name
    for node in nodes:
        for index, input_name in enumerate(node.input):
            if input_name == actor_output_name:
                node.input[index] = "action"
        for index, output_name in enumerate(node.output):
            if output_name == actor_output_name:
                node.output[index] = "action"
    output = helper.make_tensor_value_info("action", TensorProto.FLOAT, ["batch", 22])

    graph = helper.make_graph(
        nodes,
        "bfm_zero_piplus_semantic_policy",
        inputs,
        [output],
        initializer=initializers,
    )
    model = helper.make_model(
        graph,
        producer_name="sim2real",
        ir_version=7,
        opset_imports=[helper.make_opsetid("", 13)],
    )
    model.metadata_props.add(key="source_actor", value=str(actor_path))
    model.metadata_props.add(key="source_backward_encoder", value=str(encoder_path))
    model.metadata_props.add(
        key="contract",
        value="actor_state[50],last_action[22],history_actor[288],encoder_state[8,50],privileged_state[8,388],encoder_window_weight[8,1] -> action[22]",
    )
    onnx.checker.check_model(model)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    onnx.save(model, output_path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--actor", type=Path, required=True)
    parser.add_argument("--encoder", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    build(args.actor, args.encoder, args.output)


if __name__ == "__main__":
    main()
