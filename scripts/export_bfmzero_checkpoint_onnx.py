"""Export a UFO BFM-Zero checkpoint's actor and backward encoder graphs."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ufo-root", type=Path, required=True)
    parser.add_argument("--checkpoint", type=Path, required=True, help="UFO run directory containing checkpoint/")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--skip-verify", action="store_true")
    args = parser.parse_args()

    ufo_root = args.ufo_root.expanduser().resolve()
    sys.path.insert(0, str(ufo_root))
    from humanoidverse.agents.load_utils import load_model_from_checkpoint_dir
    from humanoidverse.export.backward_encoder import export_backward_encoder_from_model
    from humanoidverse.utils.helpers import export_meta_policy_as_onnx

    run_dir = args.checkpoint.expanduser().resolve()
    checkpoint_dir = run_dir / "checkpoint"
    if not (checkpoint_dir / "model" / "config.json").is_file():
        raise FileNotFoundError(f"Expected UFO checkpoint at {checkpoint_dir}")
    output_dir = args.output.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    model = load_model_from_checkpoint_dir(str(checkpoint_dir), device="cpu")
    actor_meta = export_meta_policy_as_onnx(model, output_dir, "FBcprAuxModel.onnx", z_dim=int(model.cfg.archi.z_dim))
    backward_path = output_dir / "FBcprAuxModel_z_encoder.onnx"
    export_backward_encoder_from_model(model, backward_path, verify=not bool(args.skip_verify))
    metadata = {
        "source_checkpoint": str(run_dir),
        "actor": actor_meta,
        "backward_encoder": {
            "path": str(backward_path),
            "input_keys": ["state", "last_action", "privileged_state"],
            "z_dim": int(model.cfg.archi.z_dim),
        },
    }
    (output_dir / "export.json").write_text(json.dumps(metadata, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(metadata, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
