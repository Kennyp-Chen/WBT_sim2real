from __future__ import annotations

import argparse
import os
import subprocess
import sys
from pathlib import Path

DEFAULT_POLICIES = {
    "mimic_lite": "checkpoints/mimic-lite/32x8192-huge/policy.yaml",
    "bfm_zero": "checkpoints/bfm-zero/exp_lafan40-100style_update_z10/policy.yaml",
    "bfm_zero_piplus": "checkpoints/bfm-zero/piplus/bfmzero-piplus-h0w-isaac-20260807_204741/policy.yaml",
    "scalebfm_m": "checkpoints/scalebfm/humanoid_transformer_m/policy.yaml",
    "scalebfm_xl": "checkpoints/scalebfm/humanoid_transformer_xl/policy.yaml",
    "sonic_release": "checkpoints/sonic/release/g1/policy.yaml",
    "sonic_low_latency": "checkpoints/sonic/low_latency/g1/policy.yaml",
    "teleopit": "checkpoints/teleopit/policy.yaml",
    "humanoid_gpt": "checkpoints/humanoid-gpt/policy.yaml",
    "heft_pmg": "checkpoints/heft/pmg/policy.yaml",
    "heft_compliance": "checkpoints/heft/compliance/policy.yaml",
    "holomotion": "checkpoints/holomotion/v1_4_0/policy.yaml",
    "twist2": "checkpoints/twist2/policy.yaml",
}
DEFAULT_MOTIONS = (
    "jumps1_subject1.npz",
    "fallAndGetUp1_subject1.npz",
)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record deterministic integrated sim2sim policy videos."
    )
    parser.add_argument(
        "--policy",
        action="append",
        default=[],
        metavar="NAME",
        help="Policy name to record; repeat as needed. Defaults to all policies.",
    )
    parser.add_argument(
        "--robot",
        default="g1",
        help="Robot configuration passed to integrated_sim2sim (default: g1).",
    )
    parser.add_argument(
        "--motion",
        action="append",
        default=[],
        metavar="FILE",
        help="Motion filename or path; repeat as needed. Defaults to jump and get-up.",
    )
    parser.add_argument(
        "--motions-root",
        default=".cache/motion/datasets/lafan40/motions",
    )
    parser.add_argument("--output-dir", default="outputs/policy_videos")
    parser.add_argument("--duration-s", type=float, default=15.0)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--continue-on-error", action="store_true")
    return parser.parse_args()


def _selected_policies(names: list[str], robot: str) -> dict[str, str]:
    if not names:
        if robot == "piplus_h0w":
            return {"bfm_zero_piplus": DEFAULT_POLICIES["bfm_zero_piplus"]}
        return {
            name: path
            for name, path in DEFAULT_POLICIES.items()
            if name != "bfm_zero_piplus"
        }
    unknown = sorted(set(names) - set(DEFAULT_POLICIES))
    if unknown:
        choices = ", ".join(DEFAULT_POLICIES)
        raise ValueError(f"Unknown policies {unknown}; choices: {choices}")
    return {name: DEFAULT_POLICIES[name] for name in names}


def _motion_paths(items: list[str], motions_root: Path) -> list[Path]:
    selected = items or list(DEFAULT_MOTIONS)
    paths: list[Path] = []
    for item in selected:
        path = Path(item).expanduser()
        if not path.is_absolute() and path.parent == Path("."):
            path = motions_root / path
        path = path.resolve()
        if not path.is_file():
            raise FileNotFoundError(path)
        paths.append(path)
    return paths


def main() -> None:
    args = _parse_args()
    if args.duration_s <= 0 or args.fps <= 0:
        raise ValueError("duration-s and fps must be positive")

    repo_root = Path(__file__).resolve().parents[2]
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    policies = _selected_policies(args.policy, args.robot)
    motions = _motion_paths(
        args.motion,
        (repo_root / args.motions_root).resolve(),
    )

    env = os.environ.copy()
    env.setdefault("MUJOCO_GL", "egl")
    env["HF_HUB_OFFLINE"] = "1"
    env["HF_HUB_DISABLE_TELEMETRY"] = "1"

    failures: list[str] = []
    total = len(policies) * len(motions)
    run_index = 0
    for policy_name, policy_relative in policies.items():
        policy_path = (repo_root / policy_relative).resolve()
        if not policy_path.is_file():
            raise FileNotFoundError(policy_path)
        policy_output_dir = output_dir / policy_name
        policy_output_dir.mkdir(parents=True, exist_ok=True)

        for motion_path in motions:
            run_index += 1
            output_path = policy_output_dir / f"{motion_path.stem}.mp4"
            if output_path.is_file() and output_path.stat().st_size > 0 and not args.overwrite:
                print(f"[{run_index}/{total}] skip existing {output_path}", flush=True)
                continue

            cmd = [
                sys.executable,
                str(repo_root / "sim2real/sim_env/integrated_sim2sim.py"),
                "--robot",
                args.robot,
                "--policy-config",
                str(policy_path),
                "--motion-path",
                str(motion_path),
                "--headless",
                "--initial-pause-s",
                "0",
                "--max-runtime-s",
                str(args.duration_s),
                "--video-output",
                str(output_path),
                "--video-fps",
                str(args.fps),
                "--video-width",
                str(args.width),
                "--video-height",
                str(args.height),
                "--seed",
                str(args.seed),
            ]
            print(
                f"[{run_index}/{total}] {policy_name} / {motion_path.name} -> {output_path}",
                flush=True,
            )
            result = subprocess.run(cmd, cwd=repo_root, env=env, check=False)
            if result.returncode == 0:
                continue
            output_path.unlink(missing_ok=True)
            failure = f"{policy_name}/{motion_path.name} (exit {result.returncode})"
            failures.append(failure)
            if not args.continue_on_error:
                raise subprocess.CalledProcessError(result.returncode, cmd)

    if failures:
        raise RuntimeError("Video runs failed: " + ", ".join(failures))
    print(f"Completed {total} policy/motion video runs under {output_dir}")


if __name__ == "__main__":
    main()
