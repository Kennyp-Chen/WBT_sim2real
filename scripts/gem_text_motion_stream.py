#!/usr/bin/env python3
"""Generate bounded text-conditioned GEM motion chunks.

GENMO's released text demo is a recorded-video + text pipeline: a short video
provides camera/scale context and each text prompt becomes a fixed-duration
segment. This process keeps prompt order, applies backpressure, and atomically
writes one GEM ``smpl_params.pt`` per generated segment. ``gem_chunk_pub.py``
can publish those files to the existing SONIC SMPL ZMQ endpoint.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import threading
import queue

import torch

from sim2real.teleop.gem_stream_queue import BoundedMotionChunkQueue, MotionChunkRequest


DEFAULT_GENMO_ROOT = Path("/home/sunteng/Projects/WBC_Telep/GENMO")
DEFAULT_ANCHOR = DEFAULT_GENMO_ROOT / "inputs/demo/tennis.mp4"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prompt", action="append", default=[], help="Prompt to enqueue; repeatable")
    parser.add_argument("--prompt-file", type=Path, default=None, help="UTF-8 file with one prompt per line")
    parser.add_argument("--anchor-video", type=Path, default=DEFAULT_ANCHOR)
    parser.add_argument("--genmo-root", type=Path, default=DEFAULT_GENMO_ROOT)
    parser.add_argument("--genmo-python", type=Path, default=None)
    parser.add_argument("--ckpt-path", type=Path, default=None)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/gem_stream/text"))
    parser.add_argument("--text-length", type=int, default=300, help="Frames per generated chunk at 30 Hz")
    parser.add_argument("--max-queue", type=int, default=3)
    parser.add_argument("--static-cam", action="store_true", default=True)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def _load_prompts(args: argparse.Namespace) -> list[str]:
    prompts = [str(prompt).strip() for prompt in args.prompt if str(prompt).strip()]
    if args.prompt_file is not None:
        prompts.extend(
            line.strip()
            for line in args.prompt_file.expanduser().read_text(encoding="utf-8").splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        )
    return prompts


def _extract_text_segment(source_path: Path, destination: Path, request: MotionChunkRequest) -> None:
    payload = torch.load(str(source_path), map_location="cpu", weights_only=False)
    segment_info = payload.get("segment_info", [])
    text_segments = [item for item in segment_info if item.get("type") == "text"]
    if len(text_segments) != 1:
        raise ValueError(f"Expected exactly one text segment in {source_path}, got {segment_info}")
    segment = text_segments[0]
    start, end = int(segment["start"]), int(segment["end"])
    if end <= start:
        raise ValueError(f"Empty text segment in {source_path}: {segment}")
    output: dict[str, object] = {
        "body_params_global": {},
        "body_params_incam": {},
        "K_fullimg": payload.get("K_fullimg"),
        "segment_info": [{"type": "text", "start": 0, "end": end - start, "caption": request.value}],
        "fps": 30.0,
        "source_prompt": request.value,
        "source_sequence": request.sequence,
    }
    for group in ("body_params_global", "body_params_incam"):
        params = payload.get(group)
        if not isinstance(params, dict):
            raise ValueError(f"{source_path} is missing {group}")
        output[group] = {
            key: value[start:end].clone() if hasattr(value, "clone") else value[start:end]
            for key, value in params.items()
        }
    output["K_fullimg"] = output["K_fullimg"][start:end]
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    torch.save(output, str(temporary))
    os.replace(temporary, destination)


def _run_genmo(args: argparse.Namespace, request: MotionChunkRequest, work_dir: Path) -> Path:
    genmo_root = args.genmo_root.expanduser().resolve()
    # GENMO's checkout may have an empty placeholder `.venv`; the sim2real
    # root environment is the one containing the installed GEM dependencies.
    # Keep the virtualenv symlink instead of resolving it to `/usr/bin/python`;
    # the site-packages on the virtualenv are needed by GENMO.
    python = (args.genmo_python or Path(sys.executable)).expanduser()
    ckpt = (args.ckpt_path or (genmo_root / "inputs/pretrained/gem_smpl.ckpt")).expanduser().resolve()
    if not python.is_file():
        raise FileNotFoundError(f"GENMO Python executable not found: {python}")
    if not args.anchor_video.expanduser().is_file():
        raise FileNotFoundError(f"Anchor video not found: {args.anchor_video}")
    if not ckpt.is_file():
        raise FileNotFoundError(f"GEM checkpoint not found: {ckpt}")
    work_dir.mkdir(parents=True, exist_ok=True)
    # Reuse the checked-in/demo cache when the anchor is the standard tennis
    # clip.  GENMO otherwise reruns ViTPose/ViT features for every prompt.
    cached_preprocess = genmo_root / "outputs/gem_runs" / args.anchor_video.stem / "preprocess"
    expected_preprocess = work_dir / f"{args.anchor_video.stem}_mix" / f"preprocess_{args.anchor_video.stem}"
    if cached_preprocess.is_dir() and not expected_preprocess.exists():
        expected_preprocess.mkdir(parents=True, exist_ok=True)
        for filename in ("bbx.pt", "vit_features.pt", "vitpose.pt"):
            source = cached_preprocess / filename
            if source.is_file():
                shutil.copy2(source, expected_preprocess / filename)
    command = [
        str(python),
        "scripts/demo/demo_smpl.py",
        "--input_list",
        str(args.anchor_video.expanduser().resolve()),
        f"text:{request.value}",
        "--ckpt_path",
        str(ckpt),
        "--output_root",
        str(work_dir),
        "--text_length",
        str(args.text_length),
        "--no_render",
    ]
    if args.static_cam:
        command.append("--static_cam")
    env = os.environ.copy()
    env.setdefault("HF_HUB_OFFLINE", "1")
    env.setdefault("HF_HUB_DISABLE_TELEMETRY", "1")
    subprocess.run(command, cwd=str(genmo_root), env=env, check=True)
    generated = work_dir / f"{args.anchor_video.stem}_mix/smpl_params.pt"
    if not generated.is_file():
        raise FileNotFoundError(f"GENMO did not produce {generated}")
    return generated


def _worker(args: argparse.Namespace, requests: BoundedMotionChunkQueue, stop: threading.Event) -> None:
    output_dir = args.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    while not stop.is_set():
        try:
            request = requests.get(timeout=0.1)
        except queue.Empty:
            if requests.closed:
                return
            continue
        try:
            chunk_dir = output_dir / f"chunk_{request.sequence:06d}"
            if args.dry_run:
                chunk_dir.mkdir(parents=True, exist_ok=True)
                (chunk_dir / "request.json").write_text(
                    json.dumps(request.__dict__, ensure_ascii=True, indent=2) + "\n",
                    encoding="utf-8",
                )
                print(f"[gem-text-stream] dry-run queued chunk {request.sequence}: {request.value}", flush=True)
                continue
            work_dir = output_dir / ".genmo" / f"chunk_{request.sequence:06d}"
            generated = _run_genmo(args, request, work_dir)
            target = chunk_dir / "smpl_params.pt"
            _extract_text_segment(generated, target, request)
            shutil.rmtree(work_dir, ignore_errors=True)
            print(f"[gem-text-stream] ready chunk {request.sequence}: {target}", flush=True)
        except Exception as exc:
            print(f"[gem-text-stream] chunk {request.sequence} failed: {exc}", file=sys.stderr, flush=True)
        finally:
            requests.task_done()


def main() -> None:
    args = _parse_args()
    if args.text_length <= 0:
        raise ValueError("text-length must be positive")
    prompts = _load_prompts(args)
    if not prompts and not sys.stdin.isatty():
        prompts = [line.strip() for line in sys.stdin if line.strip()]
    if not prompts:
        raise ValueError("Provide --prompt, --prompt-file, or prompts on stdin")
    requests = BoundedMotionChunkQueue(max_chunks=args.max_queue)
    stop = threading.Event()
    worker = threading.Thread(target=_worker, args=(args, requests, stop), daemon=True)
    worker.start()
    try:
        for prompt in prompts:
            request = requests.submit("text", prompt)
            if request is None:
                print(f"[gem-text-stream] queue full; rejected prompt: {prompt}", file=sys.stderr, flush=True)
        requests.join()
    finally:
        requests.close()
        stop.set()
        worker.join(timeout=2.0)


if __name__ == "__main__":
    main()
