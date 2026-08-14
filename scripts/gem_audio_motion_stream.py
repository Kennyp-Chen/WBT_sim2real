#!/usr/bin/env python3
"""Generate audio- or music-conditioned GEM SMPL motion.

GENMO's audio condition consumes mono 18 kHz waveform samples (600 samples per
30 Hz motion frame). Music conditioning consumes the training-time 35-D
per-frame ``music_embed`` feature; an MP3 alone is not a valid music condition.
The generated ``smpl_params.pt`` can be retargeted with the existing GMR
adapters and published through the existing SONIC/BFM-Zero paths.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys

import numpy as np
import torch


def _load_audio(path: Path, target_sr: int = 18000) -> tuple[torch.Tensor, int]:
    try:
        import librosa
    except ImportError:
        librosa = None
    if librosa is not None:
        waveform, _ = librosa.load(str(path.expanduser().resolve()), sr=target_sr, mono=True)
    else:
        try:
            import av
        except ImportError:
            av = None
        if av is not None:
            # PyAV handles WAV, MP3, FLAC, and common container codecs without
            # requiring a system ffmpeg executable.
            samples: list[np.ndarray] = []
            source_rates: set[int] = set()
            with av.open(str(path.expanduser().resolve())) as container:
                for frame in container.decode(audio=0):
                    array = frame.to_ndarray()
                    if array.ndim == 2:
                        array = array.mean(axis=0)
                    samples.append(np.asarray(array, dtype=np.float32))
                    if frame.sample_rate:
                        source_rates.add(int(frame.sample_rate))
            if not samples or len(source_rates) != 1:
                raise ValueError(f"Could not decode a single-rate audio stream: {path}")
            waveform = np.concatenate(samples)
            source_sr = source_rates.pop()
            if source_sr != target_sr:
                from scipy.signal import resample_poly

                divisor = int(np.gcd(source_sr, target_sr))
                waveform = resample_poly(
                    waveform, target_sr // divisor, source_sr // divisor
                ).astype(np.float32)
        else:
            if path.suffix.lower() != ".wav":
                raise RuntimeError(
                    "librosa or PyAV is required for non-WAV audio; install one in the GENMO environment"
                )
            import wave

            from scipy.signal import resample_poly

            with wave.open(str(path.expanduser().resolve()), "rb") as stream:
                source_sr = int(stream.getframerate())
                channels = int(stream.getnchannels())
                sample_width = int(stream.getsampwidth())
                frame_count = int(stream.getnframes())
                raw = stream.readframes(frame_count)
            if sample_width == 1:
                waveform = np.frombuffer(raw, dtype=np.uint8).astype(np.float32)
                waveform = (waveform - 128.0) / 128.0
            elif sample_width == 2:
                waveform = np.frombuffer(raw, dtype="<i2").astype(np.float32) / 32768.0
            elif sample_width == 4:
                waveform = np.frombuffer(raw, dtype="<i4").astype(np.float32) / 2147483648.0
            else:
                raise ValueError(f"Unsupported WAV sample width: {sample_width} bytes")
            if channels > 1:
                waveform = waveform.reshape(-1, channels).mean(axis=1)
            if source_sr != target_sr:
                divisor = int(np.gcd(source_sr, target_sr))
                waveform = resample_poly(
                    waveform, target_sr // divisor, source_sr // divisor
                ).astype(np.float32)
    waveform = np.asarray(waveform, dtype=np.float32)
    if waveform.ndim != 1 or waveform.size == 0:
        raise ValueError(f"Audio file is empty or not mono: {path}")
    return torch.from_numpy(waveform), target_sr


def _load_music_embed(path: Path) -> torch.Tensor:
    if path.suffix.lower() == ".npy":
        value = np.load(path, allow_pickle=False)
    else:
        value = torch.load(str(path), map_location="cpu", weights_only=False)
    if isinstance(value, dict):
        value = value.get("music_embed", value.get("embedding"))
    value = torch.as_tensor(value, dtype=torch.float32)
    if value.ndim != 2 or value.shape[1] != 35:
        raise ValueError(f"music_embed must have shape [frames,35], got {tuple(value.shape)}")
    if not torch.isfinite(value).all():
        raise ValueError("music_embed contains NaN or infinity")
    return value


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--audio", type=Path, help="Recorded audio file for raw-waveform conditioning")
    source.add_argument("--music-embed", type=Path, help="Precomputed [frames,35] GEM music embedding (.pt/.npy)")
    parser.add_argument("--genmo-root", type=Path, default=Path("/home/sunteng/Projects/WBC_Telep/GENMO"))
    parser.add_argument("--ckpt-path", type=Path, default=None)
    destination = parser.add_mutually_exclusive_group(required=True)
    destination.add_argument("--output", type=Path, help="One complete GEM smpl_params.pt")
    destination.add_argument("--output-dir", type=Path, help="Directory for ordered chunk_*/smpl_params.pt files")
    parser.add_argument("--chunk-frames", type=int, default=300)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--width", type=int, default=1000)
    parser.add_argument("--height", type=int, default=1000)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def _save_atomic(payload: dict[str, object], destination: Path) -> None:
    destination = destination.expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    torch.save(payload, str(temporary))
    temporary.replace(destination)


def _save_chunks(payload: dict[str, object], output_dir: Path, frames: int, chunk_frames: int) -> None:
    if chunk_frames <= 0:
        raise ValueError("chunk-frames must be positive")
    output_dir = output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    for sequence, start in enumerate(range(0, frames, chunk_frames)):
        end = min(frames, start + chunk_frames)
        chunk: dict[str, object] = {
            "body_params_global": {},
            "body_params_incam": {},
            "K_fullimg": payload["K_fullimg"][start:end],
            "fps": payload["fps"],
            "condition": payload["condition"],
            "source_start_frame": start,
            "source_end_frame": end,
        }
        for group in ("body_params_global", "body_params_incam"):
            source = payload[group]
            chunk[group] = {
                key: value[start:end] if hasattr(value, "ndim") and value.ndim > 0 and value.shape[0] == frames else value
                for key, value in source.items()
            }
        _save_atomic(chunk, output_dir / f"chunk_{sequence:06d}" / "smpl_params.pt")
        print(f"[gem-audio-stream] ready chunk {sequence}: frames {start}:{end}", flush=True)


def build_condition_data(args: argparse.Namespace) -> tuple[dict[str, object], int]:
    if args.fps <= 0 or not float(args.fps).is_integer():
        raise ValueError("fps must be a positive integer for GEM conditioning")
    if args.audio is not None:
        audio, sample_rate = _load_audio(args.audio)
        frames = max(1, int(np.ceil(audio.numel() / sample_rate * args.fps)))
        required = int(np.ceil(frames * sample_rate / args.fps))
        if audio.numel() < required:
            audio = torch.nn.functional.pad(audio, (0, required - audio.numel()))
        audio = audio[:required]
    else:
        music_embed = _load_music_embed(args.music_embed)
        frames = int(music_embed.shape[0])
    K = torch.eye(3, dtype=torch.float32)
    K[0, 0] = max(args.width, args.height)
    K[1, 1] = max(args.width, args.height)
    K[0, 2] = args.width / 2.0
    K[1, 2] = args.height / 2.0
    data: dict[str, object] = {
        "kp2d": torch.zeros(frames, 17, 3),
        "bbx_xys": torch.tensor([[args.width / 2.0, args.height / 2.0, float(max(args.width, args.height))]]).repeat(frames, 1),
        "K_fullimg": K.repeat(frames, 1, 1),
        "cam_angvel": torch.zeros(frames, 6),
        "cam_tvel": torch.zeros(frames, 3),
        "R_w2c": torch.eye(3).repeat(frames, 1, 1),
        "f_imgseq": torch.zeros(frames, 1024),
        "has_text": torch.tensor([False]),
        "caption": "",
        "length": torch.tensor(frames),
        "meta": [{"mode": "default"}],
        "mask": {
            "has_img_mask": torch.zeros(frames, dtype=torch.bool),
            "has_2d_mask": torch.zeros(frames, dtype=torch.bool),
            "has_cam_mask": torch.zeros(frames, dtype=torch.bool),
            "has_audio_mask": torch.ones(frames, dtype=torch.bool) if args.audio is not None else torch.zeros(frames, dtype=torch.bool),
            "has_music_mask": torch.ones(frames, dtype=torch.bool) if args.music_embed is not None else torch.zeros(frames, dtype=torch.bool),
        },
    }
    if args.audio is not None:
        data["audio_array"] = audio
        data["audio_fps"] = sample_rate
    else:
        data["music_embed"] = music_embed
        data["music_fps"] = float(args.fps)
    return data, frames


def main() -> None:
    args = _parse_args()
    # Resolve destinations before changing cwd to GENMO for its relative
    # body-model/config assets.
    if args.output is not None:
        args.output = args.output.expanduser().resolve()
    if args.output_dir is not None:
        args.output_dir = args.output_dir.expanduser().resolve()
    data, frames = build_condition_data(args)
    if args.dry_run:
        destination = args.output if args.output is not None else args.output_dir
        print(f"[gem-audio-stream] dry-run frames={frames} output={destination}")
        for key, value in data.items():
            if isinstance(value, torch.Tensor):
                print(f"  {key}: {tuple(value.shape)} {value.dtype}")
        return

    genmo_root = args.genmo_root.expanduser().resolve()
    # GENMO resolves body-model and auxiliary assets relative to its checkout.
    # Keep the sim2real caller's cwd independent from that asset contract.
    os.chdir(genmo_root)
    sys.path.insert(0, str(genmo_root / "scripts/demo"))
    sys.path.insert(0, str(genmo_root))
    from demo_utils import load_model

    ckpt = (args.ckpt_path or (genmo_root / "inputs/pretrained/gem_smpl.ckpt")).expanduser().resolve()
    model = load_model(str(ckpt), load_text_encoder=False)
    with torch.no_grad():
        pred = model.predict(data, static_cam=True)
    output = {
        "body_params_global": {key: value.detach().cpu() for key, value in pred["body_params_global"].items()},
        "body_params_incam": {key: value.detach().cpu() for key, value in pred["body_params_incam"].items()},
        "K_fullimg": data["K_fullimg"],
        "fps": float(args.fps),
        "condition": "audio" if args.audio is not None else "music_embed",
    }
    if args.output is not None:
        _save_atomic(output, args.output)
        print(f"[gem-audio-stream] saved {args.output} ({frames} frames)")
    else:
        _save_chunks(output, args.output_dir, frames, args.chunk_frames)


if __name__ == "__main__":
    main()
