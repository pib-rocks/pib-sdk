#!/usr/bin/env python3
"""Lightweight audio test using pib3 high-level API.

Plays a short tone, runs TTS, or records audio using the Robot/Webots backends.
All audio operations go through the pib3 package methods (play_audio, speak,
record_audio) rather than internal classes.

Usage:
  # Play a 1s 440Hz tone locally (uses Robot backend with LOCAL output)
  python examples/test_audio_simple.py --demo tone

  # Play tone on the real robot speaker
  python examples/test_audio_simple.py --demo tone --backend robot

  # Record 3s from the default microphone and play it back
  python examples/test_audio_simple.py --demo record

  # Run TTS locally (requires Piper TTS model)
  python examples/test_audio_simple.py --demo tts --text "Hello from pib3"

  # Run TTS on robot speaker
  python examples/test_audio_simple.py --demo tts --backend robot --text "Hello"

Notes:
- All demos use the pib3 Robot/Webots high-level API consistently
- TTS uses robot.speak() / webots.speak() methods
- Audio playback uses robot.play_audio() / webots.play_audio() methods
- Recording uses robot.record_audio() / local sounddevice fallback
"""

from __future__ import annotations

import argparse
from typing import Optional

import numpy as np

from pib3.backends.audio import DEFAULT_SAMPLE_RATE
from pib3 import Robot, Webots, AudioOutput, AudioInput


def generate_tone(frequency: float = 440.0, duration: float = 1.0, sr: int = DEFAULT_SAMPLE_RATE) -> np.ndarray:
    """Generate a sine wave tone as int16 samples."""
    t = np.linspace(0, duration, int(sr * duration), endpoint=False)
    return (np.sin(2 * np.pi * frequency * t) * 16000).astype(np.int16)


def parse_args(argv: Optional[list[str]] = None):
    p = argparse.ArgumentParser(
        prog="test_audio_simple.py",
        description="Test pib3 audio API with various backends and demos",
    )
    p.add_argument("--demo", choices=("tone", "tts", "record"), default="tone",
                   help="Demo to run: tone, tts, or record")
    p.add_argument("--frequency", "-f", type=float, default=440.0,
                   help="Tone frequency in Hz (default: 440)")
    p.add_argument("--duration", "-d", type=float, default=5.0,
                   help="Duration in seconds (default: 5.0)")
    p.add_argument("--text", type=str, default="Hallo von pib3",
                   help="Text for TTS demo")
    p.add_argument(
        "--backend",
        choices=("local", "robot", "webots"),
        default="local",
        help="Backend: local (laptop speaker), robot (real robot), webots (simulator)",
    )
    p.add_argument("--host", default="172.26.34.149",
                   help="Robot IP for --backend robot")
    return p.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = parse_args(argv)
    backend_name = args.backend

    # Determine output target based on backend
    backend = None
    needs_connection = True

    if backend_name == "local":
        output = AudioOutput.LOCAL
        print("Using local audio output (laptop speaker)")
        # For local-only playback, we can use Robot without ROS connection
        # since local audio uses sounddevice directly
        backend = Robot()
        needs_connection = False  # Local audio doesn't need ROS
    elif backend_name == "webots":
        output = AudioOutput.LOCAL  # Webots audio plays locally
        print("Using Webots backend (audio plays locally)")
        backend = Webots()
    else:  # robot
        output = AudioOutput.ROBOT
        print(f"Using robot backend at {args.host} (audio plays on robot speaker)")
        backend = Robot(host=args.host)

    # Connect to backend (only if needed for robot/webots audio)
    if needs_connection:
        try:
            backend.connect()
        except Exception as exc:
            print(f"Failed to connect to {backend_name} backend:", exc)
            return 1

    exit_code = 0
    try:
        if args.demo == "tone":
            tone = generate_tone(args.frequency, args.duration)
            print(f"Playing {args.frequency:.0f}Hz tone for {args.duration:.2f}s...")
            ok = backend.play_audio(tone, output=output)
            print("Result:", "OK" if ok else "FAILED")
            exit_code = 0 if ok else 2

        elif args.demo == "tts":
            print(f"Speaking: {args.text!r}")
            ok = backend.speak(args.text, output=output)
            print("Result:", "OK" if ok else "FAILED")
            exit_code = 0 if ok else 2

        elif args.demo == "record":
            print(f"Recording {args.duration:.1f}s of audio...")
            # For local backend, use LOCAL input; for robot, use ROBOT input
            if backend_name == "robot":
                audio_data = backend.record_audio(duration=args.duration, input_source=AudioInput.ROBOT)
            else:
                audio_data = backend.record_audio(duration=args.duration, input_source=AudioInput.LOCAL)
            
            if audio_data is None or len(audio_data) == 0:
                print("No audio received")
                exit_code = 3
            else:
                print(f"Recorded {len(audio_data)} samples — playing back...")
                ok = backend.play_audio(audio_data, output=output)
                print("Playback result:", "OK" if ok else "FAILED")
                exit_code = 0 if ok else 2

    finally:
        if needs_connection:
            backend.disconnect()

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
