"""Save PCM as a WAV file or capture one JPEG from RobotDogSensing.ino.

Install: python -m pip install pyserial
Examples: python capture_test.py audio COM5 --channel left --seconds 5
          python capture_test.py camera COM5
Close Arduino Serial Monitor before running this script.
"""
from __future__ import annotations

import argparse
import re
import time
import wave
from pathlib import Path


def wait_line(port, pattern: bytes, seconds: float = 15.0) -> re.Match:
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        line = port.readline()
        if not line:
            continue
        match = re.fullmatch(pattern, line.strip())
        if match:
            return match
        if b"ERROR:" in line:
            raise RuntimeError(line.decode("utf-8", errors="replace").strip())
        if len(line) < 300:
            print(line.decode("utf-8", errors="replace").strip())
    raise TimeoutError("No expected reply. Check test mode, COM port, baud and wiring.")


def read_exact(port, count: int, seconds: float) -> bytes:
    deadline = time.monotonic() + seconds
    data = bytearray()
    while len(data) < count and time.monotonic() < deadline:
        piece = port.read(min(4096, count - len(data)))
        if piece:
            data.extend(piece)
    if len(data) != count:
        raise RuntimeError(f"Truncated transfer: got {len(data)} of {count} bytes.")
    return bytes(data)


def receive_audio(port, seconds: int, output: Path) -> None:
    port.write(f"A{seconds}\n".encode("ascii"))
    header = wait_line(port, rb"PCM_BEGIN (\d+) (\d+)")
    count, sample_rate = map(int, header.groups())
    if sample_rate != 16000 or count != seconds * sample_rate * 2:
        raise RuntimeError("Unexpected PCM size or sample rate; check firmware configuration.")
    pcm = read_exact(port, count, seconds + 10)
    footer = wait_line(port, rb"PCM_END (\d+) dropped=(\d+)", seconds=5)
    if int(footer[1]) != count:
        raise RuntimeError("Firmware reported an incomplete recording; no WAV was saved.")
    drops = int(footer[2])
    if drops:
        raise RuntimeError(f"Recording lost {drops} chunks. No WAV saved; fix throughput first.")
    with wave.open(str(output), "wb") as wav:
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        wav.writeframes(pcm)
    print(f"Saved {output.resolve()} ({seconds} s, mono, signed 16-bit PCM)")


def receive_camera(port, output: Path) -> None:
    port.write(b"J\n")
    header = wait_line(port, rb"JPEG_BEGIN (\d+) (\d+) (\d+)")
    count, width, height = map(int, header.groups())
    if not 4 <= count <= 2_000_000:
        raise RuntimeError("Invalid JPEG byte count.")
    jpeg = read_exact(port, count, seconds=20)
    wait_line(port, rb"JPEG_END", seconds=5)
    if not jpeg.startswith(b"\xff\xd8") or not jpeg.endswith(b"\xff\xd9"):
        raise RuntimeError("JPEG marker check failed; no image was saved.")
    output.write_bytes(jpeg)
    print(f"Saved {output.resolve()} ({width} x {height}, {count} bytes)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("kind", choices=["audio", "camera"])
    parser.add_argument("port", help="For example COM5; see Arduino Tools > Port")
    parser.add_argument("--seconds", type=int, default=5, choices=range(1, 11))
    parser.add_argument("--channel", choices=["left", "right", "mix"], default="left")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("Install pyserial first: python -m pip install pyserial") from exc
    output = args.output or Path(
        f"mic_{args.channel}.wav" if args.kind == "audio" else "camera.jpg")
    if output.exists():
        raise SystemExit(f"Output already exists: {output}. Use --output with another filename.")
    try:
        with serial.Serial(args.port, args.baud, timeout=0.5, write_timeout=3) as port:
            # Some USB/UART bridges reset the ESP32 on open; others do not.
            time.sleep(2)
            port.reset_input_buffer()
            if args.kind == "audio":
                channel_cmd = {"left": b"L\n", "right": b"R\n", "mix": b"M\n"}
                port.write(channel_cmd[args.channel])
                wait_line(port, b"CHANNEL " + args.channel.upper().encode())
                receive_audio(port, args.seconds, output)
            else:
                receive_camera(port, output)
    except (OSError, RuntimeError, TimeoutError, serial.SerialException) as exc:
        raise SystemExit(str(exc)) from exc


if __name__ == "__main__":
    main()
