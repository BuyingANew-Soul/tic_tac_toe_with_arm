"""What does the arm report as its position? Sends NO movement commands.

Run it from the ttt folder, with the main program stopped:

    python3 tools/feedback_probe.py > probe.txt

and follow the prompts (they appear on screen, everything is also saved in
probe.txt). Send probe.txt back.

Steps:
  1. Prints everything the arm sends on its own for 3 s after connecting.
  2. Asks for the position 6 times, 0.3 s apart, without touching the arm.
  3. Torque OFF: you move the arm slowly by hand for ~6 s while it keeps reading.
  4. Torque ON (hold the arm first!): 6 more readings.
"""
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ttt.arm import CMD_FEEDBACK, CMD_TORQUE, find_port  # noqa: E402

import serial  # noqa: E402

T0 = time.time()


def say(msg):
    print(msg, flush=True)
    print(msg, file=sys.stderr, flush=True)


def read_lines(ser, seconds):
    end, buf, out = time.time() + seconds, b"", []
    while time.time() < end:
        buf += ser.read(256)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode(errors="replace").strip()
            if text:
                out.append(text)
    return out


def ask(ser, label):
    ser.write((json.dumps({"T": CMD_FEEDBACK}) + "\n").encode())
    lines = read_lines(ser, 0.3)
    fb = None
    for text in lines:
        try:
            m = json.loads(text)
        except json.JSONDecodeError:
            continue
        if m.get("T") == 1051:
            fb = m
    if fb:
        keys = " ".join(f"{k}={fb[k]:.2f}" if isinstance(fb[k], float) else f"{k}={fb[k]}"
                        for k in ("x", "y", "z", "t", "b", "s", "e") if k in fb)
        print(f"{time.time()-T0:6.2f}s {label:10s} {keys}", flush=True)
        print(f"{'':17s} raw: {json.dumps(fb)}", flush=True)
    else:
        print(f"{time.time()-T0:6.2f}s {label:10s} NO REPLY. other lines: {lines}", flush=True)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port() or "/dev/ttyUSB0"
    say(f"port {port}")
    ser = serial.Serial(port, 115200, timeout=0.05)
    ser.setDTR(False)
    ser.setRTS(False)

    say("1) listening 3 s (arm may restart when the port opens)")
    for text in read_lines(ser, 3.0):
        print(f"   arm says: {text[:200]}", flush=True)

    say("2) six readings, arm untouched")
    for i in range(6):
        ask(ser, f"still {i+1}")

    say("3) HOLD THE ARM, then press Enter: torque goes OFF and the arm goes limp")
    input()
    ser.write((json.dumps({"T": CMD_TORQUE, "cmd": 0}) + "\n").encode())
    say("   torque OFF - now move the arm slowly by hand (about 6 s)")
    for i in range(20):
        ask(ser, f"by hand {i+1}")

    say("4) HOLD THE ARM where it is, then press Enter: torque goes back ON")
    input()
    ser.write((json.dumps({"T": CMD_TORQUE, "cmd": 1}) + "\n").encode())
    time.sleep(0.5)
    for i in range(6):
        ask(ser, f"torque on {i+1}")
    ser.close()
    say("done - send probe.txt")


if __name__ == "__main__":
    main()
