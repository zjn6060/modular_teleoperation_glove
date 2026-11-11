#!/usr/bin/env python3
"""Calibrate the glove servos' zero position and range of motion.

1. Torque is disabled so the fingers can be moved by hand.
2. Put every finger's EF rod parallel to the palm plane and press Enter;
   these ticks become each servo's zero at the EF-horizontal reference.
3. Move every finger through its full range and press Enter; the extremes
   become the position limits used to clamp sim-to-real commands.

Motor IDs, joint mappings and directions are kept from an existing config.
The sim joint offset is read from the EF-horizontal reference pose, so the
newly captured physical zero and the preview use the same reference.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from driver import MotorConfig
from utils.glove_servos import CONFIG_PATH, DEFAULT_SERVOS, open_driver, save_config
from simulation.glove.reference_pose import REFERENCE_POSE, reference_angles


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--output", type=Path, default=CONFIG_PATH)
    parser.add_argument("--simulate", action="store_true")
    return parser.parse_args(argv)


def sample_until_enter(driver, prompt, on_sample):
    """Read positions continuously, showing them, until the user presses Enter."""
    print(prompt)
    done = threading.Event()
    threading.Thread(target=lambda: (sys.stdin.readline(), done.set()), daemon=True).start()
    positions = driver.read_positions()
    while not done.is_set():
        positions = driver.read_positions()
        line = on_sample(positions)
        print(f"\r{line}   ", end="", flush=True)
        time.sleep(0.05)
    print()
    return positions


def wrap_delta(position, zero):
    return (position - zero + 2048) % 4096 - 2048


def main(argv=None):
    args = parse_args(argv)
    previous = {}
    definitions = DEFAULT_SERVOS
    if args.output.exists():
        previous = {s["motor_id"]: s for s in json.loads(args.output.read_text())["servos"]}
        definitions = tuple((s["finger"], s["motor_id"], s["joint"])
                            for s in previous.values())
    offsets = reference_angles()
    for _, _, joint in definitions:
        if joint not in offsets:
            raise SystemExit(f"Unknown reference actuator: {joint}")

    names = {motor_id: finger for finger, motor_id, _ in definitions}
    motors = [MotorConfig(motor_id=motor_id, name=finger) for finger, motor_id, _ in definitions]
    driver = open_driver(motors, args.device, args.baud, args.simulate)
    with driver:
        failures = driver.disable_torque()
        if failures:
            raise SystemExit("Could not disable torque: " + "; ".join(failures))
        print("Torque disabled on all servos; fingers can be moved by hand.\n")
        for finger, motor_id, joint in definitions:
            print(f"  {finger}: ID {motor_id} -> {joint}")

        zeros = sample_until_enter(
            driver,
            "Step 1: put all four EF rods parallel to the palm plane, then press Enter.",
            lambda p: "  ".join(f"{names[i]}={p[i]}" for i in p),
        )
        low = {motor_id: 0 for motor_id in zeros}
        high = {motor_id: 0 for motor_id in zeros}

        def track(positions):
            for motor_id, position in positions.items():
                delta = wrap_delta(position, zeros[motor_id])
                low[motor_id] = min(low[motor_id], delta)
                high[motor_id] = max(high[motor_id], delta)
            return "  ".join(f"{names[i]}=[{low[i]:+d},{high[i]:+d}]" for i in positions)

        sample_until_enter(
            driver,
            "\nStep 2: move every finger through its full range, then press Enter.",
            track,
        )

    servos = []
    for finger, motor_id, joint in definitions:
        zero = zeros[motor_id]
        lo, hi = zero + low[motor_id], zero + high[motor_id]
        if lo < 0 or hi > 4095:
            raise SystemExit(
                f"{finger}: range [{lo}, {hi}] crosses the encoder wrap; "
                "re-mount the horn so the range stays within 0..4095."
            )
        servos.append({
            "finger": finger,
            "motor_id": motor_id,
            "joint": joint,
            "zero_position": zero,
            "direction": previous.get(motor_id, {}).get("direction", 1),
            "offset": offsets[joint],
            "min_position": lo,
            "max_position": hi,
        })
    save_config({"device": args.device, "baudrate": args.baud,
                 "reference_pose": REFERENCE_POSE, "servos": servos}, args.output)

    print(f"\nSaved {args.output}")
    for servo in servos:
        span = (servo["max_position"] - servo["min_position"]) * 360 / 4096
        print(f"  {servo['finger']:<6} ID {servo['motor_id']}  zero {servo['zero_position']}  "
              f"range [{servo['min_position']}, {servo['max_position']}] ({span:.0f} deg)")


if __name__ == "__main__":
    main()
