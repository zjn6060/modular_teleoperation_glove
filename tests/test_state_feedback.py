"""Read and plot live state feedback for every configured XL330 motor.

This diagnostic is read-only: it does not enable torque or write registers.
Close the figure or press Ctrl+C to stop.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from driver import (
    DriverError,
    DynamixelBus,
    MotorConfig,
    SimulatedBus,
    XL330Driver,
)
from driver.xl330 import XL330_MODEL_NUMBERS
from utils.debugging import RealtimeMotorPlot


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument(
        "--ids",
        type=int,
        nargs="+",
        help="Motor IDs to plot; omitted by default to scan IDs 0 through 10",
    )
    parser.add_argument("--names", nargs="+", help="Optional name for each motor")
    parser.add_argument("--rate", type=float, default=20.0, help="Sampling rate in Hz")
    parser.add_argument("--window", type=float, default=20.0, help="Visible history in seconds")
    parser.add_argument("--duration", type=float, default=0.0, help="Stop after N seconds; 0 runs until closed")
    parser.add_argument("--simulate", action="store_true")
    args = parser.parse_args(argv)
    if args.ids is not None and (
        len(args.ids) != len(set(args.ids))
        or any(not 0 <= value <= 252 for value in args.ids)
    ):
        parser.error("Motor IDs must be distinct values in 0..252.")
    if args.names is not None and args.ids is None:
        parser.error("--names can only be used together with explicit --ids.")
    if args.names is not None and len(args.names) != len(args.ids):
        parser.error("--names must contain one name for every motor ID.")
    if args.baud <= 0:
        parser.error("Baud must be positive.")
    if not math.isfinite(args.rate) or not 0 < args.rate <= 100:
        parser.error("Rate must be finite and in 0..100 Hz.")
    if not math.isfinite(args.window) or args.window <= 0:
        parser.error("Window must be finite and positive.")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("Duration must be finite and nonnegative.")
    return args


def discover_motor_ids(device, baud, candidates=range(11)):
    bus = DynamixelBus(device, baud)
    discovered = []
    bus.open()
    try:
        print(f"Scanning IDs {min(candidates)} through {max(candidates)}...")
        for motor_id in candidates:
            try:
                model = bus.ping(motor_id)
            except DriverError as exc:
                print(f"  ID {motor_id}: unavailable ({exc})")
                continue
            if model not in XL330_MODEL_NUMBERS:
                print(f"  ID {motor_id}: model {model}, skipped (not XL330)")
                continue
            print(f"  ID {motor_id}: XL330 model {model}")
            discovered.append(motor_id)
    finally:
        bus.close()
    return discovered


def main(argv=None):
    args = parse_args(argv)
    if args.ids is not None:
        motor_ids = args.ids
    elif args.simulate:
        motor_ids = list(range(11))
    else:
        try:
            motor_ids = discover_motor_ids(args.device, args.baud)
        except (DriverError, OSError) as exc:
            print(f"Motor scan failed: {exc}", file=sys.stderr)
            return 1
    if not motor_ids:
        print("No healthy XL330 motors found in IDs 0 through 10.", file=sys.stderr)
        return 1

    names = args.names or [f"Motor {motor_id}" for motor_id in motor_ids]
    motors = [
        MotorConfig(motor_id=motor_id, name=name)
        for motor_id, name in zip(motor_ids, names)
    ]
    driver = (
        XL330Driver(SimulatedBus(motor_ids), motors)
        if args.simulate
        else XL330Driver.from_serial(
            motors,
            device=args.device,
            baudrate=args.baud,
        )
    )
    plot = None
    started = time.monotonic()
    try:
        models = driver.open()
        for motor in motors:
            print(
                f"ID {motor.motor_id} ({motor.name}): connected, "
                f"model={models[motor.motor_id]}"
            )
        plot = RealtimeMotorPlot(motors, window_seconds=args.window)
        period = 1.0 / args.rate
        while plot.is_open and (
            args.duration == 0 or time.monotonic() - started < args.duration
        ):
            tick = time.monotonic()
            plot.update(driver.read_telemetry(), timestamp=tick)
            time.sleep(max(0.0, period - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        print("Stopped by user.")
    except (DriverError, OSError) as exc:
        print(f"State feedback failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if plot is not None:
            plot.close()
        cleanup_errors = driver.close()
        for error in cleanup_errors:
            print(f"Cleanup warning: {error}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
