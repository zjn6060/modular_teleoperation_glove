"""Read-only diagnostics for the XL330 driver."""

from __future__ import annotations

import argparse
import math
import time

from .bus import SimulatedBus
from .xl330 import MotorConfig, XL330Driver


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("check", "monitor"))
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--ids", type=int, nargs="+", default=[1, 2, 3, 4])
    parser.add_argument("--rate", type=float, default=10.0, help="Monitor rate in Hz")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds; 0 runs until Ctrl+C")
    parser.add_argument("--simulate", action="store_true")
    return parser


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = _parser()
    args = parser.parse_args(argv)
    if len(args.ids) != len(set(args.ids)) or any(not 0 <= motor_id <= 252 for motor_id in args.ids):
        parser.error("Motor IDs must be distinct values in 0..252.")
    if args.baud <= 0:
        parser.error("Baud rate must be positive.")
    if not math.isfinite(args.rate) or not 0 < args.rate <= 1000:
        parser.error("Rate must be finite and in 0..1000 Hz.")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("Duration must be finite and nonnegative.")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    motors = [MotorConfig(motor_id=motor_id) for motor_id in args.ids]
    if args.simulate:
        driver = XL330Driver(SimulatedBus(args.ids), motors)
    else:
        driver = XL330Driver.from_serial(
            motors, device=args.device, baudrate=args.baud
        )

    try:
        with driver:
            if args.command == "check":
                models = driver.ping_all()
                states = driver.read_states()
                for motor_id in driver.motor_ids:
                    state = states[motor_id]
                    print(
                        f"ID {motor_id}: model={models[motor_id]} "
                        f"position={state.position} angle={math.degrees(state.angle):.2f} deg "
                        f"hardware_error={state.hardware_error:#04x}"
                    )
                return 0

            start = time.monotonic()
            while args.duration == 0 or time.monotonic() - start < args.duration:
                tick = time.monotonic()
                states = driver.read_states()
                output = "  ".join(
                    f"ID {motor_id}: {state.position:4d} ({math.degrees(state.angle):7.2f} deg)"
                    for motor_id, state in states.items()
                )
                print(output, flush=True)
                time.sleep(max(0.0, 1.0 / args.rate - (time.monotonic() - tick)))
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
