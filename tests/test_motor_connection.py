"""Hardware connection report for every configured DYNAMIXEL motor.

Run directly; this diagnostic never enables torque or writes motor registers.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from driver import DriverError, DynamixelBus, Register, SimulatedBus
from driver.xl330 import REGISTER_SIZE


MODEL_NAMES = {
    1190: "XL330-M077",
    1200: "XL330-M288",
}


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument(
        "--ids",
        type=int,
        nargs="+",
        default=list(range(11)),
        help="Motor IDs to scan (default: 0 through 10)",
    )
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--simulate", action="store_true")
    args = parser.parse_args(argv)
    if len(args.ids) != len(set(args.ids)) or any(not 0 <= value <= 252 for value in args.ids):
        parser.error("Motor IDs must be distinct values in 0..252.")
    if args.baud <= 0 or args.retries < 1:
        parser.error("Baud must be positive and retries must be at least one.")
    return args


def read_register(bus, motor_id, register):
    return bus.read(motor_id, int(register), REGISTER_SIZE[register])


def failed_report(motor_id, detail, latency="-"):
    return {
        "id": str(motor_id),
        "status": "FAILED",
        "model": "-",
        "firmware": "-",
        "mode": "-",
        "torque": "-",
        "position": "-",
        "voltage": "-",
        "temp": "-",
        "hw_error": "-",
        "latency": latency,
        "detail": detail,
    }


def inspect_motor(bus, motor_id, retries):
    started = time.perf_counter()
    last_error = ""
    for attempt in range(1, retries + 1):
        try:
            model = bus.ping(motor_id)
            report = {
                "id": str(motor_id),
                "status": "CONNECTED",
                "model": f"{MODEL_NAMES.get(model, 'unknown')} ({model})",
                "firmware": str(read_register(bus, motor_id, Register.FIRMWARE_VERSION)),
                "mode": str(read_register(bus, motor_id, Register.OPERATING_MODE)),
                "torque": "ON" if read_register(bus, motor_id, Register.TORQUE_ENABLE) else "OFF",
                "position": str(read_register(bus, motor_id, Register.PRESENT_POSITION)),
                "voltage": f"{read_register(bus, motor_id, Register.PRESENT_INPUT_VOLTAGE) * 0.1:.1f} V",
                "temp": f"{read_register(bus, motor_id, Register.PRESENT_TEMPERATURE)} C",
                "hw_error": f"{read_register(bus, motor_id, Register.HARDWARE_ERROR):#04x}",
                "latency": f"{(time.perf_counter() - started) * 1000.0:.1f} ms",
                "detail": "OK",
            }
            return report
        except (DriverError, OSError) as exc:
            last_error = str(exc)
            if attempt < retries:
                time.sleep(0.05)
    return failed_report(
        motor_id,
        last_error,
        f"{(time.perf_counter() - started) * 1000.0:.1f} ms",
    )


def print_table(reports):
    columns = (
        ("id", "ID"),
        ("status", "STATUS"),
        ("model", "MODEL"),
        ("firmware", "FW"),
        ("mode", "MODE"),
        ("torque", "TORQUE"),
        ("position", "POSITION"),
        ("voltage", "VOLTAGE"),
        ("temp", "TEMP"),
        ("hw_error", "HW ERROR"),
        ("latency", "TIME"),
    )
    widths = {
        key: max(len(title), *(len(report[key]) for report in reports))
        for key, title in columns
    }
    header = " | ".join(title.ljust(widths[key]) for key, title in columns)
    print(header)
    print("-+-".join("-" * widths[key] for key, _ in columns))
    for report in reports:
        print(" | ".join(report[key].ljust(widths[key]) for key, _ in columns))
        if report["status"] != "CONNECTED":
            print(f"   detail: {report['detail']}")


def main(argv=None):
    args = parse_args(argv)
    bus = (
        SimulatedBus(args.ids)
        if args.simulate
        else DynamixelBus(args.device, args.baud)
    )
    try:
        bus.open()
    except (DriverError, OSError) as exc:
        reports = [failed_report(motor_id, f"Bus unavailable: {exc}") for motor_id in args.ids]
        print(f"\nDevice: {'SIMULATED' if args.simulate else args.device} @ {args.baud} baud")
        print_table(reports)
        print(f"\nSummary: 0/{len(reports)} connected, {len(reports)} failed")
        return 2

    try:
        reports = [inspect_motor(bus, motor_id, args.retries) for motor_id in args.ids]
    finally:
        bus.close()

    print(f"\nDevice: {'SIMULATED' if args.simulate else args.device} @ {args.baud} baud")
    print_table(reports)
    connected = sum(report["status"] == "CONNECTED" for report in reports)
    failed = len(reports) - connected
    print(f"\nSummary: {connected}/{len(reports)} connected, {failed} failed")
    return 0 if failed == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
