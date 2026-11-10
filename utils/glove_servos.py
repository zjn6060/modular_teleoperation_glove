"""Mapping between the glove's flexion servos and the MuJoCo horn joints."""

from __future__ import annotations

import json
from pathlib import Path

from driver import DynamixelBus, MotorConfig, SimulatedBus, XL330Driver
from driver.errors import ConfigurationError

CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "servos.json"

# (finger, motor ID, MuJoCo joint/actuator driven by the servo horn)
DEFAULT_SERVOS = (
    ("index", 2, "j2"),
    ("middle", 3, "j10"),
    ("ring", 4, "j26"),
    ("little", 5, "j18"),
)


def load_config(path: Path = CONFIG_PATH) -> dict:
    if not path.exists():
        raise ConfigurationError(
            f"{path} not found; run scripts/calibration.py first."
        )
    return json.loads(path.read_text())


def save_config(config: dict, path: Path = CONFIG_PATH) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(config, indent=2, ensure_ascii=False) + "\n")


def motor_configs(config: dict) -> list[MotorConfig]:
    return [
        MotorConfig(
            motor_id=servo["motor_id"],
            name=servo["finger"],
            zero_position=servo["zero_position"],
            direction=servo["direction"],
            min_position=servo["min_position"],
            max_position=servo["max_position"],
        )
        for servo in config["servos"]
    ]


def open_driver(motors: list[MotorConfig], device: str, baud: int, simulate: bool) -> XL330Driver:
    ids = [motor.motor_id for motor in motors]
    bus = SimulatedBus(ids) if simulate else DynamixelBus(device, baud)
    return XL330Driver(bus, motors)
