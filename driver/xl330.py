"""Class-based driver for ROBOTIS DYNAMIXEL XL330 motors."""

from __future__ import annotations

import math
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from enum import IntEnum

from .bus import DynamixelBus, RegisterBus
from .errors import CommunicationError, ConfigurationError, SafetyError


class OperatingMode(IntEnum):
    VELOCITY = 1
    POSITION = 3
    EXTENDED_POSITION = 4
    CURRENT_BASED_POSITION = 5
    PWM = 16


class Register(IntEnum):
    MODEL_NUMBER = 0
    FIRMWARE_VERSION = 6
    DRIVE_MODE = 10
    OPERATING_MODE = 11
    CURRENT_LIMIT = 38
    TORQUE_ENABLE = 64
    LED = 65
    HARDWARE_ERROR = 70
    POSITION_P_GAIN = 84
    BUS_WATCHDOG = 98
    GOAL_PWM = 100
    GOAL_CURRENT = 102
    PROFILE_ACCELERATION = 108
    PROFILE_VELOCITY = 112
    GOAL_POSITION = 116
    MOVING = 122
    PRESENT_PWM = 124
    PRESENT_CURRENT = 126
    PRESENT_VELOCITY = 128
    PRESENT_POSITION = 132
    PRESENT_INPUT_VOLTAGE = 144
    PRESENT_TEMPERATURE = 146


REGISTER_SIZE = {
    Register.MODEL_NUMBER: 2,
    Register.FIRMWARE_VERSION: 1,
    Register.DRIVE_MODE: 1,
    Register.OPERATING_MODE: 1,
    Register.CURRENT_LIMIT: 2,
    Register.TORQUE_ENABLE: 1,
    Register.LED: 1,
    Register.HARDWARE_ERROR: 1,
    Register.POSITION_P_GAIN: 2,
    Register.BUS_WATCHDOG: 1,
    Register.GOAL_PWM: 2,
    Register.GOAL_CURRENT: 2,
    Register.PROFILE_ACCELERATION: 4,
    Register.PROFILE_VELOCITY: 4,
    Register.GOAL_POSITION: 4,
    Register.MOVING: 1,
    Register.PRESENT_PWM: 2,
    Register.PRESENT_CURRENT: 2,
    Register.PRESENT_VELOCITY: 4,
    Register.PRESENT_POSITION: 4,
    Register.PRESENT_INPUT_VOLTAGE: 2,
    Register.PRESENT_TEMPERATURE: 1,
}

TICKS_PER_REVOLUTION = 4096
MIN_POSITION = 0
MAX_POSITION = TICKS_PER_REVOLUTION - 1
XL330_MODEL_NUMBERS = frozenset((1190, 1200))

# Approximate slopes derived from the 5 V stall ratings in the ROBOTIS
# e-manual. XL330 current feedback measures input current, not phase current,
# so these values are useful for trends only and are not calibrated torque.
TORQUE_CONSTANT_NM_PER_AMP = {
    1190: 0.215 / 1.47,  # XL330-M077
    1200: 0.520 / 1.47,  # XL330-M288
}


@dataclass(frozen=True, slots=True)
class MotorConfig:
    """Mechanical and logical configuration for one motor."""

    motor_id: int
    name: str = ""
    zero_position: int = 2048
    direction: int = 1
    min_position: int = MIN_POSITION
    max_position: int = MAX_POSITION

    def __post_init__(self) -> None:
        if not 0 <= self.motor_id <= 252:
            raise ConfigurationError("Motor ID must be in the range 0..252.")
        if self.direction not in (-1, 1):
            raise ConfigurationError("Motor direction must be -1 or 1.")
        if not MIN_POSITION <= self.min_position <= self.max_position <= MAX_POSITION:
            raise ConfigurationError("Position limits must satisfy 0 <= min <= max <= 4095.")
        if not self.min_position <= self.zero_position <= self.max_position:
            raise ConfigurationError("Zero position must be inside the configured limits.")

    @property
    def label(self) -> str:
        return self.name or str(self.motor_id)

    def position_to_radians(self, position: int) -> float:
        delta = (int(position) - self.zero_position + 2048) % 4096 - 2048
        return self.direction * delta * 2.0 * math.pi / TICKS_PER_REVOLUTION

    def radians_to_position(self, angle: float, *, clamp: bool = False) -> int:
        if not math.isfinite(angle):
            raise ConfigurationError("Target angle must be finite.")
        position = self.zero_position + round(
            self.direction * angle * TICKS_PER_REVOLUTION / (2.0 * math.pi)
        )
        if self.min_position <= position <= self.max_position:
            return position
        if clamp:
            return max(self.min_position, min(self.max_position, position))
        raise SafetyError(
            f"Motor {self.label}: target {position} ticks is outside "
            f"[{self.min_position}, {self.max_position}]."
        )


@dataclass(frozen=True, slots=True)
class MotorState:
    motor_id: int
    position: int
    angle: float
    hardware_error: int


@dataclass(frozen=True, slots=True)
class MotorTelemetry:
    motor_id: int
    position_raw: int
    position_rad: float
    velocity_raw: int
    velocity_rpm: float
    current_raw: int
    current_ma: float
    estimated_torque_nm: float | None
    pwm_raw: int
    pwm_percent: float
    input_voltage_v: float
    temperature_c: float
    moving: bool
    torque_enabled: bool
    hardware_error: int


def _decode_signed(value: int, bits: int) -> int:
    sign = 1 << (bits - 1)
    return int(value) - (1 << bits) if int(value) & sign else int(value)


class XL330Driver:
    """High-level controller for an arbitrary set of XL330 motors.

    Opening the driver is read-only. Motion must be enabled explicitly with
    :meth:`enable_torque`, which first commands every motor to hold its current
    position before enabling torque.
    """

    def __init__(
        self,
        bus: RegisterBus,
        motors: Iterable[MotorConfig],
        *,
        validate_model: bool = True,
    ) -> None:
        motor_tuple = tuple(motors)
        if not motor_tuple:
            raise ConfigurationError("At least one motor must be configured.")
        ids = [motor.motor_id for motor in motor_tuple]
        if len(ids) != len(set(ids)):
            raise ConfigurationError("Motor IDs must be unique.")
        names = [motor.name for motor in motor_tuple if motor.name]
        if len(names) != len(set(names)):
            raise ConfigurationError("Nonempty motor names must be unique.")
        self.bus = bus
        self.motors = motor_tuple
        self.validate_model = validate_model
        self._by_id = {motor.motor_id: motor for motor in motor_tuple}
        self._torque_enabled: set[int] = set()
        self._model_numbers: dict[int, int] = {}
        self._is_open = False

    @classmethod
    def from_serial(
        cls,
        motors: Iterable[MotorConfig],
        *,
        device: str = "/dev/ttyUSB0",
        baudrate: int = 1_000_000,
        protocol_version: float = 2.0,
        validate_model: bool = True,
    ) -> XL330Driver:
        bus = DynamixelBus(device, baudrate, protocol_version)
        return cls(bus, motors, validate_model=validate_model)

    @property
    def is_open(self) -> bool:
        return self._is_open

    @property
    def motor_ids(self) -> tuple[int, ...]:
        return tuple(self._by_id)

    @property
    def model_numbers(self) -> dict[int, int]:
        return dict(self._model_numbers)

    def open(self) -> dict[int, int]:
        if self._is_open:
            return self.ping_all()
        self.bus.open()
        try:
            models = self.ping_all()
            if self.validate_model:
                wrong = {
                    motor_id: model
                    for motor_id, model in models.items()
                    if model not in XL330_MODEL_NUMBERS
                }
                if wrong:
                    details = ", ".join(f"ID {key}: model {value}" for key, value in wrong.items())
                    raise ConfigurationError(f"Expected XL330 motors; received {details}.")
        except Exception:
            self.bus.close()
            raise
        self._model_numbers = models
        self._is_open = True
        return models

    def close(self) -> list[str]:
        """Disable torque where enabled, close the port, and return cleanup errors."""
        failures: list[str] = []
        for motor_id in tuple(self._torque_enabled):
            try:
                self._write(motor_id, Register.TORQUE_ENABLE, 0)
            except Exception as exc:  # Continue trying every motor during cleanup.
                failures.append(str(exc))
        self._torque_enabled.clear()
        try:
            self.bus.close()
        except Exception as exc:
            failures.append(str(exc))
        self._is_open = False
        self._model_numbers = {}
        return failures

    def __enter__(self) -> XL330Driver:
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def _require_open(self) -> None:
        if not self._is_open:
            raise SafetyError("Open the driver before accessing motors.")

    def _config(self, motor_id: int) -> MotorConfig:
        try:
            return self._by_id[motor_id]
        except KeyError as exc:
            raise ConfigurationError(f"Motor ID {motor_id} is not configured.") from exc

    def _read(self, motor_id: int, register: Register) -> int:
        self._require_open()
        self._config(motor_id)
        return self.bus.read(motor_id, int(register), REGISTER_SIZE[register])

    def _write(self, motor_id: int, register: Register, value: int) -> None:
        self._require_open()
        self._config(motor_id)
        self.bus.write(motor_id, int(register), REGISTER_SIZE[register], value)

    def ping_all(self) -> dict[int, int]:
        if not self.bus.is_open:
            raise SafetyError("Open the bus before pinging motors.")
        return {motor_id: self.bus.ping(motor_id) for motor_id in self.motor_ids}

    def read_register(self, motor_id: int, register: Register) -> int:
        return self._read(motor_id, register)

    def write_register(self, motor_id: int, register: Register, value: int) -> None:
        """Write a model register explicitly; application code owns its safety."""
        self._write(motor_id, register, value)

    def read_positions(self) -> dict[int, int]:
        self._require_open()
        errors = self.bus.sync_read(
            self.motor_ids, int(Register.HARDWARE_ERROR), REGISTER_SIZE[Register.HARDWARE_ERROR]
        )
        failed = {motor_id: value for motor_id, value in errors.items() if value}
        if failed:
            details = ", ".join(f"ID {key}: {value:#04x}" for key, value in failed.items())
            raise CommunicationError(f"Hardware error status is nonzero ({details}).")
        return self.bus.sync_read(
            self.motor_ids, int(Register.PRESENT_POSITION), REGISTER_SIZE[Register.PRESENT_POSITION]
        )

    def read_angles(self) -> dict[int, float]:
        return {
            motor_id: self._by_id[motor_id].position_to_radians(position)
            for motor_id, position in self.read_positions().items()
        }

    def read_states(self) -> dict[int, MotorState]:
        self._require_open()
        errors = self.bus.sync_read(
            self.motor_ids, int(Register.HARDWARE_ERROR), REGISTER_SIZE[Register.HARDWARE_ERROR]
        )
        positions = self.bus.sync_read(
            self.motor_ids, int(Register.PRESENT_POSITION), REGISTER_SIZE[Register.PRESENT_POSITION]
        )
        return {
            motor_id: MotorState(
                motor_id=motor_id,
                position=positions[motor_id],
                angle=self._by_id[motor_id].position_to_radians(positions[motor_id]),
                hardware_error=errors[motor_id],
            )
            for motor_id in self.motor_ids
        }

    def read_telemetry(self) -> dict[int, MotorTelemetry]:
        """Read a synchronized feedback snapshot for all configured motors."""
        self._require_open()
        hardware_errors = self.bus.sync_read(
            self.motor_ids,
            int(Register.HARDWARE_ERROR),
            REGISTER_SIZE[Register.HARDWARE_ERROR],
        )
        torque_enabled = self.bus.sync_read(
            self.motor_ids,
            int(Register.TORQUE_ENABLE),
            REGISTER_SIZE[Register.TORQUE_ENABLE],
        )
        fields = {
            int(register): REGISTER_SIZE[register]
            for register in (
                Register.MOVING,
                Register.PRESENT_PWM,
                Register.PRESENT_CURRENT,
                Register.PRESENT_VELOCITY,
                Register.PRESENT_POSITION,
                Register.PRESENT_INPUT_VOLTAGE,
                Register.PRESENT_TEMPERATURE,
            )
        }
        feedback = self.bus.sync_read_block(
            self.motor_ids,
            int(Register.MOVING),
            int(Register.PRESENT_TEMPERATURE) - int(Register.MOVING) + 1,
            fields,
        )

        snapshots: dict[int, MotorTelemetry] = {}
        for motor_id in self.motor_ids:
            values = feedback[motor_id]
            position = _decode_signed(values[int(Register.PRESENT_POSITION)], 32)
            velocity = _decode_signed(values[int(Register.PRESENT_VELOCITY)], 32)
            current = _decode_signed(values[int(Register.PRESENT_CURRENT)], 16)
            pwm = _decode_signed(values[int(Register.PRESENT_PWM)], 16)
            torque_constant = TORQUE_CONSTANT_NM_PER_AMP.get(
                self._model_numbers.get(motor_id)
            )
            snapshots[motor_id] = MotorTelemetry(
                motor_id=motor_id,
                position_raw=position,
                position_rad=self._by_id[motor_id].position_to_radians(position),
                velocity_raw=velocity,
                velocity_rpm=velocity * 0.229,
                current_raw=current,
                current_ma=float(current),
                estimated_torque_nm=(
                    current / 1000.0 * torque_constant
                    if torque_constant is not None
                    else None
                ),
                pwm_raw=pwm,
                pwm_percent=pwm * 0.113,
                input_voltage_v=values[int(Register.PRESENT_INPUT_VOLTAGE)] * 0.1,
                temperature_c=float(values[int(Register.PRESENT_TEMPERATURE)]),
                moving=bool(values[int(Register.MOVING)]),
                torque_enabled=bool(torque_enabled[motor_id]),
                hardware_error=hardware_errors[motor_id],
            )
        return snapshots

    def enable_torque(self, motor_ids: Iterable[int] | None = None) -> dict[int, int]:
        """Hold current positions, then enable torque on the requested motors."""
        self._require_open()
        ids = self._normalize_ids(motor_ids)
        modes = {motor_id: self._read(motor_id, Register.OPERATING_MODE) for motor_id in ids}
        invalid = {
            motor_id: mode
            for motor_id, mode in modes.items()
            if mode not in (OperatingMode.POSITION, OperatingMode.CURRENT_BASED_POSITION)
        }
        if invalid:
            details = ", ".join(f"ID {key}: mode {value}" for key, value in invalid.items())
            raise SafetyError(f"Position commands require mode 3 or 5 ({details}).")

        all_positions = self.read_positions()
        holds = {motor_id: all_positions[motor_id] for motor_id in ids}
        outside_limits = {
            motor_id: position
            for motor_id, position in holds.items()
            if not self._by_id[motor_id].min_position
            <= position
            <= self._by_id[motor_id].max_position
        }
        if outside_limits:
            details = ", ".join(
                f"ID {motor_id}: {position}" for motor_id, position in outside_limits.items()
            )
            raise SafetyError(f"Current positions are outside configured limits ({details}).")
        self.bus.sync_write(
            holds, int(Register.GOAL_POSITION), REGISTER_SIZE[Register.GOAL_POSITION]
        )
        enabled_now: list[int] = []
        try:
            for motor_id in ids:
                self._write(motor_id, Register.TORQUE_ENABLE, 1)
                enabled_now.append(motor_id)
                self._torque_enabled.add(motor_id)
        except Exception:
            for motor_id in enabled_now:
                try:
                    self._write(motor_id, Register.TORQUE_ENABLE, 0)
                except Exception:
                    pass
                self._torque_enabled.discard(motor_id)
            raise
        return holds

    def disable_torque(self, motor_ids: Iterable[int] | None = None) -> list[str]:
        self._require_open()
        ids = self._normalize_ids(motor_ids)
        failures: list[str] = []
        for motor_id in ids:
            try:
                self._write(motor_id, Register.TORQUE_ENABLE, 0)
                self._torque_enabled.discard(motor_id)
            except Exception as exc:
                failures.append(str(exc))
        return failures

    def command_positions(self, targets: Mapping[int, int]) -> None:
        self._require_open()
        normalized: dict[int, int] = {}
        for motor_id, position in targets.items():
            config = self._config(motor_id)
            if motor_id not in self._torque_enabled:
                raise SafetyError(f"Motor {config.label}: torque is not enabled by this driver.")
            position = int(position)
            if not config.min_position <= position <= config.max_position:
                raise SafetyError(
                    f"Motor {config.label}: target {position} is outside "
                    f"[{config.min_position}, {config.max_position}]."
                )
            normalized[motor_id] = position
        self.bus.sync_write(
            normalized, int(Register.GOAL_POSITION), REGISTER_SIZE[Register.GOAL_POSITION]
        )

    def command_angles(self, targets: Mapping[int, float], *, clamp: bool = False) -> None:
        positions = {
            motor_id: self._config(motor_id).radians_to_position(angle, clamp=clamp)
            for motor_id, angle in targets.items()
        }
        self.command_positions(positions)

    def configure_position_control(
        self,
        *,
        operating_mode: OperatingMode | None = None,
        current_limit: int | None = None,
        position_p_gain: int | None = None,
        profile_velocity: int | None = None,
        profile_acceleration: int | None = None,
        watchdog: int | None = None,
    ) -> None:
        """Apply explicit position-control settings while torque is disabled.

        Operating mode and current limit are EEPROM-backed on XL330 motors.
        The method changes only values supplied by the caller.
        """
        self._require_open()
        if self._torque_enabled:
            raise SafetyError("Disable torque before changing control settings.")
        if operating_mode not in (None, OperatingMode.POSITION, OperatingMode.CURRENT_BASED_POSITION):
            raise ConfigurationError("This driver supports position modes 3 and 5.")
        settings = (
            (Register.OPERATING_MODE, operating_mode),
            (Register.CURRENT_LIMIT, current_limit),
            (Register.POSITION_P_GAIN, position_p_gain),
            (Register.PROFILE_VELOCITY, profile_velocity),
            (Register.PROFILE_ACCELERATION, profile_acceleration),
            (Register.BUS_WATCHDOG, watchdog),
        )
        for register, value in settings:
            if value is None:
                continue
            if int(value) < 0:
                raise ConfigurationError(f"{register.name} must be nonnegative.")
            for motor_id in self.motor_ids:
                self._write(motor_id, register, int(value))

    def _normalize_ids(self, motor_ids: Iterable[int] | None) -> tuple[int, ...]:
        ids = self.motor_ids if motor_ids is None else tuple(motor_ids)
        if not ids:
            raise ConfigurationError("At least one motor ID is required.")
        if len(ids) != len(set(ids)):
            raise ConfigurationError("Motor IDs must be unique.")
        for motor_id in ids:
            self._config(motor_id)
        return ids

