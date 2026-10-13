"""Low-level DYNAMIXEL bus implementations.

The real bus imports ``dynamixel_sdk`` lazily, so applications can import and
test the rest of the package without serial hardware or the SDK installed.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from typing import Protocol, runtime_checkable

from .errors import CommunicationError, ConfigurationError, ConnectionError


@runtime_checkable
class RegisterBus(Protocol):
    """Transport interface consumed by model-specific motor drivers."""

    @property
    def is_open(self) -> bool: ...

    def open(self) -> None: ...

    def close(self) -> None: ...

    def ping(self, motor_id: int) -> int: ...

    def read(self, motor_id: int, address: int, size: int) -> int: ...

    def write(self, motor_id: int, address: int, size: int, value: int) -> None: ...

    def sync_read(self, motor_ids: Iterable[int], address: int, size: int) -> dict[int, int]: ...

    def sync_read_block(
        self,
        motor_ids: Iterable[int],
        start_address: int,
        length: int,
        fields: Mapping[int, int],
    ) -> dict[int, dict[int, int]]: ...

    def sync_write(self, values: Mapping[int, int], address: int, size: int) -> None: ...


class DynamixelBus:
    """A reusable DYNAMIXEL serial bus using Protocol 2.0 by default."""

    VALID_REGISTER_SIZES = (1, 2, 4)

    def __init__(
        self,
        device: str = "/dev/ttyUSB0",
        baudrate: int = 1_000_000,
        protocol_version: float = 2.0,
    ) -> None:
        if not device:
            raise ConfigurationError("A serial device is required.")
        if baudrate <= 0:
            raise ConfigurationError("Baud rate must be positive.")
        self.device = device
        self.baudrate = int(baudrate)
        self.protocol_version = float(protocol_version)
        self._sdk = None
        self._port = None
        self._packet = None
        self._is_open = False

    @property
    def is_open(self) -> bool:
        return self._is_open

    def open(self) -> None:
        if self._is_open:
            return
        try:
            import dynamixel_sdk as sdk
        except ImportError as exc:
            raise ConnectionError(
                "dynamixel_sdk is not installed; activate the myglove environment."
            ) from exc

        port = sdk.PortHandler(self.device)
        packet = sdk.PacketHandler(self.protocol_version)
        if not port.openPort():
            raise ConnectionError(
                f"Cannot open {self.device}; check the cable, serial permissions, "
                "and whether another program owns the port."
            )
        if not port.setBaudRate(self.baudrate):
            port.closePort()
            raise ConnectionError(
                f"Cannot set {self.device} to {self.baudrate} baud."
            )

        self._sdk = sdk
        self._port = port
        self._packet = packet
        self._is_open = True

    def close(self) -> None:
        if self._port is not None and self._is_open:
            self._port.closePort()
        self._is_open = False

    def __enter__(self) -> DynamixelBus:
        self.open()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def _require_open(self) -> None:
        if not self._is_open or self._packet is None or self._port is None:
            raise ConnectionError("The DYNAMIXEL bus is not open.")

    @classmethod
    def _validate_register(cls, address: int, size: int) -> None:
        if address < 0:
            raise ConfigurationError("Register address must be nonnegative.")
        if size not in cls.VALID_REGISTER_SIZES:
            raise ConfigurationError("Register size must be 1, 2, or 4 bytes.")

    @staticmethod
    def _validate_motor_id(motor_id: int) -> None:
        if not 0 <= motor_id <= 252:
            raise ConfigurationError("Motor ID must be in the range 0..252.")

    def _check_result(self, motor_id: int, operation: str, result: int, error: int) -> None:
        if result == self._sdk.COMM_SUCCESS and not error:
            return
        detail = (
            self._packet.getTxRxResult(result)
            if result != self._sdk.COMM_SUCCESS
            else self._packet.getRxPacketError(error)
        )
        raise CommunicationError(f"Motor {motor_id}: {operation}: {detail}")

    def _check_group_result(self, operation: str, result: int) -> None:
        if result != self._sdk.COMM_SUCCESS:
            detail = self._packet.getTxRxResult(result)
            raise CommunicationError(f"{operation}: {detail}")

    def ping(self, motor_id: int) -> int:
        self._validate_motor_id(motor_id)
        self._require_open()
        model, result, error = self._packet.ping(self._port, motor_id)
        self._check_result(motor_id, "ping", result, error)
        return int(model)

    def read(self, motor_id: int, address: int, size: int) -> int:
        self._validate_motor_id(motor_id)
        self._validate_register(address, size)
        self._require_open()
        method = getattr(self._packet, f"read{size}ByteTxRx")
        value, result, error = method(self._port, motor_id, address)
        self._check_result(motor_id, f"read register {address}", result, error)
        return int(value)

    def write(self, motor_id: int, address: int, size: int, value: int) -> None:
        self._validate_motor_id(motor_id)
        self._validate_register(address, size)
        self._require_open()
        if not 0 <= int(value) < 1 << (8 * size):
            raise ConfigurationError(f"Value {value} does not fit in {size} bytes.")
        method = getattr(self._packet, f"write{size}ByteTxRx")
        result, error = method(self._port, motor_id, address, int(value))
        self._check_result(motor_id, f"write register {address}", result, error)

    def sync_read(self, motor_ids: Iterable[int], address: int, size: int) -> dict[int, int]:
        self._validate_register(address, size)
        self._require_open()
        ids = tuple(motor_ids)
        group = self._sdk.GroupSyncRead(self._port, self._packet, address, size)
        for motor_id in ids:
            self._validate_motor_id(motor_id)
            if not group.addParam(motor_id):
                raise CommunicationError(f"Motor {motor_id}: could not add sync-read parameter.")
        result = group.txRxPacket()
        self._check_group_result(f"sync-read register {address}", result)
        values: dict[int, int] = {}
        for motor_id in ids:
            if not group.isAvailable(motor_id, address, size):
                raise CommunicationError(
                    f"Motor {motor_id}: sync-read register {address} returned no data."
                )
            values[motor_id] = int(group.getData(motor_id, address, size))
        group.clearParam()
        return values

    def sync_read_block(
        self,
        motor_ids: Iterable[int],
        start_address: int,
        length: int,
        fields: Mapping[int, int],
    ) -> dict[int, dict[int, int]]:
        """Read several fields contained in one contiguous control-table block."""
        if start_address < 0 or length <= 0:
            raise ConfigurationError("Sync-read block address and length must be positive.")
        for address, size in fields.items():
            self._validate_register(address, size)
            if address < start_address or address + size > start_address + length:
                raise ConfigurationError(
                    f"Field {address}:{size} is outside block "
                    f"{start_address}:{start_address + length}."
                )
        self._require_open()
        ids = tuple(motor_ids)
        group = self._sdk.GroupSyncRead(
            self._port, self._packet, start_address, length
        )
        for motor_id in ids:
            self._validate_motor_id(motor_id)
            if not group.addParam(motor_id):
                raise CommunicationError(f"Motor {motor_id}: could not add sync-read parameter.")
        result = group.txRxPacket()
        self._check_group_result(
            f"sync-read block {start_address}:{start_address + length}", result
        )
        values: dict[int, dict[int, int]] = {}
        for motor_id in ids:
            values[motor_id] = {}
            for address, size in fields.items():
                if not group.isAvailable(motor_id, address, size):
                    raise CommunicationError(
                        f"Motor {motor_id}: sync-read register {address} returned no data."
                    )
                values[motor_id][address] = int(
                    group.getData(motor_id, address, size)
                )
        group.clearParam()
        return values

    def sync_write(self, values: Mapping[int, int], address: int, size: int) -> None:
        self._validate_register(address, size)
        self._require_open()
        if not values:
            return
        group = self._sdk.GroupSyncWrite(self._port, self._packet, address, size)
        for motor_id, value in values.items():
            self._validate_motor_id(motor_id)
            if not 0 <= int(value) < 1 << (8 * size):
                raise ConfigurationError(f"Value {value} does not fit in {size} bytes.")
            encoded = list(int(value).to_bytes(size, byteorder="little", signed=False))
            if not group.addParam(motor_id, encoded):
                raise CommunicationError(f"Motor {motor_id}: could not add sync-write parameter.")
        result = group.txPacket()
        group.clearParam()
        self._check_group_result(f"sync-write register {address}", result)


class SimulatedBus:
    """In-memory bus for tests, examples, and development without hardware."""

    def __init__(
        self,
        motor_ids: Iterable[int],
        *,
        model_number: int = 1200,
        initial_position: int = 2048,
    ) -> None:
        ids = tuple(motor_ids)
        if len(ids) != len(set(ids)):
            raise ConfigurationError("Motor IDs must be unique.")
        self.model_numbers = {motor_id: model_number for motor_id in ids}
        self.registers: dict[int, dict[int, int]] = {
            motor_id: {
                10: 0,
                11: 3,
                64: 0,
                70: 0,
                84: 800,
                98: 0,
                100: 885,
                102: 200,
                108: 0,
                112: 0,
                116: initial_position,
                122: 0,
                124: 0,
                126: 0,
                128: 0,
                132: initial_position,
                144: 50,
                146: 25,
            }
            for motor_id in ids
        }
        self._is_open = False
        self.writes: list[tuple[int, int, int, int]] = []

    @property
    def is_open(self) -> bool:
        return self._is_open

    def open(self) -> None:
        self._is_open = True

    def close(self) -> None:
        self._is_open = False

    def _motor(self, motor_id: int) -> dict[int, int]:
        if not self._is_open:
            raise ConnectionError("The simulated bus is not open.")
        try:
            return self.registers[motor_id]
        except KeyError as exc:
            raise CommunicationError(f"Motor {motor_id}: no response.") from exc

    def ping(self, motor_id: int) -> int:
        self._motor(motor_id)
        return self.model_numbers[motor_id]

    def read(self, motor_id: int, address: int, size: int) -> int:
        del size
        return int(self._motor(motor_id).get(address, 0))

    def write(self, motor_id: int, address: int, size: int, value: int) -> None:
        registers = self._motor(motor_id)
        value = int(value)
        registers[address] = value
        self.writes.append((motor_id, address, size, value))
        if address == 116:
            registers[132] = value

    def sync_read(self, motor_ids: Iterable[int], address: int, size: int) -> dict[int, int]:
        return {motor_id: self.read(motor_id, address, size) for motor_id in motor_ids}

    def sync_read_block(
        self,
        motor_ids: Iterable[int],
        start_address: int,
        length: int,
        fields: Mapping[int, int],
    ) -> dict[int, dict[int, int]]:
        del start_address, length
        return {
            motor_id: {
                address: self.read(motor_id, address, size)
                for address, size in fields.items()
            }
            for motor_id in motor_ids
        }

    def sync_write(self, values: Mapping[int, int], address: int, size: int) -> None:
        for motor_id, value in values.items():
            self.write(motor_id, address, size, value)

