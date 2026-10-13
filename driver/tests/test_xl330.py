import math

import pytest

from driver import (
    CommunicationError,
    ConfigurationError,
    MotorConfig,
    SafetyError,
    SimulatedBus,
    XL330Driver,
)
from driver.xl330 import Register


def make_driver(ids=(1, 2, 3)):
    motors = [
        MotorConfig(
            motor_id=motor_id,
            name=f"motor_{motor_id}",
            zero_position=2048,
            min_position=1800,
            max_position=2300,
        )
        for motor_id in ids
    ]
    bus = SimulatedBus(ids)
    return XL330Driver(bus, motors), bus


def test_motor_configuration_and_angle_conversion():
    motor = MotorConfig(7, zero_position=2048, direction=-1, min_position=1000, max_position=3000)
    position = motor.radians_to_position(math.pi / 2)
    assert position == 1024
    assert motor.position_to_radians(position) == pytest.approx(math.pi / 2)
    with pytest.raises(SafetyError):
        motor.radians_to_position(math.pi)
    assert motor.radians_to_position(math.pi, clamp=True) == 1000


def test_configuration_rejects_duplicate_ids():
    bus = SimulatedBus((1,))
    with pytest.raises(ConfigurationError):
        XL330Driver(bus, (MotorConfig(1), MotorConfig(1)))


def test_open_validates_every_motor_before_motion():
    driver, bus = make_driver()
    bus.model_numbers[2] = 999
    with pytest.raises(ConfigurationError):
        driver.open()
    assert not bus.is_open
    assert not any(address == Register.TORQUE_ENABLE and value == 1 for _, address, _, value in bus.writes)


def test_enable_holds_all_positions_before_enabling_torque():
    driver, bus = make_driver()
    driver.open()
    holds = driver.enable_torque()

    first_enable = next(
        index
        for index, (_, address, _, value) in enumerate(bus.writes)
        if address == Register.TORQUE_ENABLE and value == 1
    )
    goals_before_enable = {
        motor_id: value
        for motor_id, address, _, value in bus.writes[:first_enable]
        if address == Register.GOAL_POSITION
    }
    assert holds == {1: 2048, 2: 2048, 3: 2048}
    assert goals_before_enable == holds

    assert driver.close() == []
    assert all(bus.registers[motor_id][Register.TORQUE_ENABLE] == 0 for motor_id in driver.motor_ids)


def test_commands_require_enabled_torque_and_respect_limits():
    driver, bus = make_driver((1,))
    driver.open()
    with pytest.raises(SafetyError):
        driver.command_positions({1: 2050})

    driver.enable_torque()
    driver.command_angles({1: math.radians(5)})
    assert bus.registers[1][Register.GOAL_POSITION] == 2105
    with pytest.raises(SafetyError):
        driver.command_positions({1: 3000})
    driver.close()


def test_enable_rejects_current_position_outside_mechanical_limits():
    driver, bus = make_driver((1,))
    bus.registers[1][Register.PRESENT_POSITION] = 2500
    driver.open()
    with pytest.raises(SafetyError, match="outside configured limits"):
        driver.enable_torque()
    assert bus.registers[1][Register.TORQUE_ENABLE] == 0
    driver.close()


def test_hardware_error_stops_position_read():
    driver, bus = make_driver((1,))
    driver.open()
    bus.registers[1][Register.HARDWARE_ERROR] = 4
    with pytest.raises(CommunicationError, match="Hardware error"):
        driver.read_positions()
    driver.close()


def test_telemetry_converts_signed_values_and_physical_units():
    driver, bus = make_driver((1,))
    bus.registers[1].update(
        {
            Register.PRESENT_POSITION: 2100,
            Register.PRESENT_VELOCITY: (1 << 32) - 2,
            Register.PRESENT_CURRENT: (1 << 16) - 100,
            Register.PRESENT_PWM: 10,
            Register.PRESENT_INPUT_VOLTAGE: 49,
            Register.PRESENT_TEMPERATURE: 31,
            Register.MOVING: 1,
            Register.TORQUE_ENABLE: 1,
        }
    )
    driver.open()
    sample = driver.read_telemetry()[1]
    assert sample.position_raw == 2100
    assert sample.velocity_raw == -2
    assert sample.velocity_rpm == pytest.approx(-0.458)
    assert sample.current_ma == -100
    assert sample.estimated_torque_nm == pytest.approx(-0.052 / 1.47)
    assert sample.pwm_percent == pytest.approx(1.13)
    assert sample.input_voltage_v == pytest.approx(4.9)
    assert sample.temperature_c == 31
    assert sample.moving
    assert sample.torque_enabled
    driver.close()


def test_context_manager_closes_bus_after_exception():
    driver, bus = make_driver((1,))
    with pytest.raises(RuntimeError):
        with driver:
            driver.enable_torque()
            raise RuntimeError("application failed")
    assert not bus.is_open
    assert bus.registers[1][Register.TORQUE_ENABLE] == 0

