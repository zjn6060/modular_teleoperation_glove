"""Real-time visualization for multi-motor telemetry."""

from __future__ import annotations

import math
import time
from collections import deque
from collections.abc import Iterable, Mapping

from driver import MotorConfig, MotorTelemetry


class RealtimeMotorPlot:
    """Open one figure per motor with position, velocity, and torque subplots."""

    def __init__(
        self,
        motors: Iterable[MotorConfig],
        *,
        window_seconds: float = 20.0,
        title: str = "XL330 real-time state feedback",
    ) -> None:
        if not math.isfinite(window_seconds) or window_seconds <= 0:
            raise ValueError("window_seconds must be finite and positive.")
        motor_list = tuple(motors)
        if not motor_list:
            raise ValueError("At least one motor is required.")

        import matplotlib.pyplot as plt

        self._plt = plt
        self.motors = motor_list
        self.window_seconds = float(window_seconds)
        self._started = time.monotonic()
        self._closed_ids: set[int] = set()
        self._history = {
            motor.motor_id: {
                key: deque()
                for key in ("time", "position", "velocity", "torque")
            }
            for motor in motor_list
        }
        self.figures = {}
        self._panels = {}

        for motor in motor_list:
            figure, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
            motor_title = f"{motor.name or 'Motor'} (ID {motor.motor_id})"
            figure.canvas.manager.set_window_title(motor_title)
            figure.suptitle(f"{title} — {motor_title}", fontsize=14)
            figure.canvas.mpl_connect(
                "close_event",
                lambda _event, motor_id=motor.motor_id: self._closed_ids.add(motor_id),
            )

            position_axis, velocity_axis, torque_axis = axes
            position_line, = position_axis.plot(
                [], [], color="tab:blue", linewidth=1.5
            )
            velocity_line, = velocity_axis.plot(
                [], [], color="tab:orange", linewidth=1.5
            )
            torque_line, = torque_axis.plot(
                [], [], color="tab:purple", linewidth=1.5
            )

            position_axis.set_ylabel("position [deg]")
            velocity_axis.set_ylabel("velocity [rpm]")
            torque_axis.set_ylabel("torque est. [mN·m]")
            torque_axis.set_xlabel("time [s]")
            for axis in axes:
                axis.grid(True, alpha=0.3)

            status = torque_axis.text(
                0.01,
                0.97,
                "waiting for data",
                transform=torque_axis.transAxes,
                ha="left",
                va="top",
                fontsize=9,
                bbox={"boxstyle": "round", "facecolor": "white", "alpha": 0.82},
            )
            figure.tight_layout(rect=(0.04, 0.04, 0.98, 0.95))
            figure.show()

            self.figures[motor.motor_id] = figure
            self._panels[motor.motor_id] = {
                "axes": axes,
                "position_line": position_line,
                "velocity_line": velocity_line,
                "torque_line": torque_line,
                "status": status,
            }

        plt.show(block=False)

    @property
    def is_open(self) -> bool:
        return any(
            motor_id not in self._closed_ids
            and self._plt.fignum_exists(figure.number)
            for motor_id, figure in self.figures.items()
        )

    def update(
        self,
        telemetry: Mapping[int, MotorTelemetry],
        *,
        timestamp: float | None = None,
    ) -> None:
        if not self.is_open:
            return
        elapsed = (time.monotonic() if timestamp is None else timestamp) - self._started
        cutoff = elapsed - self.window_seconds

        for motor in self.motors:
            motor_id = motor.motor_id
            figure = self.figures[motor_id]
            if motor_id in self._closed_ids or not self._plt.fignum_exists(figure.number):
                continue
            sample = telemetry.get(motor_id)
            if sample is None:
                continue

            history = self._history[motor_id]
            values = {
                "time": elapsed,
                "position": math.degrees(sample.position_rad),
                "velocity": sample.velocity_rpm,
                "torque": (
                    sample.estimated_torque_nm * 1000.0
                    if sample.estimated_torque_nm is not None
                    else math.nan
                ),
            }
            for key, value in values.items():
                history[key].append(value)
            while history["time"] and history["time"][0] < cutoff:
                for series in history.values():
                    series.popleft()

            panel = self._panels[motor_id]
            times = history["time"]
            panel["position_line"].set_data(times, history["position"])
            panel["velocity_line"].set_data(times, history["velocity"])
            panel["torque_line"].set_data(times, history["torque"])
            torque_text = (
                f"{sample.estimated_torque_nm * 1000.0:+.1f} mN·m"
                if sample.estimated_torque_nm is not None
                else "n/a"
            )
            panel["status"].set_text(
                f"current {sample.current_ma:+.0f} mA  "
                f"torque {torque_text}  PWM {sample.pwm_percent:+.1f}%\n"
                f"voltage {sample.input_voltage_v:.1f} V  "
                f"temperature {sample.temperature_c:.0f} °C  "
                f"torque {'ON' if sample.torque_enabled else 'OFF'}  "
                f"moving {'yes' if sample.moving else 'no'}  "
                f"HW error {sample.hardware_error:#04x}"
            )
            panel["status"].set_color(
                "darkred" if sample.hardware_error else "black"
            )
            for axis in panel["axes"]:
                axis.relim()
                axis.autoscale_view(scalex=False, scaley=True)
                axis.set_xlim(
                    max(0.0, elapsed - self.window_seconds),
                    max(self.window_seconds, elapsed),
                )
            figure.canvas.draw_idle()
            figure.canvas.flush_events()

        self._plt.pause(0.001)

    def close(self) -> None:
        for motor_id, figure in self.figures.items():
            if self._plt.fignum_exists(figure.number):
                self._plt.close(figure)
            self._closed_ids.add(motor_id)

