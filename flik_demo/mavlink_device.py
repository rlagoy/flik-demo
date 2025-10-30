"""
MavlinkDevice: A small helper around pymavlink for common tasks,
including heartbeats and servo control.
"""

import time
from typing import Iterable, Optional

from pymavlink import mavutil


class MAVLinkDevice:
    """
    Generalized MAVLink device helper.

    Parameters
    ----------
    device : str
        pymavlink connection string, e.g. "udpout:127.0.0.1:14550" or "/dev/ttyUSB0,57600".
    """

    def __init__(self, device_str: str) -> None:
        self.device_str = device_str
        self.device = None

    # --- connection & heartbeat -------------------------------------------------

    def connect(self) -> None:
        """
        Open the pymavlink connection.
        """

        self.device = mavutil.mavlink_connection(self.device_str)

    def wait_heartbeat(self, timeout: float = 5.0) -> Optional[object]:
        """Block until a HEARTBEAT is received (from the other side)."""
        if self.device is None:
            raise RuntimeError("Not connected.")
        return self.device.wait_heartbeat(timeout=timeout)

    def command_long(  # pylint: disable=too-many-arguments,too-many-positional-arguments
        self,
        command: int,
        params: Iterable[float] = (0, 0, 0, 0, 0, 0, 0),
        target_sys: Optional[int] = None,
        target_comp: Optional[int] = None,
        confirm: int = 0,
        expect_ack: bool = True,
        ack_timeout: float = 1.0,
    ) -> Optional[object]:
        """
        Send a COMMAND_LONG and optionally wait for COMMAND_ACK.

        Returns the ACK message if expect_ack is True and one arrives.
        """
        if self.device is None:
            raise RuntimeError("Not connected.")
        t_sys = target_sys if target_sys is not None else self.device.target_system or 1
        t_comp = (
            target_comp
            if target_comp is not None
            else self.device.target_component or 1
        )
        p = list(params) + [0.0] * (7 - len(list(params)))

        print(f"Sending COMMAND_LONG {command} to {t_sys}.{t_comp} with params {p}")

        self.device.mav.command_long_send(
            t_sys, t_comp, command, confirm, p[0], p[1], p[2], p[3], p[4], p[5], p[6]
        )

        if not expect_ack:
            return None

        # Wait for COMMAND_ACK on the same command
        t0 = time.time()
        while time.time() - t0 < ack_timeout:
            msg = self.device.recv_match(type="COMMAND_ACK", blocking=False)
            if msg and int(msg.command) == int(command):
                return msg
            time.sleep(0.01)
        return None

    # --- servo control ----------------------------------------------------------

    def set_servo(
        self, servo: int, pwm_us: int, *, prefer_rc_override: bool = False
    ) -> None:
        """
        Set a servo output.

        Preferred: MAV_CMD_DO_SET_SERVO (param1=servo number, param2=pulse usec).
        Fallback: RC_CHANNELS_OVERRIDE (for setups that map RC channels to servos).

        Parameters
        ----------
        servo : int
            Servo output index expected by firmware (often 1-based).
        pwm_us : int
            Pulse width in microseconds (e.g., 1000–2000).
        prefer_rc_override : bool
            If True, use RC override instead of DO_SET_SERVO.
        """
        if self.device is None:
            raise RuntimeError("Not connected.")

        if not prefer_rc_override:
            # Try MAV_CMD_DO_SET_SERVO
            return self.command_long(
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                (float(servo), float(pwm_us), 0, 0, 0, 0, 0),
                expect_ack=True,
            )

        self.set_rc_override({servo: pwm_us})
        return None

    def set_rc_override(self, channel_pwm: dict[int, int]) -> None:
        """
        Send RC_CHANNELS_OVERRIDE.

        channel_pwm keys are 1-based RC channel numbers. Values are PWM µs (1000–2000),
        or 0 to release a channel from override.
        """
        if self.device is None:
            raise RuntimeError("Not connected.")

        # Build 8-channel payload chunks per MAVLink spec.
        # Channels outside 1..8 require MAVLink2 extensions; here we handle 1..8 only.
        ch_values = [0] * 8
        for ch, pwm in channel_pwm.items():
            if 1 <= ch <= 8:
                ch_values[ch - 1] = int(pwm)

        self.device.mav.rc_channels_override_send(
            self.device.target_system or 1,
            self.device.target_component or 1,
            *ch_values,
        )

    # --- reading servo outputs (best-effort) ------------------------------------
    def read_servo_output_raw(self, timeout: float = 0.2) -> Optional[object]:
        """
        Try to read a SERVO_OUTPUT_RAW message (if the autopilot is emitting it).
        Returns the message or None if none seen in timeout.
        """
        if self.device is None:
            raise RuntimeError("Not connected.")
        t0 = time.time()
        while time.time() - t0 < timeout:
            msg = self.device.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)
            if msg:
                return msg
            time.sleep(0.01)
        return None
