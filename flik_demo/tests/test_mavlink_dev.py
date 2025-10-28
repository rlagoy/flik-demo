# test_mavlink_device.py

import pytest

from flik_demo.mavlink_device import MAVLinkDevice
from pymavlink.mavutil import mavserial

# -------------------
# Connection & heartbeat
# -------------------


def test_connect_and_wait_heartbeat():
    dev = MAVLinkDevice("/dev/tty.usbmodem2101,115200")
    dev.connect()
    msg = dev.wait_heartbeat(timeout=1.0)
    assert msg is not None
    assert isinstance(dev.device, mavserial)


def test_wait_heartbeat_raises_if_not_connected():
    dev = MAVLinkDevice("x")
    with pytest.raises(RuntimeError):
        dev.wait_heartbeat()
