"""Example test code to excercise servo motor."""

import time
import argparse

from mavlink_device import MAVLinkDevice  # pylint: disable=import-error


def main(dev: str) -> None:
    """Main hardware test function"""

    start_time = time.time()
    mav_device = MAVLinkDevice(dev)
    mav_device.connect()
    print(f"Waiting for heartbeat on {dev}")
    _ = mav_device.wait_heartbeat()
    print("Heartbeat received!")
    print(f"Connected within {time.time()-start_time} seconds.")

    input("1. Power on device with 7V. Press Enter to continue...")

    input("\n2. Measure 5V BEC output...")

    _ = mav_device.set_servo(4, 1000)
    input(
        "\n3. Setting PWM to 1000us. Measure upward deflection angle and PWM waveform."
    )

    _ = mav_device.set_servo(4, 2000)
    input(
        "\n4. Setting PWM to 2000us. Measure downward deflection angle and PWM waveform."
    )

    _ = mav_device.set_servo(4, 1500)
    input("\n5. Setting PWM to 1500us. Measure deflection angle.")

    input("\n6. Deflect servo horn and measure current.")

    input("\n7. Remove servo and measure current.")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description="FLIK Servo Motor Test")
    p.add_argument(
        "dev",
        help="MAVLink device path",
        type=str,
        default="/dev/tty.usbmodem2101,115200",
    )
    args = p.parse_args()

    main(args.dev)
