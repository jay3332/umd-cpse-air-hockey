from dualsense_controller import DualSenseController, JoyStick, active_dualsense_controller
from serial import Serial
from typing import Final

#: The bitrate of the serial connection.
SERIAL_BITRATE: Final[int] = 115_200

#: The range of the motor speed.
RANGE: Final[int] = 10_000


class Tx:
    """Serial communicator"""

    def __init__(self) -> None:
        self.serial: Serial = Serial(port='/dev/cu.usbmodem101', baudrate=SERIAL_BITRATE)
        print(self.serial.readline())

    def send(self, data: bytes) -> None:
        self.serial.write(data)

    def close(self) -> None:
        self.serial.close()

    def update_raw_motor_speeds(self, pos: JoyStick) -> None:
        x = int(pos.x * RANGE)
        y = int(pos.y * RANGE)
        self.send(b'V' + x.to_bytes(2, 'little', signed=True) + y.to_bytes(2, 'little', signed=True))


class EventHandler:
    """Handles events from the gamepad controller."""

    def __init__(self, controller: DualSenseController) -> None:
        self.controller = controller
        self.tx: Tx = Tx()
        controller.left_stick.on_change(self.on_left_stick_changed)

    def on_left_stick_changed(self, joystick: JoyStick) -> None:
        self.tx.update_raw_motor_speeds(joystick)


def main() -> None:
    with active_dualsense_controller(device_index_or_device_info=0) as controller:
        _handler = EventHandler(controller)

        while True:
            try:
                pass
            except KeyboardInterrupt:
                break


if __name__ == "__main__":
    main()
