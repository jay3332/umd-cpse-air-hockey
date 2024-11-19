from time import sleep
from typing import Final

from dualsense_controller import DualSenseController, JoyStick, active_dualsense_controller
from serial import Serial

#: The bitrate of the serial connection.
SERIAL_BITRATE: Final[int] = 115_200

#: The range of the motor speed.
RANGE: Final[int] = 10_000


class Tx:
    """Serial communicator"""

    def __init__(self) -> None:
        self.serial: Serial = Serial(port='/dev/cu.usbmodem101', baudrate=SERIAL_BITRATE)

    def send(self, data: bytes) -> None:
        self.serial.write(data)

    def close(self) -> None:
        self.serial.close()

    def update_raw_motor_speeds(self, x: int, y: int) -> None:
        x = int(x * RANGE)
        y = int(y * RANGE)
        self.send(b'V' + x.to_bytes(2, 'little', signed=True) + y.to_bytes(2, 'little', signed=True))

    def send_debug(self) -> None:
        print('sending debug')
        self.send(b'D')


class EventHandler:
    """Handles events from the gamepad controller."""

    def __init__(self, controller: DualSenseController) -> None:
        self.controller = controller
        self.forward_power: int = 0
        self.tx: Tx = Tx()

        controller.left_stick.on_change(self.on_left_stick_changed)
        controller.btn_square.on_down(self.tx.send_debug)
        # controller.left_trigger.on_change(self.on_left_trigger_changed)

    def on_left_stick_changed(self, joystick: JoyStick) -> None:
        self.tx.update_raw_motor_speeds(joystick.x, joystick.y)


def main() -> None:
    with active_dualsense_controller(device_index_or_device_info=0) as controller:
        handler = EventHandler(controller)

        try:
            while True:
                print(handler.tx.serial.read_all().decode('utf-8', errors='ignore'), end='')
                # Give serial time to send data
                sleep(1.0)
        except KeyboardInterrupt:
            handler.tx.close()
            print('Bye')


if __name__ == "__main__":
    main()
