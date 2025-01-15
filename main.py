from enum import Enum
from time import sleep
from typing import Final

from dualsense_controller import DualSenseController, JoyStick, active_dualsense_controller
from serial import Serial

#: The bitrate of the serial connection.
SERIAL_BITRATE: Final[int] = 115_200

#: The range of the motor speed.
RANGE: Final[int] = 100

#: The minimum directional theshold.
THRESHOLD: Final[int] = 20

#: Whether to allow diagonal movements.
ALLOW_DIAGONAL_TRANSLATIONS: Final[bool] = False

#: Time per tick, in seconds.
TICK_LENGTH: Final[float] = 0.01  # 0.0015


class Tx:
    """Serial communicator"""

    def __init__(self) -> None:
        self.serial: Serial = Serial(port='/dev/cu.usbmodem101', baudrate=SERIAL_BITRATE)

    def send(self, data: bytes) -> None:
        self.serial.write(data)

    def close(self) -> None:
        self.serial.close()

    def update_raw_motor_speeds(self, x: float, y: float) -> None:
        x = int(x * RANGE)
        y = int(y * RANGE)

        if abs(x) < THRESHOLD:
            x = 0
        if abs(y) < THRESHOLD:
            y = 0
        if not ALLOW_DIAGONAL_TRANSLATIONS:
            if abs(x) < abs(y):
                x = 0
            if abs(y) < abs(x):
                y = 0

        # print('sending', x, y, 'as', x.to_bytes(1, signed=True) + y.to_bytes(1, signed=True))
        self.send(x.to_bytes(1, signed=True) + y.to_bytes(1, signed=True))

    def send_debug(self) -> None:
        print('sending debug')
        self.send(b'\xff')


class AttackState(Enum):
    dormant = 0
    attacking = 1
    retracting = 2


class EventHandler:
    """Handles events from the gamepad controller."""

    ATTACK_TICKS: Final[int] = 48

    def __init__(self, controller: DualSenseController) -> None:
        self.controller = controller
        self.forward_power: int = 0
        self.tx: Tx = Tx()

        self._attack_state = AttackState.dormant
        self._attack_ticks = 0

        controller.btn_square.on_down(self.tx.send_debug)
        controller.btn_l1.on_down(self.on_attack_down)
        # controller.left_trigger.on_change(self.on_left_trigger_changed)
        # controller.left_stick.on_change(self.on_left_stick_changed)

    def on_left_stick_changed(self, joystick: JoyStick) -> None:
        y = joystick.y
        self._attack_ticks += 1
        match self._attack_state:
            case AttackState.attacking:
                y = 1.0
                if self._attack_ticks > self.ATTACK_TICKS:
                    self._attack_ticks = 0
                    self._attack_state = AttackState.retracting
            case AttackState.retracting:
                y = -1.0
                if self._attack_ticks > self.ATTACK_TICKS:
                    self._attack_state = AttackState.dormant

        self.tx.update_raw_motor_speeds(joystick.x, y)

    def on_attack_down(self) -> None:
        if self._attack_state == AttackState.dormant:
            self._attack_ticks = 0
            self._attack_state = AttackState.attacking


def main() -> None:
    with active_dualsense_controller(device_index_or_device_info=0) as controller:
        handler = EventHandler(controller)
        n = 0

        try:
            while True:
                if n % 5000 == 0:
                    print(handler.tx.serial.read_all().decode('utf-8', errors='ignore'), end='')

                handler.on_left_stick_changed(controller.left_stick.value)
                sleep(TICK_LENGTH)

        except KeyboardInterrupt:
            handler.tx.close()
            print('Bye')


if __name__ == "__main__":
    main()
