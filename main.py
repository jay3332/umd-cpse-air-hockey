from enum import Enum
from time import sleep
from typing import Final, NamedTuple

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

#: The viable interval to send individual bytes to the serial connection.
BYTE_SEND_INTERVAL: Final[float] = 0.001

#: The serial port to connect to.
SERIAL_PORT: Final[str] = '/dev/cu.usbmodem101'  # '/dev/cu.usbmodem2101'


class TxStatus(NamedTuple):
    current_x: int
    current_y: int
    is_busy: bool


class Tx:
    """Serial communicator"""

    def __init__(self) -> None:
        self.serial: Serial = Serial(port=SERIAL_PORT, baudrate=SERIAL_BITRATE, timeout=1.0)

    def send(self, data: bytes, *, fast: bool = False) -> None:
        if fast or len(data) == 1:
            self.serial.write(data)
            return

        for b in data:
            self.serial.write(bytes([b]))
            sleep(BYTE_SEND_INTERVAL)

    def close(self) -> None:
        self.serial.close()

    def send_reset(self) -> None:
        self.send(b'R')
        sleep(BYTE_SEND_INTERVAL)

    def read_status(self) -> TxStatus | None:
        self.send(b'B')
        sleep(BYTE_SEND_INTERVAL)  # Give time to respond
        data: bytes = self.serial.read(8)

        print('RECEIVED status data: ', list(data))
        if len(data) != 8 or data[0] != b'B' or data[-1] != b'\n':
            return None

        _, x1, x2, y1, y2, is_busy, checksum, _ = data
        x = int.from_bytes([x1, x2], 'big')
        y = int.from_bytes([y1, y2], 'big')
        if checksum != (x + y) % 256:
            print('checksum failed!', checksum, '!=', (x + y) % 256)
            return None

        return TxStatus(x, y, bool(is_busy))

    def send_move_to_pos(self, x: int, y: int, v: int) -> None:  # in mm and mm/s
        checksum = (x + y + v) % 256
        self.send(b'M' + x.to_bytes(2, signed=True) + y.to_bytes(2, signed=True) + v.to_bytes(2) + checksum.to_bytes(1))

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
        self.send(x.to_bytes(1, signed=True) + y.to_bytes(1, signed=True), fast=True)

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
    tx = Tx()
    sleep(1.0)
    print(tx.serial.readline())

    n = 0
    v = 0

    while True:
        n += 1

        v = min(4000, v + 300)
        tx.send_move_to_pos(0, 200, v)
        print('new v =', v)

        if n % 20 == 0:
            print(tx.serial.read_all().decode('utf-8', errors='ignore'), end='')
        sleep(0.025)

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
