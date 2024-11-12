import serial
from dualsense_controller import DualSenseController, active_dualsense_controller


class EventHandler:
    """Handles events from the gamepad controller."""

    def __init__(self, controller: DualSenseController) -> None:
        self.controller = controller
        controller.left_trigger.on_change(self.on_left_trigger)
        controller.left_stick_x.on_change(self.on_left_stick_x_changed)
        controller.left_stick_y.on_change(self.on_left_stick_y_changed)
        controller.left_stick.on_change(self.on_left_stick_changed)

    def on_left_trigger(self, value: int) -> None:
        print(f"Left trigger value: {value}")

    def on_left_stick_x_changed(self, value: int) -> None:
        print(f"Left stick X value: {value}")

    def on_left_stick_y_changed(self, value: int) -> None:
        print(f"Left stick Y value: {value}")

    def on_left_stick_changed(self, x: int, y: int) -> None:
        print(f"Left stick X value: {x}, Y value: {y}")


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
