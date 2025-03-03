from collections import deque
from time import perf_counter, sleep
from typing import Final, NamedTuple

import cv2
from matplotlive import LivePlot
import numpy as np

from main import Tx

type XY = tuple[float, float]

capture = cv2.VideoCapture(0)

CALIBRATION = (363, 38), (713, 51), (802, 650), (244, 647)
TABLE_WIDTH, TABLE_HEIGHT = 350, 800

STICK_RADIUS = 20  # mm
HOME_Y_POS = 80
MAX_VELOCITY = 2600

X_BOUNDS = STICK_RADIUS, TABLE_WIDTH - STICK_RADIUS
Y_BOUNDS = 40, 300

X_CALIB_OFFSET = 5  # Offset in mm to calibrate the x-axis

EPSILON = 1  # Acceptable "error" in mm
UPDATE_INTERVAL = 0.05  # Update interval in seconds. Make sure this is not too low to avoid blocking steppers

PERSPECTIVE_SRC = np.float32(CALIBRATION)
PERSPECTIVE_DST = np.float32([(0, 0), (TABLE_WIDTH, 0), (TABLE_WIDTH, TABLE_HEIGHT), (0, TABLE_HEIGHT)])
TRANSFORM_MATRIX = cv2.getPerspectiveTransform(PERSPECTIVE_SRC, PERSPECTIVE_DST)


def clamp(x: float, low: float, high: float) -> float:
    return max(low, min(x, high))


def is_contour_viable(contour: np.ndarray) -> bool:
    """Returns whether the contour is viable for further processing."""
    _, _, w, h = cv2.boundingRect(contour)
    if w / h > 2 or h / w > 2:  # Not a circle (more elliptical)
        return False
    return cv2.contourArea(contour) > 120


def get_puck_state(frame: cv2.Mat) -> tuple[XY, float] | None:
    """Detects the circular puck and returns the coordinates of its center."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([60, 30, 10])
    upper = np.array([90, 255, 200])
    mask = cv2.inRange(hsv, lower, upper)
    blurred = cv2.GaussianBlur(mask, (9, 9), 2)
    contours, _ = cv2.findContours(blurred, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = filter(is_contour_viable, contours)

    cv2.imshow('mask', mask)

    try:
        return max((cv2.minEnclosingCircle(contour) for contour in contours), key=lambda c: c[1])  # type: ignore
    except ValueError:
        return None


class Prediction(NamedTuple):
    x: float
    y: float
    t: float  # In how many seconds will I get there?


class PredictionEngine:
    BACKTRACK_POS_TICKS: Final[int] = 3  # Increase for accuracy at the cost of responsiveness

    def __init__(self) -> None:
        self._last_update: float = perf_counter()

        self.prev: XY = 0, 0
        self.vel_instantaneous: XY = 0, 0
        self.cum_vel: deque[XY] = deque()

    def update(self, xy: XY) -> None:
        dt = perf_counter() - self._last_update

        x2, y2 = xy
        x1, y1 = self.prev
        self.vel_instantaneous = (x2 - x1) / dt, (y2 - y1) / dt
        self.cum_vel.append(self.vel_instantaneous)
        if len(self.cum_vel) > self.BACKTRACK_POS_TICKS:
            self.cum_vel.popleft()

        self._last_update = perf_counter()
        self.prev = xy

    @property
    def velocity(self) -> XY:
        if not self.cum_vel:
            return 0, 0

        n = len(self.cum_vel)
        sx = sum(vx for vx, _ in self.cum_vel)
        sy = sum(vy for _, vy in self.cum_vel)
        return sx / n, sy / n

    def predict_puck_position(self, *, y: float | None = None, t: float | None = None) -> Prediction:
        """Predicts the position of the puck given the parameters as described below.

        If both parameters are provided, this will return the first prediction that satisfies either. For example, if
        ``y=100`` and ``t=5.0``, but the puck does not reach ``y=100`` within 5 seconds, the prediction simply returns
        the position of the puck after 5 seconds.

        Parameters
        ----------
        y: float
            Returns the prediction for the next time we reach this y-coordinate.
        t: float
            Returns the prediction after ``t`` seconds.
        """
        target_y = y
        elapsed = 0

        vx, vy = self.velocity
        x, y = self.prev

        while elapsed < t:
            if vx > 0:
                tx = (TABLE_WIDTH - x) / vx
            elif vx < 0:
                tx = x / -vx
            else:
                tx = float('inf')

            if vy > 0:
                ty = (TABLE_HEIGHT - y) / vy
            elif vy < 0:
                ty = y / -vy
            else:
                ty = float('inf')

            critical_t = min(tx, ty, t - elapsed)

            # Is target_y within the possible range of y-positions?
            if target_y is not None and (
                (vy > 0 and y <= target_y <= y + vy * critical_t)
                or (vy < 0 and y >= target_y >= y + vy * critical_t)
            ):
                # Find the exact t and x when we reach target_y
                t_to_y = (target_y - y) / vy
                x_at_y = x + vx * t_to_y
                return Prediction(x_at_y, target_y, elapsed + t_to_y)

            x += vx * critical_t
            y += vy * critical_t
            elapsed += critical_t

            # Bounce if necessary
            if critical_t == tx:
                vx = -vx
            if critical_t == ty:
                vy = -vy

        return Prediction(x, y, t)


def process_frame(frame: cv2.Mat) -> cv2.Mat:
    if puck_state := get_puck_state(frame):
        (x, y), r = puck_state
        cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 2)
        engine.update((x, y))

        # Debug predicted position at y=HOME_Y_POS
        x_hat, y_hat, t_to_home  = engine.predict_puck_position(y=HOME_Y_POS, t=5.0)
        if abs(y_hat - HOME_Y_POS) < EPSILON:
            cv2.circle(frame, (int(x_hat), int(y_hat)), 5, (0, 0, 255), -1)
            cv2.line(
                frame,
                (int(x), int(y)),
                (int(x_hat), int(y_hat)),
                (0, 0, 255),
                2
            )
            cv2.putText(
                frame,
                f'{t_to_home:.1f}s',
                (int(x_hat), int(y_hat)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 0, 255),
                1,
            )

        # Debug predicted position for every .1s increment for 3s
        # for i in range(0, 35):
        #     x_hat, y_hat, _ = engine.predict_puck_position(t=i * 0.1)
        #     cv2.circle(frame, (int(x_hat), int(y_hat)), max(1, round(5 - (5 / 36) * i)), (255, 0, 0), -1)

        # Debug velocity
        vx, vy = engine.velocity
        v = (vx ** 2 + vy ** 2) ** 0.5
        cv2.putText(frame, f'Velocity: {v} mm/s', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Debug on graph
        # plot.send('|v|', (vx ** 2 + vy ** 2) ** 0.5)
    # else:
        # plot.send('|v|', -1)

    # plot.update()
    return frame


class AdjustedTx(Tx):
    def send_move_to_pos(self, x: int, y: int, v: int) -> None:
        x = clamp(x, *X_BOUNDS)
        y = clamp(y, *Y_BOUNDS)
        super().send_move_to_pos(-int(x), int(y) - HOME_Y_POS, v)

    def wait_until_available(self) -> None:
        while True:
            status = self.read_status()
            if status is None or not status.is_busy:
                break
            sleep(UPDATE_INTERVAL)

    def attack(self) -> None:
        self.send(b'A')
        # self.send_move_to_pos(x, y, int(MAX_VELOCITY * 0.8))
        # self.wait_until_available()
        # self.send_move_to_pos(x, y + attack_mm, MAX_VELOCITY)
        # self.wait_until_available()
        # self.send_move_to_pos(x, y, MAX_VELOCITY)


if __name__ == '__main__':
    tx = AdjustedTx()
    tx.send_reset()

    engine = PredictionEngine()
    # plot = LivePlot(timestep=1 / 60, duration=2.0, ylim=(0, 2000))

    last_update = perf_counter()
    frames = 0

    # sleep(2.0)
    # tx.attack()
    # while True:
    #     print(tx.serial.read_all().decode())
    #     sleep(0.01)
    # exit()

    while True:
        ret, f = capture.read()
        if not ret:
            break

        f = cv2.warpPerspective(f, TRANSFORM_MATRIX, (TABLE_WIDTH, TABLE_HEIGHT))
        frames += 1

        if perf_counter() - last_update > UPDATE_INTERVAL:
            last_update = perf_counter()
            x_hat, y_hat, t_to_home = engine.predict_puck_position(y=HOME_Y_POS, t=3.0)
            # Make sure we don't block ourselves
            if abs(y_hat - HOME_Y_POS) < EPSILON and engine.prev[1] > HOME_Y_POS + STICK_RADIUS + 20:
                print('sending', x_hat, y_hat)
                tx.send_move_to_pos(x_hat + X_CALIB_OFFSET, y_hat, MAX_VELOCITY)

            # if t_to_home < 0.2:
            #     tx.attack()
            #     print('ATTACK!')

            print(tx.serial.read_all().decode())

        out = process_frame(f)
        cv2.imshow('out', out)

        if cv2.waitKey(1) & 0xff == ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()

    # del plot
