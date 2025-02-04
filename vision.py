from collections import deque
from time import perf_counter
from typing import Final

import cv2
from matplotlive import LivePlot
import numpy as np

type XY = tuple[float, float]

capture = cv2.VideoCapture(0)

CALIBRATION = (488, 293), (880, 286), (1029, 541), (121, 548)
TABLE_WIDTH, TABLE_HEIGHT = 350, 800

PERSPECTIVE_SRC = np.float32(CALIBRATION)
PERSPECTIVE_DST = np.float32([(0, 0), (TABLE_WIDTH, 0), (TABLE_WIDTH, TABLE_HEIGHT), (0, TABLE_HEIGHT)])
TRANSFORM_MATRIX = cv2.getPerspectiveTransform(PERSPECTIVE_SRC, PERSPECTIVE_DST)


def is_contour_viable(contour: np.ndarray) -> bool:
    """Returns whether the contour is viable for further processing."""
    _, _, w, h = cv2.boundingRect(contour)
    if w / h > 2 or h / w > 2:  # Not a circle (more elliptical)
        return False
    return cv2.contourArea(contour) > 120


def get_puck_state(frame: cv2.Mat) -> tuple[XY, float] | None:
    """Detects the circular puck and returns the coordinates of its center."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([50, 50, 40])
    upper = np.array([80, 255, 200])
    mask = cv2.inRange(hsv, lower, upper)
    blurred = cv2.GaussianBlur(mask, (9, 9), 2)
    contours, _ = cv2.findContours(blurred, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = filter(is_contour_viable, contours)

    try:
        return max((cv2.minEnclosingCircle(contour) for contour in contours), key=lambda c: c[1])  # type: ignore
    except ValueError:
        return None


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

    def predict_puck_position(self, t: float) -> XY:
        """Predicts the position of the puck in t seconds."""
        vx, vy = self.velocity
        x, y = self.prev

        while t > 0:
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

            critical_t = min(tx, ty, t)
            x += vx * critical_t
            y += vy * critical_t
            t -= critical_t

            # Bounce if necessary
            if critical_t == tx:
                vx = -vx
            if critical_t == ty:
                vy = -vy

        return x, y


def process_frame(frame: cv2.Mat) -> cv2.Mat:
    if puck_state := get_puck_state(frame):
        (x, y), r = puck_state
        cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 2)
        engine.update((x, y))

        # Debug predicted position in 0.5 seconds
        x_hat, y_hat = engine.predict_puck_position(0.5)
        cv2.circle(frame, (int(x_hat), int(y_hat)), 5, (0, 0, 255), -1)
        cv2.line(
            frame,
            (int(x), int(y)),
            (int(x_hat), int(y_hat)),
            (0, 0, 255),
            2
        )

        # Debug velocity
        vx, vy = engine.velocity
        cv2.putText(frame, f'Velocity: <{vx:.1f},{vy:.1f}>mm/s', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

        # Debug on graph
        plot.send('|v|', (vx ** 2 + vy ** 2) ** 0.5)
    else:
        plot.send('|v|', -1)

    plot.update()
    return frame


if __name__ == '__main__':
    engine = PredictionEngine()
    plot = LivePlot(timestep=1 / 60, duration=2.0, ylim=(0, 1000))

    while True:
        ret, f = capture.read()
        if not ret:
            break

        f = cv2.warpPerspective(f, TRANSFORM_MATRIX, (TABLE_WIDTH, TABLE_HEIGHT))

        out = process_frame(f)
        cv2.imshow('out', out)

        if cv2.waitKey(1) & 0xff == ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()
