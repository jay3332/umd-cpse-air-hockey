# from collections import deque
# from enum import Enum
# from time import perf_counter, sleep
# from typing import Final, NamedTuple, overload

# import cv2
# from matplotlive import LivePlot
# import numpy as np

# from main import Tx

# type XY = tuple[float, float]

# capture = cv2.VideoCapture(0)

# CALIBRATION = (104, 122), (576, 32), (598, 1239), (13, 1206)
# TABLE_WIDTH, TABLE_HEIGHT = 350, 800

# STICK_RADIUS = 20  # mm
# HOME_Y_POS = 80
# ATTACK_Y_POS = 130
# MAX_VELOCITY = 3200  # 3100

# X_MUL, Y_MUL = 1.05, 1.00
# X_BOUNDS = STICK_RADIUS, (TABLE_WIDTH - STICK_RADIUS) * X_MUL
# Y_BOUNDS = -100, 300

# X_CALIB_OFFSET = 5  # Offset in mm to calibrate the x-axis

# EPSILON = 1  # Acceptable "error" in mm
# UPDATE_INTERVAL_SLOW = UPDATE_INTERVAL = 0.1  # Update interval in seconds. Make sure this is not too low to avoid blocking steppers
# UPDATE_INTERVAL_FAST = 0.05

# PERSPECTIVE_SRC = np.float32(CALIBRATION)
# PERSPECTIVE_DST = np.float32([(0, 0), (TABLE_WIDTH, 0), (TABLE_WIDTH, TABLE_HEIGHT), (0, TABLE_HEIGHT)])
# TRANSFORM_MATRIX = cv2.getPerspectiveTransform(PERSPECTIVE_SRC, PERSPECTIVE_DST)


# @overload
# def clamp(x: int, low: int, high: int) -> int:
#     ...


# def clamp(x: float, low: float, high: float) -> float:
#     return max(low, min(x, high))


# def is_contour_viable(contour: np.ndarray) -> bool:
#     """Returns whether the contour is viable for further processing."""
#     _, _, w, h = cv2.boundingRect(contour)
#     if w / h > 2 or h / w > 2:  # Not a circle (more elliptical)
#         return False
#     return cv2.contourArea(contour) > 120


# def get_puck_state(frame: cv2.Mat) -> tuple[XY, float] | None:
#     """Detects the circular puck and returns the coordinates of its center."""
#     hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
#     lower = np.array([60, 30, 10])
#     upper = np.array([90, 255, 200])
#     mask = cv2.inRange(hsv, lower, upper)
#     blurred = cv2.GaussianBlur(mask, (9, 9), 2)
#     contours, _ = cv2.findContours(blurred, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#     contours = filter(is_contour_viable, contours)

#     cv2.imshow('mask', mask)

#     try:
#         return max((cv2.minEnclosingCircle(contour) for contour in contours), key=lambda c: c[1])  # type: ignore
#     except ValueError:
#         return None


# class Prediction(NamedTuple):
#     x: float
#     y: float
#     t: float  # In how many seconds will I get there?


# class PredictionEngine:
#     BACKTRACK_POS_TICKS: Final[int] = 3  # Increase for accuracy at the cost of responsiveness

#     def __init__(self) -> None:
#         self._last_update: float = perf_counter()

#         self.prev: XY = 0, 0
#         self.vel_instantaneous: XY = 0, 0
#         self.cum_vel: deque[XY] = deque()

#     def update(self, xy: XY) -> None:
#         dt = perf_counter() - self._last_update

#         x2, y2 = xy
#         x1, y1 = self.prev
#         self.vel_instantaneous = (x2 - x1) / dt, (y2 - y1) / dt
#         self.cum_vel.append(self.vel_instantaneous)
#         if len(self.cum_vel) > self.BACKTRACK_POS_TICKS:
#             self.cum_vel.popleft()

#         self._last_update = perf_counter()
#         self.prev = xy

#     @property
#     def velocity(self) -> XY:
#         if not self.cum_vel:
#             return 0, 0

#         n = len(self.cum_vel)
#         sx = sum(vx for vx, _ in self.cum_vel)
#         sy = sum(vy for _, vy in self.cum_vel)
#         return sx / n, sy / n

#     def predict_puck_position(self, *, y: float | None = None, t: float | None = None) -> Prediction:
#         """Predicts the position of the puck given the parameters as described below.

#         If both parameters are provided, this will return the first prediction that satisfies either. For example, if
#         ``y=100`` and ``t=5.0``, but the puck does not reach ``y=100`` within 5 seconds, the prediction simply returns
#         the position of the puck after 5 seconds.

#         Parameters
#         ----------
#         y: float
#             Returns the prediction for the next time we reach this y-coordinate.
#         t: float
#             Returns the prediction after ``t`` seconds.
#         """
#         target_y = y
#         elapsed = 0

#         vx, vy = self.velocity
#         x, y = self.prev

#         while elapsed < t:
#             if vx > 0:
#                 tx = (TABLE_WIDTH - x) / vx
#             elif vx < 0:
#                 tx = x / -vx
#             else:
#                 tx = float('inf')

#             if vy > 0:
#                 ty = (TABLE_HEIGHT - y) / vy
#             elif vy < 0:
#                 ty = y / -vy
#             else:
#                 ty = float('inf')

#             critical_t = min(tx, ty, t - elapsed)

#             # Is target_y within the possible range of y-positions?
#             if target_y is not None and (
#                 (vy > 0 and y <= target_y <= y + vy * critical_t)
#                 or (vy < 0 and y >= target_y >= y + vy * critical_t)
#             ):
#                 # Find the exact t and x when we reach target_y
#                 t_to_y = (target_y - y) / vy
#                 x_at_y = x + vx * t_to_y
#                 return Prediction(x_at_y, target_y, elapsed + t_to_y)

#             x += vx * critical_t
#             y += vy * critical_t
#             elapsed += critical_t

#             # Bounce if necessary
#             if critical_t == tx:
#                 vx = -vx
#             if critical_t == ty:
#                 vy = -vy

#         return Prediction(x, y, t)


# def process_frame(frame: cv2.Mat) -> cv2.Mat:
#     if puck_state := get_puck_state(frame):
#         (x, y), r = puck_state
#         cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 0), 2)
#         engine.update((x, y))

#         # Debug predicted position at y=HOME_Y_POS
#         x_hat, y_hat, t_to_home  = engine.predict_puck_position(y=HOME_Y_POS, t=5.0)
#         if abs(y_hat - HOME_Y_POS) < EPSILON:
#             cv2.circle(frame, (int(x_hat), int(y_hat)), 5, (0, 0, 255), -1)
#             cv2.line(
#                 frame,
#                 (int(x), int(y)),
#                 (int(x_hat), int(y_hat)),
#                 (0, 0, 255),
#                 2
#             )
#             cv2.putText(
#                 frame,
#                 f'{t_to_home:.1f}s',
#                 (int(x_hat), int(y_hat)),
#                 cv2.FONT_HERSHEY_SIMPLEX,
#                 0.5,
#                 (0, 0, 255),
#                 1,
#             )

#         # Debug predicted position for every .1s increment for 3s
#         # for i in range(0, 35):
#         #     x_hat, y_hat, _ = engine.predict_puck_position(t=i * 0.1)
#         #     cv2.circle(frame, (int(x_hat), int(y_hat)), max(1, round(5 - (5 / 36) * i)), (255, 0, 0), -1)

#         # Debug velocity
#         vx, vy = engine.velocity
#         v = (vx ** 2 + vy ** 2) ** 0.5
#         cv2.putText(frame, f'Velocity: {v} mm/s', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

#         # Debug on graph
#         # plot.send('|v|', (vx ** 2 + vy ** 2) ** 0.5)
#     # else:
#         # plot.send('|v|', -1)

#     # plot.update()
#     return frame


# class AdjustedTx(Tx):
#     def send_move_to_pos(self, x: int, y: int, v: int) -> None:
#         x = clamp(x, *X_BOUNDS)
#         y = clamp(y, *Y_BOUNDS)
#         super().send_move_to_pos(-int(x), int(y), v)

#     def wait_until_available(self) -> None:
#         while True:
#             status = self.read_status()
#             if status is None or not status.is_busy:
#                 break
#             sleep(UPDATE_INTERVAL)

#     def attack(self) -> None:
#         """Execute attack sequence based on jjrobots strategy:
#         1. Quick move to attack position
#         2. Execute attack command for forward strike
#         3. Return to defensive position
#         """
#         self.send(b'A')  # Send attack command to trigger the hardware attack sequence

#     def defend(self, x: float, y: float, t: float) -> None:
#         """Execute defensive strategy based on prediction time and position"""
#         if t < 0.3:  # Puck is very close, use max velocity
#             self.send_move_to_pos(int(x), int(y), MAX_VELOCITY)
#         else:  # Adjust velocity based on prediction time
#             vel = int(min(MAX_VELOCITY, max(2000, 5000 / t)))  # Scale velocity with time
#             self.send_move_to_pos(int(x), int(y), vel)


# class Strategy(Enum):
#     """  // predict_status == 0 => No risk
#   // predict_status == 1 => Puck is moving to our field directly
#   // predict_status == 2 => Puck is moving to our field with a bounce
#   // predict_status == 3 => ?"""
#     RETURN_TO_HOME = 0
#     DEFEND = 1
#     DEFEND_BOUNCE = 2
#     ATTACK = 3



# if __name__ == '__main__':
#     tx = AdjustedTx()
#     tx.send_reset()

#     engine = PredictionEngine()
#     last_update = perf_counter()
#     frames = 0

#     attack_ticks_remaining = 0
#     strategy = Strategy.RETURN_TO_HOME

#     while True:
#         ret, frame = capture.read()
#         if not ret:
#             break

#         frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
#         frame = cv2.warpPerspective(frame, TRANSFORM_MATRIX, (TABLE_WIDTH, TABLE_HEIGHT))
#         frames += 1

#         # Determine update interval based on puck velocity
#         vx, vy = engine.velocity
#         v_squared = vx ** 2 + vy ** 2
#         interval = UPDATE_INTERVAL_FAST if v_squared > 250_000 else UPDATE_INTERVAL_SLOW

#         if perf_counter() - last_update > interval:
#             last_update = perf_counter()
            
#             # Get prediction for defense position
#             x_hat, y_hat, t_to_home = engine.predict_puck_position(y=HOME_Y_POS, t=3.0)
            
#             # Update strategy based on prediction
#             if attack_ticks_remaining > 0:
#                 # Currently executing attack sequence
#                 target_y = -80  # Attack position
#                 attack_ticks_remaining -= 1
#                 if attack_ticks_remaining == 0:
#                     strategy = Strategy.RETURN_TO_HOME
#             else:
#                 # Check if we should attack or defend
#                 if abs(y_hat - HOME_Y_POS) < EPSILON and engine.prev[1] > HOME_Y_POS + STICK_RADIUS + 20:
#                     if t_to_home < 0.6 and vy < -1.0:  # Good conditions for attack
#                         tx.attack()
#                         attack_ticks_remaining = 4
#                         strategy = Strategy.ATTACK
#                     else:
#                         # Defend position
#                         DEFEND_LEFT_BOUND = 0
#                         DEFEND_RIGHT_BOUND = 1000
#                         x_defend = clamp(x_hat * X_MUL, DEFEND_LEFT_BOUND, DEFEND_RIGHT_BOUND)
#                         tx.defend(x_defend, HOME_Y_POS, t_to_home)
#                         strategy = Strategy.DEFEND
#                 else:
#                     # Return to home/center position
#                     tx.send_move_to_pos(TABLE_WIDTH // 2, HOME_Y_POS, int(MAX_VELOCITY * 0.6))
#                     strategy = Strategy.RETURN_TO_HOME

#             # Request status update
#             tx.send(b'S')

#         # Process and display frame
#         out = process_frame(frame)
        
#         # Add strategy indicator
#         cv2.putText(
#             out,
#             f"Strategy: {strategy.name}",
#             (10, 60),
#             cv2.FONT_HERSHEY_SIMPLEX,
#             0.5,
#             (0, 255, 0),
#             2
#         )
        
#         cv2.imshow('out', out)

#         if cv2.waitKey(1) & 0xff == ord('q'):
#             break

#     capture.release()
#     cv2.destroyAllWindows()

#     # del plot

from collections import deque
from enum import Enum
from time import perf_counter, sleep
from typing import Final, NamedTuple, overload

import cv2
from matplotlive import LivePlot
import numpy as np

from main import Tx

type XY = tuple[float, float]

capture = cv2.VideoCapture(0)

CALIBRATION = (14, 60), (706, 66), (613, 1235), (94, 1187)
TABLE_WIDTH, TABLE_HEIGHT = 350, 800

STICK_RADIUS = 20  # mm
HOME_Y_POS = 80
ATTACK_Y_POS = 130
MAX_VELOCITY = 2000  # 3100

X_MUL, Y_MUL = 1.50, 1.00
X_BOUNDS = STICK_RADIUS, (TABLE_WIDTH - STICK_RADIUS) * X_MUL
Y_BOUNDS = -100, 200

X_CALIB_OFFSET = 5  # Offset in mm to calibrate the x-axis

EPSILON = 1  # Acceptable "error" in mm
UPDATE_INTERVAL_SLOW = UPDATE_INTERVAL = 0.1  # Update interval in seconds. Make sure this is not too low to avoid blocking steppers
UPDATE_INTERVAL_FAST = 0.05

PERSPECTIVE_SRC = np.float32(CALIBRATION)
PERSPECTIVE_DST = np.float32([(0, 0), (TABLE_WIDTH, 0), (TABLE_WIDTH, TABLE_HEIGHT), (0, TABLE_HEIGHT)])
TRANSFORM_MATRIX = cv2.getPerspectiveTransform(PERSPECTIVE_SRC, PERSPECTIVE_DST)


@overload
def clamp(x: int, low: int, high: int) -> int:
    ...


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
    lower = np.array([60, 100, 10])
    upper = np.array([90, 255, 100])
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
        super().send_move_to_pos(-int(x), int(y), v)

    def wait_until_available(self) -> None:
        while True:
            status = self.read_status()
            if status is None or not status.is_busy:
                break
            sleep(UPDATE_INTERVAL)

    def attack(self) -> None:
        """Execute attack sequence based on jjrobots strategy:
        1. Quick move to attack position
        2. Execute attack command for forward strike
        3. Return to defensive position
        """
        self.send(b'A')  # Send attack command to trigger the hardware attack sequence

    def defend(self, x: float, y: float, t: float) -> None:
        """Execute defensive strategy based on prediction time and position"""
        if t < 0.3:  # Puck is very close, use max velocity
            self.send_move_to_pos(int(x), int(y), MAX_VELOCITY)
        else:  # Adjust velocity based on prediction time
            vel = int(min(MAX_VELOCITY, max(2000, 5000 / t)))  # Scale velocity with time
            self.send_move_to_pos(int(x), int(y), vel)


class Strategy(Enum):
    """  // predict_status == 0 => No risk
  // predict_status == 1 => Puck is moving to our field directly
  // predict_status == 2 => Puck is moving to our field with a bounce
  // predict_status == 3 => ?"""
    RETURN_TO_HOME = 0
    DEFEND = 1
    DEFEND_BOUNCE = 2
    ATTACK = 3



if __name__ == '__main__':
    tx = AdjustedTx()
    tx.send_reset()

    engine = PredictionEngine()
    last_update = perf_counter()
    frames = 0

    attack_ticks_remaining = 0
    strategy = Strategy.RETURN_TO_HOME

    while True:
        ret, frame = capture.read()
        if not ret:
            break

        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
        frame = cv2.warpPerspective(frame, TRANSFORM_MATRIX, (TABLE_WIDTH, TABLE_HEIGHT))
        frames += 1

        # Determine update interval based on puck velocity
        vx, vy = engine.velocity
        v_squared = vx ** 2 + vy ** 2
        interval = UPDATE_INTERVAL_FAST if v_squared > 250_000 else UPDATE_INTERVAL_SLOW

        if perf_counter() - last_update > interval:
            last_update = perf_counter()
            
            # Get prediction for defense position
            x_hat, y_hat, t_to_home = engine.predict_puck_position(y=HOME_Y_POS, t=3.0)
            
            # Update strategy based on prediction
            if attack_ticks_remaining > 0:
                # Currently executing attack sequence
                target_y = -80  # Attack position
                attack_ticks_remaining -= 1
                if attack_ticks_remaining == 0:
                    strategy = Strategy.RETURN_TO_HOME
            else:
                # Check if we should attack or defend
                if abs(y_hat - HOME_Y_POS) < EPSILON and engine.prev[1] > HOME_Y_POS + STICK_RADIUS + 20:
                    if t_to_home < 0.6 and vy < -1.0:  # Good conditions for attack
                        tx.attack()
                        attack_ticks_remaining = 4
                        strategy = Strategy.ATTACK
                    else:
                        # Defend position
                        DEFEND_LEFT_BOUND = 0
                        DEFEND_RIGHT_BOUND = 1000
                        x_defend = clamp(x_hat * X_MUL, DEFEND_LEFT_BOUND, DEFEND_RIGHT_BOUND)
                        tx.defend(x_defend, HOME_Y_POS, t_to_home)
                        strategy = Strategy.DEFEND
                else:
                    # Return to home/center position
                    tx.send_move_to_pos(int(TABLE_WIDTH // 2 * X_MUL), HOME_Y_POS, int(MAX_VELOCITY * 0.6))
                    strategy = Strategy.RETURN_TO_HOME

            # Request status update
            tx.send(b'S')

        # Process and display frame
        out = process_frame(frame)
        
        # Add strategy indicator
        cv2.putText(
            out,
            f"Strategy: {strategy.name}",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )
        
        cv2.imshow('out', out)

        if cv2.waitKey(1) & 0xff == ord('q'):
            break

    capture.release()
    cv2.destroyAllWindows()

    # del plot
