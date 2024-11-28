import cv2
import numpy as np
import time


# Constants (in millimeters not pixels dont worry jaysen)
TABLE_LENGTH = 800  # Length of the table in mm
TABLE_WIDTH = 350   # Width of the table in mm
FRICTION_COEFFICIENT = 0.02  # Coefficient of friction
GRAVITY = 9.8       # Acceleration due to gravity (m/s^2)
FRICTION_DECELERATION = FRICTION_COEFFICIENT * GRAVITY * 1000  # mm/s^2


# Variables for tracking
previous_position = None
previous_velocity = None
previous_time = None

# Initialize video capture
cap = cv2.VideoCapture(0)  # 0 for the default camera


def detect_puck(frame: cv2.Mat) -> tuple[int, int]:
    """Detect the puck in the frame and return its center coordinates in mm."""
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 50, 50])
    upper_green = np.array([85, 255, 255])
    mask = cv2.inRange(hsv_frame, lower_green, upper_green)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

   

        for contour in contours:
        (x, y), radius = cv2.minEnclosingCircle(contour)
        if radius > 5:  # ( this is to ignore  the small objects
            # Assuming a scale factor to convert pixels to mm (calibrate this) (camera observes pixels so pixels are unavoidable)
            scale_factor = 10  # Example scale for this thingy  1 pixel = 10 mm
            return (int(x * scale_factor), int(y * scale_factor)), radius * scale_factor
    return None, None


def calculate_velocity(pos1, pos2, time_delta):
    """Calculate velocity in mm/s between two positions."""
    if time_delta == 0:
        return 0, 0
    return ((pos2[0] - pos1[0]) / time_delta, (pos2[1] - pos1[1]) / time_delta)


def calculate_acceleration(vel1, vel2, time_delta):
    """Calculate acceleration in mm/s^2 between two velocities."""
    if time_delta == 0:
        return 0, 0
    return ((vel2[0] - vel1[0]) / time_delta, (vel2[1] - vel1[1]) / time_delta)


def predict_position(pos, velocity, acceleration, time_ahead):
    """Predict the puck's future position in mm."""
    future_x = pos[0] + velocity[0] * time_ahead + 0.5 * acceleration[0] * time_ahead ** 2
    future_y = pos[1] + velocity[1] * time_ahead + 0.5 * acceleration[1] * time_ahead ** 2


    # Consider table boundaries
    if future_x < 0 or future_x > TABLE_LENGTH:
        velocity[0] = -velocity[0]
        future_x = max(0, min(TABLE_LENGTH, future_x))
    if future_y < 0 or future_y > TABLE_WIDTH:
        velocity[1] = -velocity[1]
        future_y = max(0, min(TABLE_WIDTH, future_y))

     return (future_x, future_y)


while True:
    ret, frame = cap.read()
    if not ret:
        break

# Detect puck
    puck_position, radius = detect_puck(frame)
    if puck_position is not None:
        current_time = time.time()


# Calculate velocity and acceleration
     if previous_position is not None:
            time_delta = current_time - previous_time
            velocity = calculate_velocity(previous_position, puck_position, time_delta)



  # Adjust velocity for friction
            velocity = (
                max(0, velocity[0] - FRICTION_DECELERATION * time_delta),
                max(0, velocity[1] - FRICTION_DECELERATION * time_delta),
            )

            if previous_velocity is not None:
                acceleration = calculate_acceleration(previous_velocity, velocity, time_delta)
            else:
                acceleration = (0, 0)

            # Predict future position
            time_ahead = 0.1  # Predict 0.1 seconds into the future
            future_position = predict_position(puck_position, velocity, acceleration, time_ahead)

            # Draw prediction on the frame
            future_position_px = (int(future_position[0] / 10), int(future_position[1] / 10))
            cv2.circle(frame, future_position_px, 10, (0, 0, 255), -1)




        # Update previous variables
        previous_position = puck_position
        previous_velocity = velocity
        previous_time = current_time


         # Draw detected puck
        puck_position_px = (int(puck_position[0] / 10), int(puck_position[1] / 10))
        cv2.circle(frame, puck_position_px, int(radius / 10), (0, 255, 0), 2)

    # Display the frame
    cv2.imshow('Air Hockey Tracker', frame)

    # Exit on 'q' key
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()