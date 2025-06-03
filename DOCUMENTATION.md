# Air Hockey Robot
<sup>UMD Cyber-Physical Systems Engineering</sup>

## Overview

This document serves as a walkthrough of the mechanical details of the project, a guide on how to set up the robot, and documentation regarding the code in the vision and controller pipelines.

## Hardware

This project was heavily inspired by the [*Air Hockey Robot EVO*](https://www.instructables.com/Air-Hockey-Robot-EVO) project by JJRobots. Thus, much of the reference for materials and assembly are documented in the article.

The robot is controlled by a standard Arduino UNO. It is driven by two NEMA 17 Brushless Stepper Motors placed on either side of the table. The motors are wired to the CNC Shield on the Arduino, and each motor is configured with microstepping at $\frac{1}{8}\text{ step/pulse}$.

### Caveats

- You can only move the robot stick while the Arduino CNC shield is *not* powered
- The stepper motors are only open-loop controlled, and may accumulate error over time

### CoreXY Kinematics

The "H-drive" mechanism of the robot puck follows *CoreXY kinematics*, allowing movement of the puck in either dimension while keeping the motors in a static position.

To move the puck forwards and backwards, drive each motor in opposite directions. To move the puck sideways, drive each motor in the same direction.

The conversion from a movement $\langle\Delta x, \Delta y\rangle$ to motor pulses in either motor $\text{A}$ and $\text{B}$ are:

$$
\begin{align*}
\Delta a &= k(\Delta x + \Delta y) \\
\Delta b &= k(\Delta x - \Delta y) \\
\end{align*}
$$

Where $\Delta a$ pulses are sent to motor $\text{A}$, $\Delta b$ pulses are sent to motor $\text{B}$, and $k$ is some calibrated conversion rate from motor steps to the desired unit (i.e. $\rm{mm}$).

For software reference, this also implies:

$$
\begin{align*}
\Delta x &= \tfrac{k}{2}(\Delta a + \Delta b) \\
\Delta y &= \tfrac{k}{2}(\Delta a - \Delta b) \\
\end{align*}
$$

## Installation & Setup

### System Requirements

Make sure you are able to access the following:
- Python 3.12 or newer
- Rust 1.80 Nightly or newer 
- Unix-based system\*

<sup>\* Recommended, since this project has not been tested for Windows.</sup>

### Software Installation

Clone this repository:

```sh
$ git clone https://github.com/jay3332/umd-cpse-air-hockey
$ cd umd-cpse-air-hockey
```

Then, install Python dependencies:
```sh
$ pip install -r requirements.txt
```
Make sure `pip` is accessing the correct Python interpreter (you can check by running `pip -V`).

Finally, attempt to build the controller:
```sh
$ cd controller && cargo build --release
```

### AVRDUDE & Ravedude

In order to flash binaries to the Arduino, we can use [AVRDUDE](https://github.com/avrdudes/avrdude). 

Make sure you have Rust **Nightly** (the development version of Rust). You can ensure you are compiling with the Nightly channel by checking the `channel` attribute in [`rust-toolchain.toml`](/controller/rust-toolchain.toml) 

Install AVRDUDE by following the [avr-hal Quickstart Guide](https://github.com/Rahix/avr-hal?tab=readme-ov-file#quickstart). For macOS, run:

```sh
$ xcode-select --install # You may already have done this part
$ brew tap osx-cross/avr
$ brew install avr-gcc avrdude
```

Next, install the `ravedude` CLI tool:

```sh
$ cargo install ravedude
```

You also want to ensure `ravedude` is aware of the location of the serial port. We will expand on this later.

Now, you should be able to compile Rust code and flash it to the Arduino. We will do this in the next section.

### Setting up the Arduino

Your computer must be able to connect to two peripherals: the Arduino and a webcam.

1. Ensure the Arduino is properly equipped with an [Arduino Uno CNC shield](https://www.amazon.com/Shield-Expansion-Stepper-Engraver-Printer/dp/B07DXNZ9PS)
2. Ensure the CNC shield is connected to a $\text{12 V}$ power supply.
   Check that the leads of DC adapter are properly secured, and then power the CNC shield.
3. Use a mini-USB cable to connect the Arduino UNO to your computer.

Next, you want grab the device identifier of the serial port for the Arduino:
- On macOS, it should look something like ``/dev/cu.usbmodem101``. 
- On Windows, it should look something like ``COM4`` or ``COM3``.
- On Linux, it should look something like ``/dev/ttyUSB...``.

If you are having trouble finding this, refer to [this guide](https://www.mathworks.com/help/matlab/supportpkg/find-arduino-port-on-windows-mac-and-linux.html).

#### Configuring the serial port

Finally, you want to inform both ``ravedude`` *and* the `SERIAL_PORT` constant in `main.py` about this port.

*It should also be important to note that `ravedude` actually has an auto-port detection implementation, so the following step may optionally be skipped.*

Make sure when you flash code the controller, the **environment variable** ``RAVEDUDE_PORT`` is set to your serial port. 
In Unix-based systems, this can be achieved by the following:

```sh
$ RAVEDUDE_PORT=[port] [command]
```

Specifically, we will use ``cargo run`` to flash code to the Arduino, so our command will look like:
```sh
$ RAVEDUDE_PORT=[port] cargo run # For controller pipeline

# OR:
$ RAVEDUDE_PORT=[port] cargo run --bin move_to_pos # For vision pipeline
```

As a test, you can try running the second command. Make sure your working directory is `controller` (`cd controller`).

Then, update the following constant in `main.py`:

```py
# main.py
...
#: The serial port to connect to.
SERIAL_PORT: Final[str] = '/dev/cu.usbmodem101'  # <-- replace this string with your port!
...
```

## Software Calibration & Guidelines

The source code includes two parts:

- The Python-implemented [vision pipeline](/vision.py) and [PS4 controller pipeline](/main.py)
- The Rust-implemented [controller](/controller/)

### Vision Pipeline

This is the most recently updated component of the project. To run the vision pipeline, the `move_to_pos` binary must first be flashed to the Arduino:

```sh
$ cd controller
$ cargo run --bin move_to_pos
```

After it's done, terminate the process to free the serial port.

#### Calibration

The vision pipeline operates using any standard webcam. Specifically, this implementation uses the Logitech C270 webcam. 

Make sure the webcam is connected to your computer and has a complete unobstructed view of the hockey table, preferably top-down. The camera mount equipped with the table should be enough to handle these requirements.

Next, begin the calibration process of the camera by running `calibrate.py`:

```sh
$ cd ..
$ python calibrate.py
```

You should see two windows, `out` and `warped`:

![alt text](image.png)

**Make sure the orientation of the table is vertical with the robot puck on top, like the image above.** If it is not, either physically rotate the camera, or modify __both__ `vision.py` and `calibrate.py` to apply a rotation operation to each frame:

```py
# vision.py, calibrate.py
...

while True:
    ret, frame = capture.read()
    if not ret:
        break

    # Add one of these lines:
    frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    frame = cv2.rotate(frame, cv2.ROTATE_180)

    frame = cv2.warpPerspective(frame, TRANSFORM_MATRIX, (TABLE_WIDTH, TABLE_HEIGHT))
    frames += 1
    ...
```

In the calibration window labeled `out`, **click** on the corners of the table such that the "white" portion is tightly bound by the lines. You can click again to adjust the corners, or use the sliders to manually adjust coordinates.

Finally, press the `q` key to finish the calibration process. Four coordinates should be printed to standard output:

```sh
$ python calibrate.py 
...
Corners: [(52, 29), (607, 7), (619, 1243), (42, 1215)]
```

Copy the list and replace the `CALIBRATION` constant in `vision.py` with the new calibrated corners:

```py
# vision.py
...
CALIBRATION = (52, 29), (607, 7), (619, 1243), (42, 1215)  # <-- paste here
TABLE_WIDTH, TABLE_HEIGHT = 350, 800
...
```

Since the robot stick is open-loop controlled, it has no true positional awareness. 
Thus, a set origin must be calibrated, and the puck must start at this position every time we restart the vision pipeline.

**Unpower the CNC shield, physically move the robot stick as far left as possible and about ~$\text{1 inch}$ from the top, and then repower the CNC shield.** Again, this must be done every time the vision pipeline needs to be restarted.

Finally, start the vision process:

```sh
$ python vision.py
```

You should notice the robot will initially try moving to the home position, which should be centered and near the robot's goal.
If this movement is off, calibrate either the `X_MUL` and/or `HOME_POS_X` constants in `vision.py`.

### Controller Pipeline

Flash the main binary of the controller to the Arduino:
```sh
$ cd controller
$ cargo run  # Once done, terminate (ctrl+c)
$ cd ..
```

Then, ensure you have a PS5 Dualsense controller connected to your computer. This controller should be automatically detected.

Run `main.py`:
```py
$ python main.py
```

The left joystick should now be able to control the movement of the robot's hockey stick.

## Future Plans

There are few extensions to this project that I hope to implement in the future:

- Closed-loop control of the robot stick *(motor encoders + PID? vision-based detection of the stick? both?)*
- Smoother acceleration/deceleration curves (less skipping and "jerky" movement)
- Automatic calibration system (no need to manually click the corners)
- Switch from a 16 MHz Arduino UNO to a 48 MHz Adafruit(?)