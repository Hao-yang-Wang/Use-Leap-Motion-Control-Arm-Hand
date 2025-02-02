# Robot Arm and Hand Tracking System

This project involves controlling a robotic arm using real-time hand motion data. It consists of three main scripts:
- **arm_control.py**: Receives position and orientation data to control a robotic arm.
- **hand_angle.py**: Captures hand skeleton data and calculates finger joint angles using Leap Motion.
- **arm_angle.py**: Captures hand position and orientation using Leap Motion and sends it via UDP.

## Installation

Option 1: Using pip
If you are using a Python virtual environment, install dependencies using:

```sh
pip install -r requirements.txt
```

Option 2: Using Conda (Recommended)
To create an isolated Conda environment for this project, run:

```sh
conda env create -f leap_motion_env.yml
conda activate leap_motion_env
```

## Script Descriptions

### 1. arm_control.py

This script controls a robotic arm by receiving real-time position and orientation data over UDP. It communicates with a CPS robotic controller via TCP.

**Features:**
- Connects to a CPS robotic controller.
- Sets speed and position limits for the robotic arm.
- Receives data from UDP and commands the arm accordingly.
- Includes safety constraints for position and orientation limits.

**Usage:**
```sh
python arm_control.py
```

### 2. hand_angle.py

This script captures hand skeleton data from Leap Motion, computes finger joint angles, and sends them via UDP.

**Features:**
- Uses Leap Motion to track hand joints.
- Converts 3D positions into joint angles using quaternion mathematics.
- Sends joint angles via UDP to a specified receiver.
- Visualizes the hand skeleton in a graphical window.

**Usage:**
```sh
python hand_angle.py
```

### 3. arm_angle.py

This script captures hand position and orientation using Leap Motion and transmits the data via UDP.

**Features:**
- Reads real-time hand tracking data.
- Computes Euler angles from quaternions.
- Displays position and orientation data in a Tkinter GUI.
- Sends hand pose data via UDP.

**Usage:**
```sh
python arm_angle.py
```

### 4. requirements.txt

This file contains the necessary dependencies to run the project, including:
- `leap` (for Leap Motion tracking)
- `numpy`, `scipy`, and `opencv-python` (for numerical and image processing)
- `keyboard` (for key event detection)
- `dynamixel-sdk` (for motor control)

## Running the System
1. Open three terminals
2. Run `arm_control.py` to begin listening for position data.
3. Run `hand_angle.py` to track hand movements and send joint angles.
4. Run `arm_angle.py` to capture and send hand position and orientation.
5. The robotic arm will follow the received data to mimic hand movements.

## License

This project is released under the MIT License.
