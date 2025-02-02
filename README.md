# Robotic Arm and Hand Tracking Control System

This repository contains three Python scripts for controlling a robotic arm using Leap Motion hand tracking and UDP communication. The system includes:

1. **`arm_control.py`** – Controls the robotic arm based on received position and orientation data.
2. **`hand_angle.py`** – Captures and processes hand movement angles using Leap Motion and sends them via UDP.
3. **`arm_angle.py`** – Tracks hand position and orientation, displaying them in a GUI and transmitting data over UDP.

## Table of Contents
- [Dependencies](#dependencies)
- [Script Details](#script-details)
  - [arm_control.py](#arm_controlpy)
  - [hand_angle.py](#hand_anglepy)
  - [arm_angle.py](#arm_anglepy)
- [Usage](#usage)
- [License](#license)

---

## Dependencies

Ensure the following Python libraries are installed before running the scripts:

```sh
pip install numpy opencv-python keyboard scipy leap
