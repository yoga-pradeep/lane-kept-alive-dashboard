# Real-Time Lane Keep Assist (LKA) Dashboard

<p align="center">
  <img src="output.gif" alt="LKA Demo" width="900">
</p>

A real-time **C++ / OpenCV** project that processes dashcam video, detects lane boundaries, estimates the vehicle's lateral offset, computes steering correction, and visualizes the results on a live **ECU-style dashboard** with a rotating steering wheel.

This project demonstrates a complete ADAS pipeline:

**Perception → Control → Visualization**

---

## Features

* Real-time lane detection using OpenCV
* Lateral offset estimation
* Steering angle calculation
* Steering angle saturation (±30°)
* Live dashboard visualization
* Rotating steering wheel animation
* Modular C++ architecture

---

## Demo

The application displays two panels:

### Left

* Dashcam video
* Detected lane boundaries
* Lane center and vehicle center

### Right

* Lateral offset
* Steering equation
* Steering angle
* Controller status
* Rotating steering wheel

---

## System Pipeline

```text
Video Frame
      │
      ▼
Lane Detection
      │
      ▼
Lateral Offset
      │
      ▼
LKA Controller
      │
      ▼
Dashboard Visualization
```

---

## Project Modules

| Module            | Responsibility                                                       |
| ----------------- | -------------------------------------------------------------------- |
| **LaneDetector**  | Detects lane boundaries and computes lateral offset                  |
| **LkaController** | Converts lateral offset into a steering angle with ±30° safety limit |
| **DashboardUI**   | Renders the dashboard and rotating steering wheel                    |
| **main.cpp**      | Connects all modules and runs the application                        |

---

## Control Law

```text
Steering Angle = -(Lateral Offset × 15)
```

The controller uses **negative feedback** to keep the vehicle centered and limits steering commands to **±30°**.

---

## Pixel-to-Meter Conversion

```text
meters_per_pixel = 3.7 / 600 ≈ 0.00617
```

```text
Vehicle Offset = Camera Center − Lane Center
Lateral Offset = Vehicle Offset × meters_per_pixel
```

---

## Project Structure

```text
.
├── include/
├── src/
├── data/
├── output.gif
├── main.cpp
└── README.md
```

---

## Build

Install OpenCV:

```bash
brew install opencv pkg-config
```

Compile:

```bash
clang++ -std=c++17 main.cpp src/*.cpp \
-I include -I . \
$(pkg-config --cflags --libs opencv4) \
-o lka_demo
```

---

## Run

```bash
./lka_demo data/dashcam.mp4
```

Press **Q** or **ESC** to exit.

---

## Technologies

* C++
* OpenCV
* Computer Vision
* Image Processing
* Lane Detection
* ADAS
* Automotive Software

---

## Future Improvements

* Camera calibration
* Bird's-eye perspective transform
* PID controller
* Curved lane detection
* Lane confidence estimation

---

## Why this project

This project demonstrates how computer vision and control theory can be combined to build a real-time **Lane Keep Assist (LKA)** simulation using **C++** and **OpenCV**, following a modular design similar to modern ADAS software.

