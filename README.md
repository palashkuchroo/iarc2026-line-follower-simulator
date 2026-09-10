# 🤖 IARC 2026 — Line Follower Simulator

A computer-vision-based line-following simulator developed as part of robotics work for IARC 2026.

This project provides a simulation environment for testing line-following logic and robot motion without requiring physical robot hardware.

## 🚀 Features

- 🛣️ Vision-based line detection
- 🤖 Autonomous line-following logic
- 🖥️ Map-based robot simulation
- 🎯 Simulated robot position and orientation
- 🔍 Real-time debug visualization
- ⚙️ Configurable control parameters
- 🔌 Hardware-independent simulation testing

## 🧠 Architecture

```text
Map / Camera Frame
       ↓
Image Processing
       ↓
Line Detection
       ↓
LineFollower
       ↓
Motion Commands
   (vx, vy, ω)
       ↓
Robot Simulator
       ↓
Updated Position & Orientation
Project Structure
iarc2026-line-follower-simulator/
│
├── sim_test.py
└── README.md
Project Structure
iarc2026-line-follower-simulator/
│
├── sim_test.py
└── README.md
Project Structure
iarc2026-line-follower-simulator/
│
├── sim_test.py
└── README.md
Project Structure
iarc2026-line-follower-simulator/
│
├── sim_test.py
└── README.md

