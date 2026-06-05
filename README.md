# Self-Balancing Robot

## Overview
This project implements a two-wheeled self-balancing robot using a microcontroller, MPU6050 and PID control. The system models an inverted pendulum and applies state-space control methods to achieve upright stabilization on real hardware.

---

## Features
- Derived nonlinear equations of motion using Lagrangian mechanics
- Linearized state-space model about the upright equilibrium point
- Feedback controller design using pole placement
- Implementation on Arduino-based embedded system
- Complementary filter for real-time angle estimation
- Achieved sustained balancing on physical hardware (>60 seconds)

---

## Hardware
- Arduino microcontroller
- MPU6050
- DC motors
- L298N motor driver
- 3.7 Lithium Batteries (2)

---

## Software
- Embedded C++ (Arduino)
- Python
- Control Toolbox

---

## StatusPython
- Mathematical model derived and validated
- State-space controller designed and implemented
- Complementary filter implemented
- Successfully demonstrated stable balancing for 60+ seconds on repeated trials
- Ongoing improvements to robustness and performance
