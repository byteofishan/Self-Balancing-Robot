# 🤖 Self-Balancing Robot (v1.0)

A two-wheeled **self-balancing robot** built using Arduino,MPU6050, and a PID controller.  
This project demonstrates closed-loop feedback control to maintain balance on a dynamic platform — a classic control systems and robotics challenge.

---

## 🧩 Overview

This robot uses an **MPU6050 accelerometer + gyroscope** to sense its tilt angle and applies **PID (Proportional–Integral–Derivative)** control to drive the motors in real time, keeping the robot upright.  
It’s a great introduction to embedded control, sensors, and actuator feedback systems.

---

## ⚙️ Hardware Requirements

| Component | Description |
|------------|-------------|
| Arduino Uno / Nano | Main microcontroller |
| MPU6050 IMU | For angle and angular velocity sensing |
| Motor Driver | L298N / TB6612FNG / DRV8833 |
| 2× DC Gear Motors | Identical specs for balance |
| Wheels + Chassis | Rigid and centered layout |
| Battery Pack | Li-ion / Li-Po (7.4 V – 12 V) |
| Optional | Switch, regulator, jumper wires |

---

## 🪜 Working Principle

1. The **MPU6050** measures acceleration and angular velocity.  
2. A complementary filter (or simple math) estimates the **tilt angle**.  
3. The **PID controller** computes an output proportional to the tilt error.  
4. The output controls the **motor speed and direction** to restore balance.

---

## 🧠 Control Logic (PID)

The PID controller constantly adjusts motor output to correct tilt:

- **Kp**: Controls how strongly the robot reacts to angle deviation  
- **Ki**: Eliminates small steady-state drift  
- **Kd**: Adds damping to prevent oscillations  

Tuning these values is key to stable balancing.




