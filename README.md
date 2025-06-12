# TurtleBot3 Ball Tracking and Target Navigation with ROS2

## Project Goal

In this project, we control a TurtleBot3 robot to detect a red ball using its camera, follow it, and actively push it into a target area. Meanwhile, the robot utilizes an ultrasonic sensor for obstacle avoidance.

The system runs on a Jetson Nano with Raspberry Pi control, based on ROS2 Humble and OpenCV.

---

## Project Overview

### Hardware

- **TurtleBot3 (Burger or Waffle Pi)**
- **Jetson Nano** (for image processing)
- **Raspberry Pi** (controls TurtleBot3 board)
- **Ultrasonic sensor**
- **Camera (e.g., Raspberry Pi Camera Module)**

### Software

- **ROS2 Humble**
- **Python 3.x**
- **rclpy (ROS2 Python Client Library)**
- **OpenCV (for image processing)**
- **Godot RL Agents** (optional for training)
- **Stable Baselines3** (optional for reinforcement learning training)
