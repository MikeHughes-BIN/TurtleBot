# TurtleBot3 Ballverfolgung und Zielnavigation mit ROS2

## Projektziel

In diesem Projekt steuern wir einen TurtleBot3 Roboter so, dass er mithilfe seiner Kamera einen roten Ball erkennt, diesen verfolgt und aktiv in ein Ziel schiebt. Währenddessen nutzt der Roboter zusätzlich einen Ultraschall-Sensor zur Hindernisvermeidung.  

Das Projekt läuft auf einem Jetson Nano mit Raspberry Pi Steuerung, basierend auf ROS2 Humble und OpenCV.

---

## Projektübersicht

### Hardware

- **TurtleBot3 (Burger oder Waffle Pi)**
- **Jetson Nano** (für Bildverarbeitung)
- **Raspberry Pi** (Steuerung TurtleBot3-Board)
- **Ultraschall-Sensor**
- **Kamera (z. B. Raspberry Pi Camera Module)**

### Software

- **ROS2 Humble**
- **Python 3.x**
- **rclpy (ROS2 Python Client Library)**
- **OpenCV (für Bildverarbeitung)**
- **Godot RL Agents** (nur für optionales Training)
- **Stable Baselines3 (optional für RL-Training)**
