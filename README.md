# TurtleBot3 ArUco Marker Teleop

Interaktive TurtleBot3-Steuerung mit einem **physischen „Lenkrad“ aus Papier**:  
Eine Intel RealSense D435 Kamera erkennt einen ArUco-Marker und steuert damit den Roboter – ideal für **Kinder, Schulen und Workshops**, um Robotik, Computer Vision und ROS2 spielerisch zu erleben.

- Drehung des Markers → Roboter lenkt nach links/rechts  
- Abstand des Markers → Roboter fährt vorwärts, rückwärts oder langsam weiter, wenn der Marker kurz aus dem Bild verschwindet  

---

## Inhalt

- [Ziel des Projekts](#ziel-des-projekts)
- [Hardware](#hardware)
- [Software-Voraussetzungen](#software-voraussetzungen)
- [Installation](#installation)
  - [ROS2-Workspace anlegen](#ros2-workspace-anlegen)
  - [Repository klonen](#repository-klonen)
  - [Python-Umgebung (optional, empfohlen)](#python-umgebung-optional-empfohlen)
  - [Python-Abhängigkeiten installieren](#python-abhängigkeiten-installieren)
  - [Workspace bauen](#workspace-bauen)
- [Intel RealSense D435 einrichten](#intel-realsense-d435-einrichten)
- [Demo starten](#demo-starten)
- [Funktionsweise in Kurzform](#funktionsweise-in-kurzform)
- [Tipps für Workshops mit Kindern](#tipps-für-workshops-mit-kindern)
- [Troubleshooting](#troubleshooting)

---

## Ziel des Projekts

Dieses Projekt zeigt, wie man einen **TurtleBot3** mit Hilfe eines **ArUco-Markers** und einer **Intel RealSense D435** steuern kann.  
Der Marker wird dabei wie ein **Lenkrad** genutzt:

- Kinder halten ein Blatt mit einem gedruckten Marker vor die Kamera.
- Durch Drehen und Vor-/Zurückbewegen des Blatts steuern sie den Roboter.
- Die Kamera erkennt Position und Ausrichtung des Markers und das ROS2-Programm wandelt diese in Geschwindigkeit (`cmd_vel`) um.

---

## Hardware

Benötigt wird:

- **TurtleBot3 Burger**
- **Intel RealSense D435** (Stereo-Kamera)
- PC/Laptop mit:
  - Ubuntu **22.04** (oder neuer, kompatibel mit ROS2 Humble)
  - USB-3.0-Port für die RealSense
- Ein Blatt Papier mit ArUco-Marker (z. B. ID 2 aus DICT_ARUCO_ORIGINAL)

---

## Software-Voraussetzungen

- **Ubuntu 22.04**
- **ROS 2 Humble**
- TurtleBot3-ROS-Pakete (z. B. `turtlebot3`, `turtlebot3_bringup`, `turtlebot3_msgs`)
- Intel **RealSense SDK** (`librealsense2`) inkl. udev-Regeln
- Python 3
- Python-Pakete (werden gleich über `requirements.txt` installiert):
  - `numpy`
  - `opencv-contrib-python`
  - `pyrealsense2`
  - ggf. weitere je nach Code-Stand

---

## Installation

### ROS2-Workspace anlegen

Falls noch kein ROS2-Workspace existiert:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

