# TurtleBot3 Marker-Teleoperation  
### RealSense D435 + ArUco-Lenkrad (ROS2 Humble)

Dieses Projekt ermöglicht die intuitive Steuerung eines **TurtleBot3 Burger** mithilfe eines **ArUco-Markers**, der wie ein physisches Lenkrad verwendet wird.  
Die RealSense-Kamera erkennt **Ausrichtung und Entfernung** des Markers und wandelt diese automatisch in Fahrbefehle (`cmd_vel`) um.

Entwickelt für **Kinder-Workshops, Schulen und Robotik-AGs**, um spielerisch Robotik, Computer Vision und ROS2 zu vermitteln.

---

## 🖼️ Übersichtsgrafik

> *(Bitte den Pfad später durch den GitHub-Link zu deiner eigenen Datei ersetzen.)*

![Teleoperation Illustration](/mnt/data/A_2D_digital_illustration_displays_a_top-down_view.png)

---

## 🎯 Ziel des Projekts

- Steuerung des TurtleBot3 ohne Joystick oder Tastatur  
- Spielerischer Einstieg in Robotik & Computer Vision  
- Demonstration wichtiger Robotik-Konzepte:
  - Marker-Erkennung  
  - Pose-Schätzung  
  - Abstands- und Winkelregelung  
  - ROS2-Kommunikation  
- Fehlertolerante Steuerung:
  → Wenn der Marker kurz verschwindet, fährt der Roboter **mit halber letzter Geschwindigkeit** weiter.

---

## 🧰 Hardware

- **TurtleBot3 Burger**
- **Intel RealSense D435**
- PC/Laptop mit:
  - **Ubuntu 22.04**
  - **ROS2 Humble**
  - USB 3.0
- ArUco-Marker (ID **2**, Dictionary: `DICT_ARUCO_ORIGINAL`)  
  → Empfehlung: auf **Karton** montieren

---

## 🧑‍💻 Software-Voraussetzungen

- ROS2 Humble  
- RealSense SDK (**librealsense2**)  
- Python 3  
- Python-Pakete (`requirements.txt`):
  - `opencv-contrib-python`
  - `numpy`
  - `pyrealsense2`
  - `rclpy`

---

## 📦 Installation

### 1. ROS2-Workspace anlegen

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

---

### 2. Repository klonen

```bash
cd ~/ros2_ws/src
git clone https://github.com/CaioMarcasMenz/turtlebot3_aruco_teleop.git turtlebot3_aruco_teleop
```

---

### 3. Python-Umgebung (optional, aber empfohlen)

Zur sauberen Trennung aller Python-Abhängigkeiten:

```bash
cd ~/ros2_ws
python3 -m venv .venv
source .venv/bin/activate
```

---

### 4. Python-Abhängigkeiten installieren

```bash
pip install -r src/turtlebot3_aruco_teleop/requirements.txt
```

---

### 5. Workspace bauen

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

---

## 📷 Intel RealSense D435 einrichten

### 1. RealSense SDK installieren (falls noch nicht geschehen)

- `librealsense2` installieren  
- Firmware/udev-Regeln übernehmen  
- Einmal neu starten

### 2. Kamera testen

```bash
realsense-viewer
```

Wenn ein Kamerabild angezeigt wird → RealSense funktioniert.

---

## ▶️ Demo starten

### 1. TurtleBot3 vorbereiten

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
```

---

### 2. Marker-Teleoperation starten

Auf deinem Rechner (mit angeschlossener Kamera):

```bash
cd ~/ros2_ws
source .venv/bin/activate   # falls genutzt
source install/setup.bash
ros2 run turtlebot3_aruco_teleop marker_teleop
```

Ein Vollbild-Fenster öffnet sich.  
Das Live-Bild der RealSense erscheint, und der Roboter reagiert auf den Marker.

---

## 🤖 Funktionsweise

### 1. Marker-Erkennung

OpenCV erkennt den Marker mit:

- neuer API → `cv2.aruco.ArucoDetector`  
- alter API → `cv2.aruco.detectMarkers`  

→ Der Code unterstützt **beide Varianten automatisch**.

---

### 2. Pose-Schätzung

Wenn möglich:

```python
cv2.aruco.estimatePoseSingleMarkers()
```

Sonst Fallback:

```python
cv2.solvePnP()
```

---

## 🎛️ Steuerlogik

### 📌 Distanz → Vorwärts / Rückwärts

Alle Distanzwerte sind **in cm** definiert.

| Variable | Bedeutung |
|---------|-----------|
| `DIST_NEUTRAL_CAMERA_CM` | neutrale Entfernung (Stopp) |
| `DIST_MAX_FORWARD_OFFSET_CM` | maximale Annäherung |
| `DIST_MAX_BACKWARD_OFFSET_CM` | maximale Entfernung |
| `DIST_DEADZONE_OFFSET_CM` | Bereich ohne Bewegung |

**Abgeleitete Bereiche:**

- **Vorwärtsbereich**: Marker **näher** als die Totzone  
- **Totzone**: Marker im **Neutralbereich** → Roboter stoppt  
- **Rückwärtsbereich**: Marker **weiter weg** als die Totzone  

**Besonderheiten:**

- Geschwindigkeit wird **linear skaliert**  
- Distanz wird **über 3 Messungen geglättet**

---

### 📌 Drehung → Links / Rechts

Für die Rotation wird `rvec[1]` (Y-Achse) genutzt.

Parameter:

- `ANGULAR_START` – minimale Reaktionsschwelle  
- `ANGULAR_END` – maximale Sättigung  
- Glättung über die letzten 3 Werte

**Bereiche:**

- zwischen `-ANGULAR_START … +ANGULAR_START` ⇒ **geradeaus**  
- außerhalb ⇒ **proportionale Drehung** bis `BURGER_MAX_ANG_VEL`

---

### 📌 Marker außer Sicht → Fehlerrobust

Wenn der Marker nicht erkannt wird:

```python
speed_factor = 1 / (2 - marker_seen_flag)
```

- Marker sichtbar → **1.0**
- Marker weg → **0.5**

→ Der Roboter fährt **ruhig und kontrolliert weiter**, statt abrupt zu stoppen.  
Perfekt für Workshops mit Kindern.

---

## 🖥️ Visualisierung

- Vollbildmodus (`WINDOW_FULLSCREEN`)
- Bild auf **1920×1080** skaliert
- Overlay zeigt:
  - Linear- & Rotationsgeschwindigkeit  
  - Richtungstext (Links/Rechts/Vorwärts/Rückwärts/Stop)  
  - Geschätzte Distanz  
  - „Marker nicht sichtbar“-Hinweis  

---

## 👶 Tipps für Workshops

- Marker auf **A4-Karton** kleben (fühlt sich wie ein echtes Lenkrad an)  
- Kamera ca. **1 m** Abstand  
- Parcours mit Kreppband auf dem Boden markieren  
- Kindern zuerst das **Live-Kamerabild** zeigen  
- Einfaches Merkschema:
  - „**Drehen = Lenken**“
  - „**Näher ran = Gas geben**“
  - „**Weiter weg = Rückwärts**“

---

## 🔧 Kalibrierung

### Bias-Faktor

Der RealSense Z-Wert (`tvec[2]`) ist stark setupspezifisch.  
Daher wird er durch einen empirischen Faktor geteilt:

```python
bias = 13.8
tvec_z = tvec[2] / bias
```

Typischer Wertebereich: **13.5–14.3**  
→ hängt ab von Kameraabstand, Markergröße und Beleuchtung.

---
