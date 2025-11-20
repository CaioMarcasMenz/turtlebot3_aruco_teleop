#!/usr/bin/env python3

import os
import sys
import time
import argparse
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
import pyrealsense2 as rs

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name != 'nt':
    import termios


# ==========================
#   KAMERA-KALIBRIERUNG
# ==========================

cameraMatrix = np.array([[655.89097648, 0.0, 320.94706113],
                         [0.0, 654.41074471, 220.63657023],
                         [0.0, 0.0, 1.0]])
distCoeffs = np.array([-0.158630475, 0.997450260, 0.00127607205,
                       -0.00131935808, -2.03742860])

markerSize = 100.0  # in mm

# ==========================
#   ROBOTER-KONSTANTEN
# ==========================

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

# ==========================
#   DISTANZ-KONSTANTEN (cm)
# ==========================

# Neutrale Entfernung zur Kamera (zentraler Bezugspunkt)
DIST_NEUTRAL_CAMERA_CM = 100.0

# Wie weit darf der Marker MAXIMAL näher an der Kamera sein,
# bevor die maximale Vorwärtsgeschwindigkeit erreicht ist?
DIST_MAX_FORWARD_OFFSET_CM = 30.0

# Wie weit darf der Marker MAXIMAL weiter von der Kamera weg sein,
# bevor die maximale Rückwärtsgeschwindigkeit erreicht ist?
DIST_MAX_BACKWARD_OFFSET_CM = 15.0

# Größe der "Totzone" um die neutrale Entfernung,
# in der der Roboter stehen bleibt
DIST_DEADZONE_OFFSET_CM = 10.0

# ---------- abgeleitete Grenzwerte ----------

# Vorwärts-Bereich: von äußerster Vorwärtsgrenze bis zur Deadzone
DIST_FORWARD_START = DIST_NEUTRAL_CAMERA_CM - DIST_DEADZONE_OFFSET_CM / 2.0
DIST_FORWARD_END = DIST_FORWARD_START - DIST_MAX_FORWARD_OFFSET_CM

# Rückwärts-Bereich: von Deadzone bis äußerste Rückwärtsgrenze
DIST_BACKWARD_START = DIST_NEUTRAL_CAMERA_CM + DIST_DEADZONE_OFFSET_CM / 2.0
DIST_BACKWARD_END = DIST_BACKWARD_START + DIST_MAX_BACKWARD_OFFSET_CM

# Hilfskonstanten für die lineare Skalierung
FORWARD_OUTER_RANGE_CM = DIST_NEUTRAL_CAMERA_CM - (DIST_MAX_FORWARD_OFFSET_CM + DIST_DEADZONE_OFFSET_CM / 2.0)
BACKWARD_OUTER_RANGE_CM = DIST_NEUTRAL_CAMERA_CM + DIST_MAX_BACKWARD_OFFSET_CM + DIST_DEADZONE_OFFSET_CM / 2.0

# Drehwinkel-Bereich (rvec)
ANGULAR_START = 0.3
ANGULAR_END = 1.5


# ==========================
#   ANZEIGE-KONSTANTEN
# ==========================

SCREEN_WIDTH = 1920
SCREEN_HEIGHT = 1080


# ==========================
#   ZUSTANDSDATEN
# ==========================

@dataclass
class ControlState:
    # Verlauf für Mittelung
    tvec_pre_1: float = 0.0
    tvec_pre_2: float = 0.0
    rvec_pre_1: float = 0.0
    rvec_pre_2: float = 0.0

    # aktuelle Geschwindigkeiten
    velocity_linear: float = 0.0
    velocity_angular: float = 0.0

    # letzte gültige Geschwindigkeiten (wenn Marker sichtbar war)
    base_linear: float = 0.0
    base_angular: float = 0.0

    # Textfelder für Overlay
    Field1: str = ""
    Field2: str = "Vorwaerts        "


# ==========================
#   ARUCO-HILFSFUNKTIONEN
# ==========================

def create_aruco_detector(dict_name: str):
    """
    Erzeugt Dictionary + Parameter und – wenn möglich –
    einen ArucoDetector für neue OpenCV-Versionen.

    Rückgabe:
        aruco_dict, aruco_params, detector, use_new_api
    """
    ARUCO_DICT = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    }

    if ARUCO_DICT.get(dict_name, None) is None:
        print(f"[INFO] Der ArUCo-Tag '{dict_name}' wird nicht unterstützt.")
        sys.exit(0)

    print(f"[INFO] Suche nach '{dict_name}' Tags...")

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[dict_name])

    use_new_api = hasattr(cv2.aruco, "ArucoDetector")

    if use_new_api:
        aruco_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        print(f"[INFO] Verwende neue ArUco-API (ArucoDetector), OpenCV {cv2.__version__}")
    else:
        try:
            aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            aruco_params = cv2.aruco.DetectorParameters()
        detector = None
        print(f"[INFO] Verwende alte ArUco-API (detectMarkers), OpenCV {cv2.__version__}")

    return aruco_dict, aruco_params, detector, use_new_api


def detect_markers(frame, aruco_dict, aruco_params, detector, use_new_api):
    """Kapselt die ArUco-Marker-Erkennung für alte und neue OpenCV-APIs."""
    if use_new_api and detector is not None:
        corners, ids, rejected = detector.detectMarkers(frame)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame, aruco_dict, parameters=aruco_params
        )
    return corners, ids


def estimate_marker_pose(marker_corners, marker_length, camera_matrix, dist_coeffs):
    """
    Wrapper für die Pose-Schätzung:
    - nutzt cv2.aruco.estimatePoseSingleMarkers, wenn vorhanden
    - sonst cv2.solvePnP

    Rückgabe:
        rvec (3,), tvec (3,)
    """
    if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            marker_corners, marker_length, camera_matrix, dist_coeffs
        )
        rvec = rvecs[0].reshape(3)
        tvec = tvecs[0].reshape(3)
        return rvec, tvec

    # Fallback: solvePnP
    half = marker_length / 2.0
    obj_points = np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float32)

    img_points = marker_corners.reshape(4, 2).astype(np.float32)

    ok, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_matrix, dist_coeffs)
    if not ok:
        raise RuntimeError("cv2.solvePnP konnte Pose nicht bestimmen")

    return rvec.reshape(3), tvec.reshape(3)


# ==========================
#   REALSENSE / ROS-SETUP
# ==========================

def init_realsense():
    """Startet die RealSense-Pipeline und gibt sie zurück."""
    print("[INFO] Starte RealSense-Video-Stream...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    pipeline.start(config)
    time.sleep(2.0)
    return pipeline


def init_ros():
    """Initialisiert ROS2, erstellt Node und Publisher."""
    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_marker_steering')
    pub = node.create_publisher(Twist, 'cmd_vel', qos)
    return node, pub


# ==========================
#   LOGIK-FUNKTIONEN
# ==========================

def process_marker(frame, marker_corner, marker_id, state: ControlState):
    """
    Verarbeitet einen einzelnen Marker (ID 2 = Lenkrad),
    aktualisiert den Zustand und zeichnet Overlay.
    """
    pts = marker_corner.reshape((4, 2))
    (topLeft, topRight, bottomRight, bottomLeft) = pts

    topLeft = tuple(map(int, topLeft))
    topRight = tuple(map(int, topRight))
    bottomRight = tuple(map(int, bottomRight))
    bottomLeft = tuple(map(int, bottomLeft))

    # Marker-Umrandung
    cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
    cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
    cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
    cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

    # Mittelpunkt
    cX = int((topLeft[0] + bottomRight[0]) / 2.0)
    cY = int((topLeft[1] + bottomRight[1]) / 2.0)
    cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

    # Pose schätzen
    rvec, tvec = estimate_marker_pose(
        marker_corner, markerSize, cameraMatrix, distCoeffs
    )

    bias = 13.8     # empirisch ermittelt -> dadurch erhält man korrekte Entfernung -> notfalls selber anpassen, zwischen 13.5 - 14.3

    tvec_z = float(tvec[2])/bias

    # Distanz-Mittelung
    tvec_average = (tvec_z + state.tvec_pre_1 + state.tvec_pre_2) / 3.0

    # -----------------------------
    #   LINEARE GESCHWINDIGKEIT
    # -----------------------------
    if  DIST_FORWARD_END < tvec_average < DIST_FORWARD_START:
        state.velocity_linear = round(
            (1 - (round(tvec_average, 1) - FORWARD_OUTER_RANGE_CM) / DIST_MAX_FORWARD_OFFSET_CM)
            * BURGER_MAX_LIN_VEL, 2
        )
        state.Field2 = "Vorwaerts        "


    elif DIST_FORWARD_START <= tvec_average < DIST_BACKWARD_START:
        state.velocity_linear = 0.0
        state.Field2 = "Stopp            "

    elif DIST_BACKWARD_START <= tvec_average < DIST_BACKWARD_END:
        state.velocity_linear = round(
            ((round(tvec_average, 1) - DIST_NEUTRAL_CAMERA_CM - DIST_DEADZONE_OFFSET_CM/2)/DIST_MAX_BACKWARD_OFFSET_CM) * BURGER_MAX_LIN_VEL, 2)
        state.Field2 = "Rueckwaerts      "

    # -----------------------------
    #   ANGULARE GESCHWINDIGKEIT
    # -----------------------------
    rvec_toggle_param = 1 if rvec[0] > 0 else -1

    rvec_y = float(rvec[1])
    rvec_average = (rvec_y + state.rvec_pre_1 + state.rvec_pre_2) / 3.0

    if -ANGULAR_END < rvec_average < -ANGULAR_START:
        state.velocity_angular = round(
            rvec_average / (ANGULAR_END - ANGULAR_START) *
            BURGER_MAX_ANG_VEL * rvec_toggle_param,
            2
        )
        state.Field1 = "Links             "

    elif ANGULAR_START < rvec_average < ANGULAR_END:
        state.velocity_angular = round(
            rvec_average / (ANGULAR_END - ANGULAR_START) *
            BURGER_MAX_ANG_VEL * rvec_toggle_param,
            2
        )
        state.Field1 = "Rechts            "

    elif -ANGULAR_START < rvec_average < ANGULAR_START:
        state.velocity_angular = 0.0
        state.Field1 = "Geradeaus        "

    # Verlauf updaten
    state.rvec_pre_2 = state.rvec_pre_1
    state.rvec_pre_1 = rvec_y
    state.tvec_pre_2 = state.tvec_pre_1
    state.tvec_pre_1 = tvec_z

    # Basis-Geschwindigkeiten merken
    state.base_linear = state.velocity_linear
    state.base_angular = state.velocity_angular

    # Overlay
    cv2.putText(
        frame, f"ID: {marker_id}",
        (topLeft[0], topLeft[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
    )
    cv2.putText(
        frame, f"{state.Field1} angular: {state.velocity_angular:.2f}",
        (10, 20),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
    )
    cv2.putText(
        frame, f"{state.Field2} linear: {state.velocity_linear:.2f}",
        (10, 40),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
    )
    cv2.putText(
        frame, f"Distanz: {(tvec_average):.2f} cm",
        (10, 60),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
    )

    return state


def apply_no_marker_logic(frame, state: ControlState, marker_seen: bool):
    """
    Wendet den Faktor 1/(2 - marker_flag) an:
    - Marker gesehen  -> Faktor 1.0
    - Marker nicht    -> Faktor 0.5 (halbe letzte Geschwindigkeit)
    Zeichnet ggf. Hinweistext.
    """
    marker_flag = int(marker_seen)             # True -> 1, False -> 0
    speed_factor = 1.0 / (2 - marker_flag)     # 1.0 oder 0.5

    state.velocity_linear = state.base_linear * speed_factor
    state.velocity_angular = state.base_angular * speed_factor

    if not marker_seen:
        #state.Field1 = "Marker weg       "
        #state.Field2 = "Halbe Geschw.    "
        cv2.putText(
            frame, "Marker nicht sichtbar",
            (20, 20),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2
        )

    return state


def publish_twist(pub, state: ControlState):
    """Schickt die aktuelle Geschwindigkeit als Twist auf /cmd_vel."""
    twist = Twist()
    twist.linear.x = float(state.velocity_linear)
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = float(state.velocity_angular)
    pub.publish(twist)


# ==========================
#   HAUPTFUNKTION
# ==========================

def main():
    state = ControlState()

    # Argumente
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "-t",
        "--type",
        type=str,
        default="DICT_ARUCO_ORIGINAL",
        help="Typ des zu erkennenden ArUCo-Markers"
    )
    args = vars(ap.parse_args())

    aruco_dict, aruco_params, aruco_detector, use_new_api = create_aruco_detector(args["type"])

    # RealSense starten
    try:
        pipeline = init_realsense()
    except RuntimeError as e:
        print("[FEHLER] Konnte RealSense-Pipeline nicht starten:", e)
        return

    # Terminal-Einstellungen sichern (Linux)
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # ROS2 initialisieren
    node, pub = init_ros()

    # Vollbildfenster
    cv2.namedWindow("Frame", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Frame", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    try:
        while rclpy.ok():
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())

            # Marker erkennen
            corners, ids = detect_markers(
                frame, aruco_dict, aruco_params, aruco_detector, use_new_api
            )

            marker_seen = False

            if ids is not None and len(corners) > 0:
                ids = ids.flatten()
                for (markerCorner, markerID) in zip(corners, ids):
                    if markerID != 2:
                        continue
                    marker_seen = True
                    state = process_marker(frame, markerCorner, markerID, state)
                    break

            # Faktor für "Marker weg"
            state = apply_no_marker_logic(frame, state, marker_seen)

            # Bild auf Bildschirmgröße strecken und anzeigen
            resized_frame = cv2.resize(frame, (SCREEN_WIDTH, SCREEN_HEIGHT), interpolation=cv2.INTER_LINEAR)
            cv2.imshow("Frame", resized_frame)
            if cv2.waitKey(1) != -1:
                break


            # Twist publizieren
            publish_twist(pub, state)

    except Exception as e:
        print("Ein Fehler ist aufgetreten:", e)

    finally:
        # Roboter stoppen
        state.velocity_linear = 0.0
        state.velocity_angular = 0.0
        publish_twist(pub, state)

        if os.name != 'nt' and settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        cv2.destroyAllWindows()
        pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
