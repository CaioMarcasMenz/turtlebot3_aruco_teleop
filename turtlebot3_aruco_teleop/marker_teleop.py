#!/usr/bin/env python

from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

import argparse
import time
import cv2
import sys
import numpy as np
import os
import rclpy
import pyrealsense2 as rs

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

# Kamera-Kalibrierung (optional anpassen)
cameraMatrix = np.array([[655.89097648, 0.0, 320.94706113],
                         [0.0, 654.41074471, 220.63657023],
                         [0.0, 0.0, 1.0]])
distCoeffs = np.array([-0.158630475, 0.997450260, 0.00127607205, -0.00131935808, -2.03742860])
markerSize = 100  # in mm
axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]])

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')

def main():
    Field1 = ""
    Field2 = "Geradeaus"
    distance_forward_start = 70
    distance_forward_end = distance_forward_start + 30
    distance_backward_start = distance_forward_end + 5
    distance_backward_end = distance_backward_start + 10
    angular_start = 0.3
    angular_end = 1.5
    tvec_average = tvec_pre_1 = tvec_pre_2 = 0
    rvec_average = rvec_pre_1 = rvec_pre_2 = 0
    rvec_toggle_param = 1
    velocity_linear = velocity_angular = 0

    ap = argparse.ArgumentParser() 
    ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL",
                    help="Typ des zu erkennenden ArUCo-Markers")
    args = vars(ap.parse_args())

    ARUCO_DICT = {
        "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
        "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
        "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
    }

    if ARUCO_DICT.get(args["type"], None) is None:
        print("[INFO] Der ArUCo-Tag '{}' wird nicht unterstützt".format(args["type"]))
        sys.exit(0)

    print("[INFO] Suche nach '{}' Tags...".format(args["type"]))
    arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
    arucoParams = cv2.aruco.DetectorParameters() #arucoParams = cv2.aruco.DetectorParameters_create()

    print("[INFO] Starte RealSense-Video-Stream...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    pipeline.start(config)
    time.sleep(2.0)

    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    pub = node.create_publisher(Twist, 'cmd_vel', qos) 

    # Bildschirmauflösung ermitteln (z. B. 1920x1080)
    screen_width = 1920
    screen_height = 1080

    # Fenster im Vollbildmodus anlegen
    cv2.namedWindow("Frame", cv2.WND_PROP_FULLSCREEN)
    cv2.setWindowProperty("Frame", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)



    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())

            (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)

            if len(corners) > 0:
                ids = ids.flatten()

                for (markerCorner, markerID) in zip(corners, ids):
                    if markerID == 2:
                        corners = markerCorner.reshape((4, 2))
                        (topLeft, topRight, bottomRight, bottomLeft) = corners
                        topRight = (int(topRight[0]), int(topRight[1]))
                        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                        topLeft = (int(topLeft[0]), int(topLeft[1]))

                        cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                        cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                        cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                        cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

                        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                        cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)

                        rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorner, markerSize, cameraMatrix, distCoeffs)

                        tvec_average = (tvec.item(2) + tvec_pre_1 + tvec_pre_2) / 30

                        if distance_forward_start < tvec_average < distance_forward_end:
                            velocity_linear = round((1 - (round(tvec_average, 1) - distance_forward_start) /
                                                     (distance_forward_end - distance_forward_start)) *
                                                    BURGER_MAX_LIN_VEL, 2)
                            Field2 = "Vorwaerts        "
                        elif distance_backward_start < tvec_average < distance_backward_end:
                            velocity_linear = round(((round(tvec_average, 1) - distance_backward_start) /
                                                     (distance_backward_end - distance_backward_start)) *
                                                    -BURGER_MAX_LIN_VEL, 2)
                            Field2 = "Rueckwaerts      "
                        elif distance_forward_end < tvec_average < distance_backward_start:
                            velocity_linear = 0
                            Field2 = "Stopp            "

                        rvec_toggle_param = 1 if rvec.item(0) > 0 else -1
                        rvec_average = (rvec.item(1) + rvec_pre_1 + rvec_pre_2) / 3

                        if -angular_end < rvec_average < -angular_start:
                            velocity_angular = round(
                                rvec_average / (angular_end - angular_start) *
                                BURGER_MAX_ANG_VEL * rvec_toggle_param, 2)
                            Field1 = "Links            "
                        elif angular_start < rvec_average < angular_end:
                            velocity_angular = round(
                                rvec_average / (angular_end - angular_start) *
                                BURGER_MAX_ANG_VEL * rvec_toggle_param, 2)
                            Field1 = "Rechts           "
                        elif -angular_start < rvec_average < angular_start:
                            velocity_angular = 0
                            Field1 = "Geradeaus        "

                        rvec_pre_2 = rvec_pre_1
                        rvec_pre_1 = rvec.item(1)
                        tvec_pre_2 = tvec_pre_1
                        tvec_pre_1 = tvec.item(2)

                        cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(frame, f"{Field1} velocity_angular: {velocity_angular}", (10, 20),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(frame, f"{Field2} velocity_linear: {velocity_linear}", (10, 40),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        cv2.putText(frame, f"{((tvec.item(2) / 10) - 3.5):.2f} cm", (10, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
            resized_frame = cv2.resize(frame, (screen_width, screen_height))
            cv2.imshow("Frame", resized_frame)
            if cv2.waitKey(1) != -1:
                break

            twist = Twist()
            twist.linear.x = float(velocity_linear)
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(velocity_angular)
            pub.publish(twist)

    except Exception as e:
        print("Ein Fehler ist aufgetreten:", e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        if os.name != 'nt' and settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

        cv2.destroyAllWindows()
        pipeline.stop()

if __name__ == '__main__':
    main()
