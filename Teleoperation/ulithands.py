import cv2
import mediapipe as mp
import time
import math
import numpy as np
import socket as socket
import serial
import os
import requests
import json
from datetime import datetime
import serial
import struct
import time
import queue
import threading

from sklearn.metrics import euclidean_distances

#immediate ammend, ifhand track lost... put arm ovement into smooth

#POINT OF CONCERN PITCH DRAMTICALLY CRAHES AT PITCH ANGLE>100#corrected
#POINT OF CONCERN PICH ANGLE LAGS behind actual pitch


# Creating VideoCapture object

feed = cv2.VideoCapture(0)
#feed.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#feed.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
feed.set(cv2.CAP_PROP_FPS, 50)

ESP32_IP = "192.168.4.1"

SERIAL_PORT = "COM10"  # change if needed
BAUD = 115200

ser = serial.Serial(port=SERIAL_PORT, baudrate=BAUD, timeout=0.01)
ESP32_PORT = 4210   # must match ESP32 udp.begin(PORT)

udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_sock.setblocking(False)
# =========================================================
# ROARM HTTP CONFIG
# =========================================================
ROARM_IP = "192.168.4.1"


TARGET_FPS = 50
FRAME_DT = 1.0 / TARGET_FPS
last_frame = time.perf_counter()

# Creating a socket and connecting to ESP32
#client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

#Mediapipe Hand Detection setup
hands = mp.solutions.hands.Hands(
    model_complexity=0,
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.1,
    min_tracking_confidence=0.1
)

mpDraw = mp.solutions.drawing_utils

#variables to define startsequence booleans
startSequence = True

# Initialize variables
gripper_angle_last = 0
last_FPS   = 0
stable_FPS = 0
font = cv2.FONT_HERSHEY_SIMPLEX
last_servo_angle = 0
x_frame          = 640
y_frame          = 360
gripper_locked = False

sent = 0
start_send=time.perf_counter()

# Utility functions
def map_value(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])

gripper_toggle_ready = True
def get_gripper_angle(landmarks):
    global gripper_angle_last
    global gripper_locked
    global gripper_toggle_ready

    tip_index  = landmarks[8]
    tip_thumb  = landmarks[4]
    tip_middle = landmarks[12]
    base_middle = landmarks[9]

    tip_fore = landmarks[16]

    scale_ref = distance((landmarks[0].x, landmarks[0].y), (landmarks[5].x, landmarks[5].y))
    pinch_distance = distance((tip_index.x, tip_index.y), (tip_thumb.x, tip_thumb.y))
    lock_distance  = distance((tip_fore.x, tip_fore.y), (tip_thumb.x, tip_thumb.y))

    normalised_distance = round(pinch_distance / scale_ref, 2)
    normalised_distance_lock = round(lock_distance / scale_ref, 2)

    # --- Toggle logic ---
    if normalised_distance_lock <= 0.2:
        if gripper_toggle_ready:
            gripper_locked = not gripper_locked
            gripper_toggle_ready = False  # prevent rapid toggling while gesture is held
    else:
        gripper_toggle_ready = True  # ready to toggle next time gesture is made

    # --- Gripper angle smoothing ---
    gripper_angle_raw = round(map_value(normalised_distance, 0.3, 0.7, 0, 60), 1)
    gripper_angle_raw = max(0, min(60, gripper_angle_raw))
    gripper_angle = round((0.1 * gripper_angle_raw) + (0.9 * gripper_angle_last), 1)
    gripper_angle_last = gripper_angle

    return 60.0 - gripper_angle


def get_roll_angle(landmarks):
    index_base = landmarks[5]
    pinky_base = landmarks[17]
    dx = index_base.x - pinky_base.x
    dz = index_base.z - pinky_base.z
    roll_rad = math.atan2(dz, dx)
    roll_deg = round(math.degrees(roll_rad),1)

    return roll_deg

def get_yaw_angle(landmark):
    base_wrist  = landmark[0]
    base_middle = landmark[9]
    dx = base_wrist.x - base_middle.x
    dy = base_wrist.y - base_middle.y
    raw_yaw_rad = math.atan2(dx, dy)
    raw_yaw_deg = round(math.degrees(raw_yaw_rad),1)
    return raw_yaw_deg

def constrain(val, min_val, max_val):
    return max(min_val, min(val, max_val))

def adjust_gamma(image, gamma=1.5):
    invGamma = 1.0 / gamma
    table = np.array([
        ((i / 255.0) ** invGamma) * 255 for i in np.arange(256)
    ]).astype("uint8")

    return cv2.LUT(image, table)

def send_udp_xyz(x, y, z):
    payload = {
        "T": 1041,
        "x": int(x),
        "y": int(y),
        "z": int(z),
        "t": 3.0
        # "spd": 70   # optional
    }

    msg = json.dumps(payload).encode("utf-8")
    udp_sock.sendto(msg, (ESP32_IP, ESP32_PORT))
"""
def send_udp_json_end(x, y, z):
    payload = {
        "T": 104,
        "x": int(x),
        "y": int(y),
        "z": int(z),
        "t": 3.0,
        "spd": 300  # optional
        }
    msg = json.dumps(payload).encode("utf-8")
    udp_sock.sendto(msg, (ESP32_IP, ESP32_PORT))

"""

def send_udp_json_end(x, y, z):
    payload = {
        "T": 104,
        "x": int(x),
        "y": int(y),
        "z": int(z),
        "t": 3.0,
        "spd": 10   # optional
    }

    msg = json.dumps(payload).encode("utf-8")
    udp_sock.sendto(msg, (ESP32_IP, ESP32_PORT))

def send_udp_json_angle(joint, rad, speed, acc):
    payload = {
        "T": 101,
        "joint": joint,
        "rad": rad,
        "spd": speed,
        "acc": acc
    }

    msg = json.dumps(payload).encode("utf-8")
    udp_sock.sendto(msg, (ESP32_IP, ESP32_PORT))


def send_light_command(led):
    payload = {
        "T": 114,
        "led": led
    }

    msg = json.dumps(payload).encode("utf-8")
    #udp_sock.sendto(msg, (ESP32_IP, ESP32_PORT))

"""
def get_pitch_angle(landmarks):
    wrist = landmarks[0]
    #WHY AM I EVEN TAKING THE CENTER OF PALM AND NOT THE TIP or the base of the middle findeger
    palm  = landmarks[12]  # Approx center of palm
    pitch_deg = 0
    dy = palm.y - wrist.y
    dz = palm.z - wrist.z


    # Pitch = angle between hand and vertical axis, in yz plane
    pitch_rad = math.atan2(dz, dy)
    pitch_deg_raw = math.degrees(pitch_rad)

    # Normalize pitch to range 0–180:
    pitch_deg = 90.0 - pitch_deg  # Flip so forward (palm toward camera) is 0

    #if pitch angle is greater than -180 but less than 0 pitch is in upper half
    if pitch_deg_raw>-180.0 and pitch_deg_raw<-90.0:
        pitch_deg = 90.0 + (-((-180.0)-pitch_deg_raw))

    #if pitch angle is less than 180 and greater than 90 hand in in down pitch quad
    elif pitch_deg_raw<180 and pitch_deg_raw>90.0:
        pitch_deg = pitch_deg_raw-90.0

    #subtract 180 to correct bias in calculation
    pitch_deg = round(180.0 - pitch_deg,1)
    pitch_deg = constrain(pitch_deg, 60.0,100.0)
    pitch_deg = pitch_deg_raw

    return pitch_deg_raw
"""
import math

def get_pitch_angle(landmarks):
    wrist = landmarks[0]
    palm  = landmarks[12]   # center of palm (this is fine 👍)

    # MediaPipe coordinates:
    # +Y = down
    # +Z = away from camera

    dy = palm.y - wrist.y
    dz = palm.z - wrist.z

    # Signed angle in radians (range -pi to +pi)
    pitch_rad = math.atan2(dz, dy)

    # Convert to degrees
    pitch_deg = math.degrees(pitch_rad)   # -180 → +180

    # Shift so palm-facing-camera = 180°
    pitch_deg = pitch_deg + 180.0          # 0 → 360

    if pitch_deg>300:
        pitch_deg = -(360.0-pitch_deg)
        return round(pitch_deg, 1)

    elif pitch_deg>0.0 and pitch_deg<=60.0:
        return round(pitch_deg, 1)


    # Optional smoothing clamp for servo safety
    #pitch_deg = constrain(pitch_deg, 60.0, 300.0)



def send_uart_json(x, y, z):
       payload = {
           "T": 1041,
           "x": int(x),
           "y": int(y),
           "z": int(z),
           "t": 3.0
           #"spd": 70
       }

       msg = json.dumps(payload) + "\n"
       ser.write(msg.encode("utf-8"))


def send_uart_json_eoat(pitch, yaw, roll):
    payload = {
        "T": 160,
        "pitch": pitch,
        "yaw": yaw,
        "roll": roll,
        "spd": 0,
        "acc" :0
    }

    msg = json.dumps(payload) + "\n"
    ser.write(msg.encode("utf-8"))



def send_http_command_eoat(pitch, yaw, roll):
    payload = {
        "T": 160,
        "pitch": pitch,
        "yaw": yaw,
        "roll": roll,
        "spd": 0,
        "acc":0
    }

    try:
        requests.get(
            f"http://{ROARM_IP}/js",
            params={"json": json.dumps(payload)},
            timeout=HTTP_TIMEOUT
        )
    except requests.exceptions.RequestException:
        pass

def send_http_command(x, y, z):
    payload = {
        "T": 1041,
        "x": x,
        "y": y,
        "z": z,
        "t": 3.0
        #"spd":70
    }

    try:
        requests.get(
            f"http://{ROARM_IP}/js",
            params={"json": json.dumps(payload)},
            timeout=HTTP_TIMEOUT
        )
    except requests.exceptions.RequestException:
        pass


def send_end_sequence_1():
    send_udp_json_angle(2, -0.5, 400, 60);
    time.sleep(2)

    # earlier rad was at 1.0
    send_udp_json_angle(3, 2.2, 400, 60);
    time.sleep(2)

    # earlier at -3.14
    send_udp_json_angle(4, -0.1, 400, 60);
    time.sleep(2)

    send_udp_json_angle(1, 0, 400, 60);

    # time.sleep(2)

    print("end command end")

def apply_clahe(frame):
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)

    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    l = clahe.apply(l)

    lab = cv2.merge((l, a, b))
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)


# ── shared buffers ──────────────────────────────────────────────────────────
frame_q  = queue.Queue(maxsize=2)   # raw frames  (capture → process)
result_q = queue.Queue(maxsize=2)   # annotated frames (process → display)

first = True
euclidean_distance = 0

A, B, C = np.polyfit(
    [300, 245, 200, 170, 145, 130, 112, 103, 93, 87, 80, 75, 70, 67, 62, 59, 57],
    [20,  25,  30,  35,  40,  45,  50,  55,  60, 65, 70, 75, 80, 85, 90, 95, 100],
    2
)
scale_factor = 3.0

stop_event = threading.Event()

# ── THREAD 1: frame capture ─────────────────────────────────────────────────
def capture_thread():
    while not stop_event.is_set():
        ret, frame = feed.read()
        # Resize for MediaPipe
        frame = cv2.resize(
            frame,
            (640, 360),
            interpolation=cv2.INTER_LINEAR
        )
        if not ret:
            continue
        frame = cv2.flip(frame, 1)
        # drop the oldest frame if the processor hasn't caught up
        if frame_q.full():
            try:
                frame_q.get_nowait()
            except queue.Empty:
                pass
        frame_q.put(frame)
    feed.release()

# ── THREAD 2: MediaPipe processing + UART send ──────────────────────────────
def process_thread():
    global first, euclidean_distance
    global last_x, last_y, last_z
    global stable_FPS, last_FPS

    last_FPS   = 0
    stable_FPS = 0

    while not stop_event.is_set():
        try:
            frame = frame_q.get(timeout=0.1)
        except queue.Empty:
            continue

        startTime = time.perf_counter()
        res_frame = frame.copy()

        RGB_frame = cv2.cvtColor(res_frame, cv2.COLOR_BGR2RGB)
        results   = hands.process(RGB_frame)

        if results.multi_hand_landmarks:
            for hand in results.multi_hand_landmarks:
                pitch   = 0
                yaw     = 0
                roll    = 0
                gripper = 0

                mapped_x = map_value(hand.landmark[0].x, 0, 1, -35, 35)
                mapped_y = map_value(hand.landmark[0].y, 0, 1, 31, -2)
                mapped_x = max(-50, min(50, mapped_x))
                mapped_y = max(0,   min(50, mapped_y))

                x5  = int(hand.landmark[5].x  * 640)
                y5  = int(hand.landmark[5].y  * 360)
                x17 = int(hand.landmark[17].x * 640)
                y17 = int(hand.landmark[17].y * 360)

                dis = math.sqrt((x17 - x5)**2 + (y17 - y5)**2) * scale_factor
                distance_cm = abs(A * dis**2 + B * dis + C) - 50
                distance_cm -= (-25)
                distance_cm = max(-12, min(40, distance_cm))

                mapped_x    = int(mapped_x    * 10)
                mapped_y    = int(mapped_y    * 10)
                distance_cm = int(distance_cm * 10)

                roArm_x = mapped_y * 2
                roArm_y = -int(mapped_x * 1.5)
                roArm_z = distance_cm

                if first:
                    last_x, last_y, last_z = roArm_x, roArm_y, roArm_z
                    first = False
                else:
                    dx = roArm_x - last_x
                    dy = roArm_y - last_y
                    dz = roArm_z - last_z
                    euclidean_distance = math.sqrt(dx*dx + dy*dy + dz*dz)

                speed_factor = 0.8 if euclidean_distance > 50 else (0.9 if euclidean_distance > 20 else 1.0)

                smooth_x = last_x + (roArm_x - last_x) * speed_factor
                smooth_y = last_y + (roArm_y - last_y) * speed_factor
                smooth_z = last_z + (roArm_z - last_z) * speed_factor

                last_x, last_y, last_z = smooth_x, smooth_y, smooth_z

                send_uart_json(roArm_x, roArm_y, roArm_z)

                # on-frame annotations
                cv2.putText(res_frame, f"Wrist X : {mapped_x}",              (1000, 790), font, 1, (0, 255, 0), 3)
                cv2.putText(res_frame, f"Wrist Y : {mapped_y}",              (1000, 820), font, 1, (0, 255, 0), 3)
                cv2.putText(res_frame, f"Wrist Z : {distance_cm}",           (1000, 850), font, 1, (0, 255, 0), 3)
                cv2.putText(res_frame, f"Eucledian distance : {euclidean_distance}", (50, 700), font, 1, (0, 255, 0), 3)

        endTime = time.perf_counter()
        delta       = endTime - startTime
        current_FPS = 1 / delta if delta > 0 else 0
        stable_FPS  = round(0.1 * current_FPS + 0.9 * last_FPS, 1)
        last_FPS    = stable_FPS

        cv2.putText(res_frame, f"FPS: {stable_FPS}", (30, 30), font, 1, (0, 255, 0), 3)

        # pass annotated frame to display; drop if display is behind
        if result_q.full():
            try:
                result_q.get_nowait()
            except queue.Empty:
                pass
        result_q.put(res_frame)

# ── THREAD 3: display (must stay on the main thread on most platforms) ───────
# OpenCV's imshow/waitKey requires the main thread, so we start the two worker
# threads and run the display loop here.

t_capture = threading.Thread(target=capture_thread, daemon=True)
t_process = threading.Thread(target=process_thread, daemon=True)

t_capture.start()
t_process.start()

while True:
    try:
        display_frame = result_q.get(timeout=0.1)
    except queue.Empty:
        if stop_event.is_set():
            break
        continue

    cv2.imshow("Feed", display_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        stop_event.set()
        break

t_capture.join(timeout=2)
t_process.join(timeout=2)

ser.close()
cv2.destroyAllWindows()