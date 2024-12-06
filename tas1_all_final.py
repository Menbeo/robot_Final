import cv2 
import serial 
import numpy as np 
import time
#arduino = serial.Serial("INSERT USB",baudrate=9600, timeout= 1)
max_wheel = 150
def color_detec(frame):
    hsvcolor = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower = np.array([35, 50, 50])  # 7-up bottle's color - suggest from ChatGPT:')
    upper = np.array([85, 255, 255])
    mask = cv2.inRange(hsvcolor,lower,upper)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contours = max(contours, key= cv2.contourArea)
        x,y,w,h = cv2.boundingRect(largest_contours)
        return x,y,w,h
    return None
def calculate_wheel(cx,frame_center):
    error = cx - frame_center
    k_p = 0.5
    v_base = max_wheel // 2 
    correction = int(k_p * error)
    v_left = v_base - correction
    v_right = v_base + correction
    v_left = max(0, min(max_wheel, v_left))
    v_right = max(0, min(max_wheel, v_right))
    return v_left, v_right

cam = cv2.VideoCapture(1)
if not cam.isOpened(): 
        print("check camera port")
        exit()
while True:
    ret, frame = cam.read()
    if not ret: 
        print("Error reading frame, check camera.")
        break

    bottle = color_detec(frame)
    if bottle:
        x, y, w, h = bottle
        cx = x + w // 2
        cy = y + h // 2
        v_left, v_right = calculate_wheel(cx, frame.shape[1] // 2)

        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        print(f"v_left {v_left}, v_right {v_right}")
        # send data to Arduino
        # arduino.write(f"{v_left},{v_right}\n".encode())

    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Allows exiting the loop with 'q'
        break
    time.sleep(0.05)

cv2.destroyAllWindows()