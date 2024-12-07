import cv2 
import serial 
import numpy as np 
import time
base_speed = 0.2 #m/s
#arduino = serial.Serial("INSERT USB",baudrate=9600, timeout= 1)
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
    
def send_wheel(left_speed,right_speed) 
command = f"{left_speed} {right_speed}\n"
arduino.write(command.encode('utf -8')) 
except serial.SerialException as e: 
print(f"Error")

def control_robot(frame_center, cx)
error = cx - frame_center 
intergral += error 
previous_error = error 
derivative = error - previous_error 
correction = Kp*error + Ki * intergral + Kd * derivative 
base_speed = 0.2
left_speed = max(-1.0, min(1.0, base_speed - correction))
right_speed = max(-1.0, min(1.0, base_speed - correction))
send_wheel(left_speed,right_speed)
else:
send_wheel(np.random(left_speed, right_speed)
#main code 
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
        #send data to Arduino
        arduino.write(f"{v_left},{v_right}\n".encode())
    else:
        arduino.write("no bottle connect\n".encode())
    cv2.imshow("frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # Allows exiting the loop with 'q'
        break
    time.sleep(0.05)
cam.release()
cv2.destroyAllWindows()
arduino.close()
