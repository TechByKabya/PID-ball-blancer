import cv2
import serial
import time

# Connect to Arduino
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)

# OpenCV capture
cap = cv2.VideoCapture(0)

def map_val(val, in_min, in_max, out_min, out_max):
    return int((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    # Flip + convert to HSV
    frame = cv2.flip(frame, 1)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Red/Green ball color range
    lower = (29, 86, 6)
    upper = (64, 255, 255)

    mask = cv2.inRange(hsv, lower, upper)
    cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 10:
            centerX = int(x)
            centerY = int(y)

            # Map to 0–180 for servo angles
            servoX = map_val(centerX, 0, 640, 60, 120)
            servoY = map_val(centerY, 0, 480, 60, 120)

            # Send to Arduino
            ser.write(bytes([servoX, servoY]))

            cv2.circle(frame, (centerX, centerY), int(radius), (0, 255, 0), 2)

    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
