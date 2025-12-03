# Color Sorter Conveyor

A color-based object sorting system built using **Raspberry Pi**, **Python**, **OpenCV**, a **DC motor** (conveyor belt), and a **servo motor** (sorting gate).  
The camera detects the object color (red / green) and the servo redirects it while the conveyor runs at a constant speed.

---

## 🔧 Hardware Overview
- Raspberry Pi  
- USB camera / Pi Camera  
- Servo motor (sorting gate)  
- DC motor (conveyor belt)  
- Motor driver (L298N / L293D / MOSFET)  
- External motor power supply  

---

## 🧠 System Behavior
1. Conveyor (DC motor) runs continuously  
2. Camera captures objects on the belt  
3. OpenCV processes frames using HSV color thresholds  
4. Color is detected (red / green)  
5. Servo moves left or right to sort the object  

---

## 🧾 Full Code

```python
import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# =========================
#  GPIO VE PIN AYARLARI
# =========================

GPIO.setmode(GPIO.BCM)

# --- SERVO MOTOR (AYIRICI KAPI) ---
SERVO_PIN = 18
GPIO.setup(SERVO_PIN, GPIO.OUT)
pwm_servo = GPIO.PWM(SERVO_PIN, 50)  # 50 Hz
pwm_servo.start(0)

# --- DC MOTOR (KONVEYÖR) ---
MOTOR_EN = 23   # PWM pini
MOTOR_IN = 24   # yön / enable pini

GPIO.setup(MOTOR_EN, GPIO.OUT)
GPIO.setup(MOTOR_IN, GPIO.OUT)

pwm_motor = GPIO.PWM(MOTOR_EN, 1000)  # 1 kHz PWM
pwm_motor.start(0)

# =========================
#  SERVO FONKSİYONLARI
# =========================

def set_servo_angle(angle):
    """Servo açısı (derece cinsinden)."""
    duty = 2 + (angle / 18.0)
    GPIO.output(SERVO_PIN, True)
    pwm_servo.ChangeDutyCycle(duty)
    time.sleep(0.2)
    GPIO.output(SERVO_PIN, False)
    pwm_servo.ChangeDutyCycle(0)

ANGLE_LEFT = 45
ANGLE_RIGHT = 135
ANGLE_CENTER = 90

# =========================
#  DC MOTOR FONKSİYONLARI
# =========================

def motor_start(speed_percent=60):
    """Konveyör DC motorunu sabit hızda çalıştır."""
    GPIO.output(MOTOR_IN, GPIO.HIGH)
    pwm_motor.ChangeDutyCycle(speed_percent)

def motor_stop():
    """Konveyörü durdurur."""
    pwm_motor.ChangeDutyCycle(0)
    GPIO.output(MOTOR_IN, GPIO.LOW)

# =========================
#  RENK ALGILAMA AYARLARI
# =========================

cap = cv2.VideoCapture(0)

lower_green = np.array([40, 40, 40])
upper_green = np.array([80, 255, 255])

lower_red1 = np.array([0, 70, 50])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 70, 50])
upper_red2 = np.array([180, 255, 255])

def detect_color(frame):
    """Kırmızı / Yeşil veya None döndürür."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    green_pixels = cv2.countNonZero(mask_green)

    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = mask_red1 + mask_red2
    red_pixels = cv2.countNonZero(mask_red)

    threshold = 1000
    if red_pixels > threshold and red_pixels > green_pixels:
        return "red"
    elif green_pixels > threshold and green_pixels > red_pixels:
        return "green"
    else:
        return None

# =========================
#  ANA DÖNGÜ
# =========================

try:
    set_servo_angle(ANGLE_CENTER)
    time.sleep(0.5)

    motor_start(speed_percent=60)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        color = detect_color(frame)

        if color == "red":
            set_servo_angle(ANGLE_RIGHT)
            time.sleep(0.5)
            set_servo_angle(ANGLE_CENTER)

        elif color == "green":
            set_servo_angle(ANGLE_LEFT)
            time.sleep(0.5)
            set_servo_angle(ANGLE_CENTER)

        cv2.imshow("Color Sorter Conveyor", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    motor_stop()
    cap.release()
    cv2.destroyAllWindows()
    pwm_servo.stop()
    pwm_motor.stop()
    GPIO.cleanup()
