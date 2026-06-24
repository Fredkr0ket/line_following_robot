# line_follower_robot.py
# Programmer: Ferdo van Balen

from machine import Pin, PWM, I2C, ADC
from VL53L0X import VL53L0X
import time

# ============================================================
# MOTOR DRIVER
# ============================================================
class Motor:
    def __init__(self, pin_a, pin_b, freq=1000, reversed=False):
        self.a = PWM(Pin(pin_a), freq=freq)
        self.b = PWM(Pin(pin_b), freq=freq)
        self.reversed = reversed

    def run(self, speed):
        # allow PID negative values
        if self.reversed:
            speed = -speed

        # convert -100..100 → PWM duty
        speed = max(-100, min(100, speed))
        duty = int(abs(speed) * 10.23)

        if speed > 0:
            self.a.duty(duty)
            self.b.duty(0)

        elif speed < 0:
            self.a.duty(0)
            self.b.duty(duty)

        else:
            self.a.duty(0)
            self.b.duty(0)

    def stop(self):
        self.a.duty(0)
        self.b.duty(0)

    def brake(self):
        self.a.duty(1023)
        self.b.duty(1023)


# Motors
left_motor  = Motor(pin_a=18, pin_b=19, reversed=True)
right_motor = Motor(pin_a=17, pin_b=5)

def set_motors(left_speed, right_speed):
    left_motor.run(left_speed)
    right_motor.run(right_speed)

def stop():
    left_motor.stop()
    right_motor.stop()

# ============================================================
# IR SENSOR SETUP
# ============================================================
ir_sensors = [
    ADC(Pin(13)),  # sensor 1 - far left
    ADC(Pin(14)),  # sensor 2 - left
    ADC(Pin(27)),  # sensor 3 - center
    ADC(Pin(26)),  # sensor 4 - right
    ADC(Pin(25)),  # sensor 5 - far right
]

for s in ir_sensors:
    s.atten(ADC.ATTN_11DB)

# ============================================================
# TOF SENSOR SETUP
# ============================================================
i2c = I2C(0, sda=Pin(21), scl=Pin(22))
tof = VL53L0X(i2c)
tof.start()

# ============================================================
# PID CONTROLLER
# ============================================================
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error):
        self.integral += error
        derivative = error - self.prev_error

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error
        return output


pid = PID(kp=10, ki=0, kd=18)
base_speed = 100

# ============================================================
# IR CALIBRATION
# ============================================================
def calibrate_ir(surface_name):
    print("STARTING IR CALIBRATION")
    print(f"Hold sensor above {surface_name} surface!")
    print("Starting in 3 seconds...")
    #time.sleep(3)
    print(f"Collecting {surface_name} data for 5 seconds!")

    total = 0
    count = 0

    for _ in range(50):
        for s in ir_sensors:
            total += s.read()
            count += 1
        time.sleep_ms(100)

    avg = total // count
    print(surface_name, "average:", avg)
    return avg


white = calibrate_ir("WHITE")
black = calibrate_ir("BLACK")

threshold = white + (black - white) // 2
print("Threshold:", threshold)

# ============================================================
# IR PROCESSING
# ============================================================
weights = [2, 1, 0, -1, -2]

def read_line():
    line = []
    for s in ir_sensors:
        print(s.read())
        line.append(1 if s.read() < 3000 else 0)
    return line


def get_error(line):
    total = 0
    count = 0

    for i in range(5):
        if line[i] == 1:
            total += weights[i]
            count += 1

    if count == 0:
        return 0
        #return None

    return total / count

# ============================================================
# TOF CALIBRATION
# ============================================================
def calibrate_tof(distance):
    print("STARTING ToF CALIBRATION")
    print(f"Place robot {distance}mm infront of a white object")
    print("Calibration starting in 5 seconds...")
    #time.sleep(5)
    print("Taking data from sensor data for 5 seconds")
    error_sum = 0

    for _ in range(50):
        error_sum += tof.read() - distance
        time.sleep_ms(100)

    return error_sum // 50


err_100 = calibrate_tof(100)
err_300 = calibrate_tof(300)
tof_error = (err_100 + err_300) // 2

def get_distance():
    return tof.read() - tof_error

# ============================================================
# MAIN LOOP (PID LINE FOLLOWING)
# ============================================================
while True:

    # ---------- IR ----------
    line = read_line()
    error = get_error(line)

    print("Line:", line)

    if error is not None:

        correction = pid.update(error)
        
        print(f"Correction normal: {correction}")
        
        speed_factor = base_speed / 45
        print(f"Speed factor: {speed_factor}")
        correction *= speed_factor
        print(f"Corrected correction: {correction}")
        
        

        left_speed = base_speed - correction
        right_speed = base_speed + correction

        # clamp speeds
        left_speed = max(0, min(100, left_speed))
        right_speed = max(0, min(100, right_speed))

        set_motors(left_speed, right_speed)

        print("Error:", error,
              "Correction:", correction,
              "L:", left_speed,
              "R:", right_speed)

    else:
        stop()
        print("Line lost")

    # ---------- TOF ----------
    distance = get_distance()
    print("Distance:", distance, "mm")

    time.sleep_ms(100)

