# robot_v1_line_following.py
# Programmer: Ferdo van Balen

# ================== IMPORTS ==================
from machine import Pin, PWM, I2C, ADC, Encoder
from VL53L0X import VL53L0X
import time


# ================== MOTOR DRIVER ==================
class Motor:
    def __init__(self, pin_a, pin_b, freq=1000, reversed=False):
        self.a = PWM(Pin(pin_a), freq=freq)
        self.b = PWM(Pin(pin_b), freq=freq)
        self.reversed = reversed

    def run(self, speed):
        if self.reversed:
            speed = -speed

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


# ================== Motors ==================
left_motor  = Motor(pin_a=18, pin_b=19, reversed=True)
right_motor = Motor(pin_a=17, pin_b=5)

def set_motors(left_speed, right_speed):
    left_motor.run(left_speed)
    right_motor.run(right_speed)

def stop():
    left_motor.stop()
    right_motor.stop()

# ================== IR SENSOR SETUP ==================
ir_sensors = [
    ADC(Pin(13)),  # sensor 1 - far left
    ADC(Pin(14)),  # sensor 2 - left
    ADC(Pin(27)),  # sensor 3 - center
    ADC(Pin(26)),  # sensor 4 - right
    ADC(Pin(25)),  # sensor 5 - far right
]

for s in ir_sensors:
    s.atten(ADC.ATTN_11DB)

# TOF SENSOR SETUP
i2c = I2C(0, sda=Pin(21), scl=Pin(22))
tof = VL53L0X(i2c)
tof.start()

# ================== PID CONTROLLER ==================
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

# ================== IR CALIBRATION ==================
def calibrate_ir(surface_name):
    print("STARTING IR CALIBRATION")
    print(f"Hold sensor above {surface_name} surface!")
    print("Starting in 3 seconds...")
    # time.sleep(3)
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

# ================== IR PROCESSING ==================
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
        return 0 # TEST!!! should return None if no line is being seen (using 0 because test line is to thin to alwas be seen by one sensor)
        #return None

    return total / count

# ================== TOF CALIBRATION ==================
def calibrate_tof(distance):
    print("STARTING ToF CALIBRATION")
    print(f"Place robot {distance}mm infront of a white object")
    print("Calibration starting in 5 seconds...")
    # time.sleep(5)
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

# ================== ENCODER ==================
encoder_left = Encoder(0, Pin(35, Pin.IN), Pin(34, Pin.IN))   # Create second encoder for pins 32, 33 and begin counting
encoder_right = Encoder(1, Pin(32, Pin.IN), Pin(33, Pin.IN))   # Create first encoder for pins 34, 35 and begin counting

# ================== COORDINATE SYSTEM ==================
# pos_x = 0 # millimeters
# pos_y = 0 # millimeters
# pos_yaw = 0 # Degrees

position = [0,0,0] # [x, y, yaw]

# Nodes in millimeter
nodes = {
    "E6":(1485,730), "E5":(1330,730), "E4":(1180,730), "E3":(1030,730), "E2":(745,730), "E1":(0,730),
    "D2":(745 ,515), "D1":(0,515),
    "C3":(1485,365), "C2":(745,365), "C1":(0,365),
    "B2":(1485,215), "B1":(745,215),
    "A6":(1485,0), "A5":(745,0), "A4":(450,0), "A3":(300,0), "A2":(150,0), "A1":(0,0),
}

def turn(action):
    if action == "left":
        set_motors(-45, 45)
    elif action == "right":
        set_motors(45, -45)
    elif action == "reverse":
        set_motors(45, -45)
    else:
        print("unknown action")
    return

def update_position(encoder_left, encoder_right, heading, previous_position):
    average_distance = (encoder_left + encoder_right) / 2
    current_position = list()
    current_pos_y = 0
    current_pos_x = 0
    previous_pos_x = previous_position[0]
    previous_pos_y = previous_position[1]
    previous_yaw = previous_position[2]

    if heading == "north":
        current_pos_y = previous_pos_y + average_distance # adds up on Y
    elif heading == "east":
        current_pos_x = previous_pos_x - average_distance # removes from on x
    elif heading == "south":
        current_pos_y = previous_pos_y - average_distance # removes from on Y
    elif heading == "west":
        current_pos_x = previous_pos_x + average_distance # adds up on  x
    else:
        print("unknown heading")
    current_position = [current_pos_x, current_pos_y, previous_yaw]
    return current_position

# ================== MAIN LOOP (PID LINE FOLLOWING) ==================
while True:

    # ---------- IR ----------
    line = read_line()
    error = get_error(line)

    print("Line:", line)

    # ---------- ENCODER ----------
    encoder_rotations_right = encoder_right.value() / 960 #Calculating rotations of first encoder by dividing the value with the number of times the magnetic wheel has to turn for 1 rotation
    encoder_rotations_left = encoder_left.value() / 960 #Calculating rotations of second encoder by dividing the value with the number of times the magnetic wheel has to turn for 1 rotation

    encoder_distance_right = encoder_rotations_right * 157
    encoder_distance_left = encoder_rotations_left * 157
    print(f"Rotations Rl: {encoder_rotations_left} Rr: {encoder_rotations_right}") #Print value of the first encoders rotations
    print(f"Distance Rl: {encoder_distance_left} Rr: {encoder_distance_right}") #Print value of the first encoders rotations

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

    time.sleep_ms(1000)



