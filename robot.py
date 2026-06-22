#Time_of_flight.py | 22/06/2026
#Programmer: Ferdo van Balen
from machine import I2C, Pin, ADC
from VL53L0X import VL53L0X
import time


# ---- Initializing pins ----
ir_sensor_pins = [
    ADC(Pin(13)),  # sensor 1 - far left
    ADC(Pin(12)),  # sensor 2 - left
    ADC(Pin(14)),  # sensor 3 - center
    ADC(Pin(27)),  # sensor 4 - right
    ADC(Pin(26)),  # sensor 5 - far right
]

tof_sda_pin = Pin(21)  # SDA pin
tof_scl_pin = Pin(22)  # SCL pin
# ---------------------------

#Initialize IR sensor
for sensor in ir_sensor_pins:
    sensor.atten(ADC.ATTN_11DB)  # read full 0-3.3V range

#Initialize TOF sensor
ir_i2c = I2C(0, sda=tof_sda_pin, scl=tof_scl_pin)
tof = VL53L0X(ir_i2c)
tof.start()

# ------------------------------------

# ---------- PID CONTROLLER ----------
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

        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

        self.prev_error = error
        return output

pid = PID(kp=20, ki=0, kd=10)
base_speed = 50
# ------------------------------------


# ---------- CALIBRATION ----------
def calibrate_tof_sensor(distance):
    print("STARTING ToF CALIBRATION")
    print(f"Place robot {distance}mm infront of a white object")
    print("Calibration starting in 5 seconds...")
    time.sleep(5)
    print("Taking data from sensor data for 5 seconds")

    total_error = 0
    count = 0

    for _ in range(50):
        error = tof.read() - distance
        total_error += error
        count += 1
        time.sleep_ms(100)

    average_error = total_error // count
    print(f"Average error of tof sensor at a distance of {distance}mm is {average_error}")
    return average_error

def calibrate_infrared_sensor(surface_name):
    print("STARTING IR CALIBRATION")
    print(f"Hold sensor above {surface_name} surface!")
    print("Starting in 3 seconds...")
    time.sleep(3)
    print(f"Collecting {surface_name} data for 5 seconds!")

    total = 0   # sum of ALL readings from ALL sensors
    count = 0   # total number of readings taken

    # 50 iterations x 100ms = 5 seconds
    for _ in range(50):
        for sensor in ir_sensor_pins:
            total += sensor.read()  # add each sensor reading to total
            count += 1              # count every single reading
        time.sleep_ms(100)

    # Calculate one overall average across all sensors
    average = total // count
    print(f"{surface_name} average analog sensor data: {average}")
    return average

# ------------------------------------


# ---------- IR SENSOR ----------
white_average = calibrate_infrared_sensor("WHITE") # calibrate white
black_average = calibrate_infrared_sensor("BLACK") # calibrate black

# Calculate threshold value
difference = black_average - white_average  # gap between black and white
half       = difference // 2                # half of the gap
threshold  = white_average + half           # center point between black and white

print("White average: ", white_average)
print("Black average: ", black_average)
print("Threshold:     ", threshold)
print("=== CALIBRATION DONE ===")

# HELPER FUNCTION
# Reads all sensors and returns 0 for white and 1 for black
def data_parser():
    results = []
    for sensor in ir_sensor_pins:
        reading = sensor.read()
        if reading < threshold:
            results.append(1)
        else:
            results.append(0)
    return results

# Convert IR data into line error for PID
weights = [-2, -1, 0, 1, 2]

def get_line_error(line):
    total = 0
    count = 0

    for i in range(5):
        if line[i] == 1:
            total += weights[i]
            count += 1

    if count == 0:
        return None  # line lost

    return total / count

def get_ir_sensor_data():
    line = data_parser()
    print("Line: ", line)
    return line
# ------------------------------------


# ---------- TOF SENSOR ----------
# Calibrate procedure
total_error_100 = calibrate_tof_sensor(100) # calibrate 100mm
total_error_300 = calibrate_tof_sensor(300) # calibrate 300mm
total_error = (total_error_100 + total_error_300) // 2 # Calculate total error

print(f"The final error of the distance readings is {total_error}mm")

def get_tof_sensor_data():
    distance = tof.read()  # returns distance in mm
    corrected_distance = distance - total_error # calculate the corrected distance 
    print("Distance:", corrected_distance, "mm")
    return corrected_distance
# ----------------------------


# ---------- MAIN LOOP ----------
while True:

    # IR SENSOR
    line = get_ir_sensor_data()
    error = get_line_error(line)

    if error is not None:

        correction = pid.update(error)

        left_motor  = base_speed - correction
        right_motor = base_speed + correction

        print("Error:", error, "Correction:", correction)

    else:
        print("Line lost")
        ## move backwards

    # TOF SENSOR
    get_tof_sensor_data()

    time.sleep_ms(1000)