#infrared_sensor_array.py | 14/06/2026
#Programmer: Ferdo van Balen
from machine import Pin, ADC
import time

# Initialize analog pins of the infrared sensor array
ir_sensor_pins = [
    ADC(Pin(13)),  # sensor 1 - far left
    ADC(Pin(14)),  # sensor 2 - left
    ADC(Pin(27)),  # sensor 3 - center
    ADC(Pin(26)),  # sensor 4 - right
    ADC(Pin(25)),  # sensor 5 - far right
]

for sensor in sensor_pins:
    sensor.atten(ADC.ATTN_11DB)  # read full 0-3.3V range

# CALIBRATION FUNCTION
# Collects 50 readings per sensor (5 seconds at 100ms each)
# and returns the overall average across all 5 sensors
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


# Calibration sequence
# Calibrate white and black surface
white_average = calibrate_infrared_sensor("WHITE")
black_average = calibrate_infrared_sensor("BLACK")

# Calculate threshold value
difference = black_average - white_average  # gap between black and white
half       = difference // 2                # half of the gap
threshold  = white_average + half           # center point between black and white

print("White average: ", white_average)
print("Black average: ", black_average)
print("Threshold:     ", threshold)
print("=== CALIBRATION DONE ===")

# HELPER FUNCTIONsens
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

# Mail Loop
while True:
    line = data_parser()
    print("Line: ", line)
    time.sleep_ms(100)