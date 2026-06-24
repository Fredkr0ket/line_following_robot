#Time_of_flight.py | 14/06/2026
#Programmer: Ferdo van Balen
from machine import I2C, Pin
from VL53L0X import VL53L0X
import time

# Define I2C pins
sda_pin = Pin(21)  # SDA pin
scl_pin = Pin(22)  # SCL pin

# Setup I2C bus
i2c = I2C(0, sda=sda_pin, scl=scl_pin)

print(i2c)
print(i2c.scan())

# Setup TOF sensor
tof = VL53L0X(i2c)

# Start measuring
tof.start()

# Calibrate function
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


# Calibrate procedure
# calibrate 50 values from 100mm distance and 50 values of 300mm distance and get the average error
total_error_100 = calibrate_tof_sensor(100)
total_error_300 = calibrate_tof_sensor(300)

total_error = (total_error_100 + total_error_300) // 2
print(f"The final error of the distance readings is {total_error}mm")

    
while True:
    distance = tof.read()  # returns distance in mm
    corrected_distance = distance - total_error # calculate the corrected distance 
    print("Distance:", corrected_distance, "mm") 
    time.sleep_ms(1000)
    