#Encoder code | 14/06/2026
#Programmer: Ferdo van Balen
from machine import ADC, Pin
import time

ir_sensor_pins = [
    ADC(Pin(13)),  # sensor 1 - far left
    ADC(Pin(14)),  # sensor 2 - left
    ADC(Pin(27)),  # sensor 3 - center
    ADC(Pin(26)),  # sensor 4 - right
    ADC(Pin(25)),  # sensor 5 - far right
]
# Set attenuation to read full 0-3.3V range
for sensor in sensor_pins:
    sensor.atten(ADC.ATTN_11DB)

# Read all 5 sensors and print values
while True:
    readings = [sensor.read() for sensor in sensor_pins]
    print(readings)
    time.sleep_ms(500)
    

