from machine import Pin, Encoder
import time

encoder2 = Encoder(0, Pin(34, Pin.IN), Pin(35, Pin.IN))   # Create second encoder for pins 32, 33 and begin counting
encoder1 = Encoder(1, Pin(32, Pin.IN), Pin(33, Pin.IN))   # Create first encoder for pins 34, 35 and begin counting


while True:
    rotations1 = encoder1.value() / 960 #Calculating rotations of first encoder by dividing the value with the number of times the magnetic wheel has to turn for 1 rotation
    rotations2 = encoder2.value() / 960 #Calculating rotations of second encoder by dividing the value with the number of times the magnetic wheel has to turn for 1 rotation
    print(f"Rl: {rotations2} Rr: {rotations1}") #Print value of the first encoders rotations
    time.sleep(1) #Add delay 
    
