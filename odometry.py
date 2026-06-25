from machine import Pin, PWM, I2C, ADC, Encoder
from VL53L0X import VL53L0X
import time
from pathfinder import PathFinder

path_finder = PathFinder()

nodes = {
    "E6":(1485,730), "E5":(1330,730), "E4":(1180,730), "E3":(1030,730), "E2":(745,730), "E1":(0,730),
    "D2":(745 ,515), "D1":(0,515),
    "C3":(1485,365), "C2":(745,365), "C1":(0,365),
    "B2":(1485,215), "B1":(745,215),
    "A6":(1485,0), "A5":(745,0), "A4":(450,0), "A3":(300,0), "A2":(150,0), "A1":(0,0),
}

# ================== Motors ==================
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

position = [0,0,0] # [x, y, yaw]
left_motor  = Motor(pin_a=18, pin_b=19, reversed=True)
right_motor = Motor(pin_a=17, pin_b=5)

encoder_left = Encoder(0, Pin(35, Pin.IN), Pin(34, Pin.IN))  # Create second encoder for pins 32, 33 and begin counting
encoder_right = Encoder(1, Pin(32, Pin.IN), Pin(33, Pin.IN))   # Create first encoder for pins 34, 35 and begin counting

def set_motors(left_speed, right_speed):
    left_motor.run(left_speed)
    right_motor.run(right_speed)

def stop():
    left_motor.stop()
    right_motor.stop()

def read_encoder_distance():
    encoder_rotations_right = encoder_right.value() / 960 #Calculating rotations of first encoder by dividing the value with the number of times the magnetic wheel has to turn for 1 rotation
    encoder_rotations_left = encoder_left.value() / 960 #Calculating rotations of second encoder by dividing the value with the number of times the magnetic wheel has to turn for 1 rotation
    encoder_distance_right = encoder_rotations_right * 157
    encoder_distance_left = encoder_rotations_left * 157
    return encoder_distance_left, encoder_distance_right

def turn(action, previous_yaw):
    encoder_distance_left, encoder_distance_right = read_encoder_distance()
    current_yaw = 0

    if action == "left":
        left_turn_value_l = encoder_distance_left - 120
        left_turn_value_r = encoder_distance_right + 120
        print(f"left_value: {left_turn_value_l} right_value: {left_turn_value_r}")
        print(f"encoder_distance_left: {encoder_distance_left} encoder_distance_right: {encoder_distance_right}")

        current_yaw = previous_yaw - 90

        while encoder_distance_left > left_turn_value_l or encoder_distance_right < left_turn_value_r:
            encoder_distance_left, encoder_distance_right = read_encoder_distance()
            set_motors(-45, 45)
            time.sleep_ms(100)

        stop()

    elif action == "right":
        right_turn_value_l = encoder_distance_left + 120
        right_turn_value_r = encoder_distance_right - 120
        current_yaw = previous_yaw + 90
        while encoder_distance_left < right_turn_value_l or encoder_distance_right > right_turn_value_r:
            encoder_distance_left, encoder_distance_right = read_encoder_distance()
            set_motors(45, -45)
            time.sleep_ms(100)
        stop()

    elif action == "reverse":
        reverse_value_l = encoder_distance_left + 240
        reverse_value_r = encoder_distance_right - 240
        current_yaw = previous_yaw + 180
        while encoder_distance_left < reverse_value_l or encoder_distance_right > reverse_value_r:
            encoder_distance_left, encoder_distance_right = read_encoder_distance()
            set_motors(45, -45)
            time.sleep_ms(100)
        stop()
    else:
        print("unknown action")

    if current_yaw == 360:
        current_yaw = 0
    elif current_yaw > 360:
        current_yaw = 90
    elif current_yaw < 0:
        current_yaw = 270
    return current_yaw


def update_position(encoder_left, encoder_right, heading, previous_position):
    encoder_rotations_right = encoder_right.value() / 960 #Calculating rotations of first encoder by dividing the value with the number of times the magnetic wheel has to turn for 1 rotation
    encoder_rotations_left = encoder_left.value() / 960 #Calculating rotations of second encoder by dividing the value with the number of times the magnetic wheel has to turn for 1 rotation
    encoder_distance_right = encoder_rotations_right * 157
    encoder_distance_left = encoder_rotations_left * 157
    average_distance = (encoder_distance_left + encoder_distance_right) / 2
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
    encoder_left.value(0)
    encoder_right.value(0)
    return current_position

def path_to_node(coord, position, encoder_left, encoder_right):
    pos_x = position[0]
    pos_y = position[1]
    pos_yaw = position[2]
    coord_x = coord[0]
    coord_y = coord[1]

    if coord_x == pos_x and coord_y > pos_y: # NORTH
        if pos_yaw == 90:
            turn("left", pos_yaw)
        elif pos_yaw == 180:
            turn("reverse", pos_yaw)
        elif pos_yaw == 270:
            turn("right", pos_yaw)
        while coord_y > pos_y:
            set_motors(45, 45)
            position = update_position(encoder_left, encoder_right, "north", position)
            pos_y = position[2]
            time.sleep(100)

    elif coord_x == pos_x and coord_y < pos_y: # SOUTH
        if pos_yaw == 0:
            turn("reverse", pos_yaw)
        elif pos_yaw == 90:
            turn("right", pos_yaw)
        elif pos_yaw == 270:
            turn("left", pos_yaw)

    elif coord_y == pos_y and coord_x > pos_x: # WEST
        if pos_yaw == 0:
            turn("left", pos_yaw)
        elif pos_yaw == 90:
            turn("reverse", pos_yaw)
        elif pos_yaw == 180:
            turn("right", pos_yaw)

    elif coord_y == pos_y and coord_x > pos_x: #EAST
        if pos_yaw == 0:
            turn("right", pos_yaw)
        elif pos_yaw == 180:
            turn("left", pos_yaw)
        elif pos_yaw == 270:
            turn("reverse", pos_yaw)
    else:
        print("Error with coords")



while True:
    path = path_finder.astar_path_as_object("A1", "E6", ["C2"])
    for node, coord in path.items():
        # CHECK IF NODES ARE ON THE SAME Y AND SAME X AXIS AND IF THE NODES IN THE MIDDLE CAN BE SKIPPED? OR IF THERE ARE OBSTACLES IN THE WAY
        path_to_node(coord, position, encoder_left, encoder_right)
        # POP NODE FROM OBJECT




# while True:
#     print(f"previous position: {position}")
#     position[2] = turn("left", position[2])
#     print(f"updated position: {position}")
#     time.sleep(5)


    # -------- update position -------
    # print(f"encoder distance: {encoder_distance_left} | {encoder_distance_right}")
    # print(f"previous position: {position}")
    # position = update_position(encoder_distance_left, encoder_distance_right, "west", position)
    # print(f"updated position: {position}")
    # time.sleep_ms(10)
    # encoder_left.value(0)
    # encoder_right.value(0)
    # time.sleep_ms(10)







