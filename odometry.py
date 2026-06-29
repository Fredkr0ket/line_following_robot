import machine
from machine import Pin, PWM, I2C, ADC, Encoder
from VL53L0X import VL53L0X
import time, neopixel
from pathfinder import PathFinder

path_finder = PathFinder()

np = neopixel.NeoPixel(machine.Pin(2), 1) # initialising the rgb

nodes = {
    "E6":(1485,730), "E5":(1330,730), "E4":(1180,730), "E3":(1030,730), "E2":(745,730), "E1":(0,730),
    "D2":(745 ,515), "D1":(0,515),
    "C3":(1485,365), "C2":(745,365), "C1":(0,365),
    "B2":(1485,215), "B1":(745,215),
    "A6":(1485,0), "A5":(745,0), "A4":(450,0), "A3":(300,0), "A2":(150,0), "A1":(0,0),
}

ir_sensors = [
    ADC(Pin(13)),  # sensor 1 - far left
    ADC(Pin(14)),  # sensor 2 - left
    ADC(Pin(27)),  # sensor 3 - center
    ADC(Pin(26)),  # sensor 4 - right
    ADC(Pin(25)),  # sensor 5 - far right
]

slow_down_distance = 40
base_speed_robot = 50
position = [450,0,270] # [x, y, yaw]
rotation_90 = 100
rotation_180 = 200

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

# ================== PID CONTROLLER ==================
weights = [2, 1, 0, -1, -2]

def read_line():
    line = []
    for s in ir_sensors:
        line.append(1 if s.read() < 2000 else 0)
    return line

def junction_detection():
    line = read_line()
    direction = False
    if line == [0,0,1,1,1]:
        direction = True
    elif line == [1,1,1,0,0]:
        direction = True
    elif line == [0,1,1,1,0]:
        direction = True
    elif line == [0,1,1,1,1]:
        direction = True
    elif line == [1,1,1,1,0]:
        direction = True
    return direction


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


# ================== LINE FOLLOWING ==================
def line_follow(base_speed):
    line = read_line()
    error = get_error(line)

    if error is None:
        stop()
        return False

    correction = pid.update(error)

    # Scale correction to motor speed
    correction *= base_speed / 45

    left_speed = base_speed - correction
    right_speed = base_speed + correction

    # Clamp motor speeds
    left_speed = max(0, min(100, left_speed))
    right_speed = max(0, min(100, right_speed))

    set_motors(left_speed, right_speed)

    return True


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
    encoder_distance_right = encoder_rotations_right * 200
    encoder_distance_left = encoder_rotations_left * 200
    return encoder_distance_left, encoder_distance_right

def turn(action, previous_yaw, encoder_distance_90, encoder_distance_180):
    encoder_distance_left, encoder_distance_right = read_encoder_distance()
    current_yaw = 0

    if action == "left":
        print("TURNING LEFT")
        left_turn_value_l = encoder_distance_left - encoder_distance_90
        left_turn_value_r = encoder_distance_right + encoder_distance_90

        current_yaw = previous_yaw - 90

        while encoder_distance_left > left_turn_value_l or encoder_distance_right < left_turn_value_r:
            encoder_distance_left, encoder_distance_right = read_encoder_distance()
            set_motors(-45, 45)
            time.sleep_ms(100)

        stop()
        print("LEFT TURN FINISHED")

    elif action == "right":
        print("TURNING RIGHT")
        right_turn_value_l = encoder_distance_left + encoder_distance_90
        right_turn_value_r = encoder_distance_right - encoder_distance_90
        current_yaw = previous_yaw + 90
        while encoder_distance_left < right_turn_value_l or encoder_distance_right > right_turn_value_r:
            encoder_distance_left, encoder_distance_right = read_encoder_distance()
            set_motors(45, -45)
            time.sleep_ms(100)
        stop()
        print("RIGHT TURN FINISHED")

    elif action == "reverse":
        print("TURNING 180")
        reverse_value_l = encoder_distance_left + encoder_distance_180
        reverse_value_r = encoder_distance_right - encoder_distance_180
        current_yaw = previous_yaw + 180
        while encoder_distance_left < reverse_value_l or encoder_distance_right > reverse_value_r:
            encoder_distance_left, encoder_distance_right = read_encoder_distance()
            set_motors(45, -45)
            time.sleep_ms(100)
        stop()
    else:
        print("unknown action")
        print("180 TURN FINISHED")

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

def path_to_node(coord, position, encoder_left, encoder_right, base_speed):
    pos_x = position[0]
    pos_y = position[1]
    pos_yaw = position[2]
    coord_x = coord[0]
    coord_y = coord[1]
    updated_position_arr = []
    updated_yaw = None

    if coord_x == pos_x and coord_y > pos_y: # NORTH
        print("going NORTH")
        print(pos_yaw)
        print(f"positie y: {pos_y}")
        np[0] = (255, 0, 0)
        np.write()
        if pos_yaw == 90:
            updated_yaw = turn("left", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 180:
            updated_yaw = turn("reverse", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 270:
            updated_yaw = turn("right", pos_yaw, rotation_90, rotation_180)
        while coord_y > pos_y or junction == False:
            line_follow(base_speed)
            position = update_position(encoder_left, encoder_right, "north", position)
            pos_y = position[1]
            junction = junction_detection()
            if (coord_y - pos_y) < slow_down_distance:
                base_speed = 40
            time.sleep_ms(50)
        np[0] = (153, 0, 0)
        np.write()
        set_motors(45, 45)
        time.sleep_ms(500)
        print(f"positie y: {pos_y}")
        stop()
        

    elif coord_x == pos_x and coord_y < pos_y: # SOUTH
        print("goint SOUTH")
        print(f"positie y: {pos_y}")
        np[0] = (0, 255, 0)
        np.write()
        if pos_yaw == 0:
            updated_yaw = turn("reverse", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 90:
            updated_yaw = turn("right", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 270:
            updated_yaw = turn("left", pos_yaw, rotation_90, rotation_180)
        while coord_y < pos_y or junction == False:
            line_follow(base_speed)
            position = update_position(encoder_left, encoder_right, "south", position)
            pos_y = position[1]
            junction = junction_detection()
            if (pos_y - coord_y) < slow_down_distance:
                base_speed = 40
            time.sleep_ms(50)
        np[0] = (0, 153, 0)
        set_motors(45, 45)
        time.sleep_ms(500)
        print(f"positie y: {pos_y}")
        np.write()

        stop()

    elif coord_y == pos_y and coord_x > pos_x: # WEST
        print("going WEST")
        np[0] = (0, 0, 255)
        print(f"positie x: {pos_x}")
        np.write()
        if pos_yaw == 0:
            updated_yaw = turn("left", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 90:
            updated_yaw = turn("reverse", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 180:
            updated_yaw = turn("right", pos_yaw, rotation_90, rotation_180)
        while coord_x > pos_x or junction == False:
            line_follow(base_speed)
            position = update_position(encoder_left, encoder_right, "west", position)
            pos_x = position[0]
            junction = junction_detection()
            
            if (coord_x - pos_x) < slow_down_distance:
                base_speed = 50
            time.sleep_ms(50)
        np[0] = (0, 0, 153)
        np.write()
        set_motors(45, 45)
        time.sleep_ms(500)
        print(f"positie x: {pos_x}")
        stop()

    elif coord_y == pos_y and coord_x < pos_x: #EAST
        print("going EAST")
        print(f"positie x: {pos_x}")
        np[0] = (255, 0, 255)
        np.write()
        if pos_yaw == 0:
            updated_yaw = turn("right", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 180:
            updated_yaw = turn("left", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 270:
            updated_yaw = turn("reverse", pos_yaw, rotation_90, rotation_180)
        while coord_x < pos_x or junction == False:
            line_follow(base_speed)
            position = update_position(encoder_left, encoder_right, "east", position)
            pos_x = position[0]
            junction = junction_detection()
            if (pos_x - coord_x) < slow_down_distance:
                base_speed = 50
            time.sleep_ms(50)
        np[0] = (153, 0, 153)
        np.write()
        set_motors(45, 45)
        time.sleep_ms(500)
        print(f"positie x: {pos_x}")
        stop()
        
    else:
        print("Error with coords")
    
    updated_position_arr = [coord[0], coord[1], pos_yaw if updated_yaw is None else updated_yaw]

    return updated_position_arr

path = path_finder.astar_path_as_object("A4", "C3", ["A6", "B2"])
path_order = path_finder.astar("A4", "C3",["A6", "B2"])
path.pop(path_order[0])
path_order.pop(0)
print(path["B1"])
print(f"Path: {path} | Path_order: {path_order}")
while True:
    for node in path_order:
        print(f"node: {node}")
        coord = path[node]
        print(f"coords to go to: {coord}")
        print(f"current position: {position}")
        position = path_to_node(coord, position, encoder_left, encoder_right, base_speed_robot)
        path.pop(node)

