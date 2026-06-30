import machine
from machine import Pin, PWM, I2C, ADC, Encoder
from VL53L0X import VL53L0X as TOF
import time, neopixel
from pathfinder import PathFinder

# ================== INITIALIZATION ==================

path_finder = PathFinder()

np = neopixel.NeoPixel(machine.Pin(2), 1) # initialising the rgb

states = ["path_following", "box_pickup", "interrupt", "obstacle_detected"]
state = states[0]
box_order_start = ["E6", "E5", "E4", "A1"]  # destinations of boxes #starting order from left to right
box_destination = ["A4", "A3", "A2", "A1"]  # destinations of boxes #destinations of boxes black, red, green, blue
slow_down_distance = 40
base_speed_robot = 40
position = [745,365,0] # [x, y, yaw]
rotation_90 = 100
rotation_180 = 300
blocked_nodes = []
starting_node = "C2"
ending_node = "E6"
# ending_node = box_destination["black"]
kp = 14
ki = 0
kd = 16
line_value = 1750
previous_node = ""
coord_tolerance = 50

path = path_finder.astar_path_as_object(starting_node, ending_node, blocked_nodes)
path_order = path_finder.astar(starting_node, ending_node,blocked_nodes)
path.pop(path_order[0])
path_order.pop(0)
print(f"Path: {path} | Path_order: {path_order}")

# NODES MAP
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
# Interrupt initilization
# interrupt_triggered = False
# MAGNET PINS
magnet_pin = Pin(16, Pin.OUT)
# Define I2C pins
sda_pin = Pin(21)  # SDA pin
scl_pin = Pin(22)  # SCL pin

# Setup I2C bus
i2c = I2C(0, sda=sda_pin, scl=scl_pin)

print(i2c)
print(i2c.scan())

# Setup TOF sensor
tof = TOF(i2c)

# Start measuring
tof.start()

# ================== Interupt ==================
# def irq_handler(pin):
#     global button_pressed
#     button_pressed = True
# button_pressed = False
# def handle_button():
#     time.sleep_ms(10)

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
        line.append(1 if s.read() < line_value else 0)
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


pid = PID(kp=kp, ki=ki, kd=kd)


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


left_motor  = Motor(pin_a=18, pin_b=19)
right_motor = Motor(pin_a=17, pin_b=5, reversed=True)

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
    encoder_distance_right = encoder_rotations_right * 220
    encoder_distance_left = encoder_rotations_left * 220
    return encoder_distance_left, encoder_distance_right

# ================== MOVEMENT ==================
def outer_sensor_trigger(action):
    line = read_line()

    # LEFT TURN → use far left sensor
    if action == "left":
        return line[0] == 1 and line[1] == 1

    # RIGHT TURN → use far right sensor
    elif action == "right":
        return line[4] == 1 and line[3] == 1

    # REVERSE (180°) → use both outer sensors MAYBE REMOVE THISDUE TO A FULL JUNCTION THEN STOPPING ONLYU AT 90 DEGREES
    elif action == "reverse":
        return line[0] == 1 or line[4] == 1

    return False

def turn(action, previous_yaw, encoder_distance_90=rotation_90, encoder_distance_180=rotation_180):
    encoder_distance_left, encoder_distance_right = read_encoder_distance()
    current_yaw = 0

    if action == "left":
        print("TURNING LEFT")
        left_turn_value_l = encoder_distance_left - encoder_distance_90
        left_turn_value_r = encoder_distance_right + encoder_distance_90

        current_yaw = previous_yaw - 90

        while encoder_distance_left > left_turn_value_l or encoder_distance_right < left_turn_value_r:
            encoder_distance_left, encoder_distance_right = read_encoder_distance()
            set_motors(-40, 40)
            time.sleep_ms(20)
            if outer_sensor_trigger("left"):
                print("LEFT TURN STOPPED BY IR")
                break

        stop()
        print("LEFT TURN FINISHED")

    elif action == "right":
        print("TURNING RIGHT")
        right_turn_value_l = encoder_distance_left + encoder_distance_90
        right_turn_value_r = encoder_distance_right - encoder_distance_90
        current_yaw = previous_yaw + 90
        while encoder_distance_left < right_turn_value_l or encoder_distance_right > right_turn_value_r:
            encoder_distance_left, encoder_distance_right = read_encoder_distance()
            set_motors(40, -40)
            time.sleep_ms(20)
            if outer_sensor_trigger("right"):
                break
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
            time.sleep_ms(20)
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
    encoder_distance_right = encoder_rotations_right * 220
    encoder_distance_left = encoder_rotations_left * 220
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

def move_to_node(coord, position, encoder_left, encoder_right, base_speed):
    pos_x = position[0]
    pos_y = position[1]
    pos_yaw = position[2]
    coord_x = coord[0]
    coord_y = coord[1]
    updated_position_arr = []
    updated_yaw = None

    if coord_x == pos_x and coord_y > pos_y:  # NORTH
        if pos_yaw == 90:
            updated_yaw = turn("left", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 180:
            updated_yaw = turn("reverse", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 270:
            updated_yaw = turn("right", pos_yaw, rotation_90, rotation_180)
        junction = False
        while pos_y < coord_y - coord_tolerance or not junction:
            line_follow(base_speed)
            position = update_position(encoder_left, encoder_right, "north", position)
            pos_y = position[1]
            junction = junction_detection()
            time.sleep_ms(10)

        time.sleep_ms(250)

        stop()

    elif coord_x == pos_x and coord_y < pos_y:  # SOUTH
        if pos_yaw == 0:
            updated_yaw = turn("reverse", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 90:
            updated_yaw = turn("right", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 270:
            updated_yaw = turn("left", pos_yaw, rotation_90, rotation_180)
        junction = False
        while pos_y > coord_y + coord_tolerance or not junction:
            line_follow(base_speed)
            position = update_position(encoder_left, encoder_right, "south", position)
            pos_y = position[1]
            junction = junction_detection()
            time.sleep_ms(10)
        time.sleep_ms(250)
        stop()

    elif coord_y == pos_y and coord_x > pos_x:  # WEST (+X)
        if pos_yaw == 0:
            updated_yaw = turn("left", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 90:
            updated_yaw = turn("reverse", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 180:
            updated_yaw = turn("right", pos_yaw, rotation_90, rotation_180)
        junction = False
        while pos_x < coord_x - coord_tolerance or not junction:
            line_follow(base_speed)
            position = update_position(encoder_left, encoder_right, "west", position)
            pos_x = position[0]
            junction = junction_detection()
            time.sleep_ms(10)
        time.sleep_ms(250)
        print(f"positie x: {pos_x}")
        stop()

    elif coord_y == pos_y and coord_x < pos_x:  # EAST (-X)
        print("going EAST")
        print(f"positie x: {pos_x}")
        if pos_yaw == 0:
            updated_yaw = turn("right", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 180:
            updated_yaw = turn("left", pos_yaw, rotation_90, rotation_180)
        elif pos_yaw == 270:
            updated_yaw = turn("reverse", pos_yaw, rotation_90, rotation_180)
        junction = False
        while pos_x > coord_x + coord_tolerance or not junction:
            line_follow(base_speed)
            position = update_position(encoder_left, encoder_right, "east", position)
            pos_x = position[0]
            junction = junction_detection()
            time.sleep_ms(10)
        time.sleep_ms(250)
        print(f"positie x: {pos_x}")
        stop()

    else:
        print("Error with coords")

    updated_position_arr = [
        coord_x,
        coord_y,
        pos_yaw if updated_yaw is None else updated_yaw
    ]

    return updated_position_arr

def box_detection():
    while tof.read() > 70:
        line_follow(base_speed_robot)
        time.sleep_ms(10)
    stop()
    magnet_pin.value(1)
    print("TURN MAGNET ON")
    time.sleep(5)

def obstacle_detection(coord, path_order, position, distance_tof):
    coord_x = coord[0]
    coord_y = coord[1]
    node1_x, node1_y = nodes[path_order[0]] #check coordinates of node 1
    node2_x, node2_y = nodes[path_order[1]] #check coordinates of node 2
    distance_to_node2 = None

    yaw = position[2]

    if yaw == 0:
        heading = "north"
    elif yaw == 90:
        heading = "east"
    elif yaw == 180:
        heading = "south"
    elif yaw == 270:
        heading = "west"
    else:
        print("unknown yaw in obstacle detection")

    if heading == "north": #function to measure distance between coords and upcoming node to the north
        distance_to_node1 = node1_y - coord_y
        if len(path_order) >= 2 and node1_y - node2_y != 0:
            distance_to_node2 = node2_y - coord_y
    elif heading == "east": #function to measure distance between coords and upcoming node to the east
        distance_to_node1 = coord_x - node1_x
        if len(path_order) >= 2 and node1_x - node2_x != 0:
            distance_to_node2 = coord_x - node2_x
    elif heading == "south": #function to measure distance between coords and upcoming node to the south
        distance_to_node1 = coord_y - node1_y
        if len(path_order) >= 2 and node1_y - node2_y != 0:
            distance_to_node2 = coord_y - node2_y
    elif heading == "west": #function to measure distance between coords and upcoming node to the west
        distance_to_node1 = node1_x - coord_x
        if len(path_order) >= 2 and node1_x - node2_x != 0:
            distance_to_node2 = node2_x - coord_x
    else: #debug function if heading is wrong
        print("wrong heading for obstacle detection")

    if  distance_to_node2 != None and  distance_tof> (distance_to_node1 * 1.1): #check if distance to node 2 exists and if the obstacle is between node1 and node2 with 10% marge
        blocked_nodes.append(path_order[1]) #add the 2nd node from array to blocked nodes
    elif  distance_tof <= (distance_to_node1 * 1.1):
        blocked_nodes.append(path_order[0]) #add the 1st node from array to blocked nodes
    else:
        print("error with obstacle detection")
    return blocked_nodes


while True:
    print(f"State: {state}")
    #=================See==============#
    distance_tof = tof.read()
    line = read_line()
    print(line)
    junction = junction_detection()
    encoder_value_left, encoder_value_right = read_encoder_distance()
    # pressed_now = button_pressed

    #================Think=============#
    # if interrupt_triggered:
    #     if pressed_now:
    #         state = states[2]
    #         button_pressed = False
    if len(path_order) == 0: #box_pickup, if the array is empty switch to box pickup state
        state = states[1]
    elif tof.read() < 10 and state != states[1]: #obstacle detection, inactive when box_pickup is active
        state = states[3]

    #=================Act==============#
    if state == "path_following":
        np[0] = (0, 255, 0)
        np.write()
        print("State: PATH_FOLLOWING")

        node = path_order[0]
        coord = path[node]

        print(f"Target node: {node}")
        print(f"Current position: {position}")

        # move_to_node should return only when the node is reached
        position = move_to_node(
            coord,
            position,
            encoder_left,
            encoder_right,
            base_speed_robot
        )

        # Remove completed node
        previous_node = node
        path.pop(node)
        path_order.pop(0)

    elif state == "box_pickup":
        np[0] = (0, 0, 255)
        np.write()
        print("State: BOX_PICKUP")
        if previous_node[0] == "A":
            box_destination.pop(0)
            path_order = path_finder.astar(previous_node, box_order_start[0], blocked_nodes)
            if position[2] == 90:
                position[2] = turn("right", position[2])
                box_detection()
            elif position == 0:
                position[2] = turn("reverse", position[2])
                box_detection()
            elif position == 270:
                position[2] = turn("left", position[2])
                box_detection()
            elif position[2] == 180:
                box_detection()
            if distance_tof <= 30:
                magnet_pin.value(0)

        elif previous_node[0] == "E":
            box_order_start.pop(0)
            path_order = path_finder.astar(previous_node, box_destination[0], blocked_nodes)

            if position[2] == 90:
                position[2] = turn("left", position[2])
                box_detection()
            elif position[2] == 180:
                position[2] = turn("reverse", position[2])
                box_detection()
            elif position[2] == 270:
                position[2] = turn("right", position[2])
                box_detection()
            elif position[2] == 0:
                box_detection()
        set_motors(-40, -40)
        time.sleep_ms(2000)
        stop()
        turn("reverse", position[2])
        while junction_detection() == False:
            set_motors(base_speed_robot, base_speed_robot)
        path_order.pop(0)
        state = states[0]
        
        time.sleep(10)
    
    elif state == "recalibrate_path":
        stop()
        continue

    # elif state == "interrupt":
    #     np[0] = (255, 0, 255)
    #     handle_button()
        
    elif state == "obstacle_detected": #obstacle detection
        print("State: OBSTACLE_DETECTION")
        np[0] = (255, 0, 0)
        np.write()
        #obstacle_detection(coord, path_order, position, distance_tof)
        path_order = path_finder.astar(previous_node, path_order[-1], blocked_nodes)
        state = states[0]
    
    else:
        print("no valid state")






