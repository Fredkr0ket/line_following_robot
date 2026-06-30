

nodes = {
    "E6":(1485,730), "E5":(1330,730), "E4":(1180,730), "E3":(1030,730), "E2":(745,730), "E1":(0,730),
    "D2":(745 ,515), "D1":(0,515),
    "C3":(1485,365), "C2":(745,365), "C1":(0,365),
    "B2":(1485,215), "B1":(745,215),
    "A6":(1485,0), "A5":(745,0), "A4":(450,0), "A3":(300,0), "A2":(150,0), "A1":(0,0),
}

path_order = ['A5', 'B1']

heading = "west"
coord = [450, 0]

def obstacle_detection(coord, path_order, heading):
    coord_x = coord[0]
    coord_y = coord[1]
    node1_x, node1_y = nodes[path_order[0]] #check coordinates of node 1
    node2_x, node2_y = nodes[path_order[1]] #check coordinates of node 2
    distance_to_node2 = "no distance"
    print(f'Node1, X:{node1_x} Y:{ node1_y} Node2, X:{node2_x} Y:{ node2_y}') #print locations of node

    if heading == "north": #function to measure distance between coords and upcoming node to the north
        distance_to_node1 = node1_y - coord_y
        if len(path_order) >= 2 and node1_y - node2_y != 0:
            distance_to_node2 = node2_y - coord_y
        print(f'Distance to N1: {distance_to_node1} Distance to N2: {distance_to_node2}')

    elif heading == "east": #function to measure distance between coords and upcoming node to the east
        distance_to_node1 = coord_x - node1_x
        if len(path_order) >= 2 and node1_x - node2_x != 0:
            distance_to_node2 = coord_x - node2_x
        print(f'Distance to N1: {distance_to_node1} Distance to N2: {distance_to_node2}')

    elif heading == "south": #function to measure distance between coords and upcoming node to the south
        distance_to_node1 = coord_y - node1_y
        if len(path_order) >= 2 and node1_y - node2_y != 0:
            distance_to_node2 = coord_y - node2_y
        print(f'Distance to N1: {distance_to_node1} Distance to N2: {distance_to_node2}')

    elif heading == "west": #function to measure distance between coords and upcoming node to the west
        distance_to_node1 = node1_x - coord_x
        if len(path_order) >= 2 and node1_x - node2_x != 0:
            distance_to_node2 = node2_x - coord_x
        print(f'Distance to N1: {distance_to_node1} Distance to N2: {distance_to_node2}') 

    else:
        print("wrong heading for obstacle detection")
    
    

obstacle_detection(coord, path_order, heading)
