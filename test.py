

nodes = {
    "E6":(1485,730), "E5":(1330,730), "E4":(1180,730), "E3":(1030,730), "E2":(745,730), "E1":(0,730),
    "D2":(745 ,515), "D1":(0,515),
    "C3":(1485,365), "C2":(745,365), "C1":(0,365),
    "B2":(1485,215), "B1":(745,215),
    "A6":(1485,0), "A5":(745,0), "A4":(450,0), "A3":(300,0), "A2":(150,0), "A1":(0,0),
}

path_order = ['B1', 'C2']

heading = "south"
coord = [1485, 215]

def obstacle_detection(coord, path_order, heading):
    coord_x = coord[0]
    coord_y = coord[1]
    node1_x, node1_y = nodes[path_order[0]]
    node2_x, node2_y = nodes[path_order[1]]
    print(f'Node1, X:{node1_x} Y:{ node1_y} Node2, X:{node2_x} Y:{ node2_y}')
    if heading == "north":
        distance_to_node1 = node1_y - coord_y
        if len(path_order) >= 2:
            distance_to_node2 = node2_y - coord_y
        print(distance_to_node1, distance_to_node2)
    elif heading == "east":
        distance_to_node1 = coord_x - node1_x
        if len(path_order) >= 2:
            distance_to_node2 = coord_x - node2_x
        print(distance_to_node1)
    elif heading == "south":
        distance_to_node1 = coord_y - node1_y
        if len(path_order) >= 2:
            distance_to_node2 = coord_x - node2_x
        print(distance_to_node1)
    elif heading == "west":
        distance_to_node1 = coord_y - node1_y
        print(distance_to_node1) 
    else:
        print("wrong heading for obstacle detection")

obstacle_detection(coord, path_order, heading)