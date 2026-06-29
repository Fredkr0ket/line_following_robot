from pathfinder import PathFinder

path_finder = PathFinder()

path = path_finder.astar_path_as_object("A4", "C3", ["A6", "B2"])
path_order = path_finder.astar("A4", "C3",["A6", "B2"])
path.pop(path_order[0])
path_order.pop(0)
print(f"Path: {path} | Path_order: {path_order}")
while True:
    for node in path_order:
        coord = path[node]
        print(f"coords to go to: {coord}")
        # print(f"current position: {position}")
        # position = path_to_node(coord, position, encoder_left, encoder_right, base_speed_robot)
        path.pop(next(iter(path)))

