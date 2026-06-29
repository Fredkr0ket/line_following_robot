from pathfinder import PathFinder

pf = PathFinder()

path = pf.astar_path_as_object("A6", "C2")
path_order = pf.astar("A6", "C2")
while True:
    for node in path_order:
        coord = path[node]
        print(path)
        print(node)
        print(coord)
        path.pop(next(iter(path)))
        path_order.pop(0)