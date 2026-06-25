from pathfinder import PathFinder

pf = PathFinder()

path = pf.astar_path_as_object("C3", "D2", ["C2"])
print(path)