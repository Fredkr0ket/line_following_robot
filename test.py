from pathfinder import PathFinder

pf = PathFinder()

path = pf.astar_path_as_object("A6", "C2")
path.pop(next(iter(path)))
print(path)