import heapq
import networkx as nx
import matplotlib.pyplot as plt

# ============================================================
# MAP NODES
# ============================================================
nodes = {
    "E6":(10,6), "E5":(9,6), "E4":(8,6), "E3":(7,6), "E2":(5,6), "E1":(0,6),
    "D2":(5,4), "D1":(0,4),
    "C3":(10,3), "C2":(5,3), "C1":(0,3),
    "B2":(10,2), "B1":(5,2),
    "A6":(10,0), "A5":(5,0), "A4":(3,0), "A3":(2,0), "A2":(1,0), "A1":(0,0),
}

# ============================================================
# CONNECTIONS
# ============================================================
edges = {
    "E6": ["E5", "C3"],
    "E5": ["E6", "E4"],
    "E4": ["E5", "E3"],
    "E3": ["E4", "E2"],
    "E2": ["E3", "E1", "D2"],
    "E1": ["E2", "D1"],

    "D2": ["E2", "D1", "C2"],
    "D1": ["E1", "D2", "C1"],

    "C3": ["E6", "C2", "B2"],
    "C2": ["D2", "C3", "C1", "B1"],
    "C1": ["D1", "C2", "A1"],

    "B2": ["C3", "B1", "A6"],
    "B1": ["C2", "B2", "A5"],

    "A6": ["B2", "A5"],
    "A5": ["B1", "A6", "A4"],
    "A4": ["A5", "A3"],
    "A3": ["A4", "A2"],
    "A2": ["A3", "A1"],
    "A1": ["C1", "A2"],
}

blocked_nodes = ["C2", "E2"]
# ============================================================
# EDGE WEIGHTS
# ============================================================
def get_weight(a, b):
    x1, y1 = nodes[a]
    x2, y2 = nodes[b]
    return abs(x1 - x2) + abs(y1 - y2)

# ============================================================
# HEURISTIC (MANHATTAN DISTANCE)
# ============================================================
def heuristic(node, goal):
    x1, y1 = nodes[node]
    x2, y2 = nodes[goal]
    return abs(x1 - x2) + abs(y1 - y2)

# ============================================================
# A* ALGORITHM
# ============================================================
import heapq

def astar(start, goal, blocked_nodes=None):

    if blocked_nodes is None:
        blocked_nodes = set()
    else:
        blocked_nodes = set(blocked_nodes)

    open_list = []
    heapq.heappush(open_list, (0, start))

    came_from = {}
    g_score = {start: 0}

    while open_list:

        _, current = heapq.heappop(open_list)

        # skip blocked nodes immediately
        if current in blocked_nodes:
            continue

        if current == goal:

            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)

            return path[::-1]

        for neighbor in edges[current]:

            # skip blocked neighbors BEFORE doing anything else
            if neighbor in blocked_nodes:
                continue

            tentative_g = g_score[current] + get_weight(current, neighbor)

            if tentative_g < g_score.get(neighbor, float('inf')):

                came_from[neighbor] = current
                g_score[neighbor] = tentative_g

                f = tentative_g + heuristic(neighbor, goal)

                heapq.heappush(open_list, (f, neighbor))

    return None

# ============================================================
# VISUALIZATION
# ============================================================
def draw_graph(path=None, blocked_nodes=None):

    if blocked_nodes is None:
        blocked_nodes = set()
    else:
        blocked_nodes = set(blocked_nodes)

    G = nx.Graph()

    for node in nodes:
        G.add_node(node)

    for node, neighbors in edges.items():
        for neighbor in neighbors:
            G.add_edge(node, neighbor)

    pos = nodes

    plt.figure(figsize=(12, 7))

    # edges
    nx.draw_networkx_edges(G, pos, edge_color="lightgray", width=2)

    # ALL nodes default color
    nx.draw_networkx_nodes(G, pos, node_size=900, node_color="skyblue")

    # blocked nodes OVERLAY in red
    if blocked_nodes:
        nx.draw_networkx_nodes(
            G,
            pos,
            nodelist=list(blocked_nodes),
            node_color="red",
            node_size=900
        )

    # labels
    nx.draw_networkx_labels(G, pos, font_color="white", font_size=10)

    # path highlight (unchanged style)
    if path:
        path_edges = list(zip(path[:-1], path[1:]))

        nx.draw_networkx_edges(
            G,
            pos,
            edgelist=path_edges,
            edge_color="red",
            width=5
        )

        nx.draw_networkx_nodes(
            G,
            pos,
            nodelist=path,
            node_size=1000
        )

    plt.gca().invert_xaxis()
    plt.axis("equal")
    plt.axis("off")
    plt.show()

# ============================================================
# TEST
# ============================================================
start = "C3"
goal = "D2"

path = astar(start, goal, blocked_nodes)

print("Shortest Path:")
print(" -> ".join(path))

draw_graph(path, blocked_nodes)