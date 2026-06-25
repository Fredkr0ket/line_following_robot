import heapq
import networkx as nx
import matplotlib.pyplot as plt


class PathFinder:

    def __init__(self):
        self.nodes = {
            "E6":(1485,730), "E5":(1330,730), "E4":(1180,730), "E3":(1030,730), "E2":(745,730), "E1":(0,730),
            "D2":(745 ,515), "D1":(0,515),
            "C3":(1485,365), "C2":(745,365), "C1":(0,365),
            "B2":(1485,215), "B1":(745,215),
            "A6":(1485,0), "A5":(745,0), "A4":(450,0), "A3":(300,0), "A2":(150,0), "A1":(0,0),
        }


        self.edges = {
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

    def get_weight(self, a, b):
        x1, y1 = self.nodes[a]
        x2, y2 = self.nodes[b]
        return abs(x1 - x2) + abs(y1 - y2)

    def heuristic(self, node, goal):
        x1, y1 = self.nodes[node]
        x2, y2 = self.nodes[goal]
        return abs(x1 - x2) + abs(y1 - y2)

    def astar(self, start, goal, blocked_nodes=None):

        blocked_nodes = set(blocked_nodes or [])

        open_list = []
        heapq.heappush(open_list, (0, start))

        came_from = {}
        g_score = {start: 0}

        while open_list:

            _, current = heapq.heappop(open_list)

            if current in blocked_nodes:
                continue

            if current == goal:

                path = [current]

                while current in came_from:
                    current = came_from[current]
                    path.append(current)

                return path[::-1]

            for neighbor in self.edges[current]:

                if neighbor in blocked_nodes:
                    continue

                tentative_g = (
                    g_score[current]
                    + self.get_weight(current, neighbor)
                )

                if tentative_g < g_score.get(neighbor, float("inf")):

                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g

                    f = tentative_g + self.heuristic(neighbor, goal)

                    heapq.heappush(open_list, (f, neighbor))

        return None

    def get_node_from_coord(self, coord):

        for name, position in self.nodes.items():
            if position == coord:
                return name

        return None

    def get_coord_from_node(self, node):
        return self.nodes.get(node)
    
    def astar_path_as_object(self, start, goal, blocked_nodes=None):

        path = self.astar(start, goal, blocked_nodes)

        if not path:
            return None

        path_object = {}

        for node in path:
            path_object[node] = self.nodes[node]

        return path_object

    def draw_graph(self, path=None, blocked_nodes=None):

        blocked_nodes = set(blocked_nodes or [])

        G = nx.Graph()

        for node in self.nodes:
            G.add_node(node)

        for node, neighbors in self.edges.items():
            for neighbor in neighbors:
                G.add_edge(node, neighbor)

        plt.figure(figsize=(12, 7))

        nx.draw_networkx_edges(
            G,
            self.nodes,
            edge_color="lightgray",
            width=2
        )

        nx.draw_networkx_nodes(
            G,
            self.nodes,
            node_size=900,
            node_color="skyblue"
        )

        if blocked_nodes:
            nx.draw_networkx_nodes(
                G,
                self.nodes,
                nodelist=list(blocked_nodes),
                node_color="red",
                node_size=900
            )

        nx.draw_networkx_labels(
            G,
            self.nodes,
            font_color="white"
        )

        if path:

            path_edges = list(zip(path[:-1], path[1:]))

            nx.draw_networkx_edges(
                G,
                self.nodes,
                edgelist=path_edges,
                edge_color="red",
                width=5
            )

            nx.draw_networkx_nodes(
                G,
                self.nodes,
                nodelist=path,
                node_size=1000
            )

        plt.gca().invert_xaxis()
        plt.axis("equal")
        plt.axis("off")
        plt.show()

pf = PathFinder()
