import heapq
import re

class Graph:
    def __init__(self):
        self.nodes = {}

    def add_node(self, node):
        if node not in self.nodes:
            self.nodes[node] = []

    def add_edge(self, u, v, weight):
        self.add_node(u)
        self.add_node(v)
        self.nodes[u].append((v, weight))

def dijkstra(graph, start, end):
    queue = []
    heapq.heappush(queue, (0, start))
    distances = {node: float('infinity') for node in graph.nodes}
    distances[start] = 0
    shortest_path = {}

    while queue:
        current_distance, current_node = heapq.heappop(queue)

        if current_distance > distances[current_node]:
            continue

        for neighbor, weight in graph.nodes[current_node]:
            distance = current_distance + weight

            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(queue, (distance, neighbor))
                shortest_path[neighbor] = current_node

    path = []
    while end:
        path.append(end)
        end = shortest_path.get(end)
    return distances, path[::-1]

def parse_input():
    start = input("Enter the starting point: ").upper()
    total_points = int(input("Enter the total number of points (including start and end): "))
    connections = {}

    for i in range(total_points):
        point_connections = input(f"Enter connections for point {chr(ord('A') + i)}: ").upper()
        connections[chr(ord('A') + i)] = re.findall(r'(\w)(\d+)', point_connections)

    end = input("Enter the ending point: ").upper()

    return start, end, connections

def build_graph(connections):
    graph = Graph()

    for point, point_connections in connections.items():
        for neighbor, weight in point_connections:
            graph.add_edge(point, neighbor, int(weight))

    return graph

def main():
    start, end, connections = parse_input()
    graph = build_graph(connections)

    distances, path = dijkstra(graph, start, end)

    if distances[end] != float('infinity'):
        print(f"Shortest path from {start} to {end}: {''.join(path)}")
        print(f"Total distance: {distances[end]} km")
    else:
        print("No path found between the start and end points")

if __name__ == "__main__":
    main()

