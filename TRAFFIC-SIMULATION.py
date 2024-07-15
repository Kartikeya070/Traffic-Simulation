import heapq
import tkinter as tk
import random
import time
from threading import Thread


class CityGraph:
    def __init__(self):
        self.graph = {}

    def add_edge(self, start, end, weight):
        if start not in self.graph:
            self.graph[start] = []
        if end not in self.graph:
            self.graph[end] = []
        self.graph[start].append((end, weight))
        self.graph[end].append((start, weight))

    def get_edges(self):
        edges = []
        for node in self.graph:
            for neighbor, weight in self.graph[node]:
                edges.append((node, neighbor, weight))
        return edges

    def update_edge_weight(self, start, end, weight):
        for i, (neighbor, w) in enumerate(self.graph[start]):
            if neighbor == end:
                self.graph[start][i] = (end, weight)
        for i, (neighbor, w) in enumerate(self.graph[end]):
            if neighbor == start:
                self.graph[end][i] = (start, weight)




def dijkstra(graph, start, end):
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)]
    previous_nodes = {node: None for node in graph}

    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)

        if current_distance > distances[current_node]:
            continue

        for neighbor, weight in graph[current_node]:
            distance = current_distance + weight
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                previous_nodes[neighbor] = current_node
                heapq.heappush(priority_queue, (distance, neighbor))

    path = []
    while end:
        path.append(end)
        end = previous_nodes[end]
    path.reverse()

    return distances, path





class TrafficSimulator:
    def __init__(self, root, graph):
        self.root = root
        self.graph = graph
        self.canvas = tk.Canvas(root, width=800, height=600)
        self.canvas.pack()
        self.nodes = [(100, 100), (200, 100), (300, 100), (100, 200), (200, 200), (300, 200), (100, 300), (200, 300),
                      (300, 300)]
        self.node_positions = {self.nodes[i]: (self.nodes[i][0], self.nodes[i][1]) for i in range(len(self.nodes))}
        self.create_graph()
        self.draw_map()
        self.start_node = self.nodes[0]
        self.end_node = self.nodes[-1]
        self.update_thread = Thread(target=self.update_traffic)
        self.update_thread.daemon = True
        self.update_thread.start()
        self.root.after(1000, self.update_visualization)

    def create_graph(self):
        for i in range(len(self.nodes)):
            for j in range(i + 1, len(self.nodes)):
                self.graph.add_edge(self.nodes[i], self.nodes[j], random.randint(1, 10))

    def draw_map(self):
        self.canvas.delete("all")
        edges = self.graph.get_edges()
        for (start, end, weight) in edges:
            x1, y1 = self.node_positions[start]
            x2, y2 = self.node_positions[end]
            self.canvas.create_line(x1, y1, x2, y2, fill="black")
            self.canvas.create_text((x1 + x2) / 2, (y1 + y2) / 2, text=str(weight), fill="red")
        for node in self.nodes:
            x, y = self.node_positions[node]
            self.canvas.create_oval(x - 5, y - 5, x + 5, y + 5, fill="blue")

    def update_traffic(self):
        while True:
            for node in self.graph.graph:
                for i in range(len(self.graph.graph[node])):
                    end, _ = self.graph.graph[node][i]
                    new_weight = random.randint(1, 10)
                    self.graph.update_edge_weight(node, end, new_weight)
            time.sleep(10)

    def update_visualization(self):
        self.draw_map()
        self.visualize_shortest_path()
        self.root.after(1000, self.update_visualization)

    def visualize_shortest_path(self):
        distances, path = dijkstra(self.graph.graph, self.start_node, self.end_node)
        for i in range(len(path) - 1):
            x1, y1 = self.node_positions[path[i]]
            x2, y2 = self.node_positions[path[i + 1]]
            self.canvas.create_line(x1, y1, x2, y2, fill="green", width=2)



if __name__ == "__main__":
    root = tk.Tk()
    graph = CityGraph()
    simulator = TrafficSimulator(root, graph)
    root.mainloop()
