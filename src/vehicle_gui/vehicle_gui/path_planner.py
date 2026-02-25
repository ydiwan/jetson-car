import heapq
import numpy as np
import csv
import os
from collections import defaultdict
import math

class DijkstraClass:
    def __init__(self, nodes_file='nodes.csv', edges_file='edges.csv', intermediates_file='edges_intermediates.csv'):
        self.nodes = {}
        self.edges = defaultdict(list)
        self.edges_intermediates = {}
        self.load_graph(nodes_file, edges_file, intermediates_file)

    def load_graph(self, nodes_file, edges_file, intermediates_file):
        #nodes_path = os.path.join(os.path.dirname(__file__), nodes_file)
        nodes_path = os.path.join(os.path.expanduser('~'), nodes_file)
        with open(nodes_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                node_id = row[0]
                x = float(row[1])
                y = float(row[2])
                self.nodes[node_id] = {'x': x, 'y': y}

        #edges_path = os.path.join(os.path.dirname(__file__), edges_file)
        edges_path = os.path.join(os.path.expanduser('~'), edges_file)
        with open(edges_path, 'r') as f:
            reader = csv.reader(f)
            for row in reader:
                from_node = row[0]
                to_node = row[1]
                distance = float(row[2])
                self.edges[from_node].append((to_node, distance))

        intermediates_path = os.path.join(os.path.expanduser('~'), intermediates_file)
        with open(intermediates_path, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    from_node = row[0]
                    to_node = row[1]
                    intermediates = row[2:]
                    self.edges_intermediates[(from_node, to_node)] = intermediates

    def find_nearest_node(self, x, y):
        min_dist = float('inf')
        nearest_node = None

        for node_id, node_data in self.nodes.items():
            dist = math.sqrt((x - node_data['x'])**2 + (y - node_data['y'])**2)
            if dist < min_dist:
                min_dist = dist
                nearest_node = node_id

        return nearest_node, min_dist

    def dijkstra(self, start_node, end_node):
        distances = {node: float('inf') for node in self.nodes}
        distances[start_node] = 0

        # priority queue
        pq = [(0, start_node)]
        previous = {}
        visited = set()

        while pq:
            current_dist, current_node = heapq.heappop(pq)
            if current_node in visited:
                continue

            visited.add(current_node)
            if current_node == end_node:
                break

            for neighbor, edge_dist in self.edges.get(current_node, []):
                if neighbor in visited:
                    continue

                new_dist = current_dist + edge_dist

                if new_dist < distances[neighbor]:
                    distances[neighbor] = new_dist
                    previous[neighbor] = current_node
                    heapq.heappush(pq, (new_dist, neighbor))

        # reconstruct path
        path = []
        current = end_node
        while current is not None:
            path.append(current)
            current = previous.get(current)

        path.reverse()
        return path, distances[end_node]

    def get_path_with_intermediates(self, path_nodes):
        if not path_nodes or len(path_nodes) < 2:
            return path_nodes
        
        detailed_path = [path_nodes[0]]
        
        for i in range(len(path_nodes) - 1):
            from_node = path_nodes[i]
            to_node = path_nodes[i + 1]
            
            # check if this edge has intermediate points
            edge_key = (from_node, to_node)
            if edge_key in self.edges_intermediates:
                detailed_path.extend(self.edges_intermediates[edge_key])
            
            # add the destination node
            detailed_path.append(to_node)
        
        return detailed_path

    def find_path_from_coordinates(self, start_x, start_y, end_x, end_y):
        start_node, start_dist = self.find_nearest_node(start_x, start_y)
        end_node, end_dist = self.find_nearest_node(end_x, end_y)

        if start_node == end_node:
            return [start_node], [(start_x, start_y), (end_x, end_y)], 0

        path_nodes, node_distance = self.dijkstra(start_node, end_node)

        if path_nodes is None:
            return None, None, None
        
        # add any intermediate points
        detailed_path_nodes = self.get_path_with_intermediates(path_nodes)

        path_coordinates = []
        path_coordinates.append((start_x, start_y))

        for node_id in detailed_path_nodes:
            if node_id in self.nodes:
                node = self.nodes[node_id]
                path_coordinates.append((node['x'], node['y']))

        if len(path_coordinates) > 1:
            path_coordinates[-1] = (end_x, end_y)

        total_distance = start_dist + node_distance + end_dist

        return path_nodes, path_coordinates, total_distance

    def get_node_position(self, node_id):
        if node_id in self.nodes:
            return self.nodes[node_id]['x'], self.nodes[node_id]['y']
        return None, None