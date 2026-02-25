import heapq
import numpy as np

class AStar:
    def __init__(self, grid_width, grid_height, obstacle_grid):
        """
        Initialize A* pathfinder
        
        grid_width: width of the map in grid cells
        grid_height: height of the map in grid cells
        obstacle_grid: 2D array where True means obstacle
        """
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.obstacle_grid = obstacle_grid
        
    def heuristic(self, a, b):
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def get_neighbors(self, node):
        """Get valid neighboring grid cells"""
        x, y = node
        neighbors = []
        
        # Check four adjacent cells
        for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0)]:
            nx, ny = x + dx, y + dy
            
            # Check bounds
            if 0 <= nx < self.grid_width and 0 <= ny < self.grid_height:
                # Check if not obstacle
                if not self.obstacle_grid[ny][nx]:  # Note: y is first index in grid
                    neighbors.append((nx, ny))
                    
        return neighbors
    
    def find_path(self, start, goal):
        """
        Find path from start to goal
        
        start: (x, y) tuple of start position in grid coordinates
        goal: (x, y) tuple of goal position in grid coordinates
        
        Returns list of (x, y) points forming the path
        """
        # Open set (priority queue)
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        # For each node, which node it came from
        came_from = {}
        
        # Cost from start to node
        g_score = {start: 0}
        
        # Estimated total cost from start to goal through node
        f_score = {start: self.heuristic(start, goal)}
        
        # Set of visited nodes
        closed_set = set()
        
        while open_set:
            # Get node with lowest f_score
            current_f, current = heapq.heappop(open_set)
            
            # If we reached the goal
            if current == goal:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            
            # Mark as visited
            closed_set.add(current)
            
            # Check all neighbors
            for neighbor in self.get_neighbors(current):
                if neighbor in closed_set:
                    continue
                
                # Tentative g_score
                tentative_g = g_score.get(current, float('inf')) + 1
                
                # If this is a better path to the neighbor
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    # Record this path
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        # No path found
        return None