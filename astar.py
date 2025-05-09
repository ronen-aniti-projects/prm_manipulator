import numpy as np
from scipy.spatial import KDTree 
import pdb
from queue import PriorityQueue
from prm import PRM

class AStar:
    def __init__(self, prm:PRM, c_start, c_goal):
        self.connections = prm.connections
        self.nodes_kd = prm.valid_kd
        self.start = prm.add(c_start)
        self.goal = prm.add(c_goal)
        
        self.nodes = prm.nodes 
        self.path = self.search()
        
    def dist(self, c1, c2):
        return np.linalg.norm(c2 - c1, ord=2)
    
    def backtrack(self, goal):
        pass

    def search(self):
        q = PriorityQueue()    
        open = set()
        visited = set()
        parents = dict()
        g_scores = dict()
        start, goal = self.start, self.goal #TODO: Proper index required
        goal_found = True 
        
        # Handle the start node 
        q.put((0.0, start)) # Fix this later for correctness
        open.add(start)
        g_scores[start] = 0.0
        parents[start] = None
        
        # pdb.set_trace()

        while not q.empty(): 
            
            # Pop the most promising node
            _, current = q.get()
            if current in visited:
                continue
            
            # Visit the most promising node
            visited.add(current)

            # Remove the most promising node from the open set
            open.remove(current)

            # Is this the goal?
            if current == goal:
                goal_found = True 
                print("A* found the goal")
                break

            for neighbor, distance in self.connections[current]:
                if neighbor in open:
                    tentative_g_score = g_scores[current] + distance
                    if g_scores[neighbor] <= tentative_g_score:
                        continue 
                    elif g_scores[neighbor] > tentative_g_score:
                        parents[neighbor] = current
                        g_scores[neighbor] = tentative_g_score
                        f_score = g_scores[neighbor] + self.dist(self.nodes[neighbor], self.nodes[goal])
                        q.put((f_score, neighbor))
                else:
                    g_scores[neighbor] = g_scores[current] + distance
                    parents[neighbor] = current 
                    f_score = g_scores[neighbor] + self.dist(self.nodes[neighbor], self.nodes[goal])
                    q.put((f_score, neighbor))
                    open.add(neighbor)
        
        if goal_found:
            print("Goal Found")
            return self.backtrack(goal)
        else:
            print("Goal NOT Found")
            
        
        return None