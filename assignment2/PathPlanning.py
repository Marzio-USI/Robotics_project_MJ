#%%
import ast
from collections import deque
import numpy as np
import threading
import queue
import random

class PathPlanning:
    def __init__(self, nodes_list, adj_matrix : np.ndarray, Xini, Xfin):
        self.nodes_list = nodes_list
        self.adj_matrix = adj_matrix
        
        self.Xini = Xini
        self.Xfin = Xfin
        
        self.nodes = len(self.nodes_list)
        # self.edges = [[(i, j, 1) for j in range(self.nodes) if self.adj_matrix[i][j] == 1] for i in range(self.nodes)]
        
        # Remove duplicates
        # self.edges = [list(set(i)) for i in self.edges]
        
        # self.graph = [[] for _ in range(self.nodes)]
        

        self.graph = [[(i, j ,1) for j in np.where(self.adj_matrix[i])[0].tolist()] for i in range(self.nodes)]


        # print(self.graph[0])

    def compute(self):
        pass


class DFS(PathPlanning):
    def __init__(self, nodes_list, adj_matrix, Xini, Xfin):
        super().__init__(nodes_list, adj_matrix, Xini, Xfin)
        self.start_node = Xini
        self.end_node = Xfin
    
    def dfs_path(self, g):
        visited = [False] * len(g)
        predecessors = [None] * len(g)
        self.rec_profundidad(g, visited, self.start_node, predecessors)

        path = []
        current_node = self.end_node
        while current_node is not None:
            path.append(current_node)
            current_node = predecessors[current_node]
        path.reverse()
        return path

    def rec_profundidad(self, g, visited, i, predecessors):
        visited[i] = True
        for _, next_node, _ in g[i]:
            if not visited[next_node]:
                predecessors[next_node] = i
                self.rec_profundidad(g, visited, next_node, predecessors)

    def compute(self):
        return self.dfs_path(self.graph)
    



class RRTStar(PathPlanning):
    def __init__(self, nodes_list, adj_matrix):
        super().__init__(nodes_list, adj_matrix)

    def compute(self):
        pass



class BFS(PathPlanning):
    def __init__(self, nodes_list, adj_matrix, Xini, Xfin):
        super().__init__(nodes_list, adj_matrix, Xini, Xfin)
        self.start_node = Xini
        self.end_node = Xfin

    def bfs_path(self, g):
        visited = [False] * len(g)
        predecessors = [None] * len(g)
        visited[self.start_node] = True

        q = deque()
        q.append(self.start_node)

        while q:
            node = q.popleft()
            for _, next_node, _ in g[node]:
                if not visited[next_node]:
                    visited[next_node] = True
                    predecessors[next_node] = node
                    q.append(next_node)

        path = []
        current_node = self.end_node
        while current_node is not None:
            path.append(current_node)
            current_node = predecessors[current_node]
        path.reverse()

        return path

    def compute(self):
        return self.bfs_path(self.graph)

    
class Dijkstra(PathPlanning):
    def __init__(self, nodes_list, adj_matrix, Xini, Xfin):
        super().__init__(nodes_list, adj_matrix, Xini, Xfin)
        self.start_node = Xini
        self.end_node = Xfin
        
        
    def select_min(self, visited, distances):
        min_distance = float('inf')
        min_index = -1
        for i in range(len(distances)):
            if distances[i] < min_distance and not visited[i]:
                min_distance = distances[i]
                min_index = i
        return min_index
    
    def __str__(self):
        return f"Running Dijkstra with {self.nodes} nodes, starting at {self.Xini} and ending at {self.Xfin}"

    def dijkstra_path(self, g):
        n = len(g)
        visited = [False] * n
        visited[self.start_node] = True
        distances = [float('inf')] * n
        distances[self.start_node] = 0
        predecessors = [None] * n

        for start, end, cost in g[self.start_node]:
            distances[end] = cost
            predecessors[end] = self.start_node

        for _ in range(1, len(g)):
            next_node = self.select_min(visited, distances)
            if next_node is None:
                break
            visited[next_node] = True
            for start, end, cost in g[next_node]:
                if not visited[end] and distances[start] + cost < distances[end]:
                    distances[end] = distances[start] + cost
                    predecessors[end] = next_node

        path = []
        current_node = self.end_node
        while current_node is not None:
            path.append(current_node)
            current_node = predecessors[current_node]
        path.reverse()
        return path

    def compute(self):
        return self.dijkstra_path(self.graph)
    
    
class BiconnectedBFS(PathPlanning):
    def __init__(self, nodes_list, adj_matrix, Xini, Xfin):
        super().__init__(nodes_list, adj_matrix, Xini, Xfin)
        self.meeting_node = None
        self.lock = threading.Lock()

    def compute(self):
        to_visit_start = queue.Queue()
        to_visit_end = queue.Queue()
        parents_start = {}
        parents_end = {}
            
        thread_start = threading.Thread(target=self.bfs, args=(self.Xini, self.Xfin, self.graph, to_visit_start, parents_start, parents_end))
        thread_end = threading.Thread(target=self.bfs, args=(self.Xfin, self.Xini, self.graph, to_visit_end, parents_end, parents_start))
            
        thread_start.start()
        thread_end.start()
            
        thread_start.join()
        thread_end.join()

        if self.meeting_node is not None:
            path = self.build_path(parents_start, parents_end, self.meeting_node)
            return path
        else:
            return None


    def bfs(self, start, end, graph, to_visit, parents, parents_other):
        to_visit.put(start)
        parents[start] = None
        while not to_visit.empty():
            node = to_visit.get()
            for _, neighbor, _ in graph[node]: 
                if neighbor not in parents:
                    parents[neighbor] = node
                    to_visit.put(neighbor)
                elif neighbor in parents_other:
                    with self.lock:
                        if self.meeting_node is None:
                            self.meeting_node = neighbor
                            return

    def build_path(self, parents_start, parents_end, meeting_node):
        path = [meeting_node]
        while parents_start[path[0]] is not None:
            path.insert(0, parents_start[path[0]])
        while parents_end[path[-1]] is not None:
            path.append(parents_end[path[-1]])
        return path



class HybridPathFinder(PathPlanning):
    def __init__(self, nodes_list, adj_matrix, Xini, Xfin):
        super().__init__(nodes_list, adj_matrix, Xini, Xfin)
        self.max_iterations = 1000
        self.threshold = 10  
        self.tree_start = {self.Xini: None}  
        self.tree_end = {self.Xfin: None}  

    def compute(self):
        for _ in range(self.max_iterations):
            sampled_node = self.sample_node()
            for tree in [self.tree_start, self.tree_end]:
                nearest_node = self.find_nearest_node(tree, sampled_node)
                if self.distance(nearest_node, sampled_node) < self.threshold:
                    tree[sampled_node] = nearest_node
            if self.try_connect():
                return self.build_path()
        return None

    def sample_node(self):
        return random.choice(self.nodes_list)

    def find_nearest_node(self, tree, node):
        return min(tree.keys(), key=lambda n: self.distance(n, node))

    def distance(self, node1, node2):
        DJ = Dijkstra(self.nodes_list, self.adj_matrix, node1, node2)
        return len(DJ.compute())

    def try_connect(self):
        for node_start in self.tree_start:
            for node_end in self.tree_end:
                if self.distance(node_start, node_end) < self.threshold:
                    self.tree_end[node_end] = node_start  
                    return True
        return False

    def build_path(self):
        for node in self.tree_start:
            if node in self.tree_end:
                meeting_node = node
                break
        else:
            return None  

        path_start = []
        current_node = meeting_node
        while current_node is not None:
            path_start.append(current_node)
            current_node = self.tree_start.get(current_node, None)
        path_start.reverse()

        path_end = []
        current_node = meeting_node
        while current_node is not None:
            path_end.append(current_node)
            current_node = self.tree_end.get(current_node, None)
        

        return path_start + path_end[1:] 


    

# #%%
# from data_test import nodes, adj_mat
# nodes = nodes.replace(" ", ", ")
# adj_mat = adj_mat.replace(" ", ", ")
# adj_mat = adj_mat.replace("][", "],[")

# nodes_list = ast.literal_eval(nodes)
# adj_mat_list = ast.literal_eval(adj_mat)
# #%%
# path_planning = Dijkstra(nodes_list, adj_mat_list, 1, 99)
# path = path_planning.compute()
# print(path)
# # %%
# path_planning = DFS(nodes_list, adj_mat_list, 1, 99)
# path = path_planning.compute()
# print(path)
# # %%
# path_planning = BFS(nodes_list, adj_mat_list, 1, 99)
# path = path_planning.compute()
# print(path)
# #%%
# path_planning = BiconnectedBFS(nodes_list, adj_mat_list, 1, 99)
# path = path_planning.compute()
# print(path)

# # %%
# path_planning = HybridPathFinder(nodes_list, adj_mat_list, 1, 99)
# path = path_planning.compute()
# print(path)
# # %%

 

def get_algorithm(name:str, nodes : list, edges : np.ndarray, node_start: int, node_end:int):
    if name == 'DFS':
        return DFS(nodes, edges, node_start, node_end)
    elif name =='BFS':
        return BFS(nodes, edges, node_start, node_end)
    elif name == 'DJ':
        return Dijkstra(nodes, edges, node_start, node_end)
    elif name == 'BC':
        return BiconnectedBFS(nodes, edges, node_start, node_end)
    else:
        raise NotImplementedError()

# %%
