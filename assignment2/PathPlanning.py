#%%
import ast
from collections import deque
import numpy as np
class PathPlanning:
    def __init__(self, nodes_list, adj_matrix : np.ndarray, Xini, Xfin):
        self.nodes_list = nodes_list
        self.adj_matrix = adj_matrix.astype(int)
        
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
#%%

# nodes = nodes.replace(" ", ", ")
# adj_mat = adj_mat.replace(" ", ", ")
# adj_mat = adj_mat.replace("][", "],[")

# nodes_list = ast.literal_eval(nodes)
# adj_mat_list = ast.literal_eval(adj_mat)
#%%
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
# # %%
