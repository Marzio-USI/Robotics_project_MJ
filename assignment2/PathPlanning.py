#%%
class PathPlanning:
    def __init__(self, nodes_list, adj_matrix, Xini, Xfin):
        self.nodes_list = nodes_list
        self.adj_matrix = adj_matrix
        self.Xini = Xini
        self.Xfin = Xfin
        
        self.nodes = len(self.nodes_list)
        self.edges = [[(i, j, 1) for j in range(self.nodes) if self.adj_matrix[i][j] == 1] for i in range(self.nodes)]
        
        self.graph = [[] for _ in range(self.nodes)]
        
        for i in range(self.edges):
            n1, n2, self.cost = self.edges[i]
            self.graph[n1].append((n1,n2,self.cost))
            self.graph[n2].append((n2,n1,self.cost))
        
    def compute(self):
        pass


class DFS(PathPlanning):
    def __init__(self, nodes_list, adj_matrix):
        super().__init__(nodes_list, adj_matrix)
        
    
    def compute(self):
        pass
    



class RRTStar(PathPlanning):
    def __init__(self, nodes_list, adj_matrix):
        super().__init__(nodes_list, adj_matrix)

    def compute(self):
        pass


class BFS(PathPlanning):
    def __init__(self, nodes_list, adj_matrix):
        super().__init__(nodes_list, adj_matrix)

    def compute(self):
        pass
    
class Dijkstra(PathPlanning):
    def __init__(self, nodes_list, adj_matrix):
        super().__init__(nodes_list, adj_matrix)
        
    def select_min(self, visited, distances):
        min_distance = float('inf')
        min_index = -1
        for i in range(len(distances)):
            if distances[i] < min_distance and not visited[i]:
                min_distance = distances[i]
                min_index = i
        return min_index

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
            visited[next_node] = True
            for start, end, cost in g[next_node]:
                if distances[start] + cost < distances[end]:
                    distances[end] = distances[start] + cost
                    predecessors[end] = start

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
