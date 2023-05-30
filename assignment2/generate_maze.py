import numpy as np
import random

import networkx as nx
import matplotlib.pyplot as plt

n_nodes = 10*10

class GraphMaze:
    def __init__(self) -> None:
        self.nodes = np.arange(n_nodes)
        self.edges = np.zeros((n_nodes, n_nodes), dtype=int)
        self.node_coordinates = np.zeros((n_nodes, 2), dtype=float)
        self.start_node = None
        self.end_node = None
        self.generate_connection()
        self.generate_coordinates()


    def generate_coordinates(self):
        middle_point = [0.25, 0.25]
        for i in range(n_nodes):
            x = middle_point[0] + (i % 10) * 0.5
            y = middle_point[1] + (i // 10) * 0.5
            self.node_coordinates[i] = [x, y]
        

    def print_coordinates(self):
        print(self.node_coordinates)


    def generate_connection(self):
        for i in range(n_nodes):
            if (i+1) % 10 == 0:
                self.edges[i, i-1] = 1
            elif (i) % 10 == 0:
                self.edges[i, i+1] = 1
            else:
                self.edges[i, i+1] = 1
                self.edges[i, i-1] = 1

            if  i >= 90:
                self.edges[i, i-10] = 1
            elif i < 10:
                self.edges[i, i+10] = 1
            else:
                self.edges[i, i+10] = 1
                self.edges[i, i-10] = 1



    def degree(self, node):
        return np.sum(self.edges[node])
    
    def get_neighbours(self, node):
        return np.where(self.edges[node] == 1)[0]
    
    def is_a_vertical_edge(self, i, j):
        return abs(i - j) >= 10
    
    def is_a_horizontal_edge(self, i, j):
        return abs(i - j) < 10

    
    def generate_maze(self, size) -> tuple:
        verticals = []
        horizontals = []

        do_not_pick = []
        
        i = 0
        while i < size:

            in_nodes = np.random.randint(0, n_nodes)
            out_nodes = np.random.choice(self.get_neighbours(in_nodes))
            self.edges[in_nodes, out_nodes] = 0
            self.edges[out_nodes, in_nodes] = 0
            if self.dfs(in_nodes, out_nodes):
                print("DFS: Maze is connected")
                if self.is_a_vertical_edge(in_nodes, out_nodes):
                    verticals.append((in_nodes, out_nodes))
                elif self.is_a_horizontal_edge(in_nodes, out_nodes):
                    horizontals.append((in_nodes, out_nodes))
                else:
                    print("Error: Invalid edge")
                i += 1
            else:
                print('edge cannot be removed')
                self.edges[in_nodes, out_nodes] = 1
                self.edges[out_nodes, in_nodes] = 1
                do_not_pick.append((in_nodes, out_nodes))

        

        

        return self.nodes, verticals, horizontals, self.edges
    
    def dfs(self, in_nodes, out_nodes):
        start_node = 0
        visited = np.zeros(n_nodes, dtype=int)
        stack = [start_node]
        while len(stack) > 0:
            node = stack.pop()
            if visited[node] == 0:
                visited[node] = 1
                print(node)
                
                for neighbour in self.get_neighbours(node):
                    stack.append(neighbour)
        if np.sum(visited) == n_nodes:
            print("DFS: Maze is connected")
            return True
        else:
            return False
    

    def draw_maze(self, size=30):
        _, verticals, horizontals, _ = self.generate_maze(size=size)
        G = nx.Graph()
    
        # Add nodes to the graph
        for node in self.nodes:
            G.add_node(node)

        # # Add vertical and horizontal edges to the graph
        # for edge in verticals:
        #     G.add_edge(*edge, color='r')  # color vertical edges red
        # for edge in horizontals:
        #     G.add_edge(*edge, color='b')  # color horizontal edges blue

        # Add remaining edges from self.edges to the graph
        for i in range(n_nodes):
            for j in range(n_nodes):
                if self.edges[i][j] == 1:
                    G.add_edge(i, j, color='g') 
               
                       
        pos = {i: (i % 10, i // 10) for i in self.nodes}
        colors = nx.get_edge_attributes(G,'color').values()

        nx.draw(G, with_labels=True, edge_color=colors, node_color='black', pos=pos)
        plt.show()


if __name__ == '__main__':
    maze = GraphMaze()
    maze.draw_maze(size=80)
    maze.print_coordinates()



