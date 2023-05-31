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

        for i in range(n_nodes):
            x = self.node_coordinates[i][0]
            y = self.node_coordinates[i][1]
            if x <= 2.5:
                self.node_coordinates[i][0] = -2.5 + self.node_coordinates[i][0]
            elif x > 2.5:
                self.node_coordinates[i][0] -= 2.5
            
            if y <=2.5:
                self.node_coordinates[i][1] = -2.5 + self.node_coordinates[i][1]
            elif y > 2.5:
                self.node_coordinates[i][1] -= 2.5 


    def generate_edge_coordinates(self, edges):
        tmp = []
        for in_nodes, out_nodes in edges:
            in_x, in_y = self.node_coordinates[in_nodes]
            out_x, out_y = self.node_coordinates[out_nodes]
            X = (in_x + out_x)/2
            Y = (in_y + out_y)/2 
            tmp.append((X, Y))

        return tmp


        

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

        max_edges = np.sum(self.edges)//2 - (len(self.nodes) - 1)
        if size >=  max_edges/2:
            size = max_edges

        do_not_pick = []
        i = 0
        while i < size:

            in_nodes = np.random.randint(0, n_nodes)
            out_nodes = np.random.choice(self.get_neighbours(in_nodes))
            self.edges[in_nodes, out_nodes] = 0
            self.edges[out_nodes, in_nodes] = 0
            if self.dfs(in_nodes, out_nodes):
                # print("DFS: Maze is connected")
                if self.is_a_vertical_edge(in_nodes, out_nodes):
                    verticals.append((in_nodes, out_nodes))
                elif self.is_a_horizontal_edge(in_nodes, out_nodes):
                    horizontals.append((in_nodes, out_nodes))
                else:
                    print("Error: Invalid edge")
                i += 1
            else:
                # print('edge cannot be removed')
                self.edges[in_nodes, out_nodes] = 1
                self.edges[out_nodes, in_nodes] = 1
                do_not_pick.append((in_nodes, out_nodes))

        cood_vert = self.generate_edge_coordinates(verticals)
        cood_horz = self.generate_edge_coordinates(horizontals)

        

        return self.nodes, verticals, horizontals, self.edges, cood_vert, cood_horz
    
    def dfs(self, in_nodes, out_nodes):
        start_node = 0
        visited = np.zeros(n_nodes, dtype=int)
        stack = [start_node]
        while len(stack) > 0:
            node = stack.pop()
            if visited[node] == 0:
                visited[node] = 1
                # print(node)
                
                for neighbour in self.get_neighbours(node):
                    stack.append(neighbour)
        if np.sum(visited) == n_nodes:
            # print("DFS: Maze is connected")
            return True
        else:
            return False
    

    def draw_maze(self, verticals=None, horizontals=None, size=30):
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




# if __name__ == '__main__':
#     maze = GraphMaze()
#     for i in range(90, -1, -10):
#         for j in range(10):
#             print(maze.node_coordinates[i+j], end=' ')
#         print('')

#     start = np.random.choice(np.arange(0, 100))
#     print(start)

#     print(maze.node_coordinates[start])

#     print('CONVERTING')

#     new_coords = global_to_local(start, maze.node_coordinates)

#     maze = GraphMaze()
#     for i in range(90, -1, -10):
#         for j in range(10):
#             k = new_coords[i+j][0], new_coords[i+j][1]
#             print(k, end=' ')
#         print('')

#     print(new_coords[start])



    # maze.print_coordinates()
#     *_, a, b = maze.generate_maze(size=80)
#     # maze.draw_maze(size=80)
#     print(a, b, (len(a)+len(b)))
#     # maze.print_coordinates()



