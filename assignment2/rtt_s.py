import random
import math

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent

class RRTStar:
    def __init__(self, start, goal, obstacle_list, x_range, y_range, expand_dis=1.0, path_resolution=0.5, max_iter=100):
        self.start = Node(start)
        self.goal = Node(goal)
        #self.obstacle_list = obstacle_list
        self.x_range = x_range
        self.y_range = y_range
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.max_iter = max_iter
        self.node_list = []

    def planning(self):
        self.node_list = [self.start]

        for _ in range(self.max_iter):
            random_node = self.get_random_node()
            nearest_node = self.get_nearest_node(self.node_list, random_node)
            new_node = self.steer(nearest_node, random_node)

            if not self.check_collision(new_node):
                #near_nodes = self.find_near_nodes(new_node)
                #new_node = self.choose_parent(new_node, near_nodes)
                self.node_list.append(new_node)
                #self.rewire(new_node, near_nodes)

        return self.generate_final_course(len(self.node_list) - 1)

    def get_random_node(self):
        x = random.uniform(self.x_range[0], self.x_range[1])
        y = random.uniform(self.y_range[0], self.y_range[1])
        return Node((x, y))

    def get_nearest_node(self, node_list, rand_node):
        return node_list[int(math.floor(rand_node.position[0]))]

    def steer(self, from_node, to_node):
        new_node = Node(from_node.position)
        d, theta = self.calc_distance_and_angle(new_node, to_node)
        new_node.parent = from_node
        d = min(self.expand_dis, d)
        new_node.position[0] += d * math.cos(theta)
        new_node.position[1] += d * math.sin(theta)
        return new_node
