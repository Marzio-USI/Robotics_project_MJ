import rclpy
from rclpy.node import Node
import tf_transformations
import numpy as np
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import time

from sensor_msgs.msg import Range
import math
from math import sqrt
import sys
import os
dir_path  = os.path.dirname(os.path.abspath(__file__))
print(dir_path)
sys.path.append(dir_path)

from maze_spawner import spawn_maze, get_thymio_position, get_thymio_orientation


from PathPlanning import *

FACING_UP = 1.56
FACING_DOWN = -1.56
FACING_LEFT = -3.13
FACING_RIGHT = 0.0
COMPLETE_ROTATION= 2* math.pi
HALF_ROTATION = math.pi


def generate_connection(n_nodes) -> np.ndarray:
    edges = np.zeros((n_nodes, n_nodes), dtype=int)
    for i in range(n_nodes):
        if (i+1) % 10 == 0:
            edges[i, i-1] = 1
        elif (i) % 10 == 0:
            edges[i, i+1] = 1
        else:
            edges[i, i+1] = 1
            edges[i, i-1] = 1

        if  i >= 90:
            edges[i, i-10] = 1
        elif i < 10:
            edges[i, i+10] = 1
        else:
            edges[i, i+10] = 1
            edges[i, i-10] = 1
    return edges

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')



        nodes, nodes_coords, edges , start_point, end_point, node_start, node_end = spawn_maze(size=54)

        self.runtime_edges = generate_connection(len(nodes))


        self.algo = get_algorithm('BFS', nodes, self.runtime_edges, node_start, node_end)
        self.nodes_coords = nodes_coords
        self.nodes = nodes

        self.path = self.algo.compute()
        self.get_logger().info('original {}'.format(str(self.path)))
        self.prev_node = self.path.pop(0)
        self.next_node = self.path.pop(0)

        self.node_end = node_end
        self.node_start = node_start
        
        

        # realtive_node_coords = self.global_to_local(start_point, nodes_coords)

        self.start_point = start_point
        self.end_point = end_point

        
    
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)


        self.proximity_center_subscriber = self.create_subscription(Range, 'proximity/center', self.proximity_callback, 10)
        # self.proximity_left_subscriber = self.create_subscription(Range, 'proximity/left', self.proximity_callback_left, 10)
        # self.proximity_right_subscriber = self.create_subscription(Range, 'proximity/right', self.proximity_callback_right, 10)


        # self.proximity_rear_left_subscriber = self.create_subscription(Range, 'proximity/rear_left',self.proximity_callback_back, 10)
        # self.proximity_rear_right_subscriber = self.create_subscription(Range, 'proximity/rear_right', self.proximity_callback_back, 10)
        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which Thymio should be controlled.
        self.info_stop = -1.0
        self.info_stop_left = -1.0
        self.info_stop_right = -1.0

  

    def coords_to_pose2d(self, coords):
        x : float = coords[0]
        y : float= coords[1]
        return (
            x, y, 0.0
        )
    
    def linear_vel(self, angle):
        if abs(angle) < 0.017:
            return 0.14
        else:
            if abs(angle) >= (math.pi/2.5):
                return 0.005
            else:
                return 0.01

  

    def rotate(self, orientation, to_orientation):
        z:float = orientation[-1]
        difference = (to_orientation - z + HALF_ROTATION) % COMPLETE_ROTATION - HALF_ROTATION
        return difference + COMPLETE_ROTATION if difference <= -HALF_ROTATION else difference
    
    def is_upper_edge(self, node_x, node_y, node_f_x, node_f_y):
        return node_x == node_f_x and node_y < node_f_y
    
    def is_lower_edge(self, node_x, node_y, node_f_x, node_f_y):
        return node_x == node_f_x and node_y > node_f_y
    
    def is_right_edge(self, node_x, node_y, node_f_x, node_f_y):
        return node_y == node_f_y and node_x < node_f_x
    
    def is_left_edge(self, node_x, node_y, node_f_x, node_f_y):
        return node_y == node_f_y and node_x > node_f_x
    
    def looking_at_the_right_edge(self, thymio_orient, x_s, y_s, x_f, y_f):
        z = thymio_orient[-1]
        facing_up = (abs(z - FACING_UP) < 0.176) and self.is_upper_edge(x_s, y_s, x_f, y_f)
        facing_down = (abs(z - FACING_DOWN) < 0.176) and self.is_lower_edge(x_s, y_s, x_f, y_f)
        facing_right = (abs(z - FACING_RIGHT) < 0.176) and self.is_right_edge(x_s, y_s, x_f, y_f)
        facing_left = (abs(z - FACING_LEFT) < 0.176) and self.is_left_edge(x_s, y_s, x_f, y_f)
        return facing_up or facing_down or facing_right or facing_left


    def update_callback(self):
        thymio_position = self.pose3d_to_2d(get_thymio_position())
        thymio_orientation = get_thymio_orientation()


        if self.euclidean_distance(thymio_position, self.coords_to_pose2d(self.nodes_coords[self.node_end])) < 0.26:
            raise KeyboardInterrupt()

        cmd_vel = Twist() 
        if self.prev_node is not None:
            # if sensor detects something it remove edges and recompute the path
            x_s, y_s, _ = self.coords_to_pose2d(self.nodes_coords[self.prev_node])
            x_f, y_f, _ = self.coords_to_pose2d(self.nodes_coords[self.next_node])
            if self.info_stop > 0:
                if self.looking_at_the_right_edge(thymio_orientation, x_s, y_s, x_f, y_f):
                    cmd_vel.angular.z = 0.0
                    cmd_vel.linear.x = 0.0
                    self.vel_publisher.publish(cmd_vel)
                    # self.get_logger().info('removing edge {} {}'.format(self.prev_node, self.next_node))
                    self.runtime_edges[self.prev_node, self.next_node] = 0
                    self.runtime_edges[self.next_node, self.prev_node] = 0
                    self.algo = get_algorithm('BFS', self.nodes, self.runtime_edges, self.prev_node, self.node_end)
                    self.path = self.algo.compute()
                    # self.get_logger().info('new path {}'.format(str(self.path)))
                    self.prev_node = self.path.pop(0)
                    self.next_node = self.path.pop(0)
                    return

            else:
                pass
            
        
       
        if self.euclidean_distance(thymio_position, self.coords_to_pose2d(self.nodes_coords[self.next_node])) < 0.1:
            if len(self.path) == 0:
                raise KeyboardInterrupt()
            self.prev_node = self.next_node
            self.next_node = self.path.pop(0)
        x, y = thymio_position
        prev_node_x, prev_node_y , _ = self.coords_to_pose2d(self.nodes_coords[self.prev_node])
        next_node_x, next_node_y , _ = self.coords_to_pose2d(self.nodes_coords[self.next_node])
        # up x is the same, y is higher
        angle = 0.0
        correction = 0.0
        if (prev_node_x == next_node_x) and (prev_node_y < next_node_y):
            if abs(prev_node_x - x) > 0.01:
                if prev_node_x < x:
                    # go left
                    correction = math.pi / 3.5
                else:
                    correction = -math.pi / 3.5
                    
            angle = self.rotate(thymio_orientation, FACING_UP) 
        elif (prev_node_x == next_node_x) and (prev_node_y > next_node_y):
            # down (x is the same, y is lower)
            if abs(prev_node_x - x) > 0.01:
                if prev_node_x < x:
                    # go left
                    correction = -math.pi /3.5
                else:
                    correction = +math.pi / 3.5
            angle = self.rotate(thymio_orientation, FACING_DOWN)

        elif (prev_node_x < next_node_x) and (prev_node_y == next_node_y):
            # right (x is higher for next), y is the same
            angle = self.rotate(thymio_orientation, FACING_RIGHT)
            if abs(prev_node_y - y) > 0.01:
                if prev_node_y < y:
                    # go left
                    correction = -math.pi /3.5
                else:
                    correction = math.pi / 3.5
        else:
            angle = self.rotate(thymio_orientation, FACING_LEFT)
            if abs(prev_node_y - y) > 0.01:
                if prev_node_y < y:
                    # go left
                    correction = math.pi / 3.5
                else:
                    correction = -math.pi / 3.5

    
        cmd_vel.angular.z = angle + correction
        cmd_vel.linear.x = self.linear_vel(angle + correction)  
        self.vel_publisher.publish(cmd_vel)

         

    def proximity_callback(self, message):
        distance  = message.range
        if distance >= 0.0 and distance < 0.255:
            self.info_stop = distance
            self.get_logger().info('detected {}'.format(distance))
   
        else:
            self.info_stop = -1.0

    def proximity_callback_left(self, message):
        distance  = message.range
        if distance >= 0 and distance < 0.2:
            self.info_stop_left = distance
            # self.get_logger().info('detrecting left')
        else:
            self.info_stop_left = -1.0

    def proximity_callback_right(self, message):
        distance  = message.range
        if distance >= 0 and distance < 0.2:
            self.info_stop_right = distance
    
            # self.get_logger().info('detrecting right')
        else:
            self.info_stop_right = -1.0

        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
  
    
    def pose3d_to_2d(self, position):
        pose2 = (
            position[0],  # x position
            position[1],  # y position
            # yaw                # theta orientation
        )
        
        return pose2
    
    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose[0] - current_pose[0]), 2) +
                    pow((goal_pose[1] - current_pose[1]), 2))
    


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    node.start()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    main()