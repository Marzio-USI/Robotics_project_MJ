from generate_maze import GraphMaze
import rclpy
from rclpy.node import Node
import tf_transformations
import numpy as np
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import time
import sys
from sensor_msgs.msg import Range
import math
import os
dir_path = os.path.dirname(os.path.abspath(__file__))
dir_path2 = '/home/robotics23/dev_ws/src/assignment2'
# dir_path3 = '/home/robotics23/dev_ws/src/assignment2/assignment2'
# dir_path4 = '/home/robotics23/apps/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu22_04/programming/legacyRemoteApi/remoteApiBindings/lib/lib/Ubuntu20_04'
dir_path5 = '/home/robotics23/apps/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu22_04/programming/zmqRemoteApi/clients/python'
# Add the directory to sys.path
sys.path.append(dir_path)
sys.path.append(dir_path2)
# sys.path.append(dir_path3)
# sys.path.append(dir_path4)
sys.path.append(dir_path5)
# print('looking at', sys.path)
# print('dir', dir_path)

# import sim
from zmqRemoteApi import RemoteAPIClient
import time

def spawn_maze(self):
        
    print('start')
    client = RemoteAPIClient()
    print('connected')
    sim = client.getObject('sim')
    print('got sim')

    cuboid_type = sim.primitiveshape_cuboid

    maze = GraphMaze()
        nodes, verticals, horizontals, edges, cood_vert, cood_horz =  maze.generate_maze(size=40)
        node_cord = maze.node_coordinates

        # maze.draw_maze()

        wall_size_x = [0.6, 0.1, 1.0] 
        wall_size_y = [0.1, 0.6, 1.0]

        for (v_x, v_y) in cood_vert:
            wall_handle = sim.createPrimitiveShape(cuboid_type, wall_size_x)
            sim.setObjectPosition(wall_handle, -1, [v_x, v_y, 0.5])

        for (h_x, h_y) in cood_horz:
            wall_handle = sim.createPrimitiveShape(cuboid_type, wall_size_y)
            sim.setObjectPosition(wall_handle, -1, [h_x, h_y, 0.5])

        # for (n_x, n_y) in node_cord:
        #     wall_handle = sim.createPrimitiveShape(cuboid_type, [0.0, 0.0, 0.5])
        #     sim.setObjectPosition(wall_handle, -1, [n_x, n_y, 0.5])

        # spawn the 4 big walls
        wall_up_bottom = [5.2, 0.1, 1.0]
        wall_left_right = [0.1, 5.2, 1.0]

        #up 
        wall_handle = sim.createPrimitiveShape(cuboid_type, wall_up_bottom)
        sim.setObjectPosition(wall_handle, -1, [0.0, 2.5+0.05, 0.5])

        #bottom
        wall_handle = sim.createPrimitiveShape(cuboid_type, wall_up_bottom)
        sim.setObjectPosition(wall_handle, -1, [0.0, -(2.5+0.05), 0.5])

        #left
        wall_handle = sim.createPrimitiveShape(cuboid_type, wall_left_right)
        sim.setObjectPosition(wall_handle, -1, [-(2.5+0.05), 0.0 , 0.5])

        #right
        wall_handle = sim.createPrimitiveShape(cuboid_type, wall_left_right)
        sim.setObjectPosition(wall_handle, -1, [(2.5+0.05), 0.0 , 0.5])


        # maze.draw_maze(size=80)
        


    #     client = RemoteAPIClient('localhost', 19997)
    #     sim = client.getObject('sim')
    #     # sim.simxFinish(-1)
        

        

       

    # # Move the wall to a specific location
        

        # clientID = sim.simxStart('127.0.0.1',19997,True,True,5000,5)

        # if clientID!= -1:
        #     print("Connected to Remote API Server")
        #     wall_size = [1.0, 1.0, 0.1]
        #     wall_handle = sim.simxCreateBuffer(sim.primitiveshape_cuboid, wall_size)
        #     sim.simxSetObjectPosition()
            
        # else:
        #     print("Connection failed")
        #     sys.exit('Could not reconnect')
        # sim.simxFinish(clientID)

        


    def start(self):
        pass
      

def main():
    # Initialize the ROS client library

    # rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = MazeSpawner()
    # node.start()
    
    # # Keep processings events until someone manually shuts down the node
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     pass
    
    # Ensure the Thymio is stopped before exiting
    # node.stop()

if __name__ == '__main__':
    # print('looking at', sys.path)
    main()
