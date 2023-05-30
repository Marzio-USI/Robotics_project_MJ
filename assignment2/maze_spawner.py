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
dir_path3 = '/home/robotics23/dev_ws/src/assignment2/assignment2'
dir_path4 = '/home/robotics23/apps/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu22_04/programming/legacyRemoteApi/remoteApiBindings/lib/lib/Ubuntu20_04'
# Add the directory to sys.path
sys.path.append(dir_path)
sys.path.append(dir_path2)
sys.path.append(dir_path3)
sys.path.append(dir_path4)
# print('looking at', sys.path)
# print('dir', dir_path)
import sim
 
class MazeSpawner(Node):
    def __init__(self):
        super().__init__('maze_spawner')
        sim.simxFinish(-1)

    clientID = sim.simxStart('172.16.165.129',33333,True,True,5000,5)

    if clientID!= -1:
        print("Connected to Remote API Server")
    else:
        print("Connection failed")
        sys.exit('Could not reconnect')
    sim.simxFinish(clientID)


        
      

def main():
    # Initialize the ROS client library

    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = MazeSpawner()
    node.start()
    
    # Keep processings events until someone manually shuts down the node
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Ensure the Thymio is stopped before exiting
    node.stop()


if __name__ == '__main__':
    print('looking at', sys.path)
    main()
