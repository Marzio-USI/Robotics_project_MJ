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

# Add the directory to sys.path
sys.path.append(dir_path)
import sim
 
class MazeSpawner(Node):
    def __init__(self):
        super().__init__('maze_spawner')
        wall_size = [1.0, 1.0, 0.1] 

        wall_handle = sim.simCreatePureShape(0,  # Param 1 indicates the shape type: 0 = cuboid
                                            8,  # Param 2 is a combination of bit-coded settings: 8 = respondable
                                            wall_size,  # Param 3 is the size of the shape
                                            0,  # Param 4 is the mass of the shape
                                            None)  # Param 5 can be used to specify additional options

        # Move the wall to a specific location
        sim.simSetObjectPosition(wall_handle, -1, [0.0, 0.0, 0.5])


        
      
import os

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
    main()