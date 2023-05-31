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
import numpy as np
# import sim
from zmqRemoteApi import RemoteAPIClient
import time

def spawn_maze(size=40):
    

    print('start')
    client = RemoteAPIClient()
    print('connected')
    sim = client.getObject('sim')
    print('got sim')

    cuboid_type = sim.primitiveshape_cuboid

    maze = GraphMaze()
    nodes, verticals, horizontals, edges, cood_vert, cood_horz =  maze.generate_maze(size=size)
    node_cord = maze.node_coordinates

    start_point = np.random.choice(nodes)
    start_point_x, start_point_y = node_cord[start_point]

    end_point = np.random.choice(np.delete(deepcopy(nodes), start_point))
    end_point_x, end_point_y = node_cord[end_point]

    thymio_handle = sim.getObject("/MightyThymio")
    sim.setObjectPosition(thymio_handle, -1, [start_point_x, start_point_y, 0.501])


    # properties_objects = sim.objectspecialproperty_collidable sim.objectspecialproperty_measurable or sim.objectspecialproperty_detectable
    # maze.draw_maze()

    wall_size_x = [0.6, 0.1, 1.0] 
    wall_size_y = [0.1, 0.6, 1.0]
    properties_objects = sim_objectspecialproperty_collidable | sim_objectspecialproperty_measurable | sim_objectspecialproperty_detectable_ultrasonic | sim_objectspecialproperty_detectable_infrared | sim_objectspecialproperty_detectable_laser | sim_objectspecialproperty_detectable_inductive | sim_objectspecialproperty_detectable_capacitive | sim_objectspecialproperty_renderable

    for (v_x, v_y) in cood_vert:
        wall_handle = sim.createPrimitiveShape(cuboid_type, wall_size_x)
        # sim.computeMassAndInertia(wall_handle, 1.0)
        sim.setObjectPosition(wall_handle, -1, [v_x, v_y, 0.45])
        # res = sim.setObjectSpecialProperty(wall_handle, sim.objectspecialproperty_collidable)
        # sim.setObjectSpecialProperty(wall_handle, properties_objects)
        sim.setObjectInt32Param(wall_handle, sim.shapeintparam_respondable,1)
        # print(res)
        sim.resetDynamicObject(wall_handle)

    for (h_x, h_y) in cood_horz:
        wall_handle = sim.createPrimitiveShape(cuboid_type, wall_size_y)
        # res = sim.computeMassAndInertia(wall_handle, 1.0)
        # print(res)
        sim.setObjectPosition(wall_handle, -1, [h_x, h_y, 0.45])
        # sim.setObjectSpecialProperty(wall_handle, properties_objects)
        sim.setObjectInt32Param(wall_handle, sim.shapeintparam_respondable,1)

        # res = sim.setObjectSpecialProperty(wall_handle, sim.objectspecialproperty_collidable)
        # print(res)
        sim.resetDynamicObject(wall_handle)

    # for (n_x, n_y) in node_cord:
    #     wall_handle = sim.createPrimitiveShape(cuboid_type, [0.0, 0.0, 0.5])
    #     sim.setObjectPosition(wall_handle, -1, [n_x, n_y, 0.5])

    # spawn the 4 big walls
    wall_up_bottom = [5.2, 0.1, 1.0]
    wall_left_right = [0.1, 5.2, 1.0]

    #up 
    wall_handle = sim.createPrimitiveShape(cuboid_type, wall_up_bottom)
    sim.setObjectPosition(wall_handle, -1, [0.0, 2.5+0.05, 0.45])
    sim.setObjectInt32Param(wall_handle, sim.shapeintparam_respondable,1)
    sim.resetDynamicObject(wall_handle)

    #bottom
    wall_handle = sim.createPrimitiveShape(cuboid_type, wall_up_bottom)
    sim.setObjectPosition(wall_handle, -1, [0.0, -(2.5+0.05), 0.45])
    sim.setObjectInt32Param(wall_handle, sim.shapeintparam_respondable,1)
    sim.resetDynamicObject(wall_handle)

    #left
    wall_handle = sim.createPrimitiveShape(cuboid_type, wall_left_right)
    sim.setObjectPosition(wall_handle, -1, [-(2.5+0.05), 0.0 , 0.45])
    sim.setObjectInt32Param(wall_handle, sim.shapeintparam_respondable,1)
    sim.resetDynamicObject(wall_handle)

    #right
    wall_handle = sim.createPrimitiveShape(cuboid_type, wall_left_right)
    sim.setObjectPosition(wall_handle, -1, [(2.5+0.05), 0.0 , 0.45])
    sim.setObjectInt32Param(wall_handle, sim.shapeintparam_respondable,1)
    sim.resetDynamicObject(wall_handle)

    # end_point (provvisorio)
    wall_handle = sim.createPrimitiveShape(cuboid_type, [0.02, 0.02, 1.0])
    sim.setObjectPosition(wall_handle, -1, [end_point_x, end_point_y , 0.45])
    sim.setShapeColor(wall_handle, None, sim.colorcomponent_ambient_diffuse, [1.0, 0.0, 0.0])

    return nodes, node_cord, edges, (start_point_x, start_point_y), (end_point_x, end_point_y)


# if __name__ == '__main__':
#     # print('looking at', sys.path)
#     spawn_maze()
