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
from math import sqrt

dir_path1 = '/home/robotics23/dev_ws/src/assignment2'
dir_path2 = '/home/robotics23/dev_ws/src/assignment2/assignment2'
sys.path.append(dir_path1)
sys.path.append(dir_path2)
from maze_spawner import spawn_maze

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        *_, end_point = spawn_maze(size=50) 

        self.end_pose = (
            end_point[0],
            end_point[1], 
            0.0
        )
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.proximity_center_subscriber = self.create_subscription(Range, 'proximity/center', self.proximity_callback, 10)
        # self.proximity_center_left_subscriber = self.create_subscription(Range, 'proximity/center_left', self.proximity_callback_left, 10)
        # self.proximity_center_right_subscriber = self.create_subscription(Range, 'proximity/center_right', self.proximity_callback_right, 10)
        self.proximity_left_subscriber = self.create_subscription(Range, 'proximity/left', self.proximity_callback_left, 10)
        self.proximity_right_subscriber = self.create_subscription(Range, 'proximity/right', self.proximity_callback_right, 10)

        print('')

        # self.proximity_rear_left_subscriber = self.create_subscription(Range, 'proximity/rear_left',self.proximity_callback_back, 10)
        # self.proximity_rear_right_subscriber = self.create_subscription(Range, 'proximity/rear_right', self.proximity_callback_back, 10)
        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which Thymio should be controlled.
        self.info_stop = -1.0
        self.info_stop_left = -1.0
        self.info_stop_right = -1.0
        self.random_angle = None
        self.full_rotating = False
        self.move_thr = 0.995
        self.turning = False
        self.start_time = 0.0
        self.angle_turn = 0.0
        self.time_to_turn = 0.0
    

    def proximity_callback(self, message):
        distance  = message.range
        if distance >= 0 and distance < 0.2:
            self.info_stop = distance
            # self.get_logger().info('detrecting front')
        else:
            self.info_stop = -1.0

    def proximity_callback_left(self, message):
        distance  = message.range
        if distance >= 0 and distance < 0.3:
            self.info_stop_left = distance
            # self.get_logger().info('detrecting left')
        else:
            self.info_stop_left = -1.0

    def proximity_callback_right(self, message):
        distance  = message.range
        if distance >= 0 and distance < 0.3:
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
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
        
        self.get_logger().info(
            "odometry: received pose (x: {:.2f}, y: {:.2f}, theta: {:.2f})".format(*pose2d),
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
    
    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z,
            pose3.orientation.w
        )
        
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return pose2
    
    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose[0] - current_pose[0]), 2) +
                    pow((goal_pose[1] - current_pose[1]), 2))
    
    def update_callback(self):
        cmd_vel = Twist() 
        right_angle = -math.pi/6
        left_angle = math.pi/6
        small_right_angle = -math.pi/12
        small_left_angle = math.pi/12

        # if you reach the goal stop
        if self.odom_pose is not None:
            current_pose = self.pose3d_to_2d(self.odom_pose)
            if self.euclidean_distance(current_pose, self.end_pose) < 0.26:
                self.stop()
                return
            elif self.euclidean_distance(current_pose, self.end_pose) < 1.5:
                self.get_logger().info('distance to goal = {}'.format(self.euclidean_distance(current_pose, self.end_pose)))


        # robot has no right wall and no front wall
        if self.info_stop_right < 0 and (self.info_stop < 0):
            cmd_vel.linear.x = 0.3
            cmd_vel.angular.z = right_angle * 6
        elif self.info_stop_right > 0 and (self.info_stop < 0):
            cmd_vel.linear.x = 0.5
            # check for collision on the right 
            if self.info_stop_right < 0.1:
                cmd_vel.linear.x = 0.1
                cmd_vel.angular.z = small_left_angle
            elif self.info_stop_right > 0.15:
                cmd_vel.linear.x = 0.1
                cmd_vel.angular.z = small_right_angle
            else:
                cmd_vel.linear.x = 0.3
                cmd_vel.angular.z = 0.0
        elif self.info_stop_right > 0 and (self.info_stop > 0):
            # rotate to the left
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = left_angle
        elif self.info_stop > 0 and (self.info_stop_right < 0) and (self.info_stop_left < 0):
            # go left
            cmd_vel.linear.x = 0.1
            cmd_vel.angular.z = left_angle
        else:
            cmd_vel.linear.x = 0.3
            cmd_vel.angular.z = 0.0
        self.vel_publisher.publish(cmd_vel)

 
    
    def wall_on_the_right(self):
        return self.info_stop_right > 0

    def move_away_from_wall(self):
        # move away such that it detects only the right wall
        rotating_angle_right = math.pi/2
        if self.info_stop_left > 0:
            return math.pi/2
        elif self.info_stop > 0:
            return math.pi/2
        else:
            # check only if it too close to the wall
            if self.info_stop_right < 0.05:
                return math.pi/2     
        return 0.0
    

    def no_wall_encountered(self):
        return  (self.info_stop < 0) and (self.info_stop_left < 0) and (self.info_stop_right < 0)
         


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