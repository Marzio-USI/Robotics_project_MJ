import rclpy
from rclpy.node import Node
import tf_transformations

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import time
import sys
from sensor_msgs.msg import Range
import math
from math import pow, sin, cos, atan2, sqrt
from copy import deepcopy
class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        
        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

        self.proximity_center_subscriber = self.create_subscription(Range, 'proximity/center', self.proximity_callback, 10)
        self.proximity_center_left_subscriber = self.create_subscription(Range, 'proximity/center_left', self.proximity_callback, 10)
        self.proximity_center_right_subscriber = self.create_subscription(Range, 'proximity/center_right', self.proximity_callback, 10)

        self.proximity_rear_left_subscriber = self.create_subscription(Range, 'proximity/rear_left',self.proximity_callback_back, 10)
        self.proximity_rear_right_subscriber = self.create_subscription(Range, 'proximity/rear_right', self.proximity_callback_back, 10)
        
        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which Thymio should be controlled.
        self.stopping = False
        self.left = None
        self.right = None
        self.start_moving_away = False
        self.rotating = True

        self.real_distance = 0
        self.init_angle = None
        self.left_back = None
        self.right_back = None
        self.back_tollerance = 0.0005
        self.rotating_back = True
        self.start_pose = None
        self.done = False
        

    def proximity_callback(self, message):
        # "sensorthymio0/proximity_center_right_link"
        sensor = message.header.frame_id
        # print('header', message.header )
        distance  = message.range
        if distance >= 0 and distance < 0.05:
            # print(f'robot is stopping, sensor{sensor}')
            self.stopping = True
            if 'left' in sensor:
                self.left = distance
            elif 'right' in sensor :
                self.right = distance
            else:
                self.real_distance = distance

    def proximity_callback_back(self, message):
        # "sensorthymio0/proximity_center_right_link"
        sensor = message.header.frame_id
        # self.get_logger().info(f'header{message.range}')
        
        distance  = message.range
        
            # print(f'robot is stopping, sensor{sensor}')
        if 'left' in sensor:
            self.left_back = distance
        elif 'right' in sensor :
            self.right_back = distance
                



        
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
        
    def update_callback(self):
        # Let's just set some hard-coded velocities in this example
        
        cmd_vel = Twist() 
        if self.start_moving_away:
            if self.rotating_back:
                # self.get_logger().info(f'left {self.left_back}, right {self.right_back}')
                cmd_vel.linear.x  = 0.0
                if (self.left_back > 0) and (self.right_back > 0):
                    if (abs(self.left_back - self.right_back) < self.back_tollerance):
                        self.rotating_back = False
                        cmd_vel.angular.z = 0.0
                    elif self.left_back < self.right_back:
                        cmd_vel.angular.z = -math.pi/10
                    else:
                        cmd_vel.angular.z = math.pi/10
                elif self.left_back > 0:
                    cmd_vel.angular.z =  -math.pi/8
                else:
                    cmd_vel.angular.z =  math.pi/8
            else:
                if self.start_pose is None:
                    self.start_pose = self.pose3d_to_2d(self.odom_pose)
                current_pose = self.pose3d_to_2d(self.odom_pose)
                if self.euclidean_distance(self.start_pose, current_pose) < (2 - self.real_distance):
                    cmd_vel.linear.x  = 0.2
                    cmd_vel.angular.z = 0.0
                else:
                    cmd_vel.linear.x  = 0.0
                    cmd_vel.angular.z = 0.0
                    self.done = True
                          
        elif self.stopping:
            cmd_vel.linear.x  = 0.0
            if not self.start_moving_away:
                cmd_vel.angular.z = self.rotate()
        else:
            cmd_vel.linear.x  = 0.2 # [m/s]
            cmd_vel.angular.z = 0.0 # [rad/s]
        
        # Publish the command
        self.vel_publisher.publish(cmd_vel)

    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_pose[0] - current_pose[0]), 2) +
                    pow((goal_pose[1] - current_pose[1]), 2))

    def rotate(self):
        """
        rotate the robot in order to face the wall orthogonally
        """
        rotate_tollerance = 0.0004
        angle : float = 0.0
        if self.left is None:
            " rotate to the right"
            angle = -(math.pi/8)
        elif self.right is None:
            "rotate to the left"
            angle =  (math.pi/8)
        elif self.left is None and self.right is None:
            "No rotate you are already orgonally"
            angle = 0.0            
            self.start_moving_away = True
        else:
            diff = abs(self.left - self.right)
            # print(f'difference {diff}')
            if diff < rotate_tollerance:
                " stop rotating"
                angle = 0.0
                self.start_moving_away = True
            else:
                "rotate"
                if self.left > self.right:
                    "rotate right"
                    angle = -(math.pi/10)
                else:
                    "rotate left"
                    angle = (math.pi/10)
        return angle



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