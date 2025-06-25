import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros.transform_broadcaster import TransformBroadcaster

import numpy as np
import math


def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles to quaternion"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return [qx, qy, qz, qw]


class MobileDriver:
    def init(self, webots_node, properties):
        rclpy.init(args=None)
        
        self.__node = Node("mobile_driver")
        self.tfb_ = TransformBroadcaster(self.__node)
        
        self.__node.get_logger().info("Init MobileDriver")
        
        # Robot parameters
        self.wheel_separation = float(properties.get("wheel_separation", "0.6"))
        self.wheel_radius = float(properties.get("wheel_radius", "0.1"))
        
        self.__robot = webots_node.robot
        self.mobile_robot_node = self.__robot.getFromDef("MobileManipulator")
        
        if self.mobile_robot_node:
            self.robot_translation = self.mobile_robot_node.getField("translation")
            self.robot_rotation = self.mobile_robot_node.getField("rotation")
            self.robot_translation_initial = self.robot_translation.getSFVec3f()
            self.robot_rotation_initial = self.robot_rotation.getSFRotation()
        else:
            self.__node.get_logger().warn("MobileManipulator node not found in Webots")
            # Set default values
            self.robot_translation_initial = [0.0, 0.0, 0.0]
            self.robot_rotation_initial = [0.0, 0.0, 1.0, 0.0]
        
        self.__robot.timestep = 32
        
        # Initialize wheel motors
        try:
            self.left_motor = self.__robot.getDevice("left wheel motor")
            self.right_motor = self.__robot.getDevice("right wheel motor")
            
            # Set motors to velocity control mode
            self.left_motor.setPosition(float('inf'))
            self.right_motor.setPosition(float('inf'))
            self.left_motor.setVelocity(0.0)
            self.right_motor.setVelocity(0.0)
        except Exception as e:
            self.__node.get_logger().error(f"Failed to initialize wheel motors: {e}")
            self.left_motor = None
            self.right_motor = None
        
        # Initialize position sensors for wheels
        try:
            self.left_sensor = self.__robot.getDevice("left wheel sensor")
            self.right_sensor = self.__robot.getDevice("right wheel sensor")
            
            if self.left_sensor and self.right_sensor:
                self.left_sensor.enable(self.__robot.timestep)
                self.right_sensor.enable(self.__robot.timestep)
        except Exception as e:
            self.__node.get_logger().warn(f"Failed to initialize wheel sensors: {e}")
            self.left_sensor = None
            self.right_sensor = None
        
        # ROS2 subscriptions
        self.__node.create_subscription(
            Twist, "/cmd_vel", self.__cmd_vel_callback, 1
        )
        
        # ROS2 publishers
        self.joint_state_pub = self.__node.create_publisher(
            JointState, "/joint_states", 1
        )
        self.odom_pub = self.__node.create_publisher(
            Odometry, "/odom", 1
        )
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.prev_left_wheel_pos = 0.0
        self.prev_right_wheel_pos = 0.0
        self.prev_time = self.__robot.getTime()
        
        # Current velocities
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        
        self.__node.get_logger().info("MobileDriver initialized successfully")
    
    def __cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Convert to wheel velocities using differential drive kinematics
        left_wheel_vel = (linear_vel - angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        right_wheel_vel = (linear_vel + angular_vel * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Apply velocity limits
        max_wheel_vel = 10.0  # rad/s
        left_wheel_vel = max(min(left_wheel_vel, max_wheel_vel), -max_wheel_vel)
        right_wheel_vel = max(min(right_wheel_vel, max_wheel_vel), -max_wheel_vel)
        
        # Set motor velocities
        if self.left_motor and self.right_motor:
            self.left_motor.setVelocity(left_wheel_vel)
            self.right_motor.setVelocity(right_wheel_vel)
        
        # Store current command for odometry
        self.linear_velocity = linear_vel
        self.angular_velocity = angular_vel
    
    def update_odometry(self):
        """Update robot odometry based on wheel encoders"""
        if not (self.left_sensor and self.right_sensor):
            return
        
        current_time = self.__robot.getTime()
        dt = current_time - self.prev_time
        
        if dt <= 0:
            return
        
        # Get wheel positions
        try:
            self.left_wheel_pos = self.left_sensor.getValue()
            self.right_wheel_pos = self.right_sensor.getValue()
        except:
            return
        
        # Calculate wheel displacement
        left_wheel_delta = self.left_wheel_pos - self.prev_left_wheel_pos
        right_wheel_delta = self.right_wheel_pos - self.prev_right_wheel_pos
        
        # Calculate robot displacement
        left_distance = left_wheel_delta * self.wheel_radius
        right_distance = right_wheel_delta * self.wheel_radius
        
        center_distance = (left_distance + right_distance) / 2.0
        angle_change = (right_distance - left_distance) / self.wheel_separation
        
        # Update robot pose
        if abs(angle_change) < 1e-6:
            # Straight line motion
            self.x += center_distance * math.cos(self.theta)
            self.y += center_distance * math.sin(self.theta)
        else:
            # Curved motion
            radius = center_distance / angle_change
            self.x += radius * (math.sin(self.theta + angle_change) - math.sin(self.theta))
            self.y += radius * (math.cos(self.theta) - math.cos(self.theta + angle_change))
        
        self.theta += angle_change
        # Normalize angle
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # Update previous values
        self.prev_left_wheel_pos = self.left_wheel_pos
        self.prev_right_wheel_pos = self.right_wheel_pos
        self.prev_time = current_time
    
    def publish_transforms_and_odometry(self):
        """Publish TF transforms and odometry"""
        current_time = self.__robot.getTime()
        time_stamp = Time()
        time_stamp.sec = int(current_time)
        time_stamp.nanosec = int((current_time % 1) * 1e9)
        
        # Publish base_link transform
        tf = TransformStamped()
        tf.header.stamp = time_stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.1  # Height of base_link above ground
        
        quat = quaternion_from_euler(0, 0, self.theta)
        tf.transform.rotation.x = quat[0]
        tf.transform.rotation.y = quat[1]
        tf.transform.rotation.z = quat[2]
        tf.transform.rotation.w = quat[3]
        
        self.tfb_.sendTransform([tf])
        
        # Publish odometry
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.header.stamp = time_stamp
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]
        
        # Set velocity
        odom.twist.twist.linear.x = self.linear_velocity
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.angular_velocity
        
        self.odom_pub.publish(odom)
    
    def publish_joint_states(self):
        """Publish joint states for wheels"""
        current_time = self.__robot.getTime()
        time_stamp = Time()
        time_stamp.sec = int(current_time)
        time_stamp.nanosec = int((current_time % 1) * 1e9)
        
        joint_state = JointState()
        joint_state.header.stamp = time_stamp
        joint_state.name = ["left_wheel_joint", "right_wheel_joint"]
        joint_state.position = [self.left_wheel_pos, self.right_wheel_pos]
        joint_state.velocity = [0.0, 0.0]  # Could calculate from motor commands
        joint_state.effort = [0.0, 0.0]
        
        self.joint_state_pub.publish(joint_state)
    
    def step(self):
        """Main control loop step"""
        rclpy.spin_once(self.__node, timeout_sec=0)
        
        # Update odometry
        self.update_odometry()
        
        # Publish transforms and odometry
        self.publish_transforms_and_odometry()
        
        # Publish joint states
        self.publish_joint_states()


def main():
    """Main function for testing"""
    rclpy.init()
    node = Node("mobile_driver_test")
    node.get_logger().info("Mobile driver test node started")
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()