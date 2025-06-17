#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import serial
import re
from math import sin, cos, pi
import numpy as np
import tf_transformations

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, TransformStamped, Vector3
from tf2_ros import TransformBroadcaster


class ArduinoInterface(Node):
    def __init__(self):
        super().__init__('arduino_interface')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('base_width', 0.3)  # Distance between wheels in meters
        self.declare_parameter('ticks_per_meter', 4000)  # Encoder ticks per meter
        
        # Get parameters
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.base_width = self.get_parameter('base_width').get_parameter_value().double_value
        self.ticks_per_meter = self.get_parameter('ticks_per_meter').get_parameter_value().integer_value
        
        # Set up publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)
        
        # Set up tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Initialize variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_enc_left = None
        self.last_enc_right = None
        self.last_time = self.get_clock().now()
        
        # Connect to Arduino
        try:
            self.serial = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            rclpy.shutdown()
            return
        
        # Regular expression pattern to match the telemetry data format
        self.pattern = r"H:\s*([-+]?\d+\.\d+)\s*R:\s*([-+]?\d+\.\d+)\s*P:\s*([-+]?\d+\.\d+)\s*VB:(\d+)mV/(\d+)mV\s*Cliff:(\d+)\s+(\d+)\s+(\d+)\s*Enc:(\s*[-+]?\d+)/(\s*[-+]?\d+)"
        
        # Create timer for reading serial data
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50Hz (20ms)
        
    def timer_callback(self):
        """Callback for reading serial data"""
        try:
            if self.serial.in_waiting:
                line = self.serial.readline().decode('utf-8').strip()
                self.process_line(line)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            self.timer.cancel()
        except UnicodeDecodeError:
            self.get_logger().warn("Failed to decode message from Arduino")
    
    def process_line(self, line):
        """Process a line of telemetry data from the Arduino"""
        match = re.search(self.pattern, line)
        if match:
            # Extract data
            heading = float(match.group(1))  # degrees
            roll = float(match.group(2))     # degrees
            pitch = float(match.group(3))    # degrees
            
            # Convert degrees to radians
            heading_rad = heading * pi / 180.0
            roll_rad = roll * pi / 180.0
            pitch_rad = pitch * pi / 180.0
            
            # Get encoder values
            enc_left = int(match.group(9))
            enc_right = int(match.group(10))
            
            # Process data
            self.publish_imu(heading_rad, pitch_rad, roll_rad)
            self.update_odometry(enc_left, enc_right)
        else:
            self.get_logger().warn(f"Failed to parse line: {line}")
    
    def publish_imu(self, heading, pitch, roll):
        """Publish IMU data to the imu topic"""
        current_time = self.get_clock().now()
        
        # Create IMU message
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = "imu_link"
        
        # Convert Euler angles to quaternion (ZYX convention)
        q = tf_transformations.quaternion_from_euler(roll, pitch, heading)
        imu_msg.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # We don't have covariance data, so use a high value
        imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
        
        # We don't have angular velocity from the Arduino, so just set to zero
        imu_msg.angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)
        imu_msg.angular_velocity_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]  # -1 indicates this data is not available
        
        # We don't have linear acceleration data from the Arduino, so just set to zero
        imu_msg.linear_acceleration = Vector3(x=0.0, y=0.0, z=0.0)
        imu_msg.linear_acceleration_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]  # -1 indicates this data is not available
        
        # Publish IMU message
        self.imu_pub.publish(imu_msg)
    
    def update_odometry(self, enc_left, enc_right):
        """Update odometry based on encoder values"""
        current_time = self.get_clock().now()
        
        # Skip first measurement
        if self.last_enc_left is None:
            self.last_enc_left = enc_left
            self.last_enc_right = enc_right
            self.last_time = current_time
            return
        
        # Calculate distance traveled by each wheel
        delta_enc_left = enc_left - self.last_enc_left
        delta_enc_right = enc_right - self.last_enc_right
        
        # Convert encoder ticks to distance
        dist_left = delta_enc_left / self.ticks_per_meter
        dist_right = delta_enc_right / self.ticks_per_meter
        
        # Calculate average distance and rotation
        dist = (dist_right + dist_left) / 2.0
        delta_theta = (dist_right - dist_left) / self.base_width
        
        # Time difference in seconds
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        # Skip if time difference is too small
        if dt < 0.001:
            return
        
        # Calculate velocities
        if abs(dt) > 0:
            vel_x = dist / dt
            vel_theta = delta_theta / dt
        else:
            vel_x = 0.0
            vel_theta = 0.0
        
        # Update pose
        if delta_theta == 0:
            # Moving in straight line
            self.x += dist * cos(self.theta)
            self.y += dist * sin(self.theta)
        else:
            # Moving in arc
            radius = dist / delta_theta
            self.x += radius * (sin(self.theta + delta_theta) - sin(self.theta))
            self.y -= radius * (cos(self.theta + delta_theta) - cos(self.theta))
        
        self.theta += delta_theta
        
        # Save current values for next iteration
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right
        self.last_time = current_time
        
        # Create quaternion from theta
        q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        
        # Broadcast transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # Set velocity
        odom.twist.twist.linear.x = vel_x
        odom.twist.twist.angular.z = vel_theta
        
        # Set covariance
        odom.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                               0, 0.1, 0, 0, 0, 0,
                               0, 0, 0.1, 0, 0, 0,
                               0, 0, 0, 0.1, 0, 0,
                               0, 0, 0, 0, 0.1, 0,
                               0, 0, 0, 0, 0, 0.1]
        
        odom.twist.covariance = [0.1, 0, 0, 0, 0, 0,
                                0, 0.1, 0, 0, 0, 0,
                                0, 0, 0.1, 0, 0, 0,
                                0, 0, 0, 0.1, 0, 0,
                                0, 0, 0, 0, 0.1, 0,
                                0, 0, 0, 0, 0, 0.1]
        
        # Publish odometry message
        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    
    arduino_interface = ArduinoInterface()
    
    try:
        rclpy.spin(arduino_interface)
    except KeyboardInterrupt:
        pass
    finally:
        # Close serial connection
        if hasattr(arduino_interface, 'serial'):
            arduino_interface.serial.close()
        arduino_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()