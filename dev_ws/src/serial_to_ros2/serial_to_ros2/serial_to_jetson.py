import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import serial
import math

class SerialToROS2(Node):
    def __init__(self):
        super().__init__('serial_to_ros2')
        # Change serial port if needed
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

        # Publishers
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer to read serial at ~50Hz (20ms)
        self.timer = self.create_timer(0.02, self.read_serial)

        # Odometry state
        self.prev_encL = None
        self.prev_encR = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Yaw angle in radians

        # Robot parameters — update to match your hardware
        self.wheel_base = 0.3  # meters
        self.encoder_ticks_per_rev = 2048
        self.wheel_diameter = 0.065  # meters
        self.ticks_to_distance = (math.pi * self.wheel_diameter) / self.encoder_ticks_per_rev

    def read_serial(self):
        try:
            line = self.ser.readline().decode('ascii').strip()
            if not line:
                return
            # Example line format:
            # H:123.45 R:-0.12 P:1.23  VB:12000mV/11950mV   Cliff:123 124 125  Enc: 123456789/987654321

            # Parse IMU Euler angles (heading, roll, pitch)
            h_index = line.find('H:')
            r_index = line.find('R:')
            p_index = line.find('P:')
            enc_index = line.find('Enc:')

            if -1 in [h_index, r_index, p_index, enc_index]:
                self.get_logger().warn(f"Malformed line: {line}")
                return

            # Extract Euler angles as strings
            h_str = line[h_index+2 : r_index].strip()
            r_str = line[r_index+2 : p_index].strip()
            p_str = line[p_index+2 : line.find('VB:')].strip()

            heading = float(h_str)
            roll = float(r_str)
            pitch = float(p_str)

            # Extract encoder counts
            enc_str = line[enc_index+4:].strip()
            encL_str, encR_str = enc_str.split('/')
            encL = int(encL_str.strip())
            encR = int(encR_str.strip())

            # Publish IMU message
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            # Convert Euler angles (degrees) to radians
            roll_rad = math.radians(roll)
            pitch_rad = math.radians(pitch)
            yaw_rad = math.radians(heading)

            # Convert Euler to quaternion
            qx, qy, qz, qw = self.euler_to_quaternion(roll_rad, pitch_rad, yaw_rad)
            imu_msg.orientation.x = qx
            imu_msg.orientation.y = qy
            imu_msg.orientation.z = qz
            imu_msg.orientation.w = qw

            # Leave angular_velocity and linear_acceleration zero for now
            self.imu_pub.publish(imu_msg)

            # Odometry calculation
            if self.prev_encL is not None and self.prev_encR is not None:
                dL = (encL - self.prev_encL) * self.ticks_to_distance
                dR = (encR - self.prev_encR) * self.ticks_to_distance
                d_center = (dL + dR) / 2.0
                d_theta = (dR - dL) / self.wheel_base

                self.theta += d_theta
                self.x += d_center * math.cos(self.theta)
                self.y += d_center * math.sin(self.theta)

                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = 'odom'
                odom_msg.child_frame_id = 'base_link'

                odom_msg.pose.pose.position.x = self.x
                odom_msg.pose.pose.position.y = self.y
                odom_msg.pose.pose.position.z = 0.0

                qx_odom, qy_odom, qz_odom, qw_odom = self.euler_to_quaternion(0, 0, self.theta)
                odom_msg.pose.pose.orientation.x = qx_odom
                odom_msg.pose.pose.orientation.y = qy_odom
                odom_msg.pose.pose.orientation.z = qz_odom
                odom_msg.pose.pose.orientation.w = qw_odom

                # Twist can be zero for now
                self.odom_pub.publish(odom_msg)

            self.prev_encL = encL
            self.prev_encR = encR

        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
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
        return qx, qy, qz, qw

def main(args=None):
    rclpy.init(args=args)
    node = SerialToROS2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
