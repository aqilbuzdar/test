#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial, math

G = 9.80665

class MPU6050Node(Node):
    def __init__(self):
        super().__init__('mpu6050_node')
        port = self.declare_parameter('port', '/dev/arduino').get_parameter_value().string_value
        baud = self.declare_parameter('baud', 115200).get_parameter_value().integer_value
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').get_parameter_value().string_value

        self.pub = self.create_publisher(Imu, '/imu/data_raw', 50)
        try:
            self.ser = serial.Serial(port, baud, timeout=0.05)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"MPU6050 node started on {port}")
        except Exception as e:
            self.get_logger().error(f"Cannot open {port}: {e}")
            raise

        self.timer = self.create_timer(0.01, self.loop)  # 100 Hz

    def loop(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return
            # Resync on 'IMU,'
            idx = line.find('IMU,')
            if idx == -1:
                return
            payload = line[idx+4:]  # after 'IMU,'
            parts = payload.split(',')
            if len(parts) < 6:
                return

            ax, ay, az, gx, gy, gz = map(float, parts[:6])

            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id

            # Scale raw -> SI
            msg.linear_acceleration.x = (ax / 16384.0) * G
            msg.linear_acceleration.y = (ay / 16384.0) * G
            msg.linear_acceleration.z = (az / 16384.0) * G

            msg.angular_velocity.x = (gx / 131.0) * math.pi / 180.0
            msg.angular_velocity.y = (gy / 131.0) * math.pi / 180.0
            msg.angular_velocity.z = (gz / 131.0) * math.pi / 180.0

            # Orientation unknown -> -1 per REP-145
            msg.orientation_covariance[0] = -1.0
            # Reasonable small covariances (tune later)
            for i in range(9):
                msg.angular_velocity_covariance[i] = 0.02 if (i % 4 == 0) else 0.0
                msg.linear_acceleration_covariance[i] = 0.04 if (i % 4 == 0) else 0.0

            self.pub.publish(msg)
        except Exception as e:
            # swallow parse glitches
            self.get_logger().debug(f"parse error: {e}")

def main():
    rclpy.init()
    rclpy.spin(MPU6050Node())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
