#!/usr/bin/env python3

"""
BNO085 IMU Data Subscriber Example

This script demonstrates how to subscribe to and process IMU data
from the BNO085 driver.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import math

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__('imu_subscriber_example')
        
        # Subscribe to IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Statistics tracking
        self.msg_count = 0
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('IMU Subscriber started. Listening for IMU data...')
    
    def imu_callback(self, msg):
        """Process incoming IMU messages"""
        self.msg_count += 1
        
        # Calculate publishing rate every 100 messages
        if self.msg_count % 100 == 0:
            current_time = self.get_clock().now()
            duration = (current_time - self.last_time).nanoseconds / 1e9
            rate = 100.0 / duration
            self.get_logger().info(f'Publishing rate: {rate:.1f} Hz')
            self.last_time = current_time
        
        # Extract quaternion and convert to Euler angles
        quat = msg.orientation
        roll, pitch, yaw = self.quaternion_to_euler(quat.x, quat.y, quat.z, quat.w)
        
        # Extract angular velocity
        gyro = msg.angular_velocity
        
        # Extract linear acceleration
        accel = msg.linear_acceleration
        
        # Calculate calibration quality from covariance (lower = better)
        orient_cov = msg.orientation_covariance[0]  # Diagonal element
        gyro_cov = msg.angular_velocity_covariance[0]
        accel_cov = msg.linear_acceleration_covariance[0]
        
        # Log data every 60 messages (approximately once per second at 60Hz)
        if self.msg_count % 60 == 0:
            self.get_logger().info(
                f'IMU Data:\n'
                f'  Orientation (RPY): {math.degrees(roll):.1f}°, {math.degrees(pitch):.1f}°, {math.degrees(yaw):.1f}°\n'
                f'  Angular Velocity: {gyro.x:.3f}, {gyro.y:.3f}, {gyro.z:.3f} rad/s\n'
                f'  Linear Acceleration: {accel.x:.3f}, {accel.y:.3f}, {accel.z:.3f} m/s²\n'
                f'  Covariance (Orient/Gyro/Accel): {orient_cov:.4f}/{gyro_cov:.4f}/{accel_cov:.4f}'
            )
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw

def main(args=None):
    rclpy.init(args=args)
    
    imu_subscriber = ImuSubscriber()
    
    try:
        rclpy.spin(imu_subscriber)
    except KeyboardInterrupt:
        pass
    
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
