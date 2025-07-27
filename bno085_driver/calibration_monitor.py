#!/usr/bin/env python3
"""
BNO085 IMU Driver utilities and helper functions.

This module provides utility functions for working with the BNO085 IMU driver,
including calibration helpers, data conversion utilities, and diagnostic tools.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import json
import time
from typing import Dict, Any, Optional


class IMUCalibrationMonitor(Node):
    """Monitor and display IMU calibration status in real-time."""
    
    def __init__(self):
        super().__init__('imu_calibration_monitor')
        
        # Subscribe to IMU data
        self.imu_subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        # Subscribe to calibration status if available
        self.calibration_subscription = self.create_subscription(
            String,
            '/imu/calibration_status',
            self.calibration_callback,
            10
        )
        
        # Create timer for periodic status display
        self.timer = self.create_timer(2.0, self.display_status)
        
        self.last_imu_msg = None
        self.last_calibration_msg = None
        self.start_time = time.time()
        
        self.get_logger().info('IMU Calibration Monitor started')
    
    def imu_callback(self, msg: Imu):
        """Handle incoming IMU messages."""
        self.last_imu_msg = msg
    
    def calibration_callback(self, msg: String):
        """Handle incoming calibration status messages."""
        self.last_calibration_msg = msg
    
    def display_status(self):
        """Display current IMU and calibration status."""
        runtime = time.time() - self.start_time
        
        print(f"\n{'='*60}")
        print(f"IMU Status - Runtime: {runtime:.1f}s")
        print(f"{'='*60}")
        
        if self.last_imu_msg is not None:
            imu = self.last_imu_msg
            
            print(f"Orientation (quat): x={imu.orientation.x:.3f}, "
                  f"y={imu.orientation.y:.3f}, z={imu.orientation.z:.3f}, "
                  f"w={imu.orientation.w:.3f}")
            
            print(f"Angular Velocity: x={imu.angular_velocity.x:.3f}, "
                  f"y={imu.angular_velocity.y:.3f}, z={imu.angular_velocity.z:.3f}")
            
            print(f"Linear Acceleration: x={imu.linear_acceleration.x:.3f}, "
                  f"y={imu.linear_acceleration.y:.3f}, z={imu.linear_acceleration.z:.3f}")
            
            # Display covariance diagonal (uncertainty)
            orient_cov = imu.orientation_covariance[0]
            angvel_cov = imu.angular_velocity_covariance[0]
            accel_cov = imu.linear_acceleration_covariance[0]
            
            print(f"Covariance: Orient={orient_cov:.4f}, "
                  f"AngVel={angvel_cov:.4f}, Accel={accel_cov:.4f}")
        else:
            print("No IMU data received yet...")
        
        if self.last_calibration_msg is not None:
            try:
                cal_data = json.loads(self.last_calibration_msg.data)
                print(f"Calibration Status: {cal_data}")
            except json.JSONDecodeError:
                print(f"Calibration Status: {self.last_calibration_msg.data}")
        
        print(f"{'='*60}")


def main(args=None):
    """Main entry point for the calibration monitor."""
    rclpy.init(args=args)
    
    try:
        monitor = IMUCalibrationMonitor()
        print("Starting IMU Calibration Monitor...")
        print("Press Ctrl+C to stop")
        
        rclpy.spin(monitor)
    
    except KeyboardInterrupt:
        print("\nShutting down calibration monitor...")
    
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
