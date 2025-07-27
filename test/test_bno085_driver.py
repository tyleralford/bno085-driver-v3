#!/usr/bin/env python3
"""
Test script for BNO085 driver package.

This script tests the basic functionality of the BNO085 driver package
without requiring hardware connection.
"""

import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import time
import threading


class TestBNO085Driver(unittest.TestCase):
    """Test cases for BNO085 driver functionality."""
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS 2 for testing."""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS 2 after testing."""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up test fixtures."""
        self.node = Node('test_bno085_driver')
        self.imu_messages = []
        self.calibration_messages = []
        
        # Create subscriptions
        self.imu_subscription = self.node.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        self.calibration_subscription = self.node.create_subscription(
            String,
            '/imu/calibration_status',
            self.calibration_callback,
            10
        )
        
        # Start spinning in a separate thread
        self.spin_thread = threading.Thread(target=self.spin_node)
        self.spin_thread.daemon = True
        self.spin_thread.start()
    
    def tearDown(self):
        """Clean up test fixtures."""
        self.node.destroy_node()
    
    def spin_node(self):
        """Spin the node in a separate thread."""
        rclpy.spin(self.node)
    
    def imu_callback(self, msg):
        """Store received IMU messages."""
        self.imu_messages.append(msg)
    
    def calibration_callback(self, msg):
        """Store received calibration messages."""
        self.calibration_messages.append(msg)
    
    def test_package_import(self):
        """Test that the package can be imported."""
        try:
            import bno085_driver
            self.assertTrue(True, "Package imported successfully")
        except ImportError as e:
            self.fail(f"Failed to import package: {e}")
    
    def test_imu_message_structure(self):
        """Test IMU message structure validation."""
        # Create a mock IMU message
        imu_msg = Imu()
        
        # Check required fields exist
        self.assertTrue(hasattr(imu_msg, 'header'))
        self.assertTrue(hasattr(imu_msg, 'orientation'))
        self.assertTrue(hasattr(imu_msg, 'angular_velocity'))
        self.assertTrue(hasattr(imu_msg, 'linear_acceleration'))
        self.assertTrue(hasattr(imu_msg, 'orientation_covariance'))
        self.assertTrue(hasattr(imu_msg, 'angular_velocity_covariance'))
        self.assertTrue(hasattr(imu_msg, 'linear_acceleration_covariance'))


if __name__ == '__main__':
    unittest.main()
