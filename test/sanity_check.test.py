#!/usr/bin/env python3
"""
Sanity check test template
Tests basic ROS2 infrastructure and component availability
"""

import unittest
import rclpy
from rclpy.node import Node


class SanityCheckTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_rclpy_init(self):
        """Test that rclpy initializes correctly"""
        self.assertTrue(rclpy.ok())

    def test_create_node(self):
        """Test that we can create a basic node"""
        node = Node('test_node')
        self.assertIsNotNone(node)
        node.destroy_node()

    # TODO: Add tests for:
    # - Topic availability
    # - Service availability
    # - Transform tree validity
    # - Parameter server accessibility


if __name__ == '__main__':
    unittest.main()
