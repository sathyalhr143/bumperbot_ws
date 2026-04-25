#!/usr/bin/env python3

import rclpy
import math
from enum import Enum   
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class State(Enum):
    FREE = 0
    WARNING = 1
    DANGER = 2

class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop_node")

        self.declare_parameter('danger_distance', 0.2)
        self.declare_parameter('scan_topic', 'scan')
        self.declare_parameter('safety_stop_topic', 'safety_stop')
        self.danger_distance = self.get_parameter('danger_distance').get_parameter_value().double_value
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter('safety_stop_topic').get_parameter_value().string_value

        self.laser_subscriber = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.laserCallback,
            10
            )
        self.safety_stop_publisher = self.create_publisher(
            Bool,
            self.safety_stop_topic,
            10
            )
        
        self.state = State.FREE
        
    def laserCallback(self, msg: LaserScan):
        self.state = State.FREE

        for range_value in msg.ranges:
            if not math.isinf(range_value) and range_value <= self.danger_distance:
                self.state = State.DANGER
                break
        
        is_safety_stop = Bool()
        if self.state == State.DANGER:
            is_safety_stop.data = True
        elif self.state == State.FREE:
            is_safety_stop.data = False
        
        self.safety_stop_publisher.publish(is_safety_stop)


def main():
    rclpy.init()
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()