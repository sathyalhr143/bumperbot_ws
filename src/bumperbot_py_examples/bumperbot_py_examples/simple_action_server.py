#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from bumperbot_msgs.action import Fibonacci



class SimpleActionServer(Node):
    def __init__(self):
        super().__init__('simple_action_server')

        self.action_server = ActionServer(self, Fibonacci, 'fibonacci', self.fibonacciCallback)
        self.get_logger().info('Simple Action Server has been started.')

    
    def fibonacciCallback(self, goal_handle):
        self.get_logger().info('Received goal request: n=%d' % goal_handle.request.order)
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Publishing feedback: %s' % str(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(2)

        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        self.get_logger().info('Goal succeeded with result: %s' % str(result.sequence))
        return result
    
def main():
        rclpy.init()
        action_server = SimpleActionServer()
        rclpy.spin(action_server)
        action_server.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
