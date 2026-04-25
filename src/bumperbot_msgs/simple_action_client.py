#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from bumperbot_msgs.action import Fibonacci

class SimpleActionClient(Node):
    def __init__(self):
        super().__init__('simple_action_client')
        self.action_client = ActionClient(self, Fibonacci, 'fibonacci')
        self.get_logger().info('Simple Action Client has been started.')

        self.action_client.wait_for_server()

        self.goal.order = 11

        self.future = self.action_client.send_goal_async(self.goal, feedback_callback=self.feedbackCallback)
        self.future.add_done_callback(self.responseCallback)
        
    def responseCallback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            return
        self.get_logger().info('Goal accepted. Waiting for result...')
        self.future = goal_handle.get_result_async()
        self.future.add_done_callback(self.resultCallback)

    def resultCallback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result received: {result.sequence}')
        rclpy.shutdown()

    def feedbackCallback(self, feedback_msg):
        self.get_logger().info(f'Feedback received: {feedback_msg.feedback.partial_sequence}')
    
def main():
        rclpy.init()
        action_client = SimpleActionClient()
        rclpy.spin(action_client)
        action_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()