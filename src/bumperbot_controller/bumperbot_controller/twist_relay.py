#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistRelay(Node):
    def __init__(self):
        super().__init__('twist_relay')
        self.controller_subscriber = self.create_subscription(
            Twist, 
            "/bumperbot_controller/cmd_vel_unstamped", 
            self.controllerTwistCallback, 10)
        
        self.controller_publisher = self.create_publisher(TwistStamped, 
                                                          '/bumperbot_controller/cmd_vel', 
                                                          10)

        self.joy_subscriber = self.create_subscription(
            TwistStamped,
            "/input_joy/cmd_vel_stamped",
            self.joyTwistCallback, 10)
        
        self.joy_publisher = self.create_publisher(Twist, 
                                                  'joy_vel', 
                                                  10)
        

        self.key_subscriber = self.create_subscription(
            TwistStamped,
            "/key_vel",
            self.keyCallback, 10)
        
        self.key_publisher = self.create_publisher(Twist, 
                                                  '/key_vel_unstamped', 
                                                  10)
        
    def keyCallback(self, msg):
        twist_msg = msg.twist
        self.key_publisher.publish(twist_msg)

        

    def controllerTwistCallback(self, msg):
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.frame_id = "base_footprint"
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.twist = msg
        self.controller_publisher.publish(twist_stamped_msg)

    def joyTwistCallback(self, msg):
        # twist_msg = Twist()
        twist_msg = msg.twist
        self.joy_publisher.publish(twist_msg)

def main():
    rclpy.init()
    node = TwistRelay()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Node crashed with error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()