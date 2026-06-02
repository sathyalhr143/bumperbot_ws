#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial


class SimpleSerialTransmitter(Node):

    def __init__(self):
        super().__init__("simple_serial_transmitter")
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.arduino = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.1)
        self.sub_ = self.create_subscription(String, "serial_transmitter", self.msgCallback, 10)
        self.sub_

    def msgCallback(self, msg):
        self.arduino.write(msg.data.encode('utf-8'))


def main():
    rclpy.init()

    simple_serial_transmitter = SimpleSerialTransmitter()
    rclpy.spin(simple_serial_transmitter)

    simple_serial_transmitter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()