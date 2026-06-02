#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, atan2, sqrt, pi, fabs
import random
import time

random.seed(int(time.time()))

# from sensor_msgs.msg import Imu

def angle_diff(a, b):
    a = atan2(sin(a), cos(a))
    b = atan2(sin(b), cos(b))
    d1 = a - b
    d2 = 2 * pi - fabs(d1)
    if d1 > 0:
        d2 *= -1.0
    if fabs(d1) < fabs(d2):
        return d1
    else:
        return d2


class OdometryMotionModel(Node):

    def __init__(self):
        super().__init__("odometry_motion_model")
        self.is_first_odom_ = True
        self.last_odom_x= 0.0
        self.last_odom_y = 0.0
        self.last_odom_theta = 0.0
        
        self.declare_parameter("alpha1", 0.05)
        self.declare_parameter("alpha2", 0.1)
        self.declare_parameter("alpha3", 0.1)
        self.declare_parameter("alpha4", 0.1)
        self.declare_parameter("num_samples", 300)

        self.alpha1 = self.get_parameter("alpha1").get_parameter_value().double_value
        self.alpha2 = self.get_parameter("alpha2").get_parameter_value().double_value
        self.alpha3 = self.get_parameter("alpha3").get_parameter_value().double_value
        self.alpha4 = self.get_parameter("alpha4").get_parameter_value().double_value
        self.num_samples = self.get_parameter("num_samples").get_parameter_value().integer_value

        if self.num_samples >= 0:
            self.samples = PoseArray()
            self.samples.poses = [Pose() for _ in range(self.num_samples)]
        else:
            self.get_logger().fatal(f"Number of samples must be non-negative {self.num_samples}.")
            self.samples = None
            return

        self.odom_sub_ = self.create_subscription(Odometry, "bumperbot_controller/odom", self.odomCallback, 10)
        # self.imu_sub_ = self.create_subscription(Imu, "imu/out", self.imuCallback, 10)
        self.pose_array_pub_ = self.create_publisher(PoseArray, "odometry_motion_model/samples", 10)
        
        


    def odomCallback(self, odom):

        
        q = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, 
             odom.pose.pose.orientation.z, odom.pose.pose.orientation.w ]
        _,_, yaw= euler_from_quaternion(q)

        if self.is_first_odom_:
            self.last_odom_x = odom.pose.pose.position.x
            self.last_odom_y = odom.pose.pose.position.y
            self.last_odom_theta = yaw

            self.samples.header.frame_id = odom.header.frame_id
            # self.samples.header.stamp = odom.header.stamp
            self.is_first_odom_ = False
            return

        odom_x_increment = odom.pose.pose.position.x - self.last_odom_x
        odom_y_increment = odom.pose.pose.position.y - self.last_odom_y
        odom_theta_increment = angle_diff(yaw, self.last_odom_theta)
    
        
        if sqrt(odom_x_increment**2 + odom_y_increment**2) < 0.01:
            delta_rot1 = 0.0
        else:
            delta_rot1 = angle_diff(atan2(odom_y_increment, odom_x_increment), yaw) 
            
        delta_rot2 = angle_diff(odom_theta_increment, delta_rot1) - delta_rot1
        delta_transl = sqrt(odom_x_increment**2 + odom_y_increment**2)

        rot1_variance = self.alpha1 * delta_rot1 + self.alpha2 * delta_transl
        transl_variance = self.alpha3 * delta_transl + self.alpha4 * (delta_rot1 + delta_rot2)
        rot2_variance = self.alpha1 * delta_rot2 + self.alpha2 * delta_transl

        for sample in self.samples.poses:
            rot1_noise = random.gauss(0.0, rot1_variance)
            transl_noise = random.gauss(0.0, transl_variance)
            rot2_noise = random.gauss(0.0, rot2_variance)
        
            # noise free values
            delta_rot1_draw = angle_diff(delta_rot1, rot1_noise)
            delta_transl_draw = delta_transl - transl_noise
            delta_rot2_draw = angle_diff(delta_rot2, rot2_noise)

            sample_q = [sample.orientation.x, sample.orientation.y, 
                        sample.orientation.z, sample.orientation.w]
            sample_roll, sample_pitch, sample_yaw = euler_from_quaternion(sample_q)

            sample.position.x += delta_transl_draw * cos(sample_yaw + delta_rot1_draw)
            sample.position.y += delta_transl_draw * sin(sample_yaw + delta_rot1_draw)
            q = quaternion_from_euler(0.0, 0.0, sample_yaw + delta_rot1_draw + delta_rot2_draw)
            sample.orientation.x, sample.orientation.y, sample.orientation.z, sample.orientation.w = q

        self.last_odom_x = odom.pose.pose.position.x
        self.last_odom_y = odom.pose.pose.position.y
        self.last_odom_theta = yaw
        

        self.pose_array_pub_.publish(self.samples)



def main():
    rclpy.init()

    odometry_motion_model = OdometryMotionModel()
    rclpy.spin(odometry_motion_model)
    
    odometry_motion_model.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()