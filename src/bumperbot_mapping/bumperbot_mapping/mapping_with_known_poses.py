#!/usr/bin/env python3
import rclpy
from rclpy.time import Time
import math
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf_transformations import euler_from_quaternion

# global variables for probability
PRIOR_PROB = 0.5
OCCUPIED_PROB = 0.9
FREE_PROB = 0.35



class Pose:
    def __init__(self, px = 0, py = 0):
        self.x = px
        self.y = py

def coordinatesToPose(px, py, map_info: MapMetaData):
    pose = Pose()
    pose.x = round((px - map_info.origin.position.x) / map_info.resolution)
    pose.y = round((py - map_info.origin.position.y) / map_info.resolution)
    return pose

def poseOnMap(pose: Pose, map_info: MapMetaData):
    return pose.x < map_info.width and pose.x >= 0 and pose.y < map_info.height and pose.y >= 0

def poseToCell(pose: Pose, map_info: MapMetaData):
    return map_info.width * pose.y + pose.x


def bresenham(start: Pose, end: Pose):
    line = []
    dx = end.x - start.x
    dy = end.y - start.y
    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1
    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx = xsign
        xy = 0
        yx = 0
        yy = ysign
    else:
        tmp = dx
        dx = dy
        dy = tmp
        xx = 0
        xy = ysign
        yx = xsign
        yy = 0

    D = 2 * dy - dx
    y = 0

    for i in range(dx + 1):
        line.append(Pose(start.x + i * xx + y * yx, start.y + i * xy + y * yy))
        if D >= 0:
            y += 1
            D -= 2 * dx
        D += 2 * dy

    return line

def inverseSensorModel(robot_p: Pose, sensor_p: Pose):
    occ_values = []
    line = bresenham(robot_p, sensor_p)
    
    for pose in line[:-1]:
        occ_values.append((pose, FREE_PROB))
    occ_values.append((line[-1], OCCUPIED_PROB))
    return occ_values

def prob2LogOdds(p):
    return math.log(p / (1 - p))
    

def logOdds2Prob(l):
    try:
        return 1 - (1 / (1 + math.exp(l)))
    except OverflowError:
        return 1.0 if l > 0 else 0.0

prior_log_odds = prob2LogOdds(PRIOR_PROB)
occupied_log_odds = prob2LogOdds(OCCUPIED_PROB)
free_log_odds = prob2LogOdds(FREE_PROB)

class MappingWithKnownPoses(Node):
    def __init__(self, name):
        super().__init__(name)

        self.declare_parameter('width', 50.0)
        self.declare_parameter('height', 50.0)
        self.declare_parameter('resolution', 0.1)
        # self.declare_parameter('use_sim_time', True)

        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        resolution = self.get_parameter('resolution').value

        self.map_ = OccupancyGrid()

        self.map_.info.resolution = resolution
        self.map_.info.width = round(width / resolution)
        self.map_.info.height = round(height / resolution)
        self.map_.info.origin.position.x = float(-round(width / 2.0))
        self.map_.info.origin.position.y = float(-round(height / 2.0))
        self.map_.header.frame_id = 'odom'
        self.map_.data = [-1] * (self.map_.info.width * self.map_.info.height)

        self.probalility_map_ = [prior_log_odds] * (self.map_.info.width * self.map_.info.height)

        self.map_pub = self.create_publisher(OccupancyGrid, 'map', 1)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scanCallback, 10)
        self.timer = self.create_timer(1.0, self.timerCallback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def scanCallback(self, scan: LaserScan):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_.header.frame_id, 
                scan.header.frame_id, 
                Time.from_msg(scan.header.stamp),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f'Unable to transform between /odom and /base_footprint: {str(e)}')
            return

        robot_p = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, self.map_.info)

        if not poseOnMap(robot_p, self.map_.info):
            self.get_logger().error("The robot is out of the map!")
            return
        (roll, pitch, yaw) = euler_from_quaternion([
                t.transform.rotation.x, t.transform.rotation.y, 
                t.transform.rotation.z, t.transform.rotation.w])

        for i in range(len(scan.ranges)):
            r = scan.ranges[i]

            if math.isinf(r) or math.isnan(r) or r >= (scan.range_max - 0.05):
                continue
            
            theta = yaw + scan.angle_min + (scan.angle_increment * i)
            px = t.transform.translation.x + r * math.cos(theta)
            py = t.transform.translation.y + r * math.sin(theta)
            beam_p= coordinatesToPose(px, py, self.map_.info)
            if not poseOnMap(beam_p, self.map_.info):
                continue

            poses = inverseSensorModel(robot_p, beam_p)
            for pose, value in poses:
                cell = poseToCell(pose, self.map_.info)
                self.probalility_map_[cell] += prob2LogOdds(value) - prior_log_odds
               
                

                # if value == 100:
                #     self.map_.data[cell] = 100
                # elif value == 0 and self.map_.data[cell] != 100:
                #     self.map_.data[cell] = 0

    def timerCallback(self):
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_.data = [int(logOdds2Prob(cell) * 100) for cell in self.probalility_map_]
        self.map_pub.publish(self.map_)

def main():
    rclpy.init()
    node = MappingWithKnownPoses('mapping_with_known_poses')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()



