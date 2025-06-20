import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose2D

from woosh_robot_msgs.srv import ScannerData as ScannerDataSrv

import math


class ScannerBridgeNode(Node):
    def __init__(self):
        super().__init__('scanner_bridge_node')

        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.pose_pub = self.create_publisher(Pose2D, '/scan_pose', 10)

        self.cli = self.create_client(ScannerDataSrv, '/woosh_robot/robot/ScannerData')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 /woosh_robot/robot/ScannerData 服务可用中...')

        # 定时调用服务
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        req = ScannerDataSrv.Request()
        self.future = self.cli.call_async(req)
        self.future.add_done_callback(self.handle_response)

    def handle_response(self, future):
        try:
            response = future.result()
            ret = response.ret

            # 发布 LaserScan 消息
            scan_msg = LaserScan()
            scan_msg.header = Header()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = 'laser_frame'

            scan_msg.angle_min = ret.angle_min
            scan_msg.angle_max = ret.angle_max
            scan_msg.angle_increment = ret.angle_increment
            scan_msg.time_increment = ret.time_increment
            scan_msg.scan_time = ret.scan_time
            scan_msg.range_min = ret.range_min
            scan_msg.range_max = ret.range_max

            scan_msg.ranges = [
                r if ret.range_min <= r <= ret.range_max else math.nan
                for r in ret.ranges
            ]

            self.scan_pub.publish(scan_msg)

            # 发布 Pose2D 消息
            pose_msg = Pose2D()
            pose_msg.x = ret.pose.x
            pose_msg.y = ret.pose.y
            pose_msg.theta = ret.pose.theta

            self.pose_pub.publish(pose_msg)

        except Exception as e:
            self.get_logger().error(f'服务调用失败: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ScannerBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()