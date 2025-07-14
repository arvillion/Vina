import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.action import ComputePathToPose
from tf2_ros import TransformBroadcaster
import math
import time

class VisionNavNode(Node):
    def __init__(self):
        super().__init__('vision_nav_node')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.planner_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

        # 订阅目标点
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_cb,
            10
        )
        self.current_goal = None

        # 模拟视觉定位频率
        self.timer = self.create_timer(1.0, self.vision_update)

        # 状态
        self.path = []
        self.path_index = 0

    def goal_cb(self, msg: PoseStamped):
        self.get_logger().info(f"Received goal at ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")
        self.current_goal = msg

    def vision_update(self):
        # 没有目标点则不干活
        if self.current_goal is None:
            return

        # 假定视觉定位得到当前位置
        vision_x, vision_y, vision_yaw = 0.0, 0.0, 0.0

        self.broadcast_tf(vision_x, vision_y, vision_yaw)
        self.plan_path(vision_x, vision_y)

    def broadcast_tf(self, x, y, yaw):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = x
        t.transform.translation.y = y
        qz = math.sin(yaw/2.0)
        qw = math.cos(yaw/2.0)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def plan_path(self, x, y):
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.pose.position.x = x
        start_pose.pose.position.y = y
        start_pose.pose.orientation.w = 1.0

        goal_pose = self.current_goal

        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start_pose
        goal_msg.goal = goal_pose

        self.planner_client.wait_for_server()
        future = self.planner_client.send_goal_async(goal_msg)
        future.add_done_callback(self.plan_done)

    def plan_done(self, future):
        result = future.result().result
        self.get_logger().info(f"Path planned with {len(result.path.poses)} points")
        self.path = result.path.poses
        self.path_index = 0
        self.follow_path()

    def follow_path(self):
        # 简化版跟踪逻辑
        if self.path_index >= len(self.path):
            self.get_logger().info("Reached goal.")
            return

        target = self.path[self.path_index].pose.position
        dx = target.x
        dy = target.y
        dist = math.hypot(dx, dy)

        if dist < 0.1:
            self.path_index += 1
            self.follow_path()
            return

        angle = math.atan2(dy, dx)
        twist = Twist()
        twist.linear.x = 0.1
        twist.angular.z = angle
        self.cmd_vel_pub.publish(twist)

        time.sleep(0.1)
        self.follow_path()


if __name__ == '__main__':
    rclpy.init()
    node = VisionNavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()