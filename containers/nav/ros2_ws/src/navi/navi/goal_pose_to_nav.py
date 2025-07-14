# goal_pose_to_nav.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose


class GoalPoseToNavNode(Node):
    def __init__(self):
        super().__init__('goal_pose_to_nav')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info("Node started. Waiting for /goal_pose messages...")

    def goal_pose_callback(self, msg: PoseStamped):
        self.get_logger().info(f'Received new goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = msg

        # Wait for action server
        self._action_client.wait_for_server()

        # Send goal
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by action server.')
            return

        self.get_logger().info('Goal accepted. Waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Navigation finished with result code: {result}')
        # Optional: you can print result.pose or status


def main(args=None):
    rclpy.init(args=args)
    node = GoalPoseToNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()