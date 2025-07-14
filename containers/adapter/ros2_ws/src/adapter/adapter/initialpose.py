import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from woosh_robot_msgs.srv import SetRobotPose

class InitialPoseHandler(Node):
    def __init__(self):
        super().__init__('initialpose_handler')
        
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.initialpose_callback,
            qos_profile
        )
        
        # 创建客户端来调用服务
        self.cli = self.create_client(SetRobotPose, '/woosh_robot/robot/SetRobotPose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /woosh_robot/robot/SetRobotPose service...')
        
        self.get_logger().info('InitialPoseHandler node started')

    def initialpose_callback(self, msg: PoseWithCovarianceStamped):
        """回调函数，接收 /initialpose 消息，并调用服务设置机器人初始位姿"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # 计算 yaw 角（theta）
        import math
        theta = 2 * math.atan2(qz, qw)
        
        self.get_logger().info(f'Received initial pose: x={x}, y={y}, theta={theta}')
        
        # 构造服务请求
        request = SetRobotPose.Request()
        request.arg.pose.x = x
        request.arg.pose.y = y
        request.arg.pose.theta = theta
        
        # 发送请求并等待响应
        future = self.cli.call_async(request)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Successfully set robot pose: {response}')
        except Exception as e:
            self.get_logger().error(f'Failed to set robot pose: {str(e)}')


def main():
    rclpy.init()
    node = InitialPoseHandler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
