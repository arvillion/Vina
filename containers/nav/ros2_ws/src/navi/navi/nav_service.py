import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped, Point, Quaternion
from nav2_msgs.action import NavigateToPose
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from tf_transformations import quaternion_from_euler
import threading
import time
import math
from flask import Flask, request, jsonify
import json

class NavigationSDK(Node):
    def __init__(self):
        super().__init__('navigation_sdk')
        
        # 创建发布者和订阅者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 创建Nav2动作客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF2缓冲区和监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 导航状态
        self.navigation_active = False
        self.current_goal_handle = None
        
        # 默认移动参数
        self.linear_speed = 0.2  # m/s
        self.angular_speed = 0.5  # rad/s
        
        self.get_logger().info('Navigation SDK initialized')

    def move_forward(self, distance):
        """前进指定距离"""
        return self._move_linear(distance, 0.0)
    
    def move_backward(self, distance):
        """后退指定距离"""
        return self._move_linear(-distance, 0.0)
    
    def move_left(self, distance):
        """左平移指定距离"""
        return self._move_linear(0.0, distance)
    
    def move_right(self, distance):
        """右平移指定距离"""
        return self._move_linear(0.0, -distance)
    
    def _move_linear(self, x_distance, y_distance):
        """执行线性移动"""
        try:
            # 获取当前位置
            start_pose = self._get_current_pose()
            if not start_pose:
                return {"success": False, "message": "无法获取当前位置"}
            
            # 计算目标距离
            target_distance = math.sqrt(x_distance**2 + y_distance**2)
            if target_distance == 0:
                return {"success": True, "message": "移动距离为0"}
            
            # 创建速度消息
            twist = Twist()
            if abs(x_distance) > abs(y_distance):
                twist.linear.x = self.linear_speed if x_distance > 0 else -self.linear_speed
            else:
                twist.linear.y = self.linear_speed if y_distance > 0 else -self.linear_speed
            
            # 计算移动时间
            move_time = target_distance / self.linear_speed
            
            # 执行移动
            start_time = time.time()
            while (time.time() - start_time) < move_time:
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            
            # 停止移动
            self._stop_robot()
            
            return {"success": True, "message": f"成功移动 {target_distance:.2f} 米"}
            
        except Exception as e:
            self.get_logger().error(f"移动失败: {str(e)}")
            return {"success": False, "message": f"移动失败: {str(e)}"}
    
    def rotate(self, angle_degrees):
        """原地旋转指定角度（度）"""
        try:
            angle_radians = math.radians(angle_degrees)
            
            # 创建旋转速度消息
            twist = Twist()
            twist.angular.z = self.angular_speed if angle_radians > 0 else -self.angular_speed
            
            # 计算旋转时间
            rotate_time = abs(angle_radians) / self.angular_speed
            
            # 执行旋转
            start_time = time.time()
            while (time.time() - start_time) < rotate_time:
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.1)
            
            # 停止旋转
            self._stop_robot()
            
            return {"success": True, "message": f"成功旋转 {angle_degrees} 度"}
            
        except Exception as e:
            self.get_logger().error(f"旋转失败: {str(e)}")
            return {"success": False, "message": f"旋转失败: {str(e)}"}
    
    def navigate_to_pose(self, x, y, yaw=0.0, frame_id="map"):
        """导航到指定位置"""
        try:
            # 停止当前导航
            if self.navigation_active:
                self.cancel_navigation()
            
            # 等待Nav2动作服务器
            if not self.nav_client.wait_for_server(timeout_sec=5.0):
                return {"success": False, "message": "Nav2动作服务器不可用"}
            
            # 创建目标姿态
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = frame_id
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            
            goal_msg.pose.pose.position.x = float(x)
            goal_msg.pose.pose.position.y = float(y)
            goal_msg.pose.pose.position.z = 0.0
            
            # 转换欧拉角到四元数
            quat = quaternion_from_euler(0, 0, yaw)
            goal_msg.pose.pose.orientation.x = quat[0]
            goal_msg.pose.pose.orientation.y = quat[1]
            goal_msg.pose.pose.orientation.z = quat[2]
            goal_msg.pose.pose.orientation.w = quat[3]
            
            # 发送导航目标
            self.get_logger().info(f'导航到目标点: x={x}, y={y}, yaw={yaw}')
            future = self.nav_client.send_goal_async(goal_msg)
            
            # 等待目标被接受
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is None:
                return {"success": False, "message": "导航目标发送失败"}
            
            goal_handle = future.result()
            if not goal_handle.accepted:
                return {"success": False, "message": "导航目标被拒绝"}
            
            self.current_goal_handle = goal_handle
            self.navigation_active = True
            
            return {"success": True, "message": f"开始导航到 ({x}, {y})"}
            
        except Exception as e:
            self.get_logger().error(f"导航失败: {str(e)}")
            return {"success": False, "message": f"导航失败: {str(e)}"}
    
    def cancel_navigation(self):
        """停止/取消导航"""
        try:
            if self.current_goal_handle and self.navigation_active:
                # 取消当前目标
                future = self.current_goal_handle.cancel_goal_async()
                self.navigation_active = False
                self.current_goal_handle = None
                
                # 停止机器人
                self._stop_robot()
                
                return {"success": True, "message": "导航已取消"}
            else:
                return {"success": True, "message": "当前没有活动的导航任务"}
                
        except Exception as e:
            self.get_logger().error(f"取消导航失败: {str(e)}")
            return {"success": False, "message": f"取消导航失败: {str(e)}"}
    
    def _stop_robot(self):
        """停止机器人移动"""
        twist = Twist()  # 所有值默认为0
        self.cmd_vel_pub.publish(twist)
    
    def get_navigation_status(self):
        """获取导航状态"""
        try:
            current_pose = self._get_current_pose()
            return {
                "success": True,
                "navigation_active": self.navigation_active,
                "current_pose": current_pose,
                "message": "状态获取成功"
            }
        except Exception as e:
            return {"success": False, "message": f"获取状态失败: {str(e)}"}
    
    def _get_current_pose(self):
        """获取当前机器人位置"""
        try:
            # 获取从map到base_footprint的变换
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            
            return {
                "x": transform.transform.translation.x,
                "y": transform.transform.translation.y,
                "z": transform.transform.translation.z,
                "orientation": {
                    "x": transform.transform.rotation.x,
                    "y": transform.transform.rotation.y,
                    "z": transform.transform.rotation.z,
                    "w": transform.transform.rotation.w
                }
            }
        except Exception as e:
            self.get_logger().warn(f"无法获取当前位置: {str(e)}")
            return None

# Flask Web服务器
app = Flask(__name__)
nav_sdk = None

def create_response(success, message, data=None):
    """创建统一的响应格式"""
    response = {
        "success": success,
        "message": message,
        "timestamp": time.time()
    }
    if data:
        response["data"] = data
    return jsonify(response)

@app.route('/api/move/forward', methods=['POST'])
def move_forward():
    try:
        data = request.get_json()
        distance = float(data.get('distance', 1.0))
        result = nav_sdk.move_forward(distance)
        return create_response(result["success"], result["message"])
    except Exception as e:
        return create_response(False, f"请求处理失败: {str(e)}")

@app.route('/api/move/backward', methods=['POST'])
def move_backward():
    try:
        data = request.get_json()
        distance = float(data.get('distance', 1.0))
        result = nav_sdk.move_backward(distance)
        return create_response(result["success"], result["message"])
    except Exception as e:
        return create_response(False, f"请求处理失败: {str(e)}")

@app.route('/api/move/left', methods=['POST'])
def move_left():
    try:
        data = request.get_json()
        distance = float(data.get('distance', 1.0))
        result = nav_sdk.move_left(distance)
        return create_response(result["success"], result["message"])
    except Exception as e:
        return create_response(False, f"请求处理失败: {str(e)}")

@app.route('/api/move/right', methods=['POST'])
def move_right():
    try:
        data = request.get_json()
        distance = float(data.get('distance', 1.0))
        result = nav_sdk.move_right(distance)
        return create_response(result["success"], result["message"])
    except Exception as e:
        return create_response(False, f"请求处理失败: {str(e)}")

@app.route('/api/rotate', methods=['POST'])
def rotate():
    try:
        data = request.get_json()
        angle = float(data.get('angle', 90.0))
        result = nav_sdk.rotate(angle)
        return create_response(result["success"], result["message"])
    except Exception as e:
        return create_response(False, f"请求处理失败: {str(e)}")

@app.route('/api/navigate', methods=['POST'])
def navigate():
    try:
        data = request.get_json()
        x = float(data.get('x'))
        y = float(data.get('y'))
        yaw = float(data.get('yaw', 0.0))
        frame_id = data.get('frame_id', 'map')
        
        result = nav_sdk.navigate_to_pose(x, y, yaw, frame_id)
        return create_response(result["success"], result["message"])
    except Exception as e:
        return create_response(False, f"请求处理失败: {str(e)}")

@app.route('/api/cancel', methods=['POST'])
def cancel_navigation():
    try:
        result = nav_sdk.cancel_navigation()
        return create_response(result["success"], result["message"])
    except Exception as e:
        return create_response(False, f"请求处理失败: {str(e)}")

@app.route('/api/status', methods=['GET'])
def get_status():
    try:
        result = nav_sdk.get_navigation_status()
        return create_response(result["success"], result["message"], result.get("current_pose"))
    except Exception as e:
        return create_response(False, f"请求处理失败: {str(e)}")

@app.route('/api/stop', methods=['POST'])
def emergency_stop():
    try:
        nav_sdk._stop_robot()
        return create_response(True, "机器人已停止")
    except Exception as e:
        return create_response(False, f"停止失败: {str(e)}")

def run_flask_app():
    """在单独线程中运行Flask应用"""
    app.run(host='127.0.0.1', port=45567, debug=False, threaded=True)

def main():
    global nav_sdk
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建导航SDK节点
    nav_sdk = NavigationSDK()
    
    # 在单独线程中启动Flask服务器
    flask_thread = threading.Thread(target=run_flask_app, daemon=True)
    flask_thread.start()
    
    print("Navigation SDK Service started!")
    print("Web API available at: http://127.0.0.1:45567")
    print("API文档:")
    print("- POST /api/move/forward    - 前进 {distance: 1.0}")
    print("- POST /api/move/backward   - 后退 {distance: 1.0}")
    print("- POST /api/move/left       - 左移 {distance: 1.0}")
    print("- POST /api/move/right      - 右移 {distance: 1.0}")
    print("- POST /api/rotate          - 旋转 {angle: 90.0}")
    print("- POST /api/navigate        - 导航 {x: 1.0, y: 2.0, yaw: 0.0}")
    print("- POST /api/cancel          - 取消导航")
    print("- GET  /api/status          - 获取状态")
    print("- POST /api/stop            - 紧急停止")
    
    try:
        # 运行ROS2节点
        rclpy.spin(nav_sdk)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        nav_sdk.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()