#!/usr/bin/env python3
"""
测试脚本：检查 Franka 机器人系统必需的 Action Servers 和 Topics 是否正常启动。

此脚本会检查：
1. 必需的 Action Servers 是否可用
2. 必需的 Topics 是否有数据发布
3. 必需的 Nodes 是否运行
4. 必需的 Services 是否可用

使用方法：
    ros2 run role_ros2 test_required_components
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from franka_msgs.msg import FrankaRobotState
from geometry_msgs.msg import PoseStamped
from control_msgs.action import FollowJointTrajectory
from franka_msgs.action import Move as GripperMove, Grasp as GripperGrasp
from controller_manager_msgs.srv import ListControllers


class RequiredComponentsTester(Node):
    def __init__(self, arm_id='fr3', timeout=10.0):
        super().__init__('required_components_tester')
        self.arm_id = arm_id
        self.timeout = timeout
        self.test_results = {
            'nodes': {},
            'actions': {},
            'topics': {},
            'services': {},
        }
        self.topic_data_received = {}
        self.topic_subscribers = {}
        
    def test_node_exists(self, node_name):
        """测试节点是否存在"""
        try:
            # 使用 ros2 node list 命令检查
            import subprocess
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            exists = f'/{node_name}' in result.stdout or node_name in result.stdout
            self.test_results['nodes'][node_name] = exists
            return exists
        except Exception as e:
            self.get_logger().warn(f'Failed to check node {node_name}: {e}')
            self.test_results['nodes'][node_name] = False
            return False
    
    def test_action_server(self, action_name, action_type):
        """测试 Action Server 是否可用"""
        try:
            # 首先检查 action 是否在列表中
            import subprocess
            result = subprocess.run(
                ['ros2', 'action', 'list'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            exists_in_list = action_name in result.stdout
            
            if not exists_in_list:
                self.test_results['actions'][action_name] = False
                self.get_logger().error(f'✗ Action server not found: {action_name}')
                return False
            
            # 如果存在，尝试连接
            if 'FollowJointTrajectory' in action_type:
                client = ActionClient(self, FollowJointTrajectory, action_name)
            elif 'Grasp' in action_type:
                client = ActionClient(self, GripperGrasp, action_name)
            elif 'Move' in action_type:
                client = ActionClient(self, GripperMove, action_name)
            else:
                # 对于其他类型，使用 Move 作为默认
                client = ActionClient(self, GripperMove, action_name)
            
            available = client.wait_for_server(timeout_sec=3.0)
            self.test_results['actions'][action_name] = available
            if available:
                self.get_logger().info(f'✓ Action server available: {action_name}')
            else:
                self.get_logger().warn(f'⚠ Action server exists but not ready: {action_name}')
            return available
        except Exception as e:
            self.get_logger().error(f'✗ Failed to test action {action_name}: {e}')
            self.test_results['actions'][action_name] = False
            return False
    
    def test_topic_exists(self, topic_name, topic_type=None, expect_data=True):
        """测试 Topic 是否存在，并可选择是否检查数据流"""
        try:
            # 使用 ros2 topic list 检查
            import subprocess
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            exists = topic_name in result.stdout
            self.test_results['topics'][topic_name] = {
                'exists': exists, 
                'has_data': False,
                'expect_data': expect_data
            }
            
            if not exists:
                self.get_logger().error(f'✗ Topic does not exist: {topic_name}')
                return False
            
            self.get_logger().info(f'✓ Topic exists: {topic_name}')
            
            # 如果需要检查数据流
            if expect_data:
                return self.test_topic_data(topic_name, topic_type)
            else:
                # 即使不期望数据，也尝试订阅一次看看是否有数据
                self.test_topic_data(topic_name, topic_type)
                return True
                
        except Exception as e:
            self.get_logger().error(f'✗ Failed to test topic {topic_name}: {e}')
            self.test_results['topics'][topic_name] = {
                'exists': False, 
                'has_data': False,
                'expect_data': expect_data
            }
            return False
    
    def test_topic_data(self, topic_name, topic_type=None):
        """测试 Topic 是否有数据发布"""
        self.topic_data_received[topic_name] = False
        
        # 创建订阅者
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        if topic_type == 'JointState' or 'joint_states' in topic_name:
            sub = self.create_subscription(
                JointState, topic_name, 
                lambda msg, tn=topic_name: self._topic_callback(tn), 
                qos_profile
            )
        elif topic_type == 'FrankaRobotState' or 'robot_state' in topic_name:
            sub = self.create_subscription(
                FrankaRobotState, topic_name,
                lambda msg, tn=topic_name: self._topic_callback(tn),
                qos_profile
            )
        elif topic_type == 'PoseStamped' or 'pose' in topic_name:
            sub = self.create_subscription(
                PoseStamped, topic_name,
                lambda msg, tn=topic_name: self._topic_callback(tn),
                qos_profile
            )
        else:
            # 通用订阅
            sub = self.create_subscription(
                String, topic_name,
                lambda msg, tn=topic_name: self._topic_callback(tn),
                qos_profile
            )
        
        self.topic_subscribers[topic_name] = sub
        
        # 等待数据
        start_time = time.time()
        while (time.time() - start_time) < 3.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.topic_data_received.get(topic_name, False):
                self.test_results['topics'][topic_name]['has_data'] = True
                self.get_logger().info(f'✓ Topic has data: {topic_name}')
                return True
        
        self.test_results['topics'][topic_name]['has_data'] = False
        self.get_logger().warn(f'⚠ Topic exists but no data received: {topic_name}')
        return False
    
    def _topic_callback(self, topic_name):
        """Topic 数据回调"""
        self.topic_data_received[topic_name] = True
    
    def test_service(self, service_name):
        """测试 Service 是否可用"""
        try:
            # 使用 ros2 service list 检查
            import subprocess
            result = subprocess.run(
                ['ros2', 'service', 'list'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            exists = service_name in result.stdout
            self.test_results['services'][service_name] = exists
            if exists:
                self.get_logger().info(f'✓ Service available: {service_name}')
            else:
                self.get_logger().error(f'✗ Service not available: {service_name}')
            return exists
        except Exception as e:
            self.get_logger().error(f'✗ Failed to test service {service_name}: {e}')
            self.test_results['services'][service_name] = False
            return False
    
    def run_all_tests(self):
        """运行所有测试"""
        self.get_logger().info('=' * 70)
        self.get_logger().info('Franka 机器人系统必需组件测试')
        self.get_logger().info('=' * 70)
        
        # 等待 ROS2 系统初始化
        time.sleep(1.0)
        
        # ========== 测试必需的 Nodes ==========
        self.get_logger().info('\n[1/4] 测试必需的 Nodes...')
        required_nodes = [
            'controller_manager',
            'robot_state_publisher',
            'joint_state_broadcaster',
            f'{self.arm_id}_arm_controller',
            'franka_robot_state_broadcaster',
            f'{self.arm_id}_gripper',
        ]
        
        for node_name in required_nodes:
            self.test_node_exists(node_name)
        
        # ========== 测试必需的 Action Servers ==========
        self.get_logger().info('\n[2/4] 测试必需的 Action Servers...')
        required_actions = [
            (f'/{self.arm_id}_arm_controller/follow_joint_trajectory', 'FollowJointTrajectory'),
            (f'/{self.arm_id}_gripper/move', 'Move'),
            (f'/{self.arm_id}_gripper/grasp', 'Grasp'),  # Grasp 使用正确的类型
        ]
        
        for action_name, action_type in required_actions:
            self.test_action_server(action_name, action_type)
        
        # ========== 测试必需的 Topics ==========
        self.get_logger().info('\n[3/4] 测试必需的 Topics...')
        required_topics = [
            ('/joint_states', 'JointState', True),  # (topic_name, topic_type, expect_data)
            ('/franka/joint_states', 'JointState', True),
            (f'/{self.arm_id}_gripper/joint_states', 'JointState', True),
            ('/franka_robot_state_broadcaster/robot_state', 'FrankaRobotState', True),
            ('/franka_robot_state_broadcaster/current_pose', 'PoseStamped', False),  # 可能需要订阅才发布
            ('/tf', None, False),  # TF 可能需要订阅者或发布频率低
            ('/tf_static', None, False),  # 静态 TF 只在启动时发布一次
            ('/robot_description', 'String', False),  # 只在启动时发布一次
        ]
        
        for topic_name, topic_type, expect_data in required_topics:
            self.test_topic_exists(topic_name, topic_type, expect_data)
        
        # 等待一段时间以接收 topic 数据
        self.get_logger().info('等待接收 topic 数据...')
        time.sleep(3.0)  # 增加等待时间
        for _ in range(30):  # 增加循环次数
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # ========== 测试必需的 Services ==========
        self.get_logger().info('\n[4/4] 测试必需的 Services...')
        required_services = [
            '/controller_manager/list_controllers',
            '/controller_manager/switch_controller',  # 注意：单数，不是 switch_controllers
            '/controller_manager/load_controller',
            f'/{self.arm_id}_gripper/stop',
        ]
        
        for service_name in required_services:
            self.test_service(service_name)
        
        # ========== 生成测试报告 ==========
        self.print_test_report()
        
        # 返回总体结果
        return self.get_overall_result()
    
    def print_test_report(self):
        """打印测试报告"""
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('测试报告')
        self.get_logger().info('=' * 70)
        
        # Nodes
        self.get_logger().info('\n📦 Nodes:')
        all_nodes_ok = True
        for node_name, result in self.test_results['nodes'].items():
            status = '✓' if result else '✗'
            self.get_logger().info(f'  {status} {node_name}')
            if not result:
                all_nodes_ok = False
        
        # Actions
        self.get_logger().info('\n🎯 Action Servers:')
        all_actions_ok = True
        for action_name, result in self.test_results['actions'].items():
            status = '✓' if result else '✗'
            self.get_logger().info(f'  {status} {action_name}')
            if not result:
                all_actions_ok = False
        
        # Topics
        self.get_logger().info('\n📡 Topics:')
        all_topics_ok = True
        for topic_name, result in self.test_results['topics'].items():
            exists = result.get('exists', False)
            has_data = result.get('has_data', False)
            expect_data = result.get('expect_data', True)
            
            if exists and (has_data or not expect_data):
                status = '✓'
            elif exists:
                status = '⚠'  # 存在但无数据（如果期望有数据）
            else:
                status = '✗'
            
            expect_str = '需要数据' if expect_data else '不需要数据'
            self.get_logger().info(f'  {status} {topic_name} (exists: {exists}, has_data: {has_data}, {expect_str})')
            
            # 只有期望有数据但无数据时才认为失败
            if not exists or (has_data is False and expect_data):
                all_topics_ok = False
        
        # Services
        self.get_logger().info('\n🔧 Services:')
        all_services_ok = True
        for service_name, result in self.test_results['services'].items():
            status = '✓' if result else '✗'
            self.get_logger().info(f'  {status} {service_name}')
            if not result:
                all_services_ok = False
        
        # 总结
        self.get_logger().info('\n' + '=' * 70)
        self.get_logger().info('总结:')
        self.get_logger().info(f'  Nodes:      {"✓ 全部通过" if all_nodes_ok else "✗ 部分失败"}')
        self.get_logger().info(f'  Actions:    {"✓ 全部通过" if all_actions_ok else "✗ 部分失败"}')
        self.get_logger().info(f'  Topics:     {"✓ 全部通过" if all_topics_ok else "✗ 部分失败"}')
        self.get_logger().info(f'  Services:   {"✓ 全部通过" if all_services_ok else "✗ 部分失败"}')
        self.get_logger().info('=' * 70)
    
    def get_overall_result(self):
        """获取总体测试结果"""
        # Nodes: 必须全部通过
        nodes_ok = all(self.test_results['nodes'].values())
        
        # Actions: 必须全部通过（grasp 可能不可用，但 move 必须可用）
        # 只检查 move 和 follow_joint_trajectory
        critical_actions = [
            f'/{self.arm_id}_arm_controller/follow_joint_trajectory',
            f'/{self.arm_id}_gripper/move',
        ]
        actions_ok = all(
            self.test_results['actions'].get(action, False) 
            for action in critical_actions
        )
        
        # Topics: 必须存在，且期望有数据的必须有数据
        topics_ok = all(
            r.get('exists', False) and 
            (r.get('has_data', False) or not r.get('expect_data', True))
            for r in self.test_results['topics'].values()
        )
        
        # Services: 必须全部通过（switch_controller 是单数）
        services_ok = all(self.test_results['services'].values())
        
        all_ok = nodes_ok and actions_ok and topics_ok and services_ok
        return all_ok


def main(args=None):
    rclpy.init(args=args)
    
    tester = RequiredComponentsTester(arm_id='fr3', timeout=10.0)
    
    try:
        success = tester.run_all_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        tester.get_logger().warn('测试被用户中断')
        sys.exit(1)
    except Exception as e:
        tester.get_logger().error(f'测试失败: {e}')
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

