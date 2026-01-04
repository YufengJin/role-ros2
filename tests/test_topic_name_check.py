#!/usr/bin/env python3
"""
诊断脚本：检查 ROS2CameraReader 实际订阅的 topic 名称
"""

import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from role_ros2.camera_utils.multi_camera_wrapper import MultiCameraWrapper


class TopicNameCheckNode(Node):
    """检查 topic 名称的节点"""
    
    def __init__(self, camera_id: str):
        super().__init__('topic_name_check_node')
        self.camera_id = camera_id
        
        # 加载配置
        config_file = Path(__file__).parent.parent / 'role_ros2' / 'camera_utils' / 'multi_camera_config.yaml'
        self.camera_wrapper = MultiCameraWrapper(
            node=self,
            config_file=str(config_file)
        )
        
        if camera_id not in self.camera_wrapper.camera_dict:
            self.get_logger().error(f"相机 {camera_id} 未找到！")
            sys.exit(1)
        
        self.camera = self.camera_wrapper.camera_dict[camera_id]
        
        # 检查订阅者
        print(f"\n{'='*80}")
        print(f"检查相机 {camera_id} 的 topic 订阅情况")
        print(f"{'='*80}\n")
        
        # 尝试获取订阅的 topic 名称
        try:
            if hasattr(self.camera, '_rgb_sub'):
                # message_filters.Subscriber 可能没有 get_topic_name 方法
                # 尝试其他方式获取
                print(f"RGB 订阅者: {type(self.camera._rgb_sub)}")
                if hasattr(self.camera._rgb_sub, 'get_topic_name'):
                    print(f"  Topic: {self.camera._rgb_sub.get_topic_name()}")
                else:
                    print(f"  (无法获取 topic 名称)")
            
            if hasattr(self.camera, '_depth_sub'):
                print(f"Depth 订阅者: {type(self.camera._depth_sub)}")
                if hasattr(self.camera._depth_sub, 'get_topic_name'):
                    print(f"  Topic: {self.camera._depth_sub.get_topic_name()}")
                else:
                    print(f"  (无法获取 topic 名称)")
            
            if hasattr(self.camera, '_camera_info_sub'):
                print(f"CameraInfo 订阅者: {type(self.camera._camera_info_sub)}")
                if hasattr(self.camera._camera_info_sub, 'get_topic_name'):
                    print(f"  Topic: {self.camera._camera_info_sub.get_topic_name()}")
                else:
                    print(f"  (无法获取 topic 名称)")
            
            # 检查同步器
            if hasattr(self.camera, '_sync'):
                print(f"\n同步器类型: {type(self.camera._sync).__name__}")
                if hasattr(self.camera._sync, 'slop'):
                    print(f"  Slop: {self.camera._sync.slop} 秒")
            
        except Exception as e:
            print(f"检查失败: {e}")
            import traceback
            traceback.print_exc()
        
        # 等待一段时间看是否有回调
        print(f"\n等待 5 秒，检查是否有同步回调...")
        start_time = time.time()
        initial_count = getattr(self.camera, '_callback_count', 0)
        
        while time.time() - start_time < 5.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        
        final_count = getattr(self.camera, '_callback_count', 0)
        callback_received = final_count - initial_count
        
        print(f"\n回调统计:")
        print(f"  初始回调次数: {initial_count}")
        print(f"  最终回调次数: {final_count}")
        print(f"  新接收回调数: {callback_received}")
        
        if callback_received > 0:
            print(f"  ✅ 同步回调正常工作！")
        else:
            print(f"  ❌ 没有接收到同步回调")
            
            # 检查队列
            with self.camera._data_lock:
                queue_size = len(self.camera._data_queue)
            print(f"  队列大小: {queue_size}")
            
            print(f"\n可能的原因:")
            print(f"  1. Topic 名称不匹配")
            print(f"  2. 消息时间戳差异过大（超过 slop）")
            print(f"  3. 订阅未正确建立")
        
        print(f"\n{'='*80}\n")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='检查 topic 名称')
    parser.add_argument('--camera-id', type=str, required=True, help='相机ID')
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        node = TopicNameCheckNode(args.camera_id)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()

