#!/usr/bin/env python3
"""
简单测试脚本：验证 ROS2CameraReader 是否能接收回调
"""

import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from role_ros2.camera_utils.multi_camera_wrapper import MultiCameraWrapper


class SimpleTestNode(Node):
    """简单测试节点"""
    
    def __init__(self, camera_id: str):
        super().__init__('simple_camera_test_node')
        self.camera_id = camera_id
        
        # 加载配置
        config_file = Path(__file__).parent.parent / 'role_ros2' / 'camera_utils' / 'multi_camera_config.yaml'
        self.camera_wrapper = MultiCameraWrapper(
            node=self,
            config_file=str(config_file)
        )
        
        if camera_id not in self.camera_wrapper.camera_dict:
            self.get_logger().error(f"❌ 相机 {camera_id} 未找到！")
            sys.exit(1)
        
        self.camera = self.camera_wrapper.camera_dict[camera_id]
        self.get_logger().info(f"✅ 找到相机: {camera_id}")
        
        # 初始化计数器
        self.initial_callback_count = getattr(self.camera, '_callback_count', 0)
        self.start_time = time.time()
    
    def run_test(self, duration: float = 30.0):
        """运行测试"""
        print(f"\n{'='*80}")
        print(f"简单回调测试 - 相机 {self.camera_id}")
        print(f"{'='*80}")
        print(f"测试时长: {duration} 秒\n")
        
        last_report_time = self.start_time
        report_interval = 2.0
        
        while time.time() - self.start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            current_time = time.time()
            if current_time - last_report_time >= report_interval:
                elapsed = current_time - self.start_time
                callback_count = getattr(self.camera, '_callback_count', 0)
                new_callbacks = callback_count - self.initial_callback_count
                
                with self.camera._data_lock:
                    queue_size = len(self.camera._data_queue)
                
                print(f"[{elapsed:.1f}s] 回调次数: {callback_count} (新增: {new_callbacks}), 队列大小: {queue_size}")
                
                if new_callbacks > 0:
                    print(f"  ✅ 成功！已接收到 {new_callbacks} 个同步回调")
                
                last_report_time = current_time
        
        # 最终报告
        final_callback_count = getattr(self.camera, '_callback_count', 0)
        total_callbacks = final_callback_count - self.initial_callback_count
        
        print(f"\n{'='*80}")
        print("最终结果")
        print(f"{'='*80}")
        print(f"总回调次数: {total_callbacks}")
        
        with self.camera._data_lock:
            queue_size = len(self.camera._data_queue)
        print(f"队列大小: {queue_size}")
        
        if total_callbacks > 0:
            print(f"\n✅ 测试成功！TimeSynchronizer 正常工作")
            print(f"   平均 FPS: {total_callbacks / duration:.2f} Hz")
        else:
            print(f"\n❌ 测试失败！没有接收到任何回调")
            print(f"\n可能的原因：")
            print(f"  1. Topic 名称不匹配")
            print(f"  2. 消息时间戳不完全相同（TimeSynchronizer 要求完全相同）")
            print(f"  3. 订阅未正确建立")
        
        print(f"{'='*80}\n")


def main():
    import argparse
    parser = argparse.ArgumentParser(description='简单测试 ROS2CameraReader 回调')
    parser.add_argument('--camera-id', type=str, default='11022812',
                       help='相机ID (默认: 11022812)')
    parser.add_argument('--duration', type=float, default=30.0,
                       help='测试时长（秒，默认: 30）')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        node = SimpleTestNode(args.camera_id)
        node.run_test(duration=args.duration)
    except KeyboardInterrupt:
        print("\n⚠️  测试被用户中断")
    except Exception as e:
        print(f"❌ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()


