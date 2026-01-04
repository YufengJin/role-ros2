#!/usr/bin/env python3
"""
测试脚本：直接使用 ApproximateTimeSynchronizer 和 TimeSynchronizer 
来订阅三个 topic，检查同步是否正常工作
"""

import sys
import time
import argparse
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image, CameraInfo
import message_filters

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))


class TopicSyncTestNode(Node):
    """测试节点：直接测试 topic 同步"""
    
    def __init__(
        self,
        rgb_topic: str,
        depth_topic: str,
        camera_info_topic: str,
        use_approximate: bool = True,
        slop: float = 0.1
    ):
        """
        初始化测试节点
        
        Args:
            rgb_topic: RGB image topic
            depth_topic: Depth image topic
            camera_info_topic: Camera info topic
            use_approximate: 如果 True 使用 ApproximateTimeSynchronizer，否则使用 TimeSynchronizer
            slop: ApproximateTimeSynchronizer 的 slop 值（秒）
        """
        super().__init__('topic_sync_test_node')
        
        self.rgb_topic = rgb_topic
        self.depth_topic = depth_topic
        self.camera_info_topic = camera_info_topic
        self.use_approximate = use_approximate
        self.slop = slop
        
        # 统计信息
        self.callback_count = 0
        self.timestamp_diffs = []
        
        print(f"\n{'='*80}")
        print(f"Topic 同步测试")
        print(f"{'='*80}")
        print(f"RGB Topic:        {rgb_topic}")
        print(f"Depth Topic:      {depth_topic}")
        print(f"CameraInfo Topic: {camera_info_topic}")
        print(f"同步器类型:       {'ApproximateTimeSynchronizer' if use_approximate else 'TimeSynchronizer'}")
        if use_approximate:
            print(f"Slop:             {slop} 秒 ({slop*1000:.1f} ms)")
        print(f"{'='*80}\n")
        
        # 使用 ReentrantCallbackGroup 允许并发回调
        callback_group = ReentrantCallbackGroup()
        
        # 创建订阅者
        rgb_sub = message_filters.Subscriber(
            self, Image, rgb_topic, callback_group=callback_group
        )
        depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, callback_group=callback_group
        )
        camera_info_sub = message_filters.Subscriber(
            self, CameraInfo, camera_info_topic, callback_group=callback_group
        )
        
        # 创建同步器
        if use_approximate:
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [camera_info_sub, rgb_sub, depth_sub],
                queue_size=10,
                slop=slop
            )
            sync_type = "ApproximateTimeSynchronizer"
        else:
            self.sync = message_filters.TimeSynchronizer(
                [camera_info_sub, rgb_sub, depth_sub],
                queue_size=10
            )
            sync_type = "TimeSynchronizer"
        
        self.sync.registerCallback(self.sync_callback)
        
        self.get_logger().info(
            f"初始化 {sync_type}，订阅三个 topic"
        )
        
        # 添加单独的订阅回调用于诊断
        self.rgb_msg_count = 0
        self.depth_msg_count = 0
        self.camera_info_msg_count = 0
        self.last_rgb_timestamp = None
        self.last_depth_timestamp = None
        self.last_camera_info_timestamp = None
        
        # 创建单独的订阅者用于诊断
        self.rgb_diag_sub = self.create_subscription(
            Image, rgb_topic,
            self.rgb_diag_callback,
            10, callback_group=callback_group
        )
        self.depth_diag_sub = self.create_subscription(
            Image, depth_topic,
            self.depth_diag_callback,
            10, callback_group=callback_group
        )
        self.camera_info_diag_sub = self.create_subscription(
            CameraInfo, camera_info_topic,
            self.camera_info_diag_callback,
            10, callback_group=callback_group
        )
        
        self.get_logger().info("已创建诊断订阅者")
    
    def rgb_diag_callback(self, msg: Image):
        """RGB 消息诊断回调"""
        self.rgb_msg_count += 1
        ts_ns = int(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        self.last_rgb_timestamp = ts_ns
        if self.rgb_msg_count <= 3 or self.rgb_msg_count % 30 == 0:
            self.get_logger().info(
                f"[RGB] 消息 #{self.rgb_msg_count}: stamp={ts_ns/1e9:.9f}s"
            )
    
    def depth_diag_callback(self, msg: Image):
        """Depth 消息诊断回调"""
        self.depth_msg_count += 1
        ts_ns = int(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        self.last_depth_timestamp = ts_ns
        if self.depth_msg_count <= 3 or self.depth_msg_count % 30 == 0:
            self.get_logger().info(
                f"[Depth] 消息 #{self.depth_msg_count}: stamp={ts_ns/1e9:.9f}s"
            )
    
    def camera_info_diag_callback(self, msg: CameraInfo):
        """CameraInfo 消息诊断回调"""
        self.camera_info_msg_count += 1
        ts_ns = int(msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec)
        self.last_camera_info_timestamp = ts_ns
        
        # 计算时间戳差异
        if self.last_rgb_timestamp and self.last_depth_timestamp:
            rgb_diff = abs(ts_ns - self.last_rgb_timestamp) / 1e6  # ms
            depth_diff = abs(ts_ns - self.last_depth_timestamp) / 1e6  # ms
            max_diff = max(rgb_diff, depth_diff)
            
            if self.camera_info_msg_count <= 3 or self.camera_info_msg_count % 30 == 0:
                self.get_logger().info(
                    f"[CameraInfo] 消息 #{self.camera_info_msg_count}: "
                    f"stamp={ts_ns/1e9:.9f}s, "
                    f"RGB_diff={rgb_diff:.2f}ms, Depth_diff={depth_diff:.2f}ms, "
                    f"max_diff={max_diff:.2f}ms"
                )
                if self.use_approximate and max_diff > self.slop * 1000:
                    self.get_logger().warning(
                        f"⚠️  时间戳差异 {max_diff:.2f}ms 超过 slop {self.slop*1000:.1f}ms！"
                    )
    
    def sync_callback(self, camera_info_msg: CameraInfo, rgb_msg: Image, depth_msg: Image):
        """同步回调"""
        self.callback_count += 1
        
        # 提取时间戳
        info_ts_ns = int(
            camera_info_msg.header.stamp.sec * 1_000_000_000 +
            camera_info_msg.header.stamp.nanosec
        )
        rgb_ts_ns = int(
            rgb_msg.header.stamp.sec * 1_000_000_000 +
            rgb_msg.header.stamp.nanosec
        )
        depth_ts_ns = int(
            depth_msg.header.stamp.sec * 1_000_000_000 +
            depth_msg.header.stamp.nanosec
        )
        
        # 计算时间戳差异
        rgb_diff = abs(rgb_ts_ns - info_ts_ns) / 1e6  # ms
        depth_diff = abs(depth_ts_ns - info_ts_ns) / 1e6  # ms
        max_diff = max(rgb_diff, depth_diff)
        
        self.timestamp_diffs.append(max_diff)
        
        # 记录前10次和每30次回调
        if self.callback_count <= 10 or self.callback_count % 30 == 0:
            self.get_logger().info(
                f"✅ 同步回调 #{self.callback_count}: "
                f"RGB_diff={rgb_diff:.2f}ms, Depth_diff={depth_diff:.2f}ms, "
                f"max_diff={max_diff:.2f}ms"
            )
    
    def print_statistics(self):
        """打印统计信息"""
        print(f"\n{'='*80}")
        print("测试统计")
        print(f"{'='*80}")
        print(f"同步回调次数:     {self.callback_count}")
        print(f"RGB 消息数:       {self.rgb_msg_count}")
        print(f"Depth 消息数:     {self.depth_msg_count}")
        print(f"CameraInfo 消息数: {self.camera_info_msg_count}")
        
        if self.timestamp_diffs:
            diffs = np.array(self.timestamp_diffs)
            print(f"\n时间戳差异统计 (ms):")
            print(f"  平均差异: {np.mean(diffs):.2f} ms")
            print(f"  最小差异: {np.min(diffs):.2f} ms")
            print(f"  最大差异: {np.max(diffs):.2f} ms")
            print(f"  标准差:   {np.std(diffs):.2f} ms")
        
        if self.use_approximate:
            print(f"\nSlop 设置: {self.slop*1000:.1f} ms")
            if self.timestamp_diffs:
                over_slop = sum(1 for d in self.timestamp_diffs if d > self.slop * 1000)
                print(f"超过 slop 的回调数: {over_slop}/{len(self.timestamp_diffs)}")
        
        print(f"\n{'='*80}\n")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='测试 topic 同步'
    )
    parser.add_argument('--rgb-topic', type=str,
                       default='/hand_camera/zed_node/rgb/image_rect_color',
                       help='RGB image topic')
    parser.add_argument('--depth-topic', type=str,
                       default='/hand_camera/zed_node/depth/depth_registered',
                       help='Depth image topic')
    parser.add_argument('--camera-info-topic', type=str,
                       default='/hand_camera/zed_node/rgb/camera_info',
                       help='Camera info topic')
    parser.add_argument('--use-approximate', action='store_true', default=True,
                       help='使用 ApproximateTimeSynchronizer (默认)')
    parser.add_argument('--use-exact', action='store_true',
                       help='使用 TimeSynchronizer (精确同步)')
    parser.add_argument('--slop', type=float, default=0.1,
                       help='ApproximateTimeSynchronizer 的 slop 值（秒，默认 0.1）')
    parser.add_argument('--duration', type=float, default=30.0,
                       help='测试持续时间（秒，默认 30）')
    
    args = parser.parse_args()
    
    # 如果指定了 --use-exact，使用精确同步
    use_approximate = args.use_approximate and not args.use_exact
    
    rclpy.init()
    
    try:
        # 创建测试节点
        test_node = TopicSyncTestNode(
            rgb_topic=args.rgb_topic,
            depth_topic=args.depth_topic,
            camera_info_topic=args.camera_info_topic,
            use_approximate=use_approximate,
            slop=args.slop
        )
        
        print("⏳ 开始测试，等待同步回调...\n")
        
        # 运行测试
        start_time = time.time()
        last_report_time = start_time
        report_interval = 5.0
        
        while time.time() - start_time < args.duration:
            rclpy.spin_once(test_node, timeout_sec=0.1)
            
            # 定期报告
            current_time = time.time()
            if current_time - last_report_time >= report_interval:
                elapsed = current_time - start_time
                print(f"[{elapsed:.1f}s] "
                      f"同步回调: {test_node.callback_count}, "
                      f"RGB: {test_node.rgb_msg_count}, "
                      f"Depth: {test_node.depth_msg_count}, "
                      f"CameraInfo: {test_node.camera_info_msg_count}")
                last_report_time = current_time
        
        # 打印最终统计
        test_node.print_statistics()
        
        if test_node.callback_count > 0:
            print("✅ 测试成功：同步回调被调用")
            sys.exit(0)
        else:
            print("❌ 测试失败：没有同步回调被调用")
            print("\n可能的原因：")
            print("  1. 消息时间戳差异过大，超过 slop 容忍范围")
            print("  2. 三个 topic 的消息发布频率不一致")
            print("  3. 订阅没有正确建立")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n⚠️  测试被用户中断")
        if 'test_node' in locals():
            test_node.print_statistics()
        sys.exit(130)
    except Exception as e:
        print(f"❌ 测试过程中发生错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()


