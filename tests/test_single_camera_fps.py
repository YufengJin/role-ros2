#!/usr/bin/env python3
"""
测试脚本：检测单个相机的同步接收FPS和丢包情况

测试功能：
- 检测单个相机的同步接收帧率（FPS）
- 检测丢包情况（通过时间戳连续性）
- 统计丢包率
- 分析时间戳间隔分布
- 检测同步延迟
"""

import sys
import time
import argparse
from pathlib import Path
from typing import Dict, List, Tuple, Optional
from collections import deque
import threading

import numpy as np
import rclpy
from rclpy.node import Node

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from role_ros2.camera_utils.ros2_camera_reader import ROS2CameraReader
from role_ros2.camera_utils.multi_camera_wrapper import MultiCameraWrapper


class SingleCameraFPSTestNode(Node):
    """测试节点：检测单个相机的FPS和丢包情况"""
    
    def __init__(self, config_file: str = None, camera_id: str = None):
        """
        初始化测试节点
        
        Args:
            config_file: 相机配置文件路径（可选）
            camera_id: 要测试的相机ID（必需）
        """
        super().__init__('single_camera_fps_test_node')
        
        if camera_id is None:
            self.get_logger().error("❌ 必须指定 camera_id 参数！")
            sys.exit(1)
        
        self.camera_id = camera_id
        
        print(f"\n{'='*80}")
        print(f"单个相机FPS和丢包检测测试")
        print(f"{'='*80}")
        print(f"相机ID: {camera_id}\n")
        
        # 加载相机配置
        if config_file is None:
            config_file = Path(__file__).parent.parent / 'role_ros2' / 'camera_utils' / 'multi_camera_config.yaml'
        else:
            config_file = Path(config_file)
        
        # 初始化多相机包装器以获取相机配置
        try:
            self.camera_wrapper = MultiCameraWrapper(
                node=self,
                config_file=str(config_file)
            )
            self.get_logger().info("MultiCameraWrapper 初始化成功")
            
            # 检查相机是否存在
            if camera_id not in self.camera_wrapper.camera_dict:
                self.get_logger().error(f"❌ 相机 {camera_id} 未找到！")
                self.get_logger().info(f"可用相机: {list(self.camera_wrapper.camera_dict.keys())}")
                sys.exit(1)
            
            self.camera = self.camera_wrapper.camera_dict[camera_id]
            self.get_logger().info(f"✅ 找到相机: {camera_id}")
            
        except Exception as e:
            self.get_logger().error(f"初始化失败: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)
        
        # 统计数据
        self._frame_timestamps: deque = deque(maxlen=10000)  # 存储时间戳（纳秒）
        self._frame_receive_times: deque = deque(maxlen=10000)  # 存储接收时间（纳秒）
        self._frame_intervals: deque = deque(maxlen=10000)  # 存储帧间隔（毫秒）
        self._sync_delays: deque = deque(maxlen=10000)  # 存储同步延迟（毫秒）
        self._stats_lock = threading.Lock()
        
        # 回调计数器（用于监控同步回调是否被调用）
        self._callback_count = 0
        self._last_callback_time = None
    
    def test_camera_fps(
        self,
        duration: float = 30.0,
        report_interval: float = 1.0,
        expected_fps: float = 15.0
    ):
        """
        测试相机FPS和丢包
        
        Args:
            duration: 测试持续时间（秒）
            report_interval: 报告间隔（秒）
            expected_fps: 期望的FPS（用于计算丢包率）
        """
        print(f"\n{'='*80}")
        print(f"开始FPS和丢包检测测试")
        print(f"{'='*80}")
        print(f"测试持续时间: {duration} 秒")
        print(f"报告间隔: {report_interval} 秒")
        print(f"期望FPS: {expected_fps} Hz")
        print(f"{'='*80}\n")
        
        # 等待相机开始接收数据
        print("⏳ 等待相机开始接收数据...")
        wait_start = time.time()
        max_wait = 10.0
        
        # 定期打印诊断信息
        last_diag_time = wait_start
        diag_interval = 2.0
        
        while time.time() - wait_start < max_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # 定期打印诊断信息
            current_time = time.time()
            if current_time - last_diag_time >= diag_interval:
                diag = self.camera.get_diagnostics()
                print(f"   [{current_time - wait_start:.1f}s] 诊断信息:")
                print(f"      回调次数: {diag['callback_count']}")
                print(f"      队列大小: {diag['queue_size']}")
                print(f"      运行状态: {diag['is_running']}")
                print(f"      Slop设置: {diag.get('slop', 'unknown')} 秒")
                if diag.get('subscriber_info'):
                    print(f"      订阅主题: RGB={diag['subscriber_info'].get('rgb_topic', 'unknown')}")
                last_diag_time = current_time
            
            if self.camera.is_running():
                elapsed = time.time() - wait_start
                print(f"✅ 相机开始接收数据！(等待了 {elapsed:.2f} 秒)\n")
                break
            time.sleep(0.1)
        else:
            print("❌ 错误：相机在超时时间内未开始接收数据")
            # 打印最终诊断信息
            diag = self.camera.get_diagnostics()
            print(f"\n📊 最终诊断信息:")
            print(f"   回调次数: {diag['callback_count']}")
            print(f"   队列大小: {diag['queue_size']}")
            print(f"   运行状态: {diag['is_running']}")
            print(f"   Slop设置: {diag.get('slop', 'unknown')} 秒")
            if diag.get('subscriber_info'):
                print(f"   订阅主题:")
                for key, value in diag['subscriber_info'].items():
                    print(f"      {key}: {value}")
            print(f"\n💡 可能的原因:")
            print(f"   1. Slop 设置太小（当前: {diag.get('slop', 'unknown')} 秒），无法匹配时间戳差异")
            print(f"   2. RGB、Depth 或 CameraInfo 消息时间戳差异过大")
            print(f"   3. 订阅的 topic 没有数据发布")
            print(f"   4. ApproximateTimeSynchronizer 无法找到匹配的消息")
            return False
        
        # 开始测试
        test_start_time = time.time()
        last_report_time = test_start_time
        last_frame_count = 0
        
        print("📊 开始收集数据...\n")
        
        while time.time() - test_start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.01)
            
            # 读取相机数据
            data_dict, timestamp_dict = self.camera.read_camera()
            
            if data_dict and timestamp_dict:
                # 提取时间戳
                pub_t_key = f"{self.camera_id}_pub_t"
                sub_t_key = f"{self.camera_id}_sub_t"
                
                if pub_t_key in timestamp_dict:
                    pub_t_ns = timestamp_dict[pub_t_key]
                    
                    with self._stats_lock:
                        # 检查是否是新的帧（避免重复计数）
                        if not self._frame_timestamps or pub_t_ns != self._frame_timestamps[-1]:
                            self._frame_timestamps.append(pub_t_ns)
                            
                            # 计算帧间隔
                            if len(self._frame_timestamps) > 1:
                                prev_timestamp = self._frame_timestamps[-2]
                                interval_ns = pub_t_ns - prev_timestamp
                                interval_ms = interval_ns / 1e6
                                self._frame_intervals.append(interval_ms)
                            
                            # 记录接收时间
                            receive_time_ns = int(time.time() * 1e9)
                            self._frame_receive_times.append(receive_time_ns)
                            
                            # 计算同步延迟（pub_t 到 sub_t）
                            if sub_t_key in timestamp_dict:
                                sub_t_ns = timestamp_dict[sub_t_key]
                                delay_ns = sub_t_ns - pub_t_ns
                                delay_ms = delay_ns / 1e6
                                self._sync_delays.append(delay_ms)
            
            # 定期报告
            current_time = time.time()
            if current_time - last_report_time >= report_interval:
                self._print_interim_report(
                    current_time - test_start_time,
                    expected_fps
                )
                last_report_time = current_time
            
            time.sleep(0.001)  # 短暂休眠，避免CPU占用过高
        
        # 打印最终报告
        print(f"\n{'='*80}")
        print("测试完成，生成最终报告")
        print(f"{'='*80}\n")
        
        self._print_final_report(duration, expected_fps)
        
        return True
    
    def _print_interim_report(self, elapsed_time: float, expected_fps: float):
        """打印中期报告"""
        with self._stats_lock:
            frame_count = len(self._frame_timestamps)
            if frame_count == 0:
                return
            
            # 计算当前FPS
            current_fps = frame_count / elapsed_time if elapsed_time > 0 else 0
            
            # 计算丢包率
            expected_frames = int(elapsed_time * expected_fps)
            if expected_frames > 0:
                packet_loss_rate = (1.0 - frame_count / expected_frames) * 100
            else:
                packet_loss_rate = 0
            
            print(f"[{elapsed_time:.1f}s] "
                  f"帧数: {frame_count} | "
                  f"FPS: {current_fps:.2f} Hz | "
                  f"丢包率: {packet_loss_rate:.2f}%")
    
    def _print_final_report(self, duration: float, expected_fps: float):
        """打印最终报告"""
        with self._stats_lock:
            frame_count = len(self._frame_timestamps)
            
            if frame_count == 0:
                print("❌ 未接收到任何帧！")
                return
            
            # 计算总体FPS
            avg_fps = frame_count / duration if duration > 0 else 0
            
            # 计算丢包率
            expected_frames = int(duration * expected_fps)
            if expected_frames > 0:
                packet_loss_rate = (1.0 - frame_count / expected_frames) * 100
                lost_frames = expected_frames - frame_count
            else:
                packet_loss_rate = 0
                lost_frames = 0
            
            print(f"📊 总体统计:")
            print(f"   测试时长: {duration:.2f} 秒")
            print(f"   接收帧数: {frame_count}")
            print(f"   期望帧数: {expected_frames}")
            print(f"   丢失帧数: {lost_frames}")
            print(f"   平均FPS: {avg_fps:.2f} Hz")
            print(f"   期望FPS: {expected_fps:.2f} Hz")
            print(f"   丢包率: {packet_loss_rate:.2f}%")
            
            # 帧间隔统计
            if len(self._frame_intervals) > 0:
                intervals = np.array(self._frame_intervals)
                avg_interval = np.mean(intervals)
                std_interval = np.std(intervals)
                min_interval = np.min(intervals)
                max_interval = np.max(intervals)
                expected_interval = 1000.0 / expected_fps  # 毫秒
                
                print(f"\n📈 帧间隔统计 (毫秒):")
                print(f"   期望间隔: {expected_interval:.2f} ms")
                print(f"   平均间隔: {avg_interval:.2f} ms")
                print(f"   标准差: {std_interval:.2f} ms")
                print(f"   最小间隔: {min_interval:.2f} ms")
                print(f"   最大间隔: {max_interval:.2f} ms")
                
                # 检测异常间隔（可能是丢包）
                threshold = expected_interval * 1.5  # 超过期望间隔1.5倍认为是异常
                abnormal_intervals = intervals[intervals > threshold]
                if len(abnormal_intervals) > 0:
                    print(f"\n⚠️  检测到 {len(abnormal_intervals)} 个异常间隔 (> {threshold:.2f} ms):")
                    print(f"   异常间隔数量: {len(abnormal_intervals)}")
                    print(f"   异常间隔平均值: {np.mean(abnormal_intervals):.2f} ms")
                    print(f"   异常间隔最大值: {np.max(abnormal_intervals):.2f} ms")
                    # 估算因异常间隔导致的丢包
                    estimated_lost = np.sum(abnormal_intervals) / expected_interval - len(abnormal_intervals)
                    print(f"   估算丢失帧数: {estimated_lost:.1f}")
            
            # 同步延迟统计
            if len(self._sync_delays) > 0:
                delays = np.array(self._sync_delays)
                avg_delay = np.mean(delays)
                std_delay = np.std(delays)
                min_delay = np.min(delays)
                max_delay = np.max(delays)
                
                print(f"\n⏱️  同步延迟统计 (毫秒):")
                print(f"   平均延迟: {avg_delay:.2f} ms")
                print(f"   标准差: {std_delay:.2f} ms")
                print(f"   最小延迟: {min_delay:.2f} ms")
                print(f"   最大延迟: {max_delay:.2f} ms")
            
            # 时间戳连续性检查
            if len(self._frame_timestamps) > 1:
                timestamps = np.array(self._frame_timestamps)
                # 检查时间戳是否单调递增
                is_monotonic = np.all(np.diff(timestamps) > 0)
                if not is_monotonic:
                    print(f"\n⚠️  警告：时间戳不是单调递增的！")
                
                # 检查时间戳跳跃（可能的丢包）
                timestamp_diffs = np.diff(timestamps) / 1e6  # 转换为毫秒
                expected_diff = 1000.0 / expected_fps  # 期望的时间戳差（毫秒）
                large_jumps = timestamp_diffs[timestamp_diffs > expected_diff * 1.5]
                
                if len(large_jumps) > 0:
                    print(f"\n⚠️  检测到 {len(large_jumps)} 个时间戳大跳跃 (> {expected_diff * 1.5:.2f} ms):")
                    print(f"   最大跳跃: {np.max(large_jumps):.2f} ms")
                    print(f"   平均跳跃: {np.mean(large_jumps):.2f} ms")
            
            # 性能评估
            print(f"\n✅ 性能评估:")
            if packet_loss_rate < 1.0:
                print(f"   ✅ 丢包率 < 1%，性能优秀")
            elif packet_loss_rate < 5.0:
                print(f"   ⚠️  丢包率 1-5%，性能良好")
            elif packet_loss_rate < 10.0:
                print(f"   ⚠️  丢包率 5-10%，性能一般")
            else:
                print(f"   ❌ 丢包率 > 10%，性能较差")
            
            fps_ratio = avg_fps / expected_fps if expected_fps > 0 else 0
            if fps_ratio > 0.95:
                print(f"   ✅ FPS 达到期望值的 {fps_ratio*100:.1f}%，性能优秀")
            elif fps_ratio > 0.90:
                print(f"   ⚠️  FPS 达到期望值的 {fps_ratio*100:.1f}%，性能良好")
            else:
                print(f"   ❌ FPS 仅达到期望值的 {fps_ratio*100:.1f}%，性能较差")
            
            print(f"\n{'='*80}\n")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='测试单个相机的FPS和丢包情况'
    )
    parser.add_argument('--config-file', type=str, default=None,
                       help='相机配置文件路径（可选）')
    parser.add_argument('--camera-id', type=str, required=True,
                       help='要测试的相机ID（必需）')
    parser.add_argument('--duration', type=float, default=30.0,
                       help='测试持续时间（秒，默认：30.0）')
    parser.add_argument('--report-interval', type=float, default=1.0,
                       help='报告间隔（秒，默认：1.0）')
    parser.add_argument('--expected-fps', type=float, default=15.0,
                       help='期望的FPS（用于计算丢包率，默认：15.0）')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        # 创建测试节点
        test_node = SingleCameraFPSTestNode(
            config_file=args.config_file,
            camera_id=args.camera_id
        )
        
        # 运行测试
        success = test_node.test_camera_fps(
            duration=args.duration,
            report_interval=args.report_interval,
            expected_fps=args.expected_fps
        )
        
        if success:
            print("\n✅ 测试完成")
            sys.exit(0)
        else:
            print("\n❌ 测试失败")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n⚠️  测试被用户中断")
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

