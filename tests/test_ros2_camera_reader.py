#!/usr/bin/env python3
"""
测试脚本：全面测试 ROS2CameraReader 的所有功能

测试内容：
1. 初始化测试
2. 数据接收测试
3. 数据格式验证
4. 时间戳一致性测试
5. 队列管理测试
6. 读取功能测试
7. 内参获取测试
8. 运行状态检查
"""

import sys
import time
import argparse
from pathlib import Path
from typing import Dict, List, Tuple

import numpy as np
import rclpy
from rclpy.node import Node

# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))

from role_ros2.camera_utils.ros2_camera_reader import ROS2CameraReader
from role_ros2.camera_utils.multi_camera_wrapper import MultiCameraWrapper


class ROS2CameraReaderTestNode(Node):
    """测试节点：全面测试 ROS2CameraReader"""
    
    def __init__(self, config_file: str = None, camera_id: str = None):
        """
        初始化测试节点
        
        Args:
            config_file: 相机配置文件路径（可选）
            camera_id: 要测试的相机ID（必需）
        """
        super().__init__('ros2_camera_reader_test_node')
        
        if camera_id is None:
            self.get_logger().error("❌ 必须指定 camera_id 参数！")
            sys.exit(1)
        
        self.camera_id = camera_id
        
        print(f"\n{'='*80}")
        print(f"ROS2CameraReader 全面功能测试")
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
    
    def test_all_functions(self, duration: float = 30.0):
        """
        测试所有功能
        
        Args:
            duration: 测试持续时间（秒）
        """
        print(f"\n{'='*80}")
        print(f"开始全面功能测试")
        print(f"{'='*80}")
        print(f"测试时长: {duration} 秒\n")
        
        results = {}
        
        # 1. 初始化测试
        print("📋 测试 1: 初始化检查")
        results['init'] = self._test_initialization()
        print()
        
        # 2. 等待数据接收
        print("📋 测试 2: 等待数据接收")
        results['data_reception'] = self._test_data_reception(max_wait=10.0)
        print()
        
        if not results['data_reception']:
            print("❌ 数据接收失败，跳过后续测试")
            self._print_summary(results)
            return False
        
        # 3. 数据格式验证
        print("📋 测试 3: 数据格式验证")
        results['data_format'] = self._test_data_format()
        print()
        
        # 4. 时间戳一致性测试
        print("📋 测试 4: 时间戳一致性测试")
        results['timestamp_consistency'] = self._test_timestamp_consistency(duration)
        print()
        
        # 5. 队列管理测试
        print("📋 测试 5: 队列管理测试")
        results['queue_management'] = self._test_queue_management()
        print()
        
        # 6. 读取功能测试
        print("📋 测试 6: 读取功能测试")
        results['read_function'] = self._test_read_function(num_reads=20)
        print()
        
        # 7. 内参获取测试
        print("📋 测试 7: 内参获取测试")
        results['intrinsics'] = self._test_intrinsics()
        print()
        
        # 8. 运行状态检查
        print("📋 测试 8: 运行状态检查")
        results['running_status'] = self._test_running_status()
        print()
        
        # 打印总结
        self._print_summary(results)
        
        # 判断总体结果
        all_passed = all(results.values())
        return all_passed
    
    def _test_initialization(self) -> bool:
        """测试初始化"""
        try:
            # 检查相机对象是否存在
            if self.camera is None:
                print("  ❌ 相机对象为 None")
                return False
            
            # 检查相机ID
            if self.camera.camera_id != self.camera_id:
                print(f"  ❌ 相机ID不匹配: 期望 {self.camera_id}, 实际 {self.camera.camera_id}")
                return False
            
            # 检查同步器是否存在
            if not hasattr(self.camera, '_sync'):
                print("  ❌ 同步器不存在")
                return False
            
            # 检查同步器类型
            sync_type = type(self.camera._sync).__name__
            if 'TimeSynchronizer' not in sync_type:
                print(f"  ⚠️  同步器类型: {sync_type} (期望 TimeSynchronizer)")
            else:
                print(f"  ✅ 同步器类型: {sync_type}")
            
            print("  ✅ 初始化检查通过")
            return True
        except Exception as e:
            print(f"  ❌ 初始化检查失败: {e}")
            return False
    
    def _test_data_reception(self, max_wait: float = 10.0) -> bool:
        """测试数据接收"""
        # 先给订阅一些时间建立（预热）
        print("  ⏳ 等待订阅建立（预热 2 秒）...")
        warmup_start = time.time()
        while time.time() - warmup_start < 2.0:  # 等待2秒让订阅建立
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)  # 减少 sleep 时间，提高 spin 频率
        
        wait_start = time.time()
        last_log_time = wait_start
        
        while time.time() - wait_start < max_wait:
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # 同时检查回调计数和队列大小
            callback_count = getattr(self.camera, '_callback_count', 0)
            with self.camera._data_lock:
                queue_size = len(self.camera._data_queue)
            
            # 如果回调被调用或队列有数据，认为成功
            if callback_count > 0 or queue_size > 0:
                elapsed = time.time() - wait_start
                total_elapsed = time.time() - warmup_start
                print(f"  ✅ 相机开始接收数据 (等待了 {elapsed:.2f} 秒，总计 {total_elapsed:.2f} 秒)")
                print(f"  ✅ 回调次数: {callback_count}")
                print(f"  ✅ 队列大小: {queue_size}")
                return True
            
            # 每2秒打印一次状态
            current_time = time.time()
            if current_time - last_log_time >= 2.0:
                elapsed = current_time - wait_start
                print(f"  ⏳ [{elapsed:.1f}s] 等待数据... (回调次数: {callback_count}, 队列大小: {queue_size})")
                last_log_time = current_time
            
            time.sleep(0.05)  # 减少 sleep 时间，提高 spin 频率
        
        print(f"  ❌ 超时：{max_wait} 秒内未接收到数据")
        callback_count = getattr(self.camera, '_callback_count', 0)
        with self.camera._data_lock:
            queue_size = len(self.camera._data_queue)
        print(f"  📊 回调次数: {callback_count}, 队列大小: {queue_size}")
        return False
    
    def _test_data_format(self) -> bool:
        """测试数据格式"""
        try:
            # 读取数据
            data_dict, timestamp_dict = self.camera.read_camera()
            
            if not data_dict or not timestamp_dict:
                print("  ❌ 读取的数据为空")
                return False
            
            # 检查必需的键
            required_keys = ["image", "depth"]
            for key in required_keys:
                if key not in data_dict:
                    print(f"  ❌ 缺少键: {key}")
                    return False
            
            # 检查图像数据
            if self.camera_id not in data_dict["image"]:
                print(f"  ❌ 图像数据中缺少相机ID: {self.camera_id}")
                return False
            
            image = data_dict["image"][self.camera_id]
            if image is None:
                print("  ❌ 图像数据为 None")
                return False
            
            if not isinstance(image, np.ndarray):
                print(f"  ❌ 图像数据类型错误: {type(image)}")
                return False
            
            if len(image.shape) != 3 or image.shape[2] != 3:
                print(f"  ❌ 图像形状错误: {image.shape} (期望 HxWx3)")
                return False
            
            if image.dtype != np.uint8:
                print(f"  ❌ 图像数据类型错误: {image.dtype} (期望 uint8)")
                return False
            
            # 检查深度数据
            if self.camera_id not in data_dict["depth"]:
                print(f"  ❌ 深度数据中缺少相机ID: {self.camera_id}")
                return False
            
            depth = data_dict["depth"][self.camera_id]
            if depth is None:
                print("  ❌ 深度数据为 None")
                return False
            
            if not isinstance(depth, np.ndarray):
                print(f"  ❌ 深度数据类型错误: {type(depth)}")
                return False
            
            if len(depth.shape) != 2:
                print(f"  ❌ 深度形状错误: {depth.shape} (期望 HxW)")
                return False
            
            if depth.dtype != np.uint16:
                print(f"  ❌ 深度数据类型错误: {depth.dtype} (期望 uint16)")
                return False
            
            print(f"  ✅ 图像格式: {image.shape}, {image.dtype}, range=[{image.min()}, {image.max()}]")
            print(f"  ✅ 深度格式: {depth.shape}, {depth.dtype}, range=[{depth.min()}, {depth.max()}]")
            print("  ✅ 数据格式验证通过")
            return True
            
        except Exception as e:
            print(f"  ❌ 数据格式验证失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _test_timestamp_consistency(self, duration: float) -> bool:
        """测试时间戳一致性"""
        try:
            timestamps = []
            test_start = time.time()
            sample_count = 0
            max_samples = 100
            
            print(f"  📊 收集时间戳样本 (最多 {max_samples} 个)...")
            
            while time.time() - test_start < duration and sample_count < max_samples:
                rclpy.spin_once(self, timeout_sec=0.1)
                
                data_dict, timestamp_dict = self.camera.read_camera()
                if data_dict and timestamp_dict:
                    pub_t_key = f"{self.camera_id}_pub_t"
                    if pub_t_key in timestamp_dict:
                        timestamps.append(timestamp_dict[pub_t_key])
                        sample_count += 1
                
                time.sleep(0.1)
            
            if len(timestamps) < 2:
                print(f"  ⚠️  样本数量不足: {len(timestamps)}")
                return False
            
            # 检查时间戳是否单调递增
            is_monotonic = all(timestamps[i] < timestamps[i+1] for i in range(len(timestamps)-1))
            
            if not is_monotonic:
                print("  ❌ 时间戳不是单调递增的")
                return False
            
            # 计算时间戳间隔
            intervals = [(timestamps[i+1] - timestamps[i]) / 1e6 for i in range(len(timestamps)-1)]
            avg_interval = np.mean(intervals)
            std_interval = np.std(intervals)
            
            print(f"  ✅ 时间戳样本数: {len(timestamps)}")
            print(f"  ✅ 时间戳单调递增: {is_monotonic}")
            print(f"  ✅ 平均间隔: {avg_interval:.2f} ms")
            print(f"  ✅ 标准差: {std_interval:.2f} ms")
            
            # 检查时间戳是否一致（同步后的消息应该有相同的时间戳）
            # 读取最新数据并检查时间戳
            data_dict, timestamp_dict = self.camera.read_camera()
            if timestamp_dict:
                pub_t = timestamp_dict.get(f"{self.camera_id}_pub_t")
                sub_t = timestamp_dict.get(f"{self.camera_id}_sub_t")
                if pub_t and sub_t:
                    diff_ms = abs(sub_t - pub_t) / 1e6
                    print(f"  ✅ 发布-订阅时间戳差异: {diff_ms:.2f} ms")
            
            print("  ✅ 时间戳一致性测试通过")
            return True
            
        except Exception as e:
            print(f"  ❌ 时间戳一致性测试失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _test_queue_management(self) -> bool:
        """测试队列管理"""
        try:
            with self.camera._data_lock:
                queue_size = len(self.camera._data_queue)
                queue_max_size = self.camera._queue_max_size
                timestamp_order_size = len(self.camera._timestamp_order)
            
            print(f"  📊 当前队列大小: {queue_size}")
            print(f"  📊 最大队列大小: {queue_max_size}")
            print(f"  📊 时间戳顺序队列大小: {timestamp_order_size}")
            
            if queue_size > queue_max_size:
                print(f"  ❌ 队列大小超过最大值: {queue_size} > {queue_max_size}")
                return False
            
            if queue_size != timestamp_order_size:
                print(f"  ⚠️  队列大小与时间戳顺序队列大小不一致")
            
            print("  ✅ 队列管理测试通过")
            return True
            
        except Exception as e:
            print(f"  ❌ 队列管理测试失败: {e}")
            return False
    
    def _test_read_function(self, num_reads: int = 20) -> bool:
        """测试读取功能"""
        try:
            successful_reads = 0
            failed_reads = 0
            
            print(f"  📊 执行 {num_reads} 次读取测试...")
            
            for i in range(num_reads):
                rclpy.spin_once(self, timeout_sec=0.1)
                
                data_dict, timestamp_dict = self.camera.read_camera()
                
                if data_dict and timestamp_dict:
                    # 验证数据完整性
                    if ("image" in data_dict and self.camera_id in data_dict["image"] and
                        "depth" in data_dict and self.camera_id in data_dict["depth"]):
                        successful_reads += 1
                    else:
                        failed_reads += 1
                else:
                    failed_reads += 1
                
                time.sleep(0.05)
            
            success_rate = successful_reads / num_reads * 100 if num_reads > 0 else 0
            
            print(f"  📊 成功读取: {successful_reads}/{num_reads} ({success_rate:.1f}%)")
            print(f"  📊 失败读取: {failed_reads}/{num_reads}")
            
            if success_rate >= 90:
                print("  ✅ 读取功能测试通过")
                return True
            else:
                print(f"  ⚠️  成功率较低: {success_rate:.1f}%")
                return False
                
        except Exception as e:
            print(f"  ❌ 读取功能测试失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _test_intrinsics(self) -> bool:
        """测试内参获取"""
        try:
            intrinsics = self.camera.get_intrinsics()
            
            if intrinsics is None:
                print("  ⚠️  内参为 None（可能尚未接收到 CameraInfo 消息）")
                return False
            
            # 检查必需的键
            required_keys = ["K", "cameraMatrix", "width", "height"]
            for key in required_keys:
                if key not in intrinsics:
                    print(f"  ❌ 内参缺少键: {key}")
                    return False
            
            K = intrinsics.get("K")
            if K is not None:
                if not isinstance(K, np.ndarray):
                    print(f"  ❌ 内参矩阵类型错误: {type(K)}")
                    return False
                if K.shape != (3, 3):
                    print(f"  ❌ 内参矩阵形状错误: {K.shape} (期望 3x3)")
                    return False
                print(f"  ✅ 内参矩阵: {K.shape}")
            
            width = intrinsics.get("width")
            height = intrinsics.get("height")
            print(f"  ✅ 图像尺寸: {width}x{height}")
            
            print("  ✅ 内参获取测试通过")
            return True
            
        except Exception as e:
            print(f"  ❌ 内参获取测试失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _test_running_status(self) -> bool:
        """测试运行状态"""
        try:
            is_running = self.camera.is_running()
            
            if not is_running:
                print("  ❌ 相机未运行")
                return False
            
            # 检查最新时间戳
            latest_timestamp = self.camera.get_latest_pub_timestamp()
            if latest_timestamp is None:
                print("  ⚠️  最新时间戳为 None")
            else:
                print(f"  ✅ 最新时间戳: {latest_timestamp} ({latest_timestamp/1e9:.9f} s)")
            
            # 检查回调计数
            callback_count = getattr(self.camera, '_callback_count', 0)
            print(f"  ✅ 回调次数: {callback_count}")
            
            print("  ✅ 运行状态检查通过")
            return True
            
        except Exception as e:
            print(f"  ❌ 运行状态检查失败: {e}")
            return False
    
    def _print_summary(self, results: Dict[str, bool]):
        """打印测试总结"""
        print(f"\n{'='*80}")
        print("测试总结")
        print(f"{'='*80}")
        
        test_names = {
            'init': '初始化检查',
            'data_reception': '数据接收',
            'data_format': '数据格式验证',
            'timestamp_consistency': '时间戳一致性',
            'queue_management': '队列管理',
            'read_function': '读取功能',
            'intrinsics': '内参获取',
            'running_status': '运行状态检查'
        }
        
        passed = 0
        total = len(results)
        
        for key, result in results.items():
            name = test_names.get(key, key)
            status = "✅ 通过" if result else "❌ 失败"
            print(f"  {name}: {status}")
            if result:
                passed += 1
        
        print(f"\n总体结果: {passed}/{total} 测试通过")
        
        if passed == total:
            print("🎉 所有测试通过！")
        else:
            print(f"⚠️  {total - passed} 个测试失败")
        
        print(f"{'='*80}\n")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='全面测试 ROS2CameraReader 的所有功能'
    )
    parser.add_argument('--config-file', type=str, default=None,
                       help='相机配置文件路径（可选）')
    parser.add_argument('--camera-id', type=str, required=True,
                       help='要测试的相机ID（必需）')
    parser.add_argument('--duration', type=float, default=30.0,
                       help='测试持续时间（秒，默认：30.0）')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        # 创建测试节点
        test_node = ROS2CameraReaderTestNode(
            config_file=args.config_file,
            camera_id=args.camera_id
        )
        
        # 运行测试
        success = test_node.test_all_functions(duration=args.duration)
        
        if success:
            print("\n✅ 所有测试通过")
            sys.exit(0)
        else:
            print("\n❌ 部分测试失败")
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

