#!/usr/bin/env python3
"""
ROS2 Communication Test Script
Tests Topic, Service, and Action communication within a container using ROS2 standard messages.

Usage:
    python3 test_ros2_communication.py [topic|service|action|all]
    
Examples:
    python3 test_ros2_communication.py topic    # Test only topics
    python3 test_ros2_communication.py service  # Test only services
    python3 test_ros2_communication.py action  # Test only actions
    python3 test_ros2_communication.py all      # Test all (default)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import sys

# Standard ROS2 message types
from std_msgs.msg import String, Int32
from std_srvs.srv import SetBool, Empty
from example_interfaces.action import Fibonacci


class TopicTestNode(Node):
    """Test Topic publisher and subscriber"""
    
    def __init__(self):
        super().__init__('topic_test_node')
        
        # Publisher
        self.publisher = self.create_publisher(String, '/test/topic', 10)
        
        # Subscriber
        self.subscription = self.create_subscription(
            String,
            '/test/topic',
            self.topic_callback,
            10
        )
        
        self.received_count = 0
        self.published_count = 0
        
        # Timer to publish messages
        self.timer = self.create_timer(0.5, self.publish_message)
        
        self.get_logger().info('Topic test node started')
        self.get_logger().info('  Publishing to: /test/topic (std_msgs/String)')
        self.get_logger().info('  Subscribing to: /test/topic (std_msgs/String)')
    
    def publish_message(self):
        """Publish a test message"""
        msg = String()
        self.published_count += 1
        msg.data = f'Test message #{self.published_count}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')
    
    def topic_callback(self, msg):
        """Callback for received messages"""
        self.received_count += 1
        self.get_logger().info(f'Received: {msg.data} (count: {self.received_count})')


class ServiceTestNode(Node):
    """Test Service server and client"""
    
    def __init__(self):
        super().__init__('service_test_node')
        
        # Service server
        self.set_bool_srv = self.create_service(
            SetBool,
            '/test/set_bool',
            self.set_bool_callback
        )
        
        self.empty_srv = self.create_service(
            Empty,
            '/test/empty',
            self.empty_callback
        )
        
        # Service client
        self.set_bool_client = self.create_client(SetBool, '/test/set_bool')
        self.empty_client = self.create_client(Empty, '/test/empty')
        
        self.request_count = 0
        self.response_count = 0
        
        self.get_logger().info('Service test node started')
        self.get_logger().info('  Server: /test/set_bool (std_srvs/SetBool)')
        self.get_logger().info('  Server: /test/empty (std_srvs/Empty)')
        self.get_logger().info('  Client: /test/set_bool, /test/empty')
    
    def set_bool_callback(self, request, response):
        """SetBool service callback"""
        self.request_count += 1
        self.get_logger().info(f'[SetBool #{self.request_count}] Received request: data={request.data}')
        
        response.success = True
        response.message = f'Processed SetBool request #{self.request_count}, data={request.data}'
        
        self.get_logger().info(f'  Response: success={response.success}, message={response.message}')
        return response
    
    def empty_callback(self, request, response):
        """Empty service callback"""
        self.request_count += 1
        self.get_logger().info(f'[Empty #{self.request_count}] Received empty request')
        self.get_logger().info(f'  Response: Successfully processed Empty request #{self.request_count}')
        return response
    
    def wait_for_services(self, timeout=5.0):
        """Wait for services to be available"""
        self.get_logger().info('Waiting for services to be available...')
        
        if not self.set_bool_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'Service /test/set_bool not available within {timeout} seconds')
            return False
        
        if not self.empty_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'Service /test/empty not available within {timeout} seconds')
            return False
        
        self.get_logger().info('All services are available')
        return True
    
    def test_services(self, client_executor):
        """Test service calls using separate executor for client"""
        if not self.wait_for_services():
            return False
        
        # Test SetBool service
        self.get_logger().info('\n=== Testing SetBool Service ===')
        request = SetBool.Request()
        request.data = True
        
        self.get_logger().info(f'Sending SetBool request: data={request.data}')
        future = self.set_bool_client.call_async(request)
        
        # Use client executor to wait for response
        timeout = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            client_executor.spin_once(timeout_sec=0.1)
        
        if future.done():
            response = future.result()
            if response:
                self.get_logger().info(f'✓ SetBool response: success={response.success}, message={response.message}')
                self.response_count += 1
            else:
                self.get_logger().error('SetBool request failed: No response')
                return False
        else:
            self.get_logger().error('SetBool request timeout')
            return False
        
        # Test Empty service
        self.get_logger().info('\n=== Testing Empty Service ===')
        request = Empty.Request()
        
        self.get_logger().info('Sending Empty request')
        future = self.empty_client.call_async(request)
        
        # Use client executor to wait for response
        timeout = 5.0
        start_time = time.time()
        while not future.done() and (time.time() - start_time) < timeout:
            client_executor.spin_once(timeout_sec=0.1)
        
        if future.done():
            response = future.result()
            if response is not None:
                self.get_logger().info('✓ Empty response: Success')
                self.response_count += 1
            else:
                self.get_logger().error('Empty request failed: No response')
                return False
        else:
            self.get_logger().error('Empty request timeout')
            return False
        
        return True


class ActionTestNode(Node):
    """Test Action server and client"""
    
    def __init__(self):
        super().__init__('action_test_node')
        
        # Action server
        self.action_server = ActionServer(
            self,
            Fibonacci,
            '/test/fibonacci',
            self.execute_callback
        )
        
        # Action client
        self.action_client = ActionClient(self, Fibonacci, '/test/fibonacci')
        
        self.goal_count = 0
        self.result_count = 0
        
        self.get_logger().info('Action test node started')
        self.get_logger().info('  Server: /test/fibonacci (example_interfaces/Fibonacci)')
        self.get_logger().info('  Client: /test/fibonacci (example_interfaces/Fibonacci)')
    
    def cleanup(self):
        """Explicitly destroy action server and client before node destruction"""
        # Destroy action server and client explicitly to prevent InvalidHandle errors
        if hasattr(self, 'action_server') and self.action_server is not None:
            try:
                self.action_server.destroy()
            except Exception:
                pass
            self.action_server = None
        
        if hasattr(self, 'action_client') and self.action_client is not None:
            try:
                self.action_client.destroy()
            except Exception:
                pass
            self.action_client = None
    
    def execute_callback(self, goal_handle):
        """Execute action goal"""
        self.goal_count += 1
        order = goal_handle.request.order
        self.get_logger().info(f'[Fibonacci #{self.goal_count}] Received goal: order={order}')
        
        # Accept goal
        goal_handle.succeed()
        
        # Generate Fibonacci sequence
        feedback_msg = Fibonacci.Feedback()
        result = Fibonacci.Result()
        
        sequence = [0, 1]
        if order > 0:
            for i in range(2, order + 1):
                sequence.append(sequence[i-1] + sequence[i-2])
                
                # Publish feedback
                feedback_msg.sequence = sequence
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f'  Feedback: sequence length={len(sequence)}')
                
                # Simulate processing time (use time.sleep instead of rclpy.spin_once to avoid executor conflicts)
                time.sleep(0.05)
        
        # Set result
        result.sequence = sequence
        self.get_logger().info(f'✓ Completed: Fibonacci({order}) = {sequence}')
        
        return result
    
    def wait_for_server(self, timeout=5.0):
        """Wait for action server to be available"""
        self.get_logger().info('Waiting for action server to be available...')
        
        if not self.action_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(f'Action server /test/fibonacci not available within {timeout} seconds')
            return False
        
        self.get_logger().info('Action server is available')
        return True
    
    def test_action(self, client_executor):
        """Test action call using separate executor for client"""
        if not self.wait_for_server():
            return False
        
        self.get_logger().info('\n=== Testing Fibonacci Action ===')
        goal_msg = Fibonacci.Goal()
        goal_msg.order = 10
        
        self.get_logger().info(f'Sending Fibonacci goal: order={goal_msg.order}')
        
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        
        # Use client executor to wait for goal acceptance
        while not send_goal_future.done():
            client_executor.spin_once(timeout_sec=0.1)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            return False
        
        self.get_logger().info('Goal accepted, waiting for result...')
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        
        # Use client executor to wait for result
        while not result_future.done():
            client_executor.spin_once(timeout_sec=0.1)
        
        result = result_future.result().result
        if result:
            self.get_logger().info(f'✓ Fibonacci result: {result.sequence}')
            self.result_count += 1
            return True
        else:
            self.get_logger().error('Fibonacci action failed')
            return False


def test_topics(executor, duration=5.0):
    """Test topic communication"""
    print('\n' + '='*60)
    print('TEST 1: Topic Communication')
    print('='*60)
    
    topic_node = TopicTestNode()
    executor.add_node(topic_node)
    
    print(f'Running topic test for {duration} seconds...')
    print('You should see messages being published and received.')
    print('Press Ctrl+C to stop early.\n')
    
    # Run executor in background thread
    def run_executor():
        try:
            executor.spin()
        except Exception:
            pass
    
    executor_thread = threading.Thread(target=run_executor, daemon=True)
    executor_thread.start()
    
    try:
        time.sleep(duration)
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        executor.shutdown()
        executor_thread.join(timeout=1.0)
    
    print(f'\nTopic Test Summary:')
    print(f'  Published: {topic_node.published_count} messages')
    print(f'  Received: {topic_node.received_count} messages')
    
    executor.remove_node(topic_node)
    topic_node.destroy_node()
    
    return topic_node.received_count > 0


def test_services(executor):
    """Test service communication"""
    print('\n' + '='*60)
    print('TEST 2: Service Communication')
    print('='*60)
    
    service_node = ServiceTestNode()
    executor.add_node(service_node)
    
    # Start server executor in background thread
    def run_executor():
        try:
            executor.spin()
        except Exception:
            pass
    
    executor_thread = threading.Thread(target=run_executor, daemon=True)
    executor_thread.start()
    
    # Wait for services to be ready
    time.sleep(2)
    
    # Create separate executor for client operations to avoid conflicts
    client_executor = MultiThreadedExecutor()
    client_executor.add_node(service_node)
    
    # Test services using client executor
    success = service_node.test_services(client_executor)
    
    # Stop executors
    executor.shutdown()
    client_executor.shutdown()
    executor_thread.join(timeout=1.0)
    
    executor.remove_node(service_node)
    client_executor.remove_node(service_node)
    service_node.destroy_node()
    
    print(f'\nService Test Summary:')
    print(f'  Requests received: {service_node.request_count}')
    print(f'  Responses received: {service_node.response_count}')
    
    return success


def test_actions(executor):
    """Test action communication"""
    print('\n' + '='*60)
    print('TEST 3: Action Communication')
    print('='*60)
    
    action_node = ActionTestNode()
    executor.add_node(action_node)
    
    # Start server executor in background thread
    def run_executor():
        try:
            executor.spin()
        except Exception:
            pass
    
    executor_thread = threading.Thread(target=run_executor, daemon=True)
    executor_thread.start()
    
    # Wait for server to be ready
    time.sleep(2)
    
    # Create separate executor for client operations to avoid conflicts
    client_executor = MultiThreadedExecutor()
    client_executor.add_node(action_node)
    
    # Test action using client executor
    success = action_node.test_action(client_executor)
    
    # Stop executors
    executor.shutdown()
    client_executor.shutdown()
    executor_thread.join(timeout=1.0)
    
    executor.remove_node(action_node)
    client_executor.remove_node(action_node)
    
    # Explicitly destroy action server and client before destroying node
    # This prevents InvalidHandle errors during cleanup (especially in ROS2 Foxy)
    action_node.cleanup()
    action_node.destroy_node()
    
    print(f'\nAction Test Summary:')
    print(f'  Goals received: {action_node.goal_count}')
    print(f'  Results received: {action_node.result_count}')
    
    return success


def main():
    """Main function"""
    rclpy.init()
    
    # Parse command line arguments
    test_type = sys.argv[1] if len(sys.argv) > 1 else 'all'
    
    executor = MultiThreadedExecutor()
    
    results = {}
    
    try:
        if test_type in ['topic', 'all']:
            executor = MultiThreadedExecutor()
            results['topic'] = test_topics(executor, duration=5.0)
        
        if test_type in ['service', 'all']:
            executor = MultiThreadedExecutor()
            results['service'] = test_services(executor)
        
        if test_type in ['action', 'all']:
            executor = MultiThreadedExecutor()
            results['action'] = test_actions(executor)
        
        # Print final summary
        print('\n' + '='*60)
        print('FINAL SUMMARY')
        print('='*60)
        
        for test_name, success in results.items():
            status = '✓ PASSED' if success else '✗ FAILED'
            print(f'  {test_name.upper()}: {status}')
        
        all_passed = all(results.values())
        print('\n' + ('='*60))
        if all_passed:
            print('ALL TESTS PASSED ✓')
        else:
            print('SOME TESTS FAILED ✗')
        print('='*60 + '\n')
        
        sys.exit(0 if all_passed else 1)
        
    except KeyboardInterrupt:
        print('\n\nTest interrupted by user')
        sys.exit(130)
    except Exception as e:
        print(f'\n\nError during testing: {e}')
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

