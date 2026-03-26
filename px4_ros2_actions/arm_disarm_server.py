#!/usr/bin/env python3

import threading
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus
from px4_ros2_actions.action import Arm, Disarm

class ArmDisarmServer(Node):
    def __init__(self):
        super().__init__('arm_disarm_server')

        # Action Servers
        self._arm_server = ActionServer(
            self, Arm, 'arm', self.arm_callback)
            
        self._disarm_server = ActionServer(
            self, Disarm, 'disarm', self.disarm_callback)

        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscriber
        self.vehicle_command_ack_subscriber = self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.ack_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', self.status_callback, qos_profile)
            
        self.latest_ack = None
        self.ack_event = threading.Event()
        self.current_arming_state = 0

    def ack_callback(self, msg):
        self.latest_ack = msg
        self.ack_event.set()

    def status_callback(self, msg):
        self.current_arming_state = msg.arming_state

    def publish_command(self, command, param1=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def wait_for_ack(self, expected_command, timeout=5.0):
        start_time = time.time()
        self.ack_event.clear()
        self.latest_ack = None
        
        while (time.time() - start_time) < timeout:
            if self.ack_event.wait(timeout=1.0):
                self.ack_event.clear()
                if self.latest_ack and self.latest_ack.command == expected_command:
                    return self.latest_ack.result == 0 # 0 is ACCEPTED
        return False

    def arm_callback(self, goal_handle):
        self.get_logger().info('Executing Arm...')
        
        # Check current state (2 is ARMED)
        if self.current_arming_state == 2:
            self.get_logger().info("Vehicle is already ARMED")
            goal_handle.succeed()
            result = Arm.Result()
            result.outcome = True
            result.message = "Vehicle is already in Armed state"
            return result
        
        self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        
        success = self.wait_for_ack(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
        
        result = Arm.Result()
        result.outcome = success
        result.message = "Arming " + ("Succeeded" if success else "Failed")
        
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
            
        return result

    def disarm_callback(self, goal_handle):
        self.get_logger().info('Executing Disarm...')
        
        # Check current state (1 is DISARMED)
        if self.current_arming_state == 1:
            self.get_logger().info("Vehicle is already DISARMED")
            goal_handle.succeed()
            result = Disarm.Result()
            result.outcome = True
            result.message = "Vehicle is already in Disarmed state"
            return result
            
        self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        
        success = self.wait_for_ack(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM)
        
        result = Disarm.Result()
        result.outcome = success
        result.message = "Disarming " + ("Succeeded" if success else "Failed")
        
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
            
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ArmDisarmServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()
