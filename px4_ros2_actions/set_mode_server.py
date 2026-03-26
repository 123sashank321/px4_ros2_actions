#!/usr/bin/env python3

import threading
import time
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus
from px4_ros2_actions.action import SetMode

class SetModeServer(Node):
    def __init__(self):
        super().__init__('set_mode_server')

        # Action Server
        self._action_server = ActionServer(
            self, SetMode, 'set_mode', self.execute_callback)

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
            VehicleStatus, '/fmu/out/vehicle_status', self.status_callback, qos_profile)

        self.latest_ack = None
        self.ack_event = threading.Event()
        self.nav_state = 0

    def ack_callback(self, msg):
        self.latest_ack = msg
        self.ack_event.set()

    def status_callback(self, msg):
        self.nav_state = msg.nav_state

    def publish_command(self, command, param1=0.0, param2=0.0, param3=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
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
                    return self.latest_ack.result == 0
        return False

    def execute_callback(self, goal_handle):
        mode_request = goal_handle.request.mode_name.upper()
        self.get_logger().info(f'Switching Mode to {mode_request}...')

        # Mapping: 'MODE_NAME': (Main_Mode, Sub_Mode)
        # Ref: PX4 Flight Mode State Machine
        px4_mode_map = {
            'MANUAL':     (1.0, 0.0),
            'ALTITUDE':   (2.0, 0.0),
            'POSITION':   (3.0, 0.0),
            'MISSION':    (4.0, 4.0), # Main 4 (AUTO), Sub 4 (MISSION)
            'HOLD':       (4.0, 5.0), # Main 4 (AUTO), Sub 5 (LOITER)
            'LOITER':     (4.0, 5.0),
            'RTL':        (4.0, 6.0), # Main 4 (AUTO), Sub 6 (RTL)
            'LAND':       (4.0, 7.0), # Main 4 (AUTO), Sub 7 (LAND)
            'ACRO':       (5.0, 0.0),
            'OFFBOARD':   (6.0, 0.0), # Note: Requires setpoint stream > 2Hz
            'STABILIZED': (7.0, 0.0),
        }

        if mode_request not in px4_mode_map:
            self.get_logger().error(f"Unsupported mode: {mode_request}")
            goal_handle.abort()
            result = SetMode.Result()
            result.outcome = False
            result.message = f"Unsupported mode: {mode_request}"
            return result

        main_mode, sub_mode = px4_mode_map[mode_request]

        # Param1 = 1.0 (Custom Mode Flag)
        # Param2 = Main Mode
        # Param3 = Sub Mode
        self.publish_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
            1.0, 
            main_mode, 
            sub_mode
        )
        
        success = self.wait_for_ack(VehicleCommand.VEHICLE_CMD_DO_SET_MODE)
        
        result = SetMode.Result()
        result.outcome = success
        result.message = f"Switch to {mode_request} " + ("Accepted" if success else "Failed")
        
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
            
        return result

def main(args=None):
    rclpy.init(args=args)
    node = SetModeServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()
