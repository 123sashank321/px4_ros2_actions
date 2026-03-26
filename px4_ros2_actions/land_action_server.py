#!/usr/bin/env python3
"""
Vehicle-Type-Aware Land Action Server
Supports MC, FW, and VTOL landing with automatic vehicle type detection
"""

import time
import threading
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor

# PX4 Messages
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleCommandAck, VehicleStatus, VtolVehicleStatus

# Action Definitions
from px4_ros2_actions.action import Land, RTL

# Vehicle Configuration and Strategies
from vehicle_config import VehicleConfig, VehicleType
from vehicle_strategies import MCStrategy, FWStrategy, VTOLStrategy


class LandActionServer(Node):
    def __init__(self):
        super().__init__('land_action_server')

        # Load vehicle configuration
        self.vehicle_config = VehicleConfig()
        self.get_logger().info(f"Loaded vehicle configuration: {self.vehicle_config}")
        
        # Create vehicle-specific strategy
        self.strategy = self._create_strategy()
        self.get_logger().info(f"Using strategy: {self.strategy.__class__.__name__}")

        # Land Action Server
        self._land_action_server = ActionServer(
            self,
            Land,
            'land',
            self.land_execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # RTL Action Server
        self._rtl_action_server = ActionServer(
            self,
            RTL,
            'rtl',
            self.rtl_execute_callback,
            goal_callback=self.rtl_goal_callback,
            cancel_callback=self.cancel_callback
        )

        # QoS for PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            qos_profile
        )

        # Subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.vehicle_local_position_callback,
            qos_profile
        )
        self.vehicle_command_ack_subscriber = self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self.vehicle_command_ack_callback,
            qos_profile
        )
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback,
            qos_profile
        )
        self.vtol_vehicle_status_subscriber = self.create_subscription(
            VtolVehicleStatus,
            '/fmu/out/vtol_vehicle_status',
            self.vtol_vehicle_status_callback,
            qos_profile
        )

        # State Variables
        self.vehicle_local_position = VehicleLocalPosition()
        self.current_altitude = 0.0
        self.vehicle_status = VehicleStatus()
        self.vtol_vehicle_status = VtolVehicleStatus()
        self.latest_ack = None
        self.ack_event = threading.Event()

        self.get_logger().info(f'Land Action Server started for {self.vehicle_config.vehicle_type.value} vehicle')

    def _create_strategy(self):
        """Factory method to create the appropriate vehicle strategy"""
        vehicle_type = self.vehicle_config.vehicle_type
        params = self.vehicle_config.get_params_for_current_type()
        
        if vehicle_type == VehicleType.MC:
            return MCStrategy(params)
        elif vehicle_type == VehicleType.FW:
            return FWStrategy(params)
        elif vehicle_type == VehicleType.VTOL:
            return VTOLStrategy(params)
        else:
            raise ValueError(f"Unsupported vehicle type: {vehicle_type}")

    def vehicle_local_position_callback(self, msg):
        self.vehicle_local_position = msg
        self.current_altitude = -msg.z  # Z is down in NED, so -Z is up

    def vehicle_command_ack_callback(self, msg):
        self.latest_ack = msg
        self.ack_event.set()  # Wake up any waiting threads
        if msg.result != 0:
            self.get_logger().warn(f"Command {msg.command} REJECTED with code {msg.result}")

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def vtol_vehicle_status_callback(self, msg):
        self.vtol_vehicle_status = msg

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received land goal')
        return GoalResponse.ACCEPT

    def rtl_goal_callback(self, goal_request):
        self.get_logger().info(f'Received RTL goal')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, 
                                param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        """Publish a vehicle command to PX4"""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.param4 = param4
        msg.param5 = param5
        msg.param6 = param6
        msg.param7 = param7
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def land_execute_callback(self, goal_handle):
        """Main Land action execution callback - delegates to vehicle strategy"""
        self.get_logger().info(f'Executing {self.vehicle_config.vehicle_type.value} landing...')
        
        goal = goal_handle.request
        
        # Execute pre-landing actions (e.g., VTOL transition to MC)
        error = self.strategy.execute_pre_landing(goal_handle, self)
        if error:
            result = Land.Result()
            result.outcome = False
            result.message = f"Pre-landing failed: {error}"
            goal_handle.abort()
            return result

        # Get vehicle-specific land command parameters
        params = self.strategy.get_land_command_params(goal)
        
        # Send land command
        if not self.send_land_command(params):
            result = Land.Result()
            result.outcome = False
            result.message = "Land command rejected"
            goal_handle.abort()
            return result
        
        # Monitor landing progress
        result = self.monitor_landing(goal_handle)
        
        return result

    def rtl_execute_callback(self, goal_handle):
        """RTL action execution callback - Return to Launch"""
        self.get_logger().info(f'Executing RTL (Return to Launch)...')
        
        # Send RTL command
        if not self.send_rtl_command():
            result = RTL.Result()
            result.outcome = False
            result.message = "RTL command rejected"
            goal_handle.abort()
            return result
        
        # Monitor RTL progress
        result = self.monitor_rtl(goal_handle)
        
        return result

    def send_land_command(self, params: dict):
        """Sends land command and waits for ACK"""
        self.get_logger().info("Sending Land command...")
        
        # Clear previous ACK and event
        self.latest_ack = None
        self.ack_event.clear()
        
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
            param1=params.get('param1', 0.0),
            param2=params.get('param2', 0.0),
            param3=params.get('param3', 0.0),
            param4=params.get('param4', 0.0),
            param5=params.get('param5', 0.0),
            param6=params.get('param6', 0.0),
            param7=params.get('param7', 0.0)
        )

        # Wait for ACK event
        if self.ack_event.wait(timeout=10.0):
            if self.latest_ack and self.latest_ack.command == VehicleCommand.VEHICLE_CMD_NAV_LAND:
                if self.latest_ack.result == 0:
                    self.get_logger().info("Land command ACCEPTED")
                    return True
                else:
                    self.get_logger().warn(f"Land command REJECTED: {self.latest_ack.result}")
                    return False
        
        self.get_logger().warn("Land command ACK timed out!")
        return False

    def monitor_landing(self, goal_handle):
        """Monitors altitude during landing using vehicle-specific strategy"""
        feedback_msg = Land.Feedback()
        result = Land.Result()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Land.Result(outcome=False, message="Canceled")

            # Check if landing complete (vehicle-specific logic)
            is_complete, feedback_state = self.strategy.check_landing_complete(
                self.current_altitude,
                self.vehicle_status,
                self
            )

            # Publish feedback
            feedback_msg.current_altitude = self.current_altitude
            feedback_msg.state = feedback_state
            goal_handle.publish_feedback(feedback_msg)

            if is_complete:
                self.get_logger().info(f'Landing complete: {feedback_state}')
                break
            
            time.sleep(0.1)

        goal_handle.succeed()
        result.outcome = True
        result.message = "Landing complete"
        return result

    def send_rtl_command(self):
        """Sends RTL command and waits for ACK"""
        self.get_logger().info("Sending RTL (Return to Launch) command...")
        
        # Clear previous ACK and event
        self.latest_ack = None
        self.ack_event.clear()
        
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

        # Wait for ACK event
        if self.ack_event.wait(timeout=10.0):
            if self.latest_ack and self.latest_ack.command == VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH:
                if self.latest_ack.result == 0:
                    self.get_logger().info("RTL command ACCEPTED")
                    return True
                else:
                    self.get_logger().warn(f"RTL command REJECTED: {self.latest_ack.result}")
                    return False
        
        self.get_logger().warn("RTL command ACK timed out!")
        return False

    def monitor_rtl(self, goal_handle):
        """Monitors RTL progress until vehicle lands"""
        feedback_msg = RTL.Feedback()
        result = RTL.Result()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('RTL canceled')
                return RTL.Result(outcome=False, message="Canceled")

            # Publish feedback
            feedback_msg.current_altitude = self.current_altitude
            feedback_msg.distance_to_home = 0.0  # TODO: Calculate actual distance
            
            # Simple state feedback based on altitude and nav state
            if self.current_altitude < 0.2:
                feedback_msg.state = "Landed"
            elif self.current_altitude < 5.0:
                feedback_msg.state = "Final descent"
            else:
                feedback_msg.state = "Returning to launch"
            
            goal_handle.publish_feedback(feedback_msg)

            # Check if landed (altitude very low)
            if self.current_altitude < 0.2:
                self.get_logger().info(f'RTL complete - Vehicle landed')
                break
            
            time.sleep(0.5)

        goal_handle.succeed()
        result.outcome = True
        result.message = "RTL complete - Vehicle returned and landed"
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = LandActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    executor.spin()

if __name__ == '__main__':
    main()
