#!/usr/bin/env python3
"""
Vehicle-Type-Aware Takeoff Action Server
Supports MC, FW, and VTOL takeoff with automatic vehicle type detection
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

# Action Definition
from px4_ros2_actions.action import Takeoff

# Vehicle Configuration and Strategies
from vehicle_config import VehicleConfig, VehicleType
from vehicle_strategies import MCStrategy, FWStrategy, VTOLStrategy


class TakeoffActionServer(Node):
    def __init__(self):
        super().__init__('takeoff_action_server')

        # Load vehicle configuration
        self.vehicle_config = VehicleConfig()
        self.get_logger().info(f"Loaded vehicle configuration: {self.vehicle_config}")
        
        # Create vehicle-specific strategy
        self.strategy = self._create_strategy()
        self.get_logger().info(f"Using strategy: {self.strategy.__class__.__name__}")

        # Action Server
        self._action_server = ActionServer(
            self,
            Takeoff,
            'takeoff',
            self.execute_callback,
            goal_callback=self.goal_callback,
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

        self.get_logger().info(f'Takeoff Action Server started for {self.vehicle_config.vehicle_type.value} vehicle')

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
        # self.get_logger().info(f"ACK Received for Cmd {msg.command}: Result {msg.result}")
        if msg.result != 0:
            self.get_logger().warn(f"Command {msg.command} REJECTED with code {msg.result}")

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def vtol_vehicle_status_callback(self, msg):
        self.vtol_vehicle_status = msg

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Received takeoff goal: Altitude {goal_request.altitude}m')
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

    def execute_callback(self, goal_handle):
        """Main action execution callback - delegates to vehicle strategy"""
        self.get_logger().info(f'Executing {self.vehicle_config.vehicle_type.value} takeoff...')
        
        target_altitude = goal_handle.request.altitude
        goal = goal_handle.request
        
        # Ensure vehicle is armed
        if not self.ensure_armed():
            result = Takeoff.Result()
            result.outcome = False
            result.message = "Arming failed"
            goal_handle.abort()
            return result

        # Execute pre-takeoff actions (vehicle-specific)
        error = self.strategy.execute_pre_takeoff(goal_handle, self)
        if error:
            result = Takeoff.Result()
            result.outcome = False
            result.message = f"Pre-takeoff failed: {error}"
            goal_handle.abort()
            return result

        # Get vehicle-specific takeoff command parameters
        params = self.strategy.get_takeoff_command_params(goal)
        
        # Send takeoff command
        if not self.send_takeoff_command(params):
            result = Takeoff.Result()
            result.outcome = False
            result.message = "Takeoff command rejected"
            goal_handle.abort()
            return result
        
        # Monitor takeoff progress (MC phase)
        result = self.monitor_takeoff(goal_handle, target_altitude)
        
        # Execute post-takeoff actions (e.g., VTOL transition)
        if result.outcome:
            error = self.strategy.execute_post_takeoff(goal_handle, self)
            if error:
                # Don't abort - goal already succeeded from monitor_takeoff
                # Just update the result message
                self.get_logger().error(f"Post-takeoff failed: {error}")
                result.outcome = False
                result.message = f"Takeoff succeeded but post-takeoff failed: {error}"
                return result
            
            # For VTOL: If target altitude > MC altitude, continue monitoring in FW
            if self.vehicle_config.vehicle_type.value == "VTOL":
                mc_alt = self.strategy.config_params.get('mc_takeoff_altitude', 50.0)
                auto_transition = self.strategy.config_params.get('auto_transition', True)
                
                if auto_transition and target_altitude > mc_alt:
                    self.get_logger().info(f"Continuing climb in FW mode to {target_altitude}m...")
                    result = self.monitor_fw_climb(goal_handle, target_altitude)
        
        return result

    def ensure_armed(self):
        """Arms the vehicle and waits for ACK"""
        if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.get_logger().info("Vehicle already ARMED")
            return True

        self.get_logger().info("Sending Arm command...")
        
        # Clear previous ACK and event
        self.latest_ack = None
        self.ack_event.clear()
        
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        
        # Wait for ACK event (blocks until ACK arrives or timeout)
        if self.ack_event.wait(timeout=10.0):
            if self.latest_ack and self.latest_ack.command == VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM:
                if self.latest_ack.result == 0:
                    self.get_logger().info("Arm command ACCEPTED")
                    return True
                else:
                    self.get_logger().error(f"Arm command REJECTED: {self.latest_ack.result}")
                    return False
        
        self.get_logger().error("Arm command ACK timed out!")
        return False

    def send_takeoff_command(self, params: dict):
        """Sends takeoff command and waits for ACK"""
        self.get_logger().info("Sending Takeoff command...")
        
        # Clear previous ACK and event
        self.latest_ack = None
        self.ack_event.clear()
        
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
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
            if self.latest_ack and self.latest_ack.command == VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF:
                if self.latest_ack.result == 0:
                    self.get_logger().info("Takeoff command ACCEPTED")
                    return True
                else:
                    self.get_logger().warn(f"Takeoff command REJECTED: {self.latest_ack.result}")
                    return False
        
        self.get_logger().warn("Takeoff command ACK timed out!")
        return False

    def monitor_takeoff(self, goal_handle, target_altitude):
        """Monitors altitude during takeoff using vehicle-specific strategy"""
        feedback_msg = Takeoff.Feedback()
        result = Takeoff.Result()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Takeoff.Result(outcome=False, message="Canceled")

            # Check if takeoff complete (vehicle-specific logic)
            is_complete, feedback_state = self.strategy.check_takeoff_complete(
                self.current_altitude, 
                target_altitude,
                self.vehicle_status,
                self
            )

            # Publish feedback
            feedback_msg.current_altitude = self.current_altitude
            feedback_msg.state = feedback_state
            goal_handle.publish_feedback(feedback_msg)

            if is_complete:
                self.get_logger().info(f'Takeoff complete: {feedback_state}')
                break
            
            time.sleep(0.1)

        goal_handle.succeed()
        result.outcome = True
        result.message = "Takeoff complete"
        return result

    def monitor_fw_climb(self, goal_handle, target_altitude):
        """Monitors altitude during FW climb after VTOL transition"""
        feedback_msg = Takeoff.Feedback()
        result = Takeoff.Result()
        
        from px4_msgs.msg import VtolVehicleStatus
        
        # Send reposition command to tell PX4 to climb to target altitude
        self.get_logger().info(f"Climbing to {target_altitude}m in FW mode...")
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
            param1=-1.0,  # Ground speed (use default)
            param2=1.0,   # Bitmask: bit 0 = change altitude
            param3=0.0,   # Reserved
            param4=float('nan'),  # Yaw (keep current)
            param5=float('nan'),  # Latitude (keep current)
            param6=float('nan'),  # Longitude (keep current)
            param7=target_altitude  # Target altitude
        )
        
        time.sleep(1.0)  # Give PX4 time to process command
        
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('FW climb canceled')
                return Takeoff.Result(outcome=False, message="Canceled during FW climb")

            # Publish feedback
            feedback_msg.current_altitude = self.current_altitude
            feedback_msg.state = f"FW mode: Climbing to {target_altitude}m"
            goal_handle.publish_feedback(feedback_msg)

            # Check if reached target altitude in FW mode
            if self.vtol_vehicle_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW:
                if abs(self.current_altitude - target_altitude) < self.strategy.get_altitude_tolerance():
                    self.get_logger().info(f'Target altitude {target_altitude}m reached in FW mode')
                    break
            
            time.sleep(0.5)  # Check every 0.5s (slower than MC monitoring)

        # Don't call succeed() - goal already succeeded from monitor_takeoff
        result.outcome = True
        result.message = f"VTOL takeoff complete at {target_altitude}m in FW mode"
        return result


def main(args=None):
    rclpy.init(args=args)
    action_server = TakeoffActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(action_server)
    executor.spin()

if __name__ == '__main__':
    main()
