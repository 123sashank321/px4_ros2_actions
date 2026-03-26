#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from px4_ros2_actions.action import Arm, Disarm, Takeoff, Land, GoToPosition, SetMode, ExecuteMission

class MissionExecutorServer(Node):
    def __init__(self):
        super().__init__('mission_executor_server')

        # Action Server
        self._action_server = ActionServer(
            self, ExecuteMission, 'execute_mission', self.execute_callback)

        # Action Clients
        self.arm_client = ActionClient(self, Arm, 'arm')
        self.disarm_client = ActionClient(self, Disarm, 'disarm')
        self.takeoff_client = ActionClient(self, Takeoff, 'takeoff')
        self.land_client = ActionClient(self, Land, 'land')
        self.goto_client = ActionClient(self, GoToPosition, 'goto')
        self.set_mode_client = ActionClient(self, SetMode, 'set_mode')
        
        self.get_logger().info('Mission Executor initialized')

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing Mission...')
        mission_input = goal_handle.request.mission_name
        
        # Determine file path
        # If absolute path, use it. If filename, look in package share 'missions' dir.
        if os.path.isabs(mission_input):
            mission_file = mission_input
        else:
            # Look in installed package share directory
            try:
                pkg_share = get_package_share_directory('px4_ros2_actions')
                mission_file = os.path.join(pkg_share, 'missions', mission_input)
                if not mission_file.endswith('.yaml'):
                    mission_file += '.yaml'
            except Exception as e:
                self.get_logger().error(f"Could not find package path: {e}")
                goal_handle.abort()
                return ExecuteMission.Result(outcome=False, message="Package not found")

        self.get_logger().info(f"Loading mission from: {mission_file}")

        try:
            with open(mission_file, 'r') as f:
                data = yaml.safe_load(f)
                mission_steps = data.get('mission', [])
        except FileNotFoundError:
            self.get_logger().error(f"Mission file not found: {mission_file}")
            goal_handle.abort()
            return ExecuteMission.Result(outcome=False, message=f"File not found: {mission_file}")
        except Exception as e:
            self.get_logger().error(f"Error parsing mission file: {e}")
            goal_handle.abort()
            return ExecuteMission.Result(outcome=False, message=f"Parse error: {str(e)}")

        if not mission_steps:
             self.get_logger().warn("Mission file is empty or missing 'mission' key")
             
        total_steps = len(mission_steps)
        completed_steps = 0
        
        for step in mission_steps:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = ExecuteMission.Result()
                result.outcome = False
                result.message = "Mission Cancelled"
                result.steps_completed = completed_steps
                return result

            action_name = step['action']
            params = step.get('params', {})
            
            # Feedback
            feedback = ExecuteMission.Feedback()
            feedback.current_step = action_name
            feedback.progress = float(completed_steps) / total_steps
            goal_handle.publish_feedback(feedback)
            
            # Execute Step
            success = self.run_step(action_name, params)
            
            if not success:
                goal_handle.abort()
                result = ExecuteMission.Result()
                result.outcome = False
                result.message = f"Failed at step: {action_name}"
                result.steps_completed = completed_steps
                return result
                
            completed_steps += 1
            
        goal_handle.succeed()
        result = ExecuteMission.Result()
        result.outcome = True
        result.message = "Mission Completed Successfully"
        result.steps_completed = completed_steps
        return result

    def run_step(self, action_name, params):
        self.get_logger().info(f"Running step: {action_name}")
        
        if action_name == 'arm':
            return self.call_action(self.arm_client, Arm.Goal())
            
        elif action_name == 'disarm':
            return self.call_action(self.disarm_client, Disarm.Goal())
            
        elif action_name == 'takeoff':
            goal = Takeoff.Goal()
            goal.altitude = float(params.get('altitude', 2.5))
            goal.latitude = float(params.get('latitude', float('nan')))
            goal.longitude = float(params.get('longitude', float('nan')))
            goal.yaw = float(params.get('yaw', float('nan')))
            goal.pitch = float(params.get('pitch', float('nan')))
            return self.call_action(self.takeoff_client, goal)
            
        elif action_name == 'land':
            goal = Land.Goal()
            goal.altitude = float(params.get('altitude', float('nan')))
            goal.latitude = float(params.get('latitude', float('nan')))
            goal.longitude = float(params.get('longitude', float('nan')))
            goal.yaw = float(params.get('yaw', float('nan')))
            return self.call_action(self.land_client, goal)
            
        elif action_name == 'goto':
            goal = GoToPosition.Goal()
            goal.x = float(params.get('x', 0.0))
            goal.y = float(params.get('y', 0.0))
            goal.z = float(params.get('z', 0.0))
            goal.yaw = float(params.get('yaw', 0.0))
            return self.call_action(self.goto_client, goal)
            
        elif action_name == 'set_mode':
            goal = SetMode.Goal()
            goal.mode_name = params.get('mode_name', 'MODE')
            return self.call_action(self.set_mode_client, goal)
            
        else:
            self.get_logger().error(f"Unknown action: {action_name}")
            return False

    def call_action(self, client, goal):
        if not client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"Action server not available")
            return False
            
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False
            
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        return result.outcome

def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutorServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()
