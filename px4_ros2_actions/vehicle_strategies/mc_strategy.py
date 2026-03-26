#!/usr/bin/env python3
"""
Multicopter Strategy
Implements takeoff and landing logic for multicopter vehicles
"""

from typing import Tuple, Optional
from .base_vehicle_strategy import BaseVehicleStrategy
from px4_msgs.msg import VehicleStatus


class MCStrategy(BaseVehicleStrategy):
    """Strategy for Multicopter (MC) vehicles"""
    
    def get_takeoff_command_params(self, goal) -> dict:
        """
        MC Takeoff: Simple vertical climb to target altitude.
        
        Returns:
            param1: pitch (not used for MC, NaN)
            param4: yaw angle (use goal.yaw or NaN for current)
            param5: latitude (use goal.latitude or NaN for current)
            param6: longitude (use goal.longitude or NaN for current)
            param7: altitude (target altitude)
        """
        return {
            'param1': float('nan'),  # Pitch not used for MC
            'param4': goal.yaw if goal.yaw != 0.0 else float('nan'),
            'param5': goal.latitude if goal.latitude != 0.0 else float('nan'),
            'param6': goal.longitude if goal.longitude != 0.0 else float('nan'),
            'param7': goal.altitude
        }
    
    def get_land_command_params(self, goal) -> dict:
        """
        MC Landing: Descend vertically at current or specified position.
        
        Returns:
            param4: yaw angle
            param5: latitude (or NaN for current position)
            param6: longitude (or NaN for current position)
            param7: altitude (usually 0 or ground level)
        """
        return {
            'param4': goal.yaw if goal.yaw != 0.0 else float('nan'),
            'param5': goal.latitude if goal.latitude != 0.0 else float('nan'),
            'param6': goal.longitude if goal.longitude != 0.0 else float('nan'),
            'param7': goal.altitude if goal.altitude != 0.0 else float('nan')
        }
    
    def check_takeoff_complete(self, current_altitude: float, target_altitude: float,
                               vehicle_status, vehicle_interface) -> Tuple[bool, str]:
        """
        MC takeoff is complete when altitude is within tolerance of target.
        """
        tolerance = self.get_altitude_tolerance()
        altitude_diff = abs(current_altitude - target_altitude)
        
        if altitude_diff < tolerance:
            return True, f"Target altitude reached: {current_altitude:.2f}m"
        else:
            return False, f"Climbing to {target_altitude}m (current: {current_altitude:.2f}m)"
    
    def check_landing_complete(self, current_altitude: float, vehicle_status,
                               vehicle_interface) -> Tuple[bool, str]:
        """
        MC landing is complete when altitude is near ground level.
        """
        if current_altitude < 0.2:  # Within 20cm of ground
            return True, "Landed"
        else:
            return False, f"Descending (current: {current_altitude:.2f}m)"
    
    def execute_pre_takeoff(self, goal_handle, vehicle_interface) -> Optional[str]:
        """No special pre-takeoff actions needed for MC"""
        return None
    
    def execute_post_takeoff(self, goal_handle, vehicle_interface) -> Optional[str]:
        """No special post-takeoff actions needed for MC"""
        return None
    
    def execute_pre_landing(self, goal_handle, vehicle_interface) -> Optional[str]:
        """No special pre-landing actions needed for MC"""
        return None
