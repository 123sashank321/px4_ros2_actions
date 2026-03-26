#!/usr/bin/env python3
"""
Fixed Wing Strategy
Implements takeoff and landing logic for fixed-wing vehicles
"""

import time
from typing import Tuple, Optional
from .base_vehicle_strategy import BaseVehicleStrategy
from px4_msgs.msg import VehicleStatus, VehicleCommand


class FWStrategy(BaseVehicleStrategy):
    """Strategy for Fixed Wing (FW) vehicles"""
    
    def get_takeoff_command_params(self, goal) -> dict:
        """
        FW Takeoff: Runway takeoff with pitch angle.
        
        Returns:
            param1: Takeoff pitch angle in degrees
            param4: yaw angle
            param5: latitude
            param6: longitude
            param7: altitude
        """
        pitch = self.config_params.get('takeoff_pitch', 15.0)
        
        return {
            'param1': goal.pitch if goal.pitch != 0.0 else pitch,
            'param4': goal.yaw if goal.yaw != 0.0 else float('nan'),
            'param5': goal.latitude if goal.latitude != 0.0 else float('nan'),
            'param6': goal.longitude if goal.longitude != 0.0 else float('nan'),
            'param7': goal.altitude
        }
    
    def get_land_command_params(self, goal) -> dict:
        """
        FW Landing: Approach and landing pattern.
        
        Returns:
            param4: yaw angle
            param5: latitude
            param6: longitude
            param7: altitude (ground level)
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
        FW takeoff is complete when altitude is reached and in FW flight mode.
        """
        tolerance = self.get_altitude_tolerance()
        altitude_diff = abs(current_altitude - target_altitude)
        
        # Check if altitude reached
        if altitude_diff < tolerance:
            # Additional check: ensure in proper flight mode (LOITER or similar)
            if vehicle_status.nav_state in [VehicleStatus.NAVIGATION_STATE_AUTO_LOITER,
                                            VehicleStatus.NAVIGATION_STATE_OFFBOARD]:
                return True, f"Takeoff complete at {current_altitude:.2f}m"
            else:
                return False, f"Climbing (alt: {current_altitude:.2f}m, transitioning mode)"
        else:
            return False, f"Climbing to {target_altitude}m (current: {current_altitude:.2f}m)"
    
    def check_landing_complete(self, current_altitude: float, vehicle_status,
                               vehicle_interface) -> Tuple[bool, str]:
        """
        FW landing is complete when on ground.
        """
        # Check if landed (altitude very low and nav_state indicates landing complete)
        if current_altitude < 0.5:
            return True, "Landed"
        
        # Provide feedback based on altitude ranges (approach, flare, etc.)
        flare_alt = self.config_params.get('flare_altitude', 10.0)
        approach_alt = self.config_params.get('approach_altitude', 50.0)
        
        if current_altitude > approach_alt:
            return False, f"Approaching ({current_altitude:.2f}m)"
        elif current_altitude > flare_alt:
            return False, f"Descending ({current_altitude:.2f}m)"
        else:
            return False, f"Flare phase ({current_altitude:.2f}m)"
    
    def execute_pre_takeoff(self, goal_handle, vehicle_interface) -> Optional[str]:
        """No special pre-takeoff actions needed for FW"""
        return None
    
    def execute_post_takeoff(self, goal_handle, vehicle_interface) -> Optional[str]:
        """No special post-takeoff actions needed for FW"""
        return None
    
    def execute_pre_landing(self, goal_handle, vehicle_interface) -> Optional[str]:
        """No special pre-landing actions needed for FW"""
        return None
