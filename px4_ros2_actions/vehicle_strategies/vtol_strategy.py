#!/usr/bin/env python3
"""
VTOL Strategy
Implements takeoff and landing logic for VTOL vehicles
Follows the logic from comms_new.py: MC takeoff → transition to FW
"""

import time
from typing import Tuple, Optional
from .base_vehicle_strategy import BaseVehicleStrategy
from px4_msgs.msg import VehicleStatus, VehicleCommand


class VTOLStrategy(BaseVehicleStrategy):
    """
    Strategy for VTOL vehicles.
    
    Takeoff: MC mode climb → automatic transition to FW mode
    Landing: FW mode → transition to MC → vertical descent
    """
    
    def get_takeoff_command_params(self, goal) -> dict:
        """
        VTOL Takeoff: Starts as MC takeoff.
        
        Returns:
            param1: pitch (not used for initial MC takeoff)
            param4: yaw angle
            param5: latitude
            param6: longitude
            param7: MC takeoff altitude
        """
        mc_takeoff_alt = self.config_params.get('mc_takeoff_altitude', 10.0)
        
        return {
            'param1': float('nan'),
            'param4': goal.yaw if goal.yaw != 0.0 else float('nan'),
            'param5': goal.latitude if goal.latitude != 0.0 else float('nan'),
            'param6': goal.longitude if goal.longitude != 0.0 else float('nan'),
            'param7': mc_takeoff_alt  # Use MC takeoff altitude
        }
    
    def get_land_command_params(self, goal) -> dict:
        """
        VTOL Landing: Land in MC mode.
        
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
        VTOL takeoff monitoring during MC phase.
        
        Completion criteria:
        - Reaches mc_takeoff_altitude in MC mode
        - Then execute_post_takeoff() handles the transition to FW
        """
        tolerance = self.get_altitude_tolerance()
        mc_takeoff_alt = self.config_params.get('mc_takeoff_altitude', 50.0)
        
        # Get VTOL state from vtol_vehicle_status
        vtol_status = vehicle_interface.vtol_vehicle_status
        from px4_msgs.msg import VtolVehicleStatus
        
        # Check VTOL state
        is_fw_mode = (vtol_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW)
        is_mc_mode = (vtol_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_MC)
        is_transitioning_to_fw = (vtol_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_TRANSITION_TO_FW)
        
        # During MC climb phase: complete when MC altitude reached
        if is_mc_mode:
            if abs(current_altitude - mc_takeoff_alt) < tolerance:
                # MC takeoff complete - execute_post_takeoff will handle transition
                return True, f"MC climb complete at {current_altitude:.2f}m, ready for transition"
            else:
                return False, f"MC mode: Climbing to {mc_takeoff_alt}m (current: {current_altitude:.2f}m)"
        
        # If already transitioning or in FW (shouldn't happen in normal flow)
        elif is_transitioning_to_fw:
            return False, f"Transitioning to FW mode ({current_altitude:.2f}m)"
        elif is_fw_mode:
            return True, f"Already in FW mode at {current_altitude:.2f}m"
        else:
            return False, f"VTOL state unknown ({current_altitude:.2f}m)"
    
    def check_landing_complete(self, current_altitude: float, vehicle_status,
                               vehicle_interface) -> Tuple[bool, str]:
        """
        VTOL landing is complete when on ground in MC mode.
        """
        if current_altitude < 0.2:
            return True, "Landed"
        
        # Get VTOL state from vtol_vehicle_status
        vtol_status = vehicle_interface.vtol_vehicle_status
        from px4_msgs.msg import VtolVehicleStatus
        
        # Provide feedback based on VTOL state
        is_mc_mode = (vtol_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_MC)
        is_fw_mode = (vtol_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW)
        is_transitioning_to_mc = (vtol_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_TRANSITION_TO_MC)
        is_transitioning_to_fw = (vtol_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_TRANSITION_TO_FW)
        
        if is_transitioning_to_mc or is_transitioning_to_fw:
            return False, f"Transitioning to MC mode ({current_altitude:.2f}m)"
        elif is_mc_mode:
            return False, f"MC mode: Descending ({current_altitude:.2f}m)"
        elif is_fw_mode:
            return False, f"FW mode: Preparing for transition ({current_altitude:.2f}m)"
        else:
            return False, f"VTOL state unknown: Descending ({current_altitude:.2f}m)"
    
    def execute_pre_takeoff(self, goal_handle, vehicle_interface) -> Optional[str]:
        """No special pre-takeoff actions for VTOL"""
        return None
    
    def execute_post_takeoff(self, goal_handle, vehicle_interface) -> Optional[str]:
        """
        Post-takeoff for VTOL: Handle transition to FW if enabled.
        
        This is called after MC takeoff altitude is reached.
        If auto_transition is enabled, triggers VTOL transition to FW mode.
        """
        auto_transition = self.config_params.get('auto_transition', True)
        
        if not auto_transition:
            return None  # No transition needed
        
        # Get VTOL status
        vtol_status = vehicle_interface.vtol_vehicle_status
        from px4_msgs.msg import VtolVehicleStatus
        
        # Check if already in FW mode
        if vtol_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW:
            vehicle_interface.get_logger().info("Already in FW mode")
            return None
        
        # Check if vehicle is VTOL capable
        if not vehicle_interface.vehicle_status.is_vtol:
            vehicle_interface.get_logger().warn("Vehicle is not VTOL capable, skipping transition")
            return None
        
        # IMPORTANT: PX4 requires vehicle to be in AUTO_LOITER for safe transition
        # Following the pattern from comms_new.py
        if vehicle_interface.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            vehicle_interface.get_logger().info("Switching to AUTO_LOITER (HOLD) mode before transition...")
            vehicle_interface.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,  # Custom main mode
                param2=4.0   # LOITER sub-mode
            )
            # Wait for mode change
            time.sleep(2.0)
            # Verify mode changed
            if vehicle_interface.vehicle_status.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
                vehicle_interface.get_logger().warn(f"Failed to enter LOITER mode, current: {vehicle_interface.vehicle_status.nav_state}")
        
        # Trigger transition to FW mode
        vehicle_interface.get_logger().info("Initiating VTOL transition to FW mode...")
        vehicle_interface.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION,
            param1=4.0  # 4.0 = transition to FW mode
        )
        
        # Wait for transition to complete
        timeout = self.config_params.get('transition_timeout', 30.0)
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                return "Transition canceled by user"
            
            # Update status
            vtol_status = vehicle_interface.vtol_vehicle_status
            
            # Check if transition complete
            if vtol_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW:
                vehicle_interface.get_logger().info("Transition to FW mode complete")
                time.sleep(0.5)  # Brief stabilization
                return None  # Success
            
            time.sleep(0.1)
        
        return "VTOL transition to FW timed out"
    
    def execute_pre_landing(self, goal_handle, vehicle_interface) -> Optional[str]:
        """
        Pre-landing for VTOL: 
        1. If in FW mode and above transition altitude, descend to transition altitude in FW
        2. Then transition to MC mode
        
        This ensures safe altitude for FW→MC transition.
        """
        vtol_status = vehicle_interface.vtol_vehicle_status
        from px4_msgs.msg import VtolVehicleStatus
        
        # Get transition altitude from config
        transition_alt = self.config_params.get('transition_altitude', 50.0)
        current_alt = vehicle_interface.current_altitude
        
        # Check if already in MC mode
        if vtol_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_MC:
            vehicle_interface.get_logger().info("Already in MC mode, ready to land")
            return None
        
        # If in FW mode, descend to transition altitude first
        if vtol_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW:
            # If above transition altitude, descend in FW mode first
            if current_alt > transition_alt + 5.0:  # 5m buffer
                vehicle_interface.get_logger().info(f"Descending to transition altitude {transition_alt}m in FW mode...")
                vehicle_interface.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
                    param1=-1.0,  # Ground speed (use default)
                    param2=1.0,   # Bitmask: bit 0 = change altitude
                    param3=0.0,   # Reserved
                    param4=float('nan'),  # Yaw (keep current)
                    param5=float('nan'),  # Latitude (keep current)
                    param6=float('nan'),  # Longitude (keep current)
                    param7=transition_alt  # Transition altitude
                )
                
                # Monitor descent in FW mode
                timeout = 60.0  # 1 minute max for descent
                start_time = time.time()
                tolerance = self.config_params.get('altitude_tolerance', 0.5)
                
                while time.time() - start_time < timeout:
                    if goal_handle.is_cancel_requested:
                        return "FW descent canceled by user"
                    
                    current_alt = vehicle_interface.current_altitude
                    
                    # Check if reached transition altitude
                    if abs(current_alt - transition_alt) < tolerance:
                        vehicle_interface.get_logger().info(f"Reached transition altitude {transition_alt}m")
                        break
                    
                    time.sleep(0.5)
                else:
                    vehicle_interface.get_logger().warn("FW descent to transition altitude timed out, continuing anyway")
            
            # Now initiate transition to MC mode
            vehicle_interface.get_logger().info("Initiating VTOL transition to MC mode...")
            vehicle_interface.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION,
                param1=3.0  # 3.0 = transition to MC mode
            )
            
            # Wait for transition to complete
            timeout = self.config_params.get('transition_timeout', 30.0)
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    return "Transition canceled by user"
                
                # Update status
                vtol_status = vehicle_interface.vtol_vehicle_status
                
                # Check if transition complete
                if vtol_status.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_MC:
                    vehicle_interface.get_logger().info("Transition to MC mode complete")
                    time.sleep(0.5)  # Brief stabilization
                    return None  # Success
                
                time.sleep(0.1)
            
            return "VTOL transition to MC timed out"
        
        return None
