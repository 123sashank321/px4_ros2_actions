#!/usr/bin/env python3
"""
Base Vehicle Strategy
Abstract base class defining the interface for vehicle-specific behaviors
"""

from abc import ABC, abstractmethod
from typing import Tuple, Optional
from px4_msgs.msg import VehicleCommand


class BaseVehicleStrategy(ABC):
    """
    Abstract base class for vehicle-specific takeoff and landing strategies.
    Each vehicle type (MC, FW, VTOL) implements this interface.
    """
    
    def __init__(self, config_params: dict):
        """
        Initialize strategy with vehicle-specific parameters.
        
        Args:
            config_params: Dictionary of vehicle-specific configuration
        """
        self.config_params = config_params
    
    @abstractmethod
    def get_takeoff_command_params(self, goal) -> dict:
        """
        Get PX4 VehicleCommand parameters for takeoff.
        
        Args:
            goal: Action goal request
            
        Returns:
            Dictionary of param1-param7 for VEHICLE_CMD_NAV_TAKEOFF
        """
        pass
    
    @abstractmethod
    def get_land_command_params(self, goal) -> dict:
        """
        Get PX4 VehicleCommand parameters for landing.
        
        Args:
            goal: Action goal request
            
        Returns:
            Dictionary of param1-param7 for VEHICLE_CMD_NAV_LAND
        """
        pass
    
    @abstractmethod
    def check_takeoff_complete(self, current_altitude: float, target_altitude: float, 
                               vehicle_status, vehicle_interface) -> Tuple[bool, str]:
        """
        Check if takeoff is complete.
        
        Args:
            current_altitude: Current altitude in meters
            target_altitude: Target altitude in meters
            vehicle_status: VehicleStatus message
            vehicle_interface: Reference to action server for additional checks
            
        Returns:
            Tuple of (is_complete, feedback_message)
        """
        pass
    
    @abstractmethod
    def check_landing_complete(self, current_altitude: float, vehicle_status, 
                               vehicle_interface) -> Tuple[bool, str]:
        """
        Check if landing is complete.
        
        Args:
            current_altitude: Current altitude in meters
            vehicle_status: VehicleStatus message
            vehicle_interface: Reference to action server for additional checks
            
        Returns:
            Tuple of (is_complete, feedback_message)
        """
        pass
    
    @abstractmethod
    def execute_pre_takeoff(self, goal_handle, vehicle_interface) -> Optional[str]:
        """
        Execute any pre-takeoff actions specific to this vehicle type.
        
        Args:
            goal_handle: Action goal handle
            vehicle_interface: Reference to action server
            
        Returns:
            Error message if failed, None if successful
        """
        pass
    
    @abstractmethod
    def execute_post_takeoff(self, goal_handle, vehicle_interface) -> Optional[str]:
        """
        Execute any post-takeoff actions (e.g., VTOL transition).
        
        Args:
            goal_handle: Action goal handle
            vehicle_interface: Reference to action server
            
        Returns:
            Error message if failed, None if successful
        """
        pass
    
    @abstractmethod
    def execute_pre_landing(self, goal_handle, vehicle_interface) -> Optional[str]:
        """
        Execute any pre-landing actions (e.g., VTOL transition to MC).
        
        Args:
            goal_handle: Action goal handle
            vehicle_interface: Reference to action server
            
        Returns:
            Error message if failed, None if successful
        """
        pass
    
    def get_altitude_tolerance(self) -> float:
        """Get altitude tolerance for this vehicle type"""
        return self.config_params.get('altitude_tolerance', 0.3)
