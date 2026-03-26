#!/usr/bin/env python3
"""
Vehicle Configuration Loader
Loads and manages vehicle-type specific configuration from JSON
"""

import json
import os
from enum import Enum
from pathlib import Path


class VehicleType(Enum):
    """Enumeration of supported vehicle types"""
    MC = "MC"       # Multicopter
    FW = "FW"       # Fixed Wing
    VTOL = "VTOL"   # Vertical Takeoff and Landing


class VehicleConfig:
    """
    Vehicle configuration loader and manager.
    Loads configuration from JSON file and provides type-safe access.
    """
    
    def __init__(self, config_path: str = None):
        """
        Initialize vehicle configuration.
        
        Args:
            config_path: Path to vehicle_config.json. If None, uses default location.
        """
        if config_path is None:
            # Use ROS2 package share directory for config files
            try:
                from ament_index_python.packages import get_package_share_directory
                package_share = get_package_share_directory('px4_ros2_actions')
                config_path = Path(package_share) / 'config' / 'vehicle_config.json'
            except Exception as e:
                # Fallback: try relative to this file (for development)
                package_path = Path(__file__).parent.parent
                config_path = package_path / 'config' / 'vehicle_config.json'
        
        self.config_path = Path(config_path)
        self._config = self._load_config()
        self._validate_config()
    
    def _load_config(self) -> dict:
        """Load configuration from JSON file"""
        if not self.config_path.exists():
            raise FileNotFoundError(f"Configuration file not found: {self.config_path}")
        
        with open(self.config_path, 'r') as f:
            return json.load(f)
    
    def _validate_config(self):
        """Validate configuration structure"""
        if 'vehicle' not in self._config:
            raise ValueError("Configuration must contain 'vehicle' key")
        
        vehicle_config = self._config['vehicle']
        
        if 'type' not in vehicle_config:
            raise ValueError("Vehicle configuration must contain 'type' key")
        
        # Validate vehicle type
        vehicle_type_str = vehicle_config['type']
        if vehicle_type_str not in [vt.value for vt in VehicleType]:
            raise ValueError(f"Invalid vehicle type: {vehicle_type_str}. "
                           f"Must be one of: {[vt.value for vt in VehicleType]}")
    
    @property
    def vehicle_type(self) -> VehicleType:
        """Get the current vehicle type"""
        type_str = self._config['vehicle']['type']
        return VehicleType(type_str)
    
    def get_mc_params(self) -> dict:
        """Get multicopter-specific parameters"""
        return self._config['vehicle'].get('mc', {})
    
    def get_fw_params(self) -> dict:
        """Get fixed-wing-specific parameters"""
        return self._config['vehicle'].get('fw', {})
    
    def get_vtol_params(self) -> dict:
        """Get VTOL-specific parameters"""
        return self._config['vehicle'].get('vtol', {})
    
    def get_params_for_current_type(self) -> dict:
        """Get parameters for the currently configured vehicle type"""
        if self.vehicle_type == VehicleType.MC:
            return self.get_mc_params()
        elif self.vehicle_type == VehicleType.FW:
            return self.get_fw_params()
        elif self.vehicle_type == VehicleType.VTOL:
            return self.get_vtol_params()
        else:
            return {}
    
    def __repr__(self) -> str:
        return f"VehicleConfig(type={self.vehicle_type.value}, config_path={self.config_path})"
