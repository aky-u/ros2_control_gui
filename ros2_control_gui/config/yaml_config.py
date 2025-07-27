"""
YAML configuration file management for ROS 2 control.
"""

import yaml
import os
from typing import Optional, List, Dict, Any


class YamlConfigManager:
    """Manages loading and parsing of YAML configuration files"""
    
    def __init__(self):
        self.config_data: Optional[Dict[str, Any]] = None
        self.config_file_path: Optional[str] = None
    
    def load_config(self, yaml_file_path: str) -> bool:
        """Load YAML configuration file"""
        try:
            with open(yaml_file_path, 'r') as f:
                self.config_data = yaml.safe_load(f)
            self.config_file_path = yaml_file_path
            return True
        except Exception as e:
            print(f"Failed to load YAML config from {yaml_file_path}: {e}")
            return False
    
    def is_loaded(self) -> bool:
        """Check if configuration is loaded"""
        return self.config_data is not None
    
    def get_config_filename(self) -> str:
        """Get the filename of the loaded config"""
        if self.config_file_path:
            return os.path.basename(self.config_file_path)
        return "No config loaded"
    
    def get_controller_joint_names(self, controller_name: str) -> Optional[List[str]]:
        """Get joint names for a controller from loaded YAML config"""
        if not self.config_data:
            return None
            
        # Common YAML structures for ROS2 control configurations
        search_paths = [
            # Direct controller name as key (most common)
            [controller_name, "ros__parameters"],
            [controller_name],
            # Under controller_manager section
            ["controller_manager", "ros__parameters", controller_name],
            # Under controllers section
            ["controllers", controller_name],
            # Under ros__parameters section
            ["ros__parameters", controller_name],
            # Try with / prefix (some configs use this)
            [f"/{controller_name}", "ros__parameters"],
            [f"/{controller_name}"],
        ]
        
        for path in search_paths:
            result = self._get_nested_value(self.config_data, path)
            if result and isinstance(result, dict):
                joint_names = self._extract_joint_names_from_config(result)
                if joint_names:
                    return joint_names
        
        # Try recursive search
        result = self._find_controller_in_nested_config(self.config_data, controller_name)
        return result

    def _get_nested_value(self, data: dict, path: list):
        """Get nested value from dictionary using path list"""
        current = data
        for key in path:
            if isinstance(current, dict) and key in current:
                current = current[key]
            else:
                return None
        return current

    def _extract_joint_names_from_config(self, config: dict) -> Optional[List[str]]:
        """Extract joint names from controller configuration"""
        # Common keys where joint names are stored
        joint_keys = ['joints', 'joint_names', 'controlled_joints']
        
        for key in joint_keys:
            if key in config:
                joints = config[key]
                if isinstance(joints, list) and joints:
                    return joints
                elif isinstance(joints, str):
                    # Sometimes joints are stored as space/comma separated string
                    return [j.strip() for j in joints.replace(',', ' ').split() if j.strip()]
        
        return None

    def _find_controller_in_nested_config(self, config: dict, controller_name: str) -> Optional[List[str]]:
        """Recursively search for controller configuration in nested structure"""
        if isinstance(config, dict):
            for key, value in config.items():
                if key == controller_name and isinstance(value, dict):
                    joint_names = self._extract_joint_names_from_config(value)
                    if joint_names:
                        return joint_names
                elif isinstance(value, dict):
                    result = self._find_controller_in_nested_config(value, controller_name)
                    if result:
                        return result
        
        return None
