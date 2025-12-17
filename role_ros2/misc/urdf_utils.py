"""
Utilities for parsing URDF files to extract joint names
"""

import xml.etree.ElementTree as ET
from typing import List, Optional
import os


def get_joint_names_from_urdf(urdf_file: str, arm_id: str = "fr3") -> List[str]:
    """
    Extract joint names from URDF file
    
    Args:
        urdf_file: Path to URDF file
        arm_id: Arm ID prefix (e.g., 'fr3')
    
    Returns:
        List of joint names in order
    """
    if not os.path.exists(urdf_file):
        raise FileNotFoundError(f"URDF file not found: {urdf_file}")
    
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    
    joint_names = []
    
    # Find all revolute joints (arm joints)
    for joint in root.findall('joint'):
        joint_name = joint.get('name', '')
        joint_type = joint.get('type', '')
        
        # Only include revolute joints (arm joints)
        if joint_type == 'revolute' and joint_name.startswith(f'{arm_id}_'):
            joint_names.append(joint_name)
    
    # Sort joints by number to ensure correct order
    def get_joint_number(name):
        # Extract number from joint name (e.g., "fr3_panda_joint1" -> 1)
        try:
            parts = name.split('joint')
            if len(parts) > 1:
                return int(parts[1])
        except:
            pass
        return 999  # Put unknown joints at the end
    
    joint_names.sort(key=get_joint_number)
    
    return joint_names


def get_gripper_joint_names(arm_id: str = "fr3") -> List[str]:
    """
    Get gripper joint names (standard Franka gripper has 2 finger joints)
    
    Args:
        arm_id: Arm ID prefix
    
    Returns:
        List of gripper joint names
    """
    return [f'{arm_id}_finger_joint1', f'{arm_id}_finger_joint2']


def get_all_joint_names(urdf_file: str, arm_id: str = "fr3", include_gripper: bool = True) -> List[str]:
    """
    Get all joint names (arm + gripper) from URDF
    
    Args:
        urdf_file: Path to URDF file
        arm_id: Arm ID prefix
        include_gripper: Whether to include gripper joints
    
    Returns:
        List of all joint names
    """
    arm_joints = get_joint_names_from_urdf(urdf_file, arm_id)
    
    if include_gripper:
        gripper_joints = get_gripper_joint_names(arm_id)
        return arm_joints + gripper_joints
    
    return arm_joints

