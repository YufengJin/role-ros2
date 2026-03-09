#!/usr/bin/env python3
"""
Convert MuJoCo XML model to URDF format for ROS2.

Author: Chaser Robotics Team
"""

import xml.etree.ElementTree as ET
import numpy as np
import os
import sys
import yaml
from pathlib import Path


def quat_to_rpy(quat):
    """Convert quaternion [w, x, y, z] to roll-pitch-yaw [r, p, y]"""
    w, x, y, z = quat
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)
    
    return [roll, pitch, yaw]


def parse_quat(quat_str):
    """Parse quaternion string to list"""
    if quat_str:
        parts = quat_str.strip().split()
        if len(parts) == 4:
            return [float(x) for x in parts]
    return [1, 0, 0, 0]  # Default identity quaternion


def parse_euler(euler_str):
    """Parse euler angles string to list"""
    if euler_str:
        parts = euler_str.strip().split()
        if len(parts) == 3:
            return [float(x) for x in parts]
    return [0, 0, 0]


def parse_pos(pos_str):
    """Parse position string to list"""
    if pos_str:
        parts = pos_str.strip().split()
        if len(parts) == 3:
            return [float(x) for x in parts]
    return [0, 0, 0]


def parse_inertia(inertia_str):
    """Parse inertia string (fullinertia format: ixx iyy izz ixy ixz iyz)"""
    if inertia_str:
        parts = inertia_str.strip().split()
        if len(parts) == 6:
            ixx, iyy, izz, ixy, ixz, iyz = [float(x) for x in parts]
            return {
                'ixx': ixx, 'iyy': iyy, 'izz': izz,
                'ixy': ixy, 'ixz': ixz, 'iyz': iyz
            }
    return {'ixx': 0.001, 'iyy': 0.001, 'izz': 0.001, 'ixy': 0, 'ixz': 0, 'iyz': 0}


def mujoco_to_urdf(mujoco_file, output_file, arm_id='fr3', generate_config=True, config_file=None):
    """
    Convert MuJoCo XML to URDF template and optionally generate joint_names.yaml
    
    NOTE: The generated URDF is a TEMPLATE and may not be fully ROS2-compliant.
    Manual review and adjustment is REQUIRED before use in production.
    
    The output file will be automatically renamed to *_template.urdf to indicate
    that it requires manual verification.
    
    Args:
        mujoco_file: Path to input MuJoCo XML file
        output_file: Path to output URDF file (will be renamed to *_template.urdf)
        arm_id: Arm ID prefix (e.g., 'fr3', 'panda')
        generate_config: If True, generate joint_names.yaml
        config_file: Path to joint_names.yaml (if None, auto-detect)
    
    Returns:
        tuple: (success: bool, joint_names: dict) where joint_names contains arm_joints and gripper_joints
    """
    tree = ET.parse(mujoco_file)
    root = tree.getroot()
    
    # Create URDF root
    urdf = ET.Element('robot')
    urdf.set('name', f'{arm_id}_robot')
    
    # Add base_link first (root link with minimal inertial for ROS2 compatibility)
    base_link = ET.SubElement(urdf, 'link')
    base_link.set('name', 'base_link')
    
    # Add minimal inertial to base_link (required for ROS2 robot_state_publisher)
    base_inertial = ET.SubElement(base_link, 'inertial')
    base_origin = ET.SubElement(base_inertial, 'origin')
    base_origin.set('xyz', '0 0 0')
    base_origin.set('rpy', '0 0 0')
    base_mass = ET.SubElement(base_inertial, 'mass')
    base_mass.set('value', '0.001')
    base_inertia = ET.SubElement(base_inertial, 'inertia')
    base_inertia.set('ixx', '0.001')
    base_inertia.set('iyy', '0.001')
    base_inertia.set('izz', '0.001')
    base_inertia.set('ixy', '0')
    base_inertia.set('ixz', '0')
    base_inertia.set('iyz', '0')
    
    # Store joint names for config generation
    joint_names_list = []
    
    # Get mesh directory
    mesh_dir = os.path.dirname(mujoco_file)
    compiler = root.find('compiler')
    if compiler is not None:
        meshdir = compiler.get('meshdir', './mesh')
        if not os.path.isabs(meshdir):
            mesh_dir = os.path.join(os.path.dirname(mujoco_file), meshdir)
    
    # Process bodies recursively
    worldbody = root.find('worldbody')
    if worldbody is None:
        print("Error: No worldbody found in MuJoCo file")
        return False, None
    
    # Find base link (first body in worldbody)
    base_body = worldbody.find('body')
    if base_body is None:
        print("Error: No base body found")
        return False, None
    
    def process_body(body_elem, parent_link_name=None, link_name_prefix=''):
        """Recursively process body elements"""
        body_name = body_elem.get('name', 'unnamed')
        link_name = f'{arm_id}_{body_name}' if body_name.startswith('panda_') else f'{arm_id}_{body_name}'
        
        # Create link
        link = ET.SubElement(urdf, 'link')
        link.set('name', link_name)
        
        # Process inertial
        inertial_elem = body_elem.find('inertial')
        if inertial_elem is not None:
            inertial = ET.SubElement(link, 'inertial')
            pos = parse_pos(inertial_elem.get('pos', '0 0 0'))
            mass = float(inertial_elem.get('mass', '1.0'))
            
            origin = ET.SubElement(inertial, 'origin')
            origin.set('xyz', f'{pos[0]} {pos[1]} {pos[2]}')
            origin.set('rpy', '0 0 0')
            
            mass_elem = ET.SubElement(inertial, 'mass')
            mass_elem.set('value', str(mass))
            
            inertia_elem = ET.SubElement(inertial, 'inertia')
            fullinertia = inertial_elem.get('fullinertia', None)
            diaginertia = inertial_elem.get('diaginertia', None)
            
            if fullinertia:
                # Full inertia matrix (ixx iyy izz ixy ixz iyz)
                inertia_vals = parse_inertia(fullinertia)
                inertia_elem.set('ixx', str(inertia_vals['ixx']))
                inertia_elem.set('iyy', str(inertia_vals['iyy']))
                inertia_elem.set('izz', str(inertia_vals['izz']))
                inertia_elem.set('ixy', str(inertia_vals['ixy']))
                inertia_elem.set('ixz', str(inertia_vals['ixz']))
                inertia_elem.set('iyz', str(inertia_vals['iyz']))
            elif diaginertia:
                # Diagonal inertia matrix (ixx iyy izz only)
                parts = diaginertia.strip().split()
                if len(parts) == 3:
                    ixx, iyy, izz = [float(x) for x in parts]
                    inertia_elem.set('ixx', str(ixx))
                    inertia_elem.set('iyy', str(iyy))
                    inertia_elem.set('izz', str(izz))
                    inertia_elem.set('ixy', '0')
                    inertia_elem.set('ixz', '0')
                    inertia_elem.set('iyz', '0')
                else:
                    # Default if parsing fails
                    inertia_elem.set('ixx', '0.001')
                    inertia_elem.set('iyy', '0.001')
                    inertia_elem.set('izz', '0.001')
                    inertia_elem.set('ixy', '0')
                    inertia_elem.set('ixz', '0')
                    inertia_elem.set('iyz', '0')
            else:
                # Default inertia
                inertia_elem.set('ixx', '0.001')
                inertia_elem.set('iyy', '0.001')
                inertia_elem.set('izz', '0.001')
                inertia_elem.set('ixy', '0')
                inertia_elem.set('ixz', '0')
                inertia_elem.set('iyz', '0')
        
        # Process visual - find all geoms in this body
        for geom in body_elem.findall('geom'):
            if geom.get('type') == 'mesh':
                visual = ET.SubElement(link, 'visual')
                origin = ET.SubElement(visual, 'origin')
                origin.set('xyz', '0 0 0')
                origin.set('rpy', '0 0 0')
                
                geometry = ET.SubElement(visual, 'geometry')
                mesh = ET.SubElement(geometry, 'mesh')
                mesh_file = geom.get('mesh', '')
                if not mesh_file:
                    mesh_file = geom.get('file', '')
                if mesh_file:
                    # Use relative path from package - mesh files are in robot_ik/franka/mesh/
                    mesh.set('filename', f'package://role_ros2/robot_ik/franka/mesh/{mesh_file}.obj')
                
                material = ET.SubElement(visual, 'material')
                material.set('name', 'panda_material')
                color = ET.SubElement(material, 'color')
                color.set('rgba', '0.8 0.8 0.8 1')
                
                # Also add collision geometry
                collision = ET.SubElement(link, 'collision')
                origin_coll = ET.SubElement(collision, 'origin')
                origin_coll.set('xyz', '0 0 0')
                origin_coll.set('rpy', '0 0 0')
                geometry_coll = ET.SubElement(collision, 'geometry')
                mesh_coll = ET.SubElement(geometry_coll, 'mesh')
                if mesh_file:
                    # Use relative path from package - mesh files are in robot_ik/franka/mesh/
                    mesh_coll.set('filename', f'package://role_ros2/robot_ik/franka/mesh/{mesh_file}.obj')
        
        # Process joint if this is not the base
        if parent_link_name is not None:
            joint_elem = body_elem.find('joint')
            if joint_elem is not None:
                # Body has a joint - create revolute joint
                joint = ET.SubElement(urdf, 'joint')
                joint_name = joint_elem.get('name', 'unnamed_joint')
                full_joint_name = f'{arm_id}_{joint_name}'
                joint.set('name', full_joint_name)
                joint.set('type', 'revolute')
                
                # Store joint name for config generation
                joint_names_list.append(full_joint_name)
                
                parent = ET.SubElement(joint, 'parent')
                parent.set('link', parent_link_name)
                
                child = ET.SubElement(joint, 'child')
                child.set('link', link_name)
                
                origin = ET.SubElement(joint, 'origin')
                pos = parse_pos(body_elem.get('pos', '0 0 0'))
                quat = parse_quat(body_elem.get('quat', None))
                euler = parse_euler(body_elem.get('euler', None))
                
                if euler and euler != [0, 0, 0]:
                    origin.set('rpy', f'{euler[0]} {euler[1]} {euler[2]}')
                elif quat and quat != [1, 0, 0, 0]:
                    rpy = quat_to_rpy(quat)
                    origin.set('rpy', f'{rpy[0]} {rpy[1]} {rpy[2]}')
                else:
                    origin.set('rpy', '0 0 0')
                
                origin.set('xyz', f'{pos[0]} {pos[1]} {pos[2]}')
                
                axis = ET.SubElement(joint, 'axis')
                axis_xyz = parse_pos(joint_elem.get('axis', '0 0 1'))
                axis.set('xyz', f'{axis_xyz[0]} {axis_xyz[1]} {axis_xyz[2]}')
                
                limit = ET.SubElement(joint, 'limit')
                joint_range = joint_elem.get('range', '-3.14 3.14')
                range_parts = joint_range.split()
                if len(range_parts) == 2:
                    limit.set('lower', range_parts[0])
                    limit.set('upper', range_parts[1])
                else:
                    limit.set('lower', '-3.14')
                    limit.set('upper', '3.14')
                limit.set('effort', '87')
                limit.set('velocity', '2.175')
            else:
                # Body has no joint - create a fixed joint to parent
                # This handles cases like link8 and hand which don't have joints in MuJoCo
                joint = ET.SubElement(urdf, 'joint')
                joint.set('name', f'{arm_id}_{body_name}_fixed_joint')
                joint.set('type', 'fixed')
                
                parent = ET.SubElement(joint, 'parent')
                parent.set('link', parent_link_name)
                
                child = ET.SubElement(joint, 'child')
                child.set('link', link_name)
                
                origin = ET.SubElement(joint, 'origin')
                # Get position and orientation from body attributes
                pos = parse_pos(body_elem.get('pos', '0 0 0'))
                quat = parse_quat(body_elem.get('quat', None))
                euler = parse_euler(body_elem.get('euler', None))
                
                # Set rotation (rpy)
                if euler and euler != [0, 0, 0]:
                    origin.set('rpy', f'{euler[0]} {euler[1]} {euler[2]}')
                elif quat and quat != [1, 0, 0, 0]:
                    rpy = quat_to_rpy(quat)
                    origin.set('rpy', f'{rpy[0]} {rpy[1]} {rpy[2]}')
                else:
                    origin.set('rpy', '0 0 0')
                
                # Set position (xyz)
                origin.set('xyz', f'{pos[0]} {pos[1]} {pos[2]}')
        
        # Process child bodies
        for child_body in body_elem.findall('body'):
            process_body(child_body, link_name)
        
        return link_name
    
    # Process base link (this will be fr3_panda_link0)
    # Pass 'base_link' as parent so it automatically creates a fixed joint
    # (robot base body has no joint, so process_body will create a fixed joint)
    robot_base_link_name = process_body(base_body, 'base_link')
    
    # The fixed joint was already created by process_body since base_body has no joint
    # Find and rename it to have a consistent naming pattern
    for joint in urdf.findall('joint'):
        if joint.find('parent') is not None and joint.find('parent').get('link') == 'base_link':
            if joint.find('child') is not None and joint.find('child').get('link') == robot_base_link_name:
                # Rename to consistent pattern
                joint.set('name', f'{arm_id}_base_joint')
                break
    
    # Ensure robot base link has inertial properties (required for ROS2)
    # Find the robot base link element
    robot_base_link_elem = None
    for link in urdf.findall('link'):
        if link.get('name') == robot_base_link_name:
            robot_base_link_elem = link
            break
    
    if robot_base_link_elem is not None:
        inertial_elem = robot_base_link_elem.find('inertial')
        if inertial_elem is None:
            # Add minimal inertial if missing
            inertial = ET.SubElement(robot_base_link_elem, 'inertial')
            origin = ET.SubElement(inertial, 'origin')
            origin.set('xyz', '0 0 0')
            origin.set('rpy', '0 0 0')
            mass = ET.SubElement(inertial, 'mass')
            mass.set('value', '0.001')
            inertia = ET.SubElement(inertial, 'inertia')
            inertia.set('ixx', '0.001')
            inertia.set('iyy', '0.001')
            inertia.set('izz', '0.001')
            inertia.set('ixy', '0')
            inertia.set('ixz', '0')
            inertia.set('iyz', '0')
            print(f"  Note: Added minimal inertial to {robot_base_link_name} (required for ROS2)")
    
    # Rename output file to *_template.urdf to indicate it needs manual review
    output_path = Path(output_file)
    if not output_path.name.endswith('_template.urdf'):
        # Change filename to template format
        if output_path.name.endswith('.urdf'):
            template_name = output_path.name.replace('.urdf', '_template.urdf')
        else:
            template_name = f"{output_path.stem}_template.urdf"
        output_file = str(output_path.parent / template_name)
        print(f"  Note: Output file renamed to template format: {Path(output_file).name}")
    
    # Write URDF
    tree = ET.ElementTree(urdf)
    ET.indent(tree, space='  ')
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    
    print(f"Successfully converted {mujoco_file} to {output_file}")
    print("\n" + "=" * 60)
    print("⚠️  IMPORTANT: This is a TEMPLATE URDF file")
    print("=" * 60)
    print("The generated URDF may not be fully ROS2-compliant and requires:")
    print("  1. Manual review of all link and joint definitions")
    print("  2. Verification of inertial properties (especially for base_link)")
    print("  3. Check mesh file paths are correct")
    print("  4. Validation using: check_urdf <urdf_file>")
    print("  5. Testing with robot_state_publisher before production use")
    print("=" * 60)
    
    # Extract joint names from generated URDF
    arm_joints = []
    gripper_joints = []
    
    # Sort joint names to ensure correct order
    joint_names_list.sort()
    
    for joint_name in joint_names_list:
        if 'finger' in joint_name.lower() or 'gripper' in joint_name.lower():
            gripper_joints.append(joint_name)
        else:
            arm_joints.append(joint_name)
    
    # If no gripper joints found, add defaults
    if not gripper_joints:
        gripper_joints = [
            f'{arm_id}_panda_finger_joint1',
            f'{arm_id}_panda_finger_joint2'
        ]
        print(f"  Note: No gripper joints found, using defaults: {gripper_joints}")
    
    joint_names = {
        'arm_id': arm_id,
        'arm_joints': arm_joints,
        'gripper_joints': gripper_joints
    }
    
    # Generate joint_names.yaml if requested
    if generate_config:
        if config_file is None:
            # Auto-detect config file location
            output_dir = Path(output_file).parent
            # Try to find config directory (should be at same level as role_ros2)
            if 'robot_ik' in str(output_dir):
                # Go up: robot_ik/franka -> robot_ik -> role_ros2 -> role-ros2 -> config
                config_dir = output_dir.parent.parent.parent.parent / 'config'
            else:
                config_dir = Path(output_file).parent / 'config'
            config_file = config_dir / 'joint_names.yaml'
        
        config_file = Path(config_file)
        config_file.parent.mkdir(parents=True, exist_ok=True)
        
        config = {
            'arm_id': arm_id,
            'arm_joints': arm_joints,
            'gripper_joints': gripper_joints,
            'auto_extract_from_urdf': False,
        }
        
        with open(config_file, 'w') as f:
            f.write("# Joint Names Configuration\n")
            f.write("# Auto-generated from URDF - DO NOT EDIT MANUALLY\n")
            f.write(f"# Generated from: {Path(mujoco_file).name}\n")
            f.write(f"# Regenerate using: python3 scripts/mujoco_to_urdf.py <xml_file> <urdf_file> {arm_id}\n\n")
            yaml.dump(config, f, default_flow_style=False, sort_keys=False)
        
        print(f"✓ Generated joint_names.yaml: {config_file}")
        print(f"  - Arm joints ({len(arm_joints)}): {', '.join(arm_joints)}")
        print(f"  - Gripper joints ({len(gripper_joints)}): {', '.join(gripper_joints)}")
    
    return True, joint_names


def validate_urdf_tf_tree(urdf_file):
    """
    Validate URDF TF tree structure for ROS2 compatibility and visualize tree.
    
    Checks:
    1. Only one root link (base_link)
    2. All links connected via joints
    3. No orphaned links
    4. Joint names match expected format
    5. No circular references
    
    Args:
        urdf_file: Path to URDF file
    
    Returns:
        tuple: (is_valid: bool, issues: list, tree_structure: dict)
    """
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    
    issues = []
    tree_structure = {}
    
    # Find all links and joints
    links = {link.get('name'): link for link in root.findall('link')}
    joints = {joint.get('name'): joint for joint in root.findall('joint')}
    
    # Find root links (links that are not children of any joint)
    child_links = set()
    parent_child_map = {}
    for joint_name, joint in joints.items():
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is not None and child is not None:
            parent_link = parent.get('link')
            child_link = child.get('link')
            child_links.add(child_link)
            if child_link not in parent_child_map:
                parent_child_map[child_link] = []
            parent_child_map[child_link].append({
                'parent': parent_link,
                'joint': joint_name,
                'type': joint.get('type', 'unknown')
            })
    
    root_links = [name for name in links.keys() if name not in child_links]
    
    # Check 1: Only one root link
    if len(root_links) == 0:
        issues.append("ERROR: No root link found (all links are children)")
    elif len(root_links) > 1:
        issues.append(f"ERROR: Multiple root links found: {root_links}")
    elif root_links[0] != 'base_link':
        issues.append(f"WARNING: Root link is '{root_links[0]}', expected 'base_link'")
    else:
        print(f"  ✓ Root link: {root_links[0]}")
    
    # Check 2: All links (except base_link) should be children of a joint
    orphaned_links = [name for name in links.keys() if name != 'base_link' and name not in child_links]
    if orphaned_links:
        issues.append(f"ERROR: Orphaned links (not connected via joint): {orphaned_links}")
    else:
        print(f"  ✓ All links connected via joints ({len(links)} links, {len(joints)} joints)")
    
    # Check 3: All joints have valid parent and child
    for joint_name, joint in joints.items():
        parent = joint.find('parent')
        child = joint.find('child')
        if parent is None or child is None:
            issues.append(f"ERROR: Joint '{joint_name}' missing parent or child")
        elif parent.get('link') not in links:
            issues.append(f"ERROR: Joint '{joint_name}' references non-existent parent link: {parent.get('link')}")
        elif child.get('link') not in links:
            issues.append(f"ERROR: Joint '{joint_name}' references non-existent child link: {child.get('link')}")
    
    # Check 4: No circular references (each link should have only one parent)
    for child_link, parents in parent_child_map.items():
        if len(parents) > 1:
            issues.append(f"ERROR: Link '{child_link}' has multiple parents: {[p['parent'] for p in parents]}")
    
    # Build tree structure for visualization
    def build_tree(link_name, depth=0):
        """Recursively build tree structure"""
        children = []
        for child_link, parents in parent_child_map.items():
            for parent_info in parents:
                if parent_info['parent'] == link_name:
                    joint_info = {
                        'joint': parent_info['joint'],
                        'type': parent_info['type'],
                        'link': child_link
                    }
                    sub_children = build_tree(child_link, depth + 1)
                    joint_info['children'] = sub_children
                    children.append(joint_info)
        return children
    
    if root_links:
        tree_structure = {
            'root': root_links[0],
            'children': build_tree(root_links[0])
        }
    
    # Visualize tree
    def print_tree(node, prefix="", is_last=True):
        """Print tree structure"""
        if isinstance(node, dict):
            if 'root' in node:
                print(f"\n  TF Tree Structure:")
                print(f"  {node['root']}")
                for i, child in enumerate(node.get('children', [])):
                    is_last_child = (i == len(node['children']) - 1)
                    print_tree(child, "  ", is_last_child)
        elif isinstance(node, dict) and 'joint' in node:
            connector = "└── " if is_last else "├── "
            joint_type = node.get('type', 'unknown')
            print(f"{prefix}{connector}[{joint_type}] {node['joint']}")
            print(f"{prefix}{'    ' if is_last else '│   '}└── {node['link']}")
            for i, child in enumerate(node.get('children', [])):
                is_last_child = (i == len(node['children']) - 1)
                print_tree(child, prefix + ('    ' if is_last else '│   '), is_last_child)
    
    if tree_structure:
        print_tree(tree_structure)
    
    # Summary
    if not issues:
        print(f"\n  ✓ URDF TF tree structure is valid for ROS2")
        return True, [], tree_structure
    else:
        print(f"\n  ✗ Found {len(issues)} issue(s):")
        for issue in issues:
            print(f"    - {issue}")
        return False, issues, tree_structure


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 mujoco_to_urdf.py <input_mujoco.xml> <output.urdf> [arm_id] [--no-config]")
        print("\nOptions:")
        print("  --no-config    Skip generating joint_names.yaml")
        print("\nNote: Output file will be automatically renamed to *_template.urdf")
        print("      to indicate that manual review is required.")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    arm_id = 'fr3'
    generate_config = True
    
    # Parse arguments
    for i, arg in enumerate(sys.argv[3:], start=3):
        if arg == '--no-config':
            generate_config = False
        elif not arg.startswith('--'):
            arm_id = arg
    
    if not os.path.exists(input_file):
        print(f"Error: Input file {input_file} not found")
        sys.exit(1)
    
    print("=" * 60)
    print(f"Converting MuJoCo XML to URDF Template (arm_id: {arm_id})")
    print("=" * 60)
    print("⚠️  Note: Generated URDF will be a TEMPLATE requiring manual review")
    print("=" * 60)
    
    # Auto-detect config file location
    config_file = None
    if generate_config:
        output_path = Path(output_file)
        # Try to find config directory
        if 'robot_ik' in str(output_path):
            config_dir = output_path.parent.parent.parent.parent / 'config'
            config_file = config_dir / 'joint_names.yaml'
    
    success, joint_names = mujoco_to_urdf(input_file, output_file, arm_id, generate_config, config_file)
    
    if success:
        print("\n" + "=" * 60)
        print("Validating URDF TF tree structure...")
        print("=" * 60)
        is_valid, issues, _ = validate_urdf_tf_tree(output_file)
        
        if is_valid:
            print("\n✓ URDF template conversion and validation completed!")
            print("\n⚠️  REMINDER: This is a TEMPLATE file. Manual review required before use.")
            print(f"   Output file: {output_file}")
        else:
            print("\n⚠ URDF template generated but validation found issues.")
            print("   Please review and fix before using in production.")
            print(f"   Output file: {output_file}")
            sys.exit(1)
    else:
        print("\n✗ URDF conversion failed")
        sys.exit(1)

