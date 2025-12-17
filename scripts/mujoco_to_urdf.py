#!/usr/bin/env python3
"""
Convert MuJoCo XML model to URDF format for ROS2
"""

import xml.etree.ElementTree as ET
import numpy as np
import os
import sys
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


def mujoco_to_urdf(mujoco_file, output_file, arm_id='fr3'):
    """Convert MuJoCo XML to URDF"""
    tree = ET.parse(mujoco_file)
    root = tree.getroot()
    
    # Create URDF root
    urdf = ET.Element('robot')
    urdf.set('name', f'{arm_id}_robot')
    
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
        return False
    
    # Find base link (first body in worldbody)
    base_body = worldbody.find('body')
    if base_body is None:
        print("Error: No base body found")
        return False
    
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
            if fullinertia:
                inertia_vals = parse_inertia(fullinertia)
                inertia_elem.set('ixx', str(inertia_vals['ixx']))
                inertia_elem.set('iyy', str(inertia_vals['iyy']))
                inertia_elem.set('izz', str(inertia_vals['izz']))
                inertia_elem.set('ixy', str(inertia_vals['ixy']))
                inertia_elem.set('ixz', str(inertia_vals['ixz']))
                inertia_elem.set('iyz', str(inertia_vals['iyz']))
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
                    # Use relative path from package
                    mesh.set('filename', f'package://role_ros2/meshes/franka/{mesh_file}.obj')
                
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
                    mesh_coll.set('filename', f'package://role_ros2/meshes/franka/{mesh_file}.obj')
        
        # Process joint if this is not the base
        if parent_link_name is not None:
            joint_elem = body_elem.find('joint')
            if joint_elem is not None:
                joint = ET.SubElement(urdf, 'joint')
                joint_name = joint_elem.get('name', 'unnamed_joint')
                joint.set('name', f'{arm_id}_{joint_name}')
                joint.set('type', 'revolute')
                
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
        
        # Process child bodies
        for child_body in body_elem.findall('body'):
            process_body(child_body, link_name)
        
        return link_name
    
    # Process base link
    base_link_name = process_body(base_body, None)
    
    # Write URDF
    tree = ET.ElementTree(urdf)
    ET.indent(tree, space='  ')
    tree.write(output_file, encoding='utf-8', xml_declaration=True)
    
    print(f"Successfully converted {mujoco_file} to {output_file}")
    return True


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: python3 mujoco_to_urdf.py <input_mujoco.xml> <output.urdf> [arm_id]")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2]
    arm_id = sys.argv[3] if len(sys.argv) > 3 else 'fr3'
    
    if not os.path.exists(input_file):
        print(f"Error: Input file {input_file} not found")
        sys.exit(1)
    
    mujoco_to_urdf(input_file, output_file, arm_id)

