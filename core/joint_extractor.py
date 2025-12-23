# -*- coding: utf-8 -*-
"""
Joint Data Extraction using Globalize-Then-Relativize Strategy

This module implements robust joint position extraction that handles:
- Nested components
- Rotated sub-assemblies
- Random component origins
- Deep hierarchies

Key insight: Never trust local coordinates. Always calculate in World Space first,
then relativize to parent frame.
"""

import adsk, adsk.core, adsk.fusion
import math
from . import math_operation as math_op


def matrix_multiply(m1: adsk.core.Matrix3D, m2: adsk.core.Matrix3D) -> adsk.core.Matrix3D:
    """
    Multiply two 4x4 transformation matrices using Fusion API.
    Result = M1 * M2
    """
    # Create copy of m1 to avoid side effects
    result = m1.copy()
    # transformBy is equivalent to post-multiplying: result = result * m2
    result.transformBy(m2)
    return result

def matrix_invert(matrix: adsk.core.Matrix3D) -> adsk.core.Matrix3D:
    """
    Invert a 4x4 transformation matrix using Fusion API.
    """
    inv_matrix = matrix.copy()
    inv_matrix.invert()
    return inv_matrix


def convert_to_urdf_coords(point_cm_yup: adsk.core.Vector3D) -> adsk.core.Vector3D:
    """
    Convert from Fusion (cm, Y-up) to URDF (m, Z-up).
    
    Transformation:
    1. Scale: cm → m (divide by 100)
    2. Basis change: Y-up → Z-up (rotate -90° around X-axis)
    
    Rotation matrix for -90° around X:
    [1   0   0]
    [0   0   1]
    [0  -1   0]
    
    Transformation:
    x_urdf = x_fusion / 100
    y_urdf = z_fusion / 100
    z_urdf = -y_fusion / 100
    
    Args:
        point_cm_yup: Point in Fusion coordinates (cm, Y-up)
    
    Returns:
        adsk.core.Vector3D: Point in URDF coordinates (m, Z-up)
    """
    x = point_cm_yup.x / 100.0  # cm to m
    y = point_cm_yup.y / 100.0
    z = point_cm_yup.z / 100.0
    
    # Y-up to Z-up: [x, z, -y]
    # FIX: Keep Y-up (Fusion native) to match local mesh export
    x_urdf = x
    y_urdf = y
    z_urdf = z
    
    return adsk.core.Vector3D.create(x_urdf, y_urdf, z_urdf)


def get_full_world_transform(occurrence: adsk.fusion.Occurrence) -> adsk.core.Matrix3D:
    """
    Get the true World Space transform of an occurrence by traversing up the assembly context.
    
    Args:
        occurrence: The occurrence to get transform for
    
    Returns:
        adsk.core.Matrix3D: World Space transformation matrix
    """
    try:
        # Start with the occurrence's local transform (relative to parent)
        transform = occurrence.transform2.copy()
        
        # Traverse up the assembly context chain
        current = occurrence
        while current.assemblyContext:
            parent_transform = current.assemblyContext.transform2.copy()
            # World = Parent * Local
            # transformBy does: result = result * other. 
            # We want: Parent * Local. 
            # So: Parent.transformBy(Local)
            parent_transform.transformBy(transform) 
            transform = parent_transform
            
            current = current.assemblyContext
            
        return transform
    except Exception as e:
        # Fallback
        return occurrence.transform2

def get_joint_world_position(joint: adsk.fusion.Joint, 
                            parent_occ: adsk.fusion.Occurrence) -> adsk.core.Matrix3D:
    """
    Get the World Space position and orientation of a joint.
    """
    try:
        # Get parent occurrence's full world transform
        parent_world = get_full_world_transform(parent_occ)
        
        # CRITICAL: Use geometryOrOriginTwo (parent side of joint)
        geometry_or_origin = joint.geometryOrOriginTwo
        
        if geometry_or_origin is None:
            return parent_world
        
        # Check if it's a JointGeometry or JointOrigin
        try:
            # Try as JointGeometry first
            geometry = geometry_or_origin.geometry if hasattr(geometry_or_origin, 'geometry') else geometry_or_origin
            origin_point = geometry.origin
            
            # Get axes
            try:
                z_axis = geometry.primaryAxisVector  
                x_axis = geometry.secondaryAxisVector
                y_axis = geometry.thirdAxisVector
            except:
                # Fallback
                z_axis = geometry.zAxis
                x_axis = geometry.xAxis
                y_axis = geometry.yAxis
        except:
             # Fallback
            try:
                origin_point = geometry_or_origin.origin if hasattr(geometry_or_origin, 'origin') else joint.geometryOrOriginTwo.geometry.origin
                z_axis = joint.geometryOrOriginTwo.primaryAxisVector
                x_axis = joint.geometryOrOriginTwo.secondaryAxisVector
                y_axis = joint.geometryOrOriginTwo.thirdAxisVector
            except:
                return parent_world

        # Normalize axes
        z_len = (z_axis.x**2 + z_axis.y**2 + z_axis.z**2)**0.5
        if z_len > 0: z_axis = adsk.core.Vector3D.create(z_axis.x/z_len, z_axis.y/z_len, z_axis.z/z_len)
        
        x_len = (x_axis.x**2 + x_axis.y**2 + x_axis.z**2)**0.5
        if x_len > 0: x_axis = adsk.core.Vector3D.create(x_axis.x/x_len, x_axis.y/x_len, x_axis.z/x_len)
        
        y_len = (y_axis.x**2 + y_axis.y**2 + y_axis.z**2)**0.5
        if y_len > 0: y_axis = adsk.core.Vector3D.create(y_axis.x/y_len, y_axis.y/y_len, y_axis.z/y_len)
        
        # Build joint matrix in LOCAL frame (relative to parent's component origin)
        local_joint_matrix = adsk.core.Matrix3D.create()
        local_joint_matrix.setCell(0, 0, x_axis.x)
        local_joint_matrix.setCell(0, 1, x_axis.y)
        local_joint_matrix.setCell(0, 2, x_axis.z)
        local_joint_matrix.setCell(1, 0, y_axis.x)
        local_joint_matrix.setCell(1, 1, y_axis.y)
        local_joint_matrix.setCell(1, 2, y_axis.z)
        local_joint_matrix.setCell(2, 0, z_axis.x)
        local_joint_matrix.setCell(2, 1, z_axis.y)
        local_joint_matrix.setCell(2, 2, z_axis.z)
        local_joint_matrix.translation = adsk.core.Vector3D.create(origin_point.x, origin_point.y, origin_point.z)
        
        # CRITICAL: Transform from parent's local frame to world frame
        # World_Joint = Parent_World * Local_Joint
        # Fusion: m1.transformBy(m2) -> m1 = m1 * m2
        world_joint_matrix = parent_world.copy()
        world_joint_matrix.transformBy(local_joint_matrix)
        
        return world_joint_matrix
        
    except Exception as e:
        return parent_occ.transform2


def extract_joint_data(joint: adsk.fusion.Joint, 
                       parent_occ: adsk.fusion.Occurrence,
                       child_occ: adsk.fusion.Occurrence,
                       design: adsk.fusion.Design = None,
                       reference_matrix: adsk.core.Matrix3D = None) -> dict:
    """
    Extract URDF-compatible joint data using accurate reference frame.
    
    Args:
        joint: The Fusion Joint object
        parent_occ: Parent occurrence (anchor point)
        child_occ: Child occurrence (moving part)
        design: Design object (optional)
        reference_matrix: Optional explicit World Transform of the Reference Frame (A).
                         If provided, calculates T_ref_joint = A^-1 * B
                         If None, uses parent_occ's World Transform as A.
    
    Returns:
        dict: {
            'xyz': adsk.core.Vector3D (URDF coordinates in meters),
            'rpy': [roll, pitch, yaw] (Euler angles in radians),
            'type': str
        }
    """
    try:
        # Step A: Get World Space transforms
        if reference_matrix:
             parent_world_matrix = reference_matrix
        else:
             parent_world_matrix = get_full_world_transform(parent_occ)
             
        joint_world_matrix = get_joint_world_position(joint, parent_occ)
        
        # DEBUG: Log matrices for debugging
        try:
            from ..commands.ACDC4Robot import constants
            text_palette = constants.get_text_palette()
            
            def mat_to_str(m):
                return f"[{m.getCell(0,3):.2f}, {m.getCell(1,3):.2f}, {m.getCell(2,3):.2f}]"

            def mat_full_str(m):
                 rows = []
                 for i in range(4):
                     rows.append(f"[{m.getCell(i,0):.3f}, {m.getCell(i,1):.3f}, {m.getCell(i,2):.3f}, {m.getCell(i,3):.3f}]")
                 return " | ".join(rows)

            if text_palette:
                text_palette.writeText(f"[JOINT] Parent: {parent_occ.name}")
                if reference_matrix:
                    text_palette.writeText(f"  Reference Frame (Explicit): {mat_full_str(parent_world_matrix)}")
                else:
                    text_palette.writeText(f"  Reference Frame (Component): {mat_full_str(parent_world_matrix)}")
                text_palette.writeText(f"  Joint World:  {mat_full_str(joint_world_matrix)}")
        except:
            pass
        
        # Step B: Invert parent matrix
        parent_inv = matrix_invert(parent_world_matrix)
        
        # Step C: Relativize (calculate relative transform)
        relative_transform = matrix_multiply(parent_inv, joint_world_matrix)
        
        try:
             if text_palette:
                 text_palette.writeText(f"  Ref Inv:      {mat_full_str(parent_inv)}")
                 text_palette.writeText(f"  Relative:     {mat_full_str(relative_transform)}")
        except:
             pass

        # Step D: Convert units and axes (Fusion Y-up -> URDF Z-up)
        # We need to transform the relative matrix from Fusion Basis to URDF Basis
        # T_urdf = B * T_fusion * B_inv
        # Where B is the rotation matrix from Fusion to URDF
        # Fusion: X=Right, Y=Up, Z=Front
        # URDF:   X=Forward, Y=Left, Z=Up (Standard ROS)
        # B = [1 0 0]  (X -> X)
        #     [0 0 1]  (Y -> Z)
        #     [0 -1 0] (Z -> -Y)
        
        # Create Basis Matrix B
        fusion_to_urdf = adsk.core.Matrix3D.create()
        fusion_to_urdf.setCell(1, 1, 0)
        fusion_to_urdf.setCell(1, 2, 1)
        fusion_to_urdf.setCell(2, 1, -1)
        fusion_to_urdf.setCell(2, 2, 0)
        
        # Create B_inv (Transpose of B for rotation matrix)
        urdf_to_fusion = fusion_to_urdf.copy()
        urdf_to_fusion.invert()
        
        # Calculate T_urdf = B * T_fusion * B_inv
        # Fusion API: m1.transformBy(m2) -> m1 * m2
        # We want: B * (T_fusion * B_inv)
        
        # 1. T_fusion * B_inv
        # Be careful with Fusion API multiplication order!
        # matrix_multiply implementation: result = m1 * m2
        
        # Scale T_fusion first (cm -> m)
        # Scaling affects translation but not rotation
        relative_transform_m = relative_transform.copy()
        relative_transform_m.translation = adsk.core.Vector3D.create(
            relative_transform.translation.x / 100.0,
            relative_transform.translation.y / 100.0,
            relative_transform.translation.z / 100.0
        )
        
        # T_urdf = B * T_fusion * B_inv
        temp = matrix_multiply(relative_transform_m, urdf_to_fusion) # T * B_inv
        urdf_transform = matrix_multiply(fusion_to_urdf, temp) # B * (T * B_inv)
        
        # Extract format
        origin_urdf = urdf_transform.translation
        rpy = math_op.matrix3d_2_pose(urdf_transform)[3:6]
        
        # Get joint type
        joint_type = get_joint_type(joint)
        
        # DEBUG: Log final result
        try:
            from ..commands.ACDC4Robot import constants
            text_palette = constants.get_text_palette()
            if text_palette:
                text_palette.writeText(f"[JOINT] Result: xyz=({origin_urdf.x:.4f}, {origin_urdf.y:.4f}, {origin_urdf.z:.4f}), rpy=({rpy[0]:.4f}, {rpy[1]:.4f}, {rpy[2]:.4f})")
        except:
            pass
        
        return {
            'xyz': origin_urdf,
            'rpy': rpy,
            'type': joint_type,
            'parent_name': parent_occ.name,
            'child_name': child_occ.name
        }
        
    except Exception as e:
        # Fallback: use basic extraction (less robust but won't crash)
        try:
            from ..commands.ACDC4Robot import constants
            text_palette = constants.get_text_palette()
            if text_palette:
                text_palette.writeText(f"[ERROR] extract_joint_data failed: {str(e)}")
        except:
            pass

        try:
            parent_transform = parent_occ.transform2
            joint_transform = get_joint_world_position(joint, parent_occ)
            parent_inv = matrix_invert(parent_transform)
            relative = matrix_multiply(parent_inv, joint_transform)
            
            origin_urdf = convert_to_urdf_coords(relative.translation)
            rpy = math_op.matrix3d_2_pose(relative)[3:6]
            
            return {
                'xyz': origin_urdf,
                'rpy': rpy,
                'type': get_joint_type(joint),
                'parent_name': parent_occ.name,
                'child_name': child_occ.name
            }
        except:
            # Last resort fallback
            return {
                'xyz': adsk.core.Vector3D.create(0, 0, 0),
                'rpy': [0, 0, 0],
                'type': 'fixed',
                'parent_name': parent_occ.name,
                'child_name': child_occ.name
            }


def get_joint_type(joint: adsk.fusion.Joint) -> str:
    """
    Get the URDF joint type from Fusion joint.
    
    Args:
        joint: Fusion Joint object
    
    Returns:
        str: URDF joint type ('revolute', 'prismatic', 'fixed', 'continuous')
    """
    try:
        joint_type = joint.jointMotion.jointType
        
        if joint_type == adsk.fusion.JointTypes.RevoluteJointType:
            # Check if it has limits (revolute) or not (continuous)
            motion = joint.jointMotion
            if hasattr(motion, 'rotationLimits') and motion.rotationLimits.isMaximumValueParametric:
                return 'revolute'
            else:
                return 'continuous'
        elif joint_type == adsk.fusion.JointTypes.SliderJointType:
            return 'prismatic'
        elif joint_type == adsk.fusion.JointTypes.PlanarJointType:
            return 'planar'
        elif joint_type == adsk.fusion.JointTypes.BallJointType:
            return 'ball'
        else:
            return 'fixed'
    except:
        return 'fixed'
