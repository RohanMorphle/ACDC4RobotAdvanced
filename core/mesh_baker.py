# -*- coding: utf-8 -*-
"""
Mesh Baking Utility - Transforms meshes to their joint frames before export
"""

import adsk, adsk.core, adsk.fusion
from . import math_operation as math_op

def get_baked_mesh_body(body: adsk.fusion.BRepBody, bake_transform: adsk.core.Matrix3D):
    """
    Create a temporary copy of a body and apply a transform to it.
    
    Parameters:
    -----------
    body: adsk.fusion.BRepBody
        The original body to copy and transform
    bake_transform: adsk.core.Matrix3D
        The transform matrix to apply (World * Joint_Inverse)
    
    Returns:
    --------
    adsk.fusion.BRepBody
        The temporary transformed body (lives in TemporaryBRepManager)
    """
    try:
        # Get the temporary BRep manager
        temp_mgr = adsk.fusion.TemporaryBRepManager.get()
        
        # Copy the body into the temporary manager
        temp_body = temp_mgr.copy(body)
        
        # Apply the transform to move the body to the joint frame
        temp_mgr.transform(temp_body, bake_transform)
        
        return temp_body
    except Exception as e:
        return body  # Fall back to original if baking fails


def calculate_bake_transform(link_occurrence: adsk.fusion.Occurrence, parent_joint: adsk.fusion.Joint = None) -> adsk.core.Matrix3D:
    """
    Calculate the transform matrix to bake a body to its joint frame.
    
    Parameters:
    -----------
    link_occurrence: adsk.fusion.Occurrence
        The occurrence representing the link
    parent_joint: adsk.fusion.Joint
        The parent joint (None for base link)
    
    Returns:
    --------
    adsk.core.Matrix3D
        The final transform: World_Position * Joint_Inverse
    """
    try:
        # Step A: Get the world transform of the occurrence
        world_transform = link_occurrence.transform2
        
        # Step B & C: Calculate the target frame (joint or world origin)
        if parent_joint is None:
            # Base link: target is world origin (identity)
            target_transform = adsk.core.Matrix3D.create()
            target_transform.translation = adsk.core.Point3D.create(0, 0, 0)
            joint_inverse = target_transform
        else:
            # Child link: target is the parent joint's origin
            # Get the joint's geometry matrix
            joint_geometry = parent_joint.geometryOrOriginOne
            if joint_geometry is not None:
                joint_transform = joint_geometry.transform
            else:
                # Fallback: use joint's occurrence transform
                joint_transform = parent_joint.occurrenceTwo.transform2
            
            # Calculate inverse of joint transform
            joint_inverse = adsk.core.Matrix3D.create()
            joint_inverse.setWithMatrix3D(joint_transform)
            joint_inverse.invert()
        
        # Final bake transform: World * Joint_Inverse
        bake_transform = adsk.core.Matrix3D.create()
        bake_transform.setWithMatrix3D(world_transform)
        bake_transform.transformBy(joint_inverse)
        
        return bake_transform
    except Exception as e:
        # Fallback: return identity matrix if calculation fails
        return adsk.core.Matrix3D.create()


def transform_point(point: adsk.core.Point3D, transform: adsk.core.Matrix3D) -> list:
    """
    Apply a 4x4 transformation matrix to a 3D point.
    
    Parameters:
    -----------
    point: adsk.core.Point3D
        The point to transform
    transform: adsk.core.Matrix3D
        The transformation matrix
    
    Returns:
    --------
    list: [x, y, z] in meters
    """
    try:
        # Create a vector from the point
        vec = adsk.core.Vector3D.create(point.x, point.y, point.z)
        
        # Transform the point using the matrix
        transformed_pt = adsk.core.Point3D.create(point.x, point.y, point.z)
        transformed_pt.transformBy(transform)
        
        # Convert to meters (from cm) and return
        return [transformed_pt.x * 0.01, transformed_pt.y * 0.01, transformed_pt.z * 0.01]
    except Exception as e:
        return [0, 0, 0]
