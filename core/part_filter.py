# -*- coding: utf-8 -*-
"""
Small Part Filter - Filters out parts based on bounding box diagonal size
Prevents small/cosmetic parts from being included in robot export
"""

import adsk, adsk.core, adsk.fusion
import math
from ..commands.ACDC4Robot import constants


def log_message(msg: str):
    """Log a message to Fusion's text palette"""
    try:
        text_palette = constants.get_text_palette()
        if text_palette:
            text_palette.writeText(f"[FILTER] {msg}")
    except:
        pass


def convert_to_cm(value: float, unit: str) -> float:
    """
    Convert user input to Fusion's internal units (cm).
    
    Parameters:
    -----------
    value: float
        The size value from user input
    unit: str
        User's selected unit ('mm', 'cm', 'm')
    
    Returns:
    --------
    float: Value converted to centimeters
    """
    if unit == 'mm':
        return value / 10.0  # 1mm = 0.1cm
    elif unit == 'cm':
        return value  # Already in cm
    elif unit == 'm':
        return value * 100.0  # 1m = 100cm
    else:
        return value  # Default to cm


def get_bounding_box_diagonal(body: adsk.fusion.BRepBody) -> float:
    """
    Calculate the diagonal of a body's bounding box.
    Result is in Fusion's internal units (cm).
    
    Parameters:
    -----------
    body: adsk.fusion.BRepBody
        The body to measure
    
    Returns:
    --------
    float: Diagonal length in cm
    """
    try:
        bbox = body.boundingBox
        dx = abs(bbox.maxPoint.x - bbox.minPoint.x)
        dy = abs(bbox.maxPoint.y - bbox.minPoint.y)
        dz = abs(bbox.maxPoint.z - bbox.minPoint.z)
        
        # Calculate diagonal: sqrt(dx² + dy² + dz²)
        diagonal = math.sqrt(dx**2 + dy**2 + dz**2)
        
        return diagonal
    except Exception as e:
        log_message(f"ERROR getting bbox: {str(e)}")
        return float('inf')  # If we can't get bbox, assume it's not small


def count_joints_per_link(links: list, design: adsk.fusion.Design) -> dict:
    """
    Count how many joints reference each link.
    
    Parameters:
    -----------
    links: list[Link]
        List of Link objects
    design: adsk.fusion.Design
        The design to search for joints
    
    Returns:
    --------
    dict: Maps link name → number of joints connected to it
    """
    # Create a mapping from fullPathName to link name
    path_to_name = {}
    
    for link in links:
        try:
            occ = link.get_link_occ()
            # Use link.name if available, otherwise use occurrence.name
            link_name = link.name if link.name else occ.name
            path_to_name[occ.fullPathName] = link_name
            log_message(f"DEBUG: Registered link {link_name} with path {occ.fullPathName}")
        except Exception as e:
            log_message(f"DEBUG: Error registering link: {str(e)}")
            pass
    
    joint_count = {}
    
    # Initialize count for all links
    for path, link_name in path_to_name.items():
        joint_count[link_name] = 0
    
    # Count joints connected to each link
    try:
        root = design.rootComponent
        for joint in root.allJoints:
            try:
                # Get parent and child occurrences
                parent_occ = joint.occurrenceOne
                child_occ = joint.occurrenceTwo
                
                # Check if parent is in our links
                if parent_occ and parent_occ.fullPathName in path_to_name:
                    parent_name = path_to_name[parent_occ.fullPathName]
                    joint_count[parent_name] += 1
                    log_message(f"DEBUG: Joint connects to parent {parent_name}")
                
                # Check if child is in our links
                if child_occ and child_occ.fullPathName in path_to_name:
                    child_name = path_to_name[child_occ.fullPathName]
                    joint_count[child_name] += 1
                    log_message(f"DEBUG: Joint connects to child {child_name}")
                    
            except Exception as e:
                log_message(f"DEBUG: Error processing joint: {str(e)}")
                pass
    except Exception as e:
        log_message(f"ERROR counting joints: {str(e)}")
    
    log_message(f"DEBUG: Final joint count = {joint_count}")
    return joint_count


def should_filter_part(occurrence: adsk.fusion.Occurrence, 
                      threshold_cm: float,
                      num_joints: int) -> bool:
    """
    Determine if a part should be filtered out.
    
    NEW LOGIC:
    - REMOVE if: Has EXACTLY 1 joint AND is below threshold (fastener/screw)
    - KEEP if: Has 2+ joints (connector piece)
    - KEEP if: Has 0 joints (floating piece)
    - KEEP if: Above threshold (large piece)
    
    Parameters:
    -----------
    occurrence: adsk.fusion.Occurrence
        The occurrence to check
    threshold_cm: float
        Minimum diagonal size in cm (Fusion internal units)
    num_joints: int
        Number of joints connected to this link
    
    Returns:
    --------
    bool: True if part should be FILTERED OUT, False if it should be KEPT
    """
    part_name = occurrence.name
    
    try:
        # Get bounding box
        if occurrence.bRepBodies.count == 0:
            log_message(f"KEEP: {part_name} (no bodies)")
            return False
        
        # Calculate max diagonal across all bodies
        max_diagonal = 0.0
        for i in range(occurrence.bRepBodies.count):
            body = occurrence.bRepBodies.item(i)
            diagonal = get_bounding_box_diagonal(body)
            max_diagonal = max(max_diagonal, diagonal)
        
        # Decision logic based on joint count
        if num_joints >= 2:
            log_message(f"KEEP: {part_name} (connector - {num_joints} joints, diagonal={max_diagonal:.2f}cm)")
            return False
        elif num_joints == 0:
            log_message(f"KEEP: {part_name} (floating - {num_joints} joints, diagonal={max_diagonal:.2f}cm)")
            return False
        elif max_diagonal >= threshold_cm:
            log_message(f"KEEP: {part_name} (large - {max_diagonal:.2f}cm >= {threshold_cm:.2f}cm, joints={num_joints})")
            return False
        else:
            # FILTER: Exactly 1 joint AND small
            log_message(f"FILTER: {part_name} (fastener - {max_diagonal:.2f}cm < {threshold_cm:.2f}cm, joints={num_joints})")
            return True
            
    except Exception as e:
        log_message(f"KEEP: {part_name} (error checking: {str(e)})")
        return False  # Safe default: keep if we can't analyze


def filter_links(links: list, threshold_value: float, threshold_unit: str, design: adsk.fusion.Design) -> list:
    """
    Filter a list of Link objects based on safer logic.
    
    SAFER APPROACH:
    - Only removes parts with EXACTLY 1 joint (fasteners/screws)
    - Keeps connector parts (2+ joints)
    - Keeps floating parts (0 joints)
    - Respects size threshold
    - Preserves kinematic chains
    
    Parameters:
    -----------
    links: list[Link]
        List of Link objects to filter
    threshold_value: float
        Threshold value from user input
    threshold_unit: str
        Unit of threshold value ('mm', 'cm', 'm')
    design: adsk.fusion.Design
        The design to analyze joints
    
    Returns:
    --------
    list[Link]: Filtered list of Link objects (small fasteners removed)
    """
    if not links or threshold_value <= 0:
        return links
    
    # Convert threshold to cm
    threshold_cm = convert_to_cm(threshold_value, threshold_unit)
    
    log_message(f"=== FILTER START ===")
    log_message(f"Threshold: {threshold_value}{threshold_unit} ({threshold_cm:.4f}cm)")
    log_message(f"Analyzing {len(links)} exported links")
    
    # Count joints per link
    joint_count = count_joints_per_link(links, design)
    log_message(f"Joint distribution: {dict(sorted(joint_count.items(), key=lambda x: x[1], reverse=True))}")
    
    filtered_links = []
    removed_count = 0
    
    for link in links:
        occurrence = link.get_link_occ()
        # Use occurrence.name since link.name might be None
        occ_name = link.name if link.name else occurrence.name
        num_joints = joint_count.get(occ_name, 0)
        
        # Check if part should be filtered
        if not should_filter_part(occurrence, threshold_cm, num_joints):
            filtered_links.append(link)
        else:
            removed_count += 1
    
    log_message(f"=== FILTER END ===")
    log_message(f"Removed {removed_count} small fasteners")
    log_message(f"Keeping {len(filtered_links)} structural/large parts")
    
    return filtered_links
