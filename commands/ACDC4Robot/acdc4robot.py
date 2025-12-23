# -*- coding: utf-8 -*-
# Author: nuofan
"""
Export robot description format files from Fusion360 design
"""

import adsk, adsk.core, adsk.fusion, traceback
import os, sys
from ...core.link import Link
from ...core.joint import Joint
from . import constants
from ...core import write
from ...core import utils
from ...core import part_filter
import time

def get_link_joint_list(design: adsk.fusion.Design):
    """
    Get the link list and joint list to export

    Return:
    link_list: [Link]
        a list contains all the links that will be exported
    joint_list: [Joint]
        a list contains all the joint that will be exported
    """
    root = design.rootComponent
    link_list = []
    joint_list = []
    occs: adsk.fusion.OccurrenceList = root.allOccurrences
    
    # try to solve the nested components problem
    # but still not fully tested
    for occ in occs:
        # TODO: it seems use occ.joints.count will make it usable with occurrences? Test it
        if occ.component.joints.count > 0:
            # textPalette.writeText(str(occ.fullPathName))
            continue
        else:
            # Only occurrence contains zero joint and has zero childOccurrences 
            # can be seen as a link
            if occ.childOccurrences.count > 0:
                # textPalette.writeText(str(occ.fullPathName))
                # textPalette.writeText(str(occ.childOccurrences.count))
                continue
            else:
                # textPalette.writeText(str(occ.fullPathName))
                # textPalette.writeText(str(occ.childOccurrences.count))
                if occ.isLightBulbOn:
                    # only the occurrence light bulb on that the occurrence will be exported
                    link_list.append(Link(occ)) # add link objects into link_list

    # Apply small part filter if enabled
    exclude_small = constants.get_exclude_small_parts()
    
    if exclude_small:
        threshold_value = constants.get_size_threshold_value()
        threshold_unit = constants.get_size_threshold_unit()
        
        try:
            text_palette = constants.get_text_palette()
            if text_palette:
                text_palette.writeText(f"[FILTER] Calling filter_links with threshold={threshold_value}{threshold_unit}")
        except:
            pass
        
        link_list = part_filter.filter_links(link_list, threshold_value, threshold_unit, design)
        
        try:
            text_palette = constants.get_text_palette()
            if text_palette:
                text_palette.writeText(f"[FILTER] Filter returned {len(link_list)} links")
        except:
            pass

    # AUTO-GROUND DETECTION: Ensure grounded component is always the root link
    # This fixes coordinate frame inconsistencies when joint hierarchy is wrong
    ground_link_index = -1
    
    try:
        text_palette = constants.get_text_palette()
        if text_palette:
            text_palette.writeText(f"[AUTO-GROUND] Scanning {len(link_list)} links for grounded component...")
    except:
        pass
    
    for i, link in enumerate(link_list):
        try:
            occ = link.get_link_occ()
            comp = occ.component
            
            # Debug: show each component and its ground status
            try:
                text_palette = constants.get_text_palette()
                if text_palette:
                    text_palette.writeText(f"[AUTO-GROUND]   Link {i}: {occ.name}, isGrounded={comp.isGrounded}")
            except:
                pass
            
            if comp.isGrounded:
                ground_link_index = i
                try:
                    text_palette = constants.get_text_palette()
                    if text_palette:
                        text_palette.writeText(f"[AUTO-GROUND] Detected grounded component: {occ.name}")
                        text_palette.writeText(f"[AUTO-GROUND] Moving to root position for consistent coordinate frame")
                except:
                    pass
                break
        except:
            pass
    
    # Move grounded link to front of list (becomes URDF root)
    if ground_link_index > 0:  # Only reorder if ground is not already first
        ground_link = link_list.pop(ground_link_index)
        link_list.insert(0, ground_link)
        try:
            text_palette = constants.get_text_palette()
            if text_palette:
                text_palette.writeText(f"[AUTO-GROUND] Reordered link list - ground component is now root")
        except:
            pass
    elif ground_link_index == 0:
        try:
            text_palette = constants.get_text_palette()
            if text_palette:
                text_palette.writeText(f"[AUTO-GROUND] Ground component already at root position")
        except:
            pass
    else:
        try:
            text_palette = constants.get_text_palette()
            if text_palette:
                text_palette.writeText(f"[AUTO-GROUND] WARNING: No grounded component found!")
                text_palette.writeText(f"[AUTO-GROUND] URDF root will be: {link_list[0].get_name() if link_list else 'NONE'}")
        except:
            pass

    # Build list of remaining link names for joint validation
    remaining_link_names = set()
    for link in link_list:
        try:
            occ = link.get_link_occ()
            remaining_link_names.add(occ.name)
        except:
            pass
    
    for joint in root.allJoints:
        try:
            # Get parent and child occurrence names - validate joint is accessible first
            parent_name = joint.occurrenceOne.name if joint.occurrenceOne else None
            child_name = joint.occurrenceTwo.name if joint.occurrenceTwo else None
            
            # Only add joint if both parent and child links exist
            if parent_name in remaining_link_names and child_name in remaining_link_names:
                joint_list.append(Joint(joint))
            else:
                try:
                    text_palette = constants.get_text_palette()
                    if text_palette:
                        text_palette.writeText(f"[FILTER] Skipping joint: parent={parent_name}, child={child_name}")
                except:
                    pass
        except Exception as e:
            # Skip invalid/broken joints that throw API errors
            try:
                text_palette = constants.get_text_palette()
                if text_palette:
                    text_palette.writeText(f"[ERROR] Skipping invalid joint: {str(e)}")
            except:
                pass

    return link_list, joint_list

def export_stl(design: adsk.fusion.Design, save_dir: str, links: list[Link]):
    """
    export each component's stl file into "save_dir/mesh"

    Parameters
    ---------
    design: adsk.fusion.Design
        current active design
    save_dir: str
        the directory to store the export stl file
    """
    # create a single exportManager instance
    export_manager = design.exportManager
    # set the directory for the mesh file
    try: os.mkdir(save_dir + "/meshes")
    except: pass
    mesh_dir = save_dir + "/meshes"

    text_palette = constants.get_text_palette()

    for link in links:
        visual_body: adsk.fusion.BRepBody = link.get_visual_body()
        col_body: adsk.fusion.BRepBody = link.get_collision_body()
        # Use OBJ if link.mesh_format is "obj", otherwise use STL
        use_obj = (link.mesh_format == "obj")
        
        occ = link.get_link_occ()
        link_name = link.get_name()

        if text_palette:
             text_palette.writeText(f"[EXPORT] Processing Link: {link_name}")

        if (visual_body is None) and (col_body is None):
            # export the whole occurrence
            mesh_name = mesh_dir + "/" + link_name
            
            # FIX: Use component to export in LOCAL coordinates combined with URDF transform
            export_source = occ.component
            if text_palette:
                text_palette.writeText(f"  -> Exporting Component (Local Frame): {export_source.name}")

            # Retry logic for export
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    if use_obj:
                        export_options = export_manager.createOBJExportOptions(export_source, mesh_name)
                    else:
                        export_options = export_manager.createSTLExportOptions(export_source, mesh_name)
                        # Force MM to match URDF scale 0.001
                        export_options.meshUnit = adsk.fusion.MeshUnits.MillimeterMeshUnit
                        # Ensure binary format for STL
                        export_options.isBinaryFormat = True
                        export_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                    
                    export_options.sendToPrintUtility = False
                    export_manager.execute(export_options)
                    
                    if text_palette:
                        text_palette.writeText(f"  -> SUCCESS: Exported {link_name}")
                    break  # Success, exit retry loop
                    
                except Exception as e:
                    if attempt < max_retries - 1:
                        if text_palette:
                            text_palette.writeText(f"  -> RETRY {attempt + 1}/{max_retries}: Export failed for {link_name}: {str(e)}")
                    else:
                        # Final attempt failed - check if it's a nested assembly
                        is_assembly = False
                        try:
                            is_assembly = export_source.occurrences.count > 0
                        except:
                            pass
                        
                        error_msg = f"FATAL ERROR: Failed to export {link_name} after {max_retries} attempts\n"
                        error_msg += f"Component: {export_source.name}\n"
                        error_msg += f"Format: {'OBJ' if use_obj else 'STL'}\n"
                        
                        if is_assembly:
                            error_msg += f"REASON: THIS IS A NESTED ASSEMBLY! URDF requires flat components.\n"
                            error_msg += f"FIX: Flatten this assembly or dissolve it into individual bodies.\n"
                        
                        error_msg += f"Error: {str(e)}"
                        if text_palette:
                            text_palette.writeText(f"  -> {error_msg}")
                        raise RuntimeError(error_msg)

        elif (visual_body is not None) and (col_body is not None):
            # export visual and collision geometry seperately
            visual_mesh_name = mesh_dir + "/" + link_name + "_visual"
            
            # FIX: Check if body is a Proxy (from occurrence) and get native object (Local)
            if visual_body.assemblyContext:
               visual_export_source = visual_body.nativeObject
               if text_palette:
                    text_palette.writeText(f"  -> Exporting Visual Body PROXY as NATIVE (Local): {visual_export_source.name}")
            else:
               visual_export_source = visual_body
               if text_palette:
                    text_palette.writeText(f"  -> Exporting Visual Body DIRECT (Local): {visual_export_source.name}")

            # Retry logic for visual export
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    if use_obj:
                        visual_exp_options = export_manager.createOBJExportOptions(visual_export_source, visual_mesh_name)
                    else:
                        visual_exp_options = export_manager.createSTLExportOptions(visual_export_source, visual_mesh_name)
                        # Force MM
                        visual_exp_options.meshUnit = adsk.fusion.MeshUnits.MillimeterMeshUnit
                        visual_exp_options.isBinaryFormat = True
                        visual_exp_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
                    
                    visual_exp_options.sendToPrintUtility = False
                    export_manager.execute(visual_exp_options)
                    
                    if text_palette:
                        text_palette.writeText(f"  -> SUCCESS: Exported visual for {link_name}")
                    break
                    
                except Exception as e:
                    if attempt < max_retries - 1:
                        if text_palette:
                            text_palette.writeText(f"  -> RETRY {attempt + 1}/{max_retries}: Visual export failed for {link_name}: {str(e)}")
                    else:
                        # Check if parent component is assembly
                        is_assembly = False
                        try:
                            is_assembly = occ.component.occurrences.count > 0
                        except:
                            pass
                        
                        error_msg = f"FATAL ERROR: Failed to export visual for {link_name} after {max_retries} attempts\n"
                        error_msg += f"Body: {visual_export_source.name}\n"
                        error_msg += f"Format: {'OBJ' if use_obj else 'STL'}\n"
                        
                        if is_assembly:
                            error_msg += f"REASON: Parent component is a NESTED ASSEMBLY! URDF requires flat components.\n"
                            error_msg += f"FIX: Flatten this assembly or dissolve it into individual bodies.\n"
                        
                        error_msg += f"Error: {str(e)}"
                        if text_palette:
                            text_palette.writeText(f"  -> {error_msg}")
                        raise RuntimeError(error_msg)

            col_mesh_name = mesh_dir + "/" + link_name + "_collision"
            
            # FIX: Check if body is a Proxy (from occurrence) and get native object (Local)
            if col_body.assemblyContext:
               col_export_source = col_body.nativeObject
               if text_palette:
                    text_palette.writeText(f"  -> Exporting Collision Body PROXY as NATIVE (Local): {col_export_source.name}")
            else:
               col_export_source = col_body
               if text_palette:
                    text_palette.writeText(f"  -> Exporting Collision Body DIRECT (Local): {col_export_source.name}")

            # Retry logic for collision export
            for attempt in range(max_retries):
                try:
                    if use_obj:
                        col_exp_options = export_manager.createOBJExportOptions(col_export_source, col_mesh_name)
                    else:
                        col_exp_options = export_manager.createSTLExportOptions(col_export_source, col_mesh_name)
                        # Force MM
                        col_exp_options.meshUnit = adsk.fusion.MeshUnits.MillimeterMeshUnit
                        col_exp_options.isBinaryFormat = True
                        col_exp_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                    
                    col_exp_options.sendToPrintUtility = False
                    export_manager.execute(col_exp_options)
                    
                    if text_palette:
                        text_palette.writeText(f"  -> SUCCESS: Exported collision for {link_name}")
                    break
                    
                except Exception as e:
                    if attempt < max_retries - 1:
                        if text_palette:
                            text_palette.writeText(f"  -> RETRY {attempt + 1}/{max_retries}: Collision export failed for {link_name}: {str(e)}")
                    else:
                        # Check if parent component is assembly
                        is_assembly = False
                        try:
                            is_assembly = occ.component.occurrences.count > 0
                        except:
                            pass
                        
                        error_msg = f"FATAL ERROR: Failed to export collision for {link_name} after {max_retries} attempts\n"
                        error_msg += f"Body: {col_export_source.name}\n"
                        error_msg += f"Format: {'OBJ' if use_obj else 'STL'}\n"
                        
                        if is_assembly:
                            error_msg += f"REASON: Parent component is a NESTED ASSEMBLY! URDF requires flat components.\n"
                            error_msg += f"FIX: Flatten this assembly or dissolve it into individual bodies.\n"
                        
                        error_msg += f"Error: {str(e)}"
                        if text_palette:
                            text_palette.writeText(f"  -> {error_msg}")
                        raise RuntimeError(error_msg)

        elif (visual_body is None) and (col_body is not None):
            error_message = "Please set two bodies, one for visual and one for collision. \n"
            error_message = error_message + "Body for visual missing."
            utils.error_box(error_message)
            utils.terminate_box()
        elif (visual_body is not None) and (col_body is None):
            error_message = "Please set two bodies, one for visual and one for collision. \n"
            error_message = error_message + "Body for collision missing."
            utils.error_box(error_message)
            utils.terminate_box()


def run():
    # Initialization
    app = adsk.core.Application.get()
    ui = app.userInterface
    product = app.activeProduct
    design = adsk.fusion.Design.cast(product)

    msg_box_title = "ACDC4Robot Message"
    
    # open a text palette for debuging
    textPalette = ui.palettes.itemById("TextCommands")
    if not textPalette.isVisible:
        textPalette.isVisible = True
    constants.set_text_palette(textPalette)

    try:
        # Set design type into do not capture design history
        design.designType = adsk.fusion.DesignTypes.DirectDesignType

        # # Check the length unit of Fusion360
        # if design.unitsManager.defaultLengthUnits != "m":
        #     ui.messageBox("Please set length unit to 'm'!", msg_box_title)
        #     return 0 # exit run() function
        
        root = design.rootComponent # get root component
        allComp = design.allComponents
        robot_name = root.name.split()[0]
        constants.set_robot_name(robot_name)
        
        # Set the folder to store exported files
        folder_dialog = ui.createFolderDialog()
        folder_dialog.title = "Chose your folder to export"
        dialog_result = folder_dialog.showDialog() # show folder dialog
        save_folder = ""
        if dialog_result == adsk.core.DialogResults.DialogOK:
            save_folder = folder_dialog.folder
        else:
            ui.messageBox("ACDC4Robot was canceled", msg_box_title)
            return 0 # exit run() function
        
        save_folder = save_folder + "/" + robot_name
        try: os.mkdir(save_folder)
        except: pass

        ui.messageBox("Start ACDC4Robot Add-IN", msg_box_title)

        # get all the link & joint elements to export
        link_list, joint_list = get_link_joint_list(design)
        
        # Get currently selected components to export as OBJ
        obj_link_names = set()
        try:
            for sel_item in ui.activeSelections:
                if sel_item.entity and hasattr(sel_item.entity, 'name'):
                    obj_link_names.add(sel_item.entity.name)
        except:
            pass
        
        # Mark selected links as OBJ format
        for link in link_list:
            if link.get_name() in obj_link_names or link.link.name in obj_link_names:
                link.mesh_format = "obj"
        
        if obj_link_names:
            ui.messageBox("Selected components will be exported as OBJ, others as STL", msg_box_title)

        rdf = constants.get_rdf()
        simulator = constants.get_sim_env()

        if rdf == None:
            ui.messageBox("Robot description format is None.\n" +
                          "Please choose one robot description format", msg_box_title)
        elif rdf == "URDF":
            if simulator  == "None":
                ui.messageBox("Simulation environment is None.\n" +
                              "Please select a simulation environment.", msg_box_title)
            elif simulator == "Gazebo":
                # write to .urdf file
                write.write_urdf(link_list, joint_list, save_folder, robot_name)
                # export mesh files
                export_stl(design, save_folder, link_list)
                ui.messageBox("Finished exporting URDF for Gazebo.", msg_box_title)
                
            elif simulator == "PyBullet":
                # write to .urdf file
                write.write_urdf(link_list, joint_list, save_folder, robot_name)
                # export mesh files
                export_stl(design, save_folder, link_list)
                # generate pybullet script
                write.write_hello_pybullet(rdf, robot_name, save_folder)
                ui.messageBox("Finished exporting URDF for PyBullet.", msg_box_title)
            
            elif simulator == "MuJoCo":
                # write to .urdf file
                write.write_urdf(link_list, joint_list, save_folder, robot_name)
                # export mesh files
                export_stl(design, save_folder, link_list)
                
                ui.messageBox("Finished exporting URDF for MuJoCo.", msg_box_title)

        elif rdf == "SDFormat":
            if simulator == "None":
                ui.messageBox("Simulation environment is None.\n" + 
                              "Please select a simulation environment.", msg_box_title)
            elif simulator == "Gazebo":
                # write to .sdf file
                write.write_sdf(link_list, joint_list, save_folder, robot_name)
                # write a model cofig file
                author = constants.get_author_name()
                des = constants.get_model_description()
                write.write_sdf_config(save_folder, robot_name, author, des)
                # export stl files
                export_stl(design, save_folder, link_list)
                ui.messageBox("Finished exporting SDFormat for Gazebo.", msg_box_title)
            elif simulator == "PyBullet":
                # write to .sdf file
                write.write_sdf(link_list, joint_list, save_folder, robot_name)
                # export stl files
                export_stl(design, save_folder, link_list)
                # generate pybullet script
                write.write_hello_pybullet(rdf,robot_name, save_folder)
                ui.messageBox("Finished exporting SDFormat for PyBullet.", msg_box_title)
            
            elif simulator == "MuJoCo":
                ui.messageBox("MuJoCo does not support SDFormat. \n" +
                              "Please select PyBullet or Gazebo as simulation environment.", msg_box_title)

        elif rdf == "MJCF":
            if simulator == "None":
                ui.messageBox("Simulation environment is None. \n" +
                              "Please select a simulation environment.", msg_box_title)
            elif simulator == "Gazebo":
                ui.messageBox("Gazebo does not support MJCF. \n"+
                              "Please select MuJoCo for simulation.", msg_box_title)
            elif simulator == "PyBullet":
                ui.messageBox("PyBullet does not support MJCF. \n" +
                              "Please select MuJoCo for simulation.", msg_box_title)
            elif simulator == "MuJoCo":
                # write to .xml file
                write.write_mjcf(root, robot_name, save_folder)
                # export stl files
                export_stl(design, save_folder, link_list)
                time.stop(0.1)
                ui.messageBox("Finished exporting MJCF for MuJoCo.", msg_box_title)
        
    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
