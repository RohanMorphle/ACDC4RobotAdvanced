# -*- coding: utf-8 -*-

# initialize global constants
SDF_FILE_DIR = "" 
ROBOT_NAME = ""
TEXT_PALETTE = None # adsk.core.Application.palettes
AUTHOR_NAME = "" # author name in model.config
MODEL_DESCRIPTION = "" # model description in model.config
ROBOT_DESCRIPTION_FORMAT = "" # robot description format such as urdf, sdf, mjcf, etc.
SIMULATION_ENVIRONMENT = "" # simulation environment such as Gazebo, PyBullet, MuJoCo, etc.

# Small part filter settings
EXCLUDE_SMALL_PARTS = False # Whether to filter out small parts
SIZE_THRESHOLD_VALUE = 5.0 # Threshold value (in user's selected unit)
SIZE_THRESHOLD_UNIT = "mm" # Unit: 'mm', 'cm', or 'm'

def set_sdf_file_dir(sdf_file_dir: str):
    global SDF_FILE_DIR
    SDF_FILE_DIR = sdf_file_dir

def get_sdf_file_dir():
    return SDF_FILE_DIR

def set_robot_name(robot_name: str):
    global ROBOT_NAME
    ROBOT_NAME = robot_name

def get_robot_name():
    return ROBOT_NAME

def set_text_palette(text_palette):
    global TEXT_PALETTE
    TEXT_PALETTE = text_palette

def set_rdf(robot_description_format):
    global ROBOT_DESCRIPTION_FORMAT
    ROBOT_DESCRIPTION_FORMAT = robot_description_format

def set_sim_env(sim_env):
    global SIMULATION_ENVIRONMENT
    SIMULATION_ENVIRONMENT = sim_env

def get_text_palette():
    return TEXT_PALETTE

def set_author_name(author_name: str):
    global AUTHOR_NAME
    AUTHOR_NAME = author_name

def get_author_name():
    return AUTHOR_NAME

def set_model_description(model_description: str):
    global MODEL_DESCRIPTION
    MODEL_DESCRIPTION = model_description

def get_model_description():
    return MODEL_DESCRIPTION

def set_rdf(robot_description_format: str):
    global ROBOT_DESCRIPTION_FORMAT
    ROBOT_DESCRIPTION_FORMAT = robot_description_format

def get_rdf() -> str:
    """
    Return:
    ---------
    ROBOT_DESCRIPTION_FORMAT: str
        "URDF", "SDFormat", ...
    """
    return ROBOT_DESCRIPTION_FORMAT

def set_sim_env(sim_env: str):
    global SIMULATION_ENVIRONMENT
    SIMULATION_ENVIRONMENT = sim_env

def get_sim_env() -> str:
    """
    Return:
    ---------
    SIMULATION_ENVIRONMENT: str
        "Gazebo", "PyBullet", ...
    """
    return SIMULATION_ENVIRONMENT

# Small part filter functions
def set_exclude_small_parts(exclude: bool):
    global EXCLUDE_SMALL_PARTS
    EXCLUDE_SMALL_PARTS = exclude

def get_exclude_small_parts() -> bool:
    return EXCLUDE_SMALL_PARTS

def set_size_threshold_value(value: float):
    global SIZE_THRESHOLD_VALUE
    SIZE_THRESHOLD_VALUE = value

def get_size_threshold_value() -> float:
    return SIZE_THRESHOLD_VALUE

def set_size_threshold_unit(unit: str):
    global SIZE_THRESHOLD_UNIT
    SIZE_THRESHOLD_UNIT = unit

def get_size_threshold_unit() -> str:
    return SIZE_THRESHOLD_UNIT