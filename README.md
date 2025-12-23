[README.md](https://github.com/user-attachments/files/24310824/README.md)
# Morphle Advance Urdf Exporter

A comprehensive Fusion 360 Add-In for exporting robot models to URDF, SDFormat (SDF), and MJCF formats, specifically tailored for Morphle's advanced robotics workflows.

This exporter combines robust coordinate system handling from the upstream ACDC4Robot repository with custom features for production-grade model flexibility.

## Features

### 1. Accurate Coordinate Positioning
*   **Upstream Alignment**: Implements the "Globalize-Then-Relativize" strategy for joint origin extraction, ensuring 100% compatibility with standard robotics simulators (Gazebo, PyBullet, MuJoCo).
*   **Correct Axis Handling**: Maintains strict alignment between Link and Joint frames.

### 2. flexible Mesh Support
*   **Dual Format**: Support for both **STL** (millimeter precision) and **OBJ** (centimeter scale) formats.
*   **Dynamic Visuals**: Automatically configures URDF/SDF output to reference the correct file extension and apply the necessary scale factors (`0.001` for STL, `0.01` for OBJ).

### 3. Smart Filtering (Advanced Feature)
*   **Small Part Exclusion**: Automatically filters out fasteners, screws, and debris smaller than a user-defined threshold (e.g., < 5cm diagonal).
*   **Component Logic**: Intelligently preserves small structural connectors if they serve as critical joint parents/children, ensuring the kinematic chain remains unbroken.

### 4. Simulator Ready
*   **Gazebo**: Full URDF/SDF support with collision/visual/inertial blocks.
*   **PyBullet**: Generates `hello_pybullet.py` boilerplates.
*   **MuJoCo**: Direct MJCF XML export support.

## Usage

1.  **Install**: Place the `.bundle` in your Fusion 360 `ApplicationPlugins` folder.
2.  **Run**: Open Fusion 360, go to `Tools -> Add-Ins`, and start `ACDC4Robot`.
3.  **Configure**:
    *   Select your `Robot Description Format` (URDF/SDF/MJCF).
    *   Choose the `Simulation Environment` (PyBullet/Gazebo/MuJoCo).
    *   (Optional) Enable "Exclude Small Parts" and set a size threshold.
    *   (Optional) Select specific components to export as OBJ (default is STL).
4.  **Export**: Click the command button to generate the package.

## Project Structure

*   `ACDC4Robot.py`: Main entry point and orchestrator.
*   `core/urdf.py`: URDF generation logic (Custom patched for dynamic extensions).
*   `core/joint.py` & `core/link.py`: Geometric extraction logic (Reference implementation).
*   `core/part_filter.py`: Custom filtering logic.

---
*Maintained by Morphle Engineering*
