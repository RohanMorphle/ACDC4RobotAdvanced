# Small Part Filter Implementation - COMPLETE

## Overview
Implemented a "Small Part Filter" feature that allows users to exclude small parts from robot export based on bounding box diagonal size.

## Files Modified

### 1. **core/part_filter.py** (NEW)
Helper module for filtering parts based on size.

**Key Functions:**

- `convert_to_cm(value, unit)` → Converts user input (mm/cm/m) to Fusion's internal cm units
  - 1 mm = 0.1 cm
  - 1 cm = 1 cm  
  - 1 m = 100 cm

- `get_bounding_box_diagonal(body)` → Calculates diagonal of bounding box
  - Formula: $D = \sqrt{dx^2 + dy^2 + dz^2}$ (Result in cm)

- `is_structural_part(occurrence, design)` → Checks if part is parent of any joint
  - ALWAYS KEEPS structural parts (safety check)

- `should_filter_part(occurrence, threshold_cm, design)` → Determines if part should be filtered
  - Returns True if part is smaller than threshold AND not structural
  - Returns False if part should be KEPT

- `filter_links(links, threshold_value, threshold_unit, design)` → Main filter function
  - Applies unit conversion
  - Filters link list
  - Returns filtered list

---

### 2. **commands/ACDC4Robot/entry.py** (UPDATED)
UI dialog and event handlers.

**Changes:**

#### UI Controls Added (lines ~108-120):
```python
# Checkbox to enable/disable
exclude_small_parts = inputs.addBoolValueInput("exclude_small_parts", "Exclude Small Parts?", False)

# Unit dropdown (visible when checkbox enabled)
size_unit_dropdown = inputs.addDropDownCommandInput("size_unit_selection", "Unit", ...)
# Options: mm (default), cm, m

# Size threshold input (visible when checkbox enabled)
size_threshold_input = inputs.addValueInput("size_threshold_value", "Min Diagonal Size", "", default=5.0)
```

#### Input Change Handler (lines ~190-200):
- Shows/hides filter inputs when checkbox state changes
- Unit dropdown and threshold input only visible when "Exclude Small Parts" is checked

#### Command Execute Handler (lines ~150-167):
- Reads filter settings from UI
- Stores in constants for use during export
- Passes to `acdc4robot.run()`

---

### 3. **commands/ACDC4Robot/constants.py** (UPDATED)
Global constants and getters/setters for filter settings.

**New Constants:**
```python
EXCLUDE_SMALL_PARTS = False
SIZE_THRESHOLD_VALUE = 5.0
SIZE_THRESHOLD_UNIT = "mm"
```

**New Functions:**
- `set_exclude_small_parts(bool)`
- `get_exclude_small_parts() → bool`
- `set_size_threshold_value(float)`
- `get_size_threshold_value() → float`
- `set_size_threshold_unit(str)`
- `get_size_threshold_unit() → str`

---

### 4. **commands/ACDC4Robot/acdc4robot.py** (UPDATED)
Main export logic.

**Changes:**

- Import: `from ...core import part_filter`

- `get_link_joint_list()` modified (lines ~43-44):
  ```python
  # Apply small part filter if enabled
  if constants.get_exclude_small_parts():
      threshold_value = constants.get_size_threshold_value()
      threshold_unit = constants.get_size_threshold_unit()
      link_list = part_filter.filter_links(link_list, threshold_value, threshold_unit, design)
  ```

---

## Algorithm Flow

### User Perspective:
1. User checks "Exclude Small Parts?" checkbox
2. Unit dropdown appears (mm, cm, m)
3. Size threshold input appears (default 5.0)
4. User sets threshold (e.g., "5" mm)
5. Click OK → Export runs with filter applied

### Internal Logic:

```
INPUT: threshold_value=5, threshold_unit="mm"
   ↓
CONVERT to cm: 5 / 10 = 0.5 cm
   ↓
FOR EACH LINK in link_list:
   ├─ Check if structural (parent of joint)
   │   ├─ YES → KEEP (never filter structural)
   │   ├─ NO  → Continue
   │
   ├─ Get bounding box: {minPoint, maxPoint}
   ├─ Calculate diagonal: D = sqrt(dx² + dy² + dz²)  [in cm]
   │
   ├─ IF D < 0.5 cm AND not structural
   │   └─ FILTER OUT (remove from list)
   ├─ ELSE
   │   └─ KEEP (add to filtered list)
   ↓
OUTPUT: filtered_link_list
```

---

## Unit Conversion Examples

| User Input | Unit | Conversion | Internal (cm) |
|-----------|------|-----------|--------------|
| 5         | mm   | 5 / 10    | 0.5 cm      |
| 0.5       | cm   | 0.5 × 1   | 0.5 cm      |
| 0.005     | m    | 0.005 × 100 | 0.5 cm  |
| 25        | mm   | 25 / 10   | 2.5 cm      |
| 1         | cm   | 1 × 1     | 1 cm        |

---

## Safety Features

### 1. Structural Part Protection
- Parts that are parents of joints are NEVER filtered
- Prevents breaking robot kinematics

### 2. Fallback Behavior
- If bounding box calculation fails → KEEP part (safe default)
- If structural check fails → ASSUME structural → KEEP part

### 3. Default Values
- Filter OFF by default (user must enable)
- Default threshold: 5mm (cosmetic parts typically < 5mm)
- Default unit: mm (most intuitive for small parts)

---

## Testing Checklist

- [ ] Enable filter, set threshold to 5mm, confirm small parts removed
- [ ] Check that parts connected by joints are preserved
- [ ] Verify unit conversion: 5mm = 0.5cm = 0.005m all give same result
- [ ] Test with different thresholds (1mm, 10mm, 100mm)
- [ ] Confirm filter OFF removes no parts
- [ ] Verify URDF has correct number of links

---

## Integration Complete ✅

The feature is fully integrated and ready to use:
1. UI controls show/hide dynamically
2. Filter settings stored in constants
3. Applied during link list creation
4. Structural parts protected
5. Unit conversion automatic
