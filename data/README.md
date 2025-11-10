# AstraTag Data Files

This directory contains the marker dictionary and sampling keypoints that define the AstraTag detection system.

## Files

### `new_dictionary.json`
**Purpose**: Defines all known markers with their binary signatures and 3D geometry.

**Contents**: 
- 20 marker definitions (IDs 0-19, plus extras)
- 4 orientations per marker (0°, 90°, 180°, 270°)
- 48-bit binary signatures for each orientation
- 3D world coordinates for pose estimation

**Usage**: Loaded at runtime via `load_tag_dictionary("data/new_dictionary.json")`

**Format**:
```json
{
  "marker_id": {
    "orientation": {
      "signature": "48-bit binary string",
      "world_points": [[x,y,z], [x,y,z], ...]
    }
  }
}
```

### `keypoints.txt`
**Purpose**: Defines the 48 triangular sampling regions used to extract marker signatures.

**Contents**: 48 triangular regions in normalized 700×700 coordinate space

**Usage**: Loaded at runtime via `load_keypoints("data/keypoints.txt")`

**Format**:
```
triangle_0,x1,y1,x2,y2,x3,y3
triangle_1,x1,y1,x2,y2,x3,y3
...
triangle_47,x1,y1,x2,y2,x3,y3
```

## Installation

These files are **required** for AstraTag detection to work. They define the marker system's visual appearance and encoding scheme.

### System-wide Installation
```bash
sudo mkdir -p /usr/local/share/astratag
sudo cp data/* /usr/local/share/astratag/
```

### User Installation
```bash
mkdir -p ~/.local/share/astratag
cp data/* ~/.local/share/astratag/
```

### In-place Usage
```bash
# Run from project root
./build/astratag_detect
# Will automatically find data/ directory
```

## Creating Custom Dictionaries

To create your own marker dictionary:

1. **Design markers**: Create unique binary patterns (48 bits each)
2. **Define geometry**: Specify 3D coordinates for each marker corner/feature
3. **Generate JSON**: Follow the format in `new_dictionary.json`
4. **Test**: Verify with example application

Example Python script to generate dictionary:
```python
import json

dictionary = {}
for marker_id in range(20):
    dictionary[str(marker_id)] = {
        "0": {
            "signature": generate_signature(),  # Your function
            "world_points": [[0,0,0], [1,0,0], [1,1,0], [0,1,0], ...]
        },
        # Add 90, 180, 270 degree rotations
    }

with open("custom_dictionary.json", "w") as f:
    json.dump(dictionary, f, indent=2)
```

## Modifying Sampling Regions

The keypoints define how signatures are extracted. To modify:

1. Open `keypoints.txt`
2. Each line: `triangle_N,x1,y1,x2,y2,x3,y3`
3. Coordinates are in 700×700 normalized space
4. Must have exactly 48 triangular regions
5. Regenerate dictionary signatures after changing keypoints

## Notes

- These files define the **marker encoding scheme**
- Changing them requires regenerating all printed markers
- Keep backups when creating custom dictionaries
- Dictionary and keypoints must be consistent with each other
