# pyRevit Tunnel Lining Placement

One-click automated placement of adaptive tunnel lining segments along Civil 3D alignments in Revit.

![Tunnel Placement](icon.png) 

## What it does

- Extracts alignment geometry from linked DWG (Civil 3D feature lines)
- Automatically builds segment chain from a single click
- Places multiple adaptive families along the alignment
- Calculates and assigns rotation parameters for each segment
- Handles straight lines, polylines, and tessellated arcs

## Performance

~40 seconds for 127 segments × 5 families (635 total components)

## Installation

1. Clone this repository to your pyRevit extensions folder:
   ```
   %appdata%\pyRevit\Extensions\
   ```

2. Reload pyRevit

3. Find "Tunnel Adaptive" button in the toolbar

## Usage

1. Link Civil 3D DWG with tunnel alignment into Revit
2. Load adaptive tunnel families (see requirements below)
3. Run the script
4. Enter family names (comma-separated)
5. Click on the **first** line segment — this defines the direction
6. Script automatically finds connected segments and places components

## Family Requirements

Adaptive families must have:
- **2 placement points** (start and end of segment)
- **Rotation_start** parameter (angle, radians)
- **Rotation_end** parameter (angle, radians)

Default families: `Invert_segment, Haunch_right, Haunch_left, Crown, Base_segment`

## Configuration

Edit these values in `script.py` if needed:

```python
ARC_SEGMENT_LENGTH_MM = 10000.0  # Arc tessellation length
MAX_GAP_FT = 50.0                # Maximum gap between segments
MIN_LENGTH_MM = 100.0            # Minimum segment length
ANGLE_OFFSET_DEG = -90.0         # Rotation offset
```

## Workflow

This tool is part of a Civil 3D → Revit infrastructure workflow:

1. Create alignment in Civil 3D
2. Export as feature line to DWG
3. Link DWG into Revit
4. Run this script to place tunnel lining

For the full workflow explanation, see my [LinkedIn post](https://linkedin.com/in/aiamkovoi).

## Credits

Approach to fast pyRevit plugins inspired by [Erik Frits](https://www.linkedin.com/in/erik-frits/) and [LearnPyRevit](https://learnpyrevit.com/).

## License

MIT License — free to use and modify.

## Author

**Aleksandr Iamkovoi** — BIM Modeller | MSc in Civil Engineering

- [LinkedIn](https://linkedin.com/in/aiamkovoi)
- [GitHub](https://github.com/aiamkovoi)
  
Code developed with assistance from Claude AI.
