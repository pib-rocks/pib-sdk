# Unit Conventions

Joint angle units and conversions across different components.

## Unit Summary

| Context | Unit | Range (typical) |
|---------|------|-----------------|
| Canonical (Trajectory/Webots) | radians | -π/2 to +π/2 |
| User API (default) | percentage | 0% to 100% |

| Real robot (ROS) | centidegrees | -9000 to +9000 |

## Percentage System

The percentage system provides an intuitive interface for joint control:

- **0%**: Joint at minimum limit (fully closed/back)
- **50%**: Joint at midpoint
- **100%**: Joint at maximum limit (fully open/forward)

```python
from pib3 import Robot, Joint

with Robot() as robot:
    # Move to middle position
    robot.set_joint(Joint.ELBOW_LEFT, 50.0)

    # Fully extend
    robot.set_joint(Joint.ELBOW_LEFT, 100.0)

    # Fully retract
    robot.set_joint(Joint.ELBOW_LEFT, 0.0)
```

!!! note "Calibration Required"
    The percentage system requires calibrated joint limits. See the [Calibration Guide](../getting-started/calibration.md).

### Conversion Formula

```python
def percent_to_radians(percent, min_rad, max_rad):
    """Convert percentage to radians."""
    return min_rad + (percent / 100.0) * (max_rad - min_rad)

def radians_to_percent(radians, min_rad, max_rad):
    """Convert radians to percentage."""
    return ((radians - min_rad) / (max_rad - min_rad)) * 100.0
```

## Radians

Direct angular control in radians:

```python
from pib3 import Robot, Joint
import math

with Robot() as robot:
    # Set exact angle
    robot.set_joint(Joint.ELBOW_LEFT, math.pi / 4, unit="rad")  # 45 degrees

    # Read in radians
    pos = robot.get_joint(Joint.ELBOW_LEFT, unit="rad")
    print(f"Elbow at {math.degrees(pos):.1f} degrees")
```

## Canonical Format (Webots)

The canonical format uses **Webots motor radians** directly. This is the native format for Webots simulation and the internal storage format for trajectories.

Webots motors use sensible radian ranges (e.g., -π/2 to +π/2 for the head motor).

### No Offset for Webots

The `WebotsBackend` uses the canonical format directly:

```python
class WebotsBackend(RobotBackend):
    WEBOTS_OFFSET = 0.0  # No conversion needed

    def _to_backend_format(self, radians):
        return radians  # Identity

    def _from_backend_format(self, values):
        return values  # Identity
```



## Real Robot (ROS) Format

The real robot expects positions in **centidegrees** (hundredths of a degree):

```
centidegrees = degrees(radians) * 100
centidegrees = radians * (180 / π) * 100
```

### Example Conversions

| Radians | Degrees | Centidegrees |
|---------|---------|--------------|
| 0.0 | 0° | 0 |
| π/4 | 45° | 4500 |
| π/2 | 90° | 9000 |
| π | 180° | 18000 |

### Handled Automatically

The `RealRobotBackend` converts automatically:

```python
class RealRobotBackend(RobotBackend):
    def _to_backend_format(self, radians):
        return np.round(np.degrees(radians) * 100).astype(int)

    def _from_backend_format(self, centidegrees):
        return np.radians(np.array(centidegrees) / 100.0)
```

## Trajectory Format

Trajectories are stored in **canonical Webots motor radians**:

```json
{
  "format_version": "1.0",
  "unit": "radians",
  "coordinate_frame": "webots",
  "waypoints": [
    [0.5, 1.2, 0.8, ...],
    [0.6, 1.3, 0.9, ...]
  ]
}
```

Backends convert to their native format when executing:

- **Webots**: Uses waypoints directly (no conversion)

- **Real Robot**: Converts to centidegrees

## Comparison Table

| Operation | Webots | Real Robot |
|-----------|-------|--------|------------|
| Internal storage | radians | radians | radians |
| Backend conversion | -1.0 rad | none | to centideg |
| Position sensor | +1.0 rad | none | from centideg |
| API (percentage) | ✓ | ✓ | ✓ |
| API (radians) | ✓ | ✓ | ✓ |

## Code Examples

### Working with Different Units

```python
from pib3 import Robot, Joint
import math

# Both backends support the same API
def demonstrate_units(backend):
    # Percentage (recommended for most uses)
    backend.set_joint(Joint.ELBOW_LEFT, 50.0)
    pos_pct = backend.get_joint(Joint.ELBOW_LEFT)
    print(f"Percentage: {pos_pct:.1f}%")

    # Radians (for precise control)
    backend.set_joint(Joint.ELBOW_LEFT, math.pi / 3, unit="rad")
    pos_rad = backend.get_joint(Joint.ELBOW_LEFT, unit="rad")
    print(f"Radians: {pos_rad:.3f}")

# Works the same with both backends


with Robot(host="172.26.34.149") as robot:
    demonstrate_units(robot)
```

### Manual Conversion

```python
from pib3.backends.base import RobotBackend
import math

# If you need manual conversion (rarely needed)
def manual_convert():
    # Get calibration data
    limits = RobotBackend.JOINT_LIMITS  # Dict of {name: (min, max)}

    joint = "elbow_left"
    min_rad, max_rad = limits[joint]

    # Percent to radians
    percent = 75.0
    radians = min_rad + (percent / 100.0) * (max_rad - min_rad)

    # Radians to degrees
    degrees = math.degrees(radians)

    # To centidegrees (for ROS)
    centideg = int(degrees * 100)

    print(f"{percent}% = {radians:.3f} rad = {degrees:.1f}° = {centideg} centideg")
```
