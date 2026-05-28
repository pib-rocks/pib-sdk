# Code Style Guide

Coding conventions and standards for pib3.

## Python Style

We follow [PEP 8](https://pep8.org/) with these specifics:

### Line Length

- Maximum 100 characters (relaxed from PEP 8's 79)
- Docstrings and comments: 80 characters preferred

### Imports

```python
# Standard library
import math
import os
from pathlib import Path
from typing import Dict, List, Optional

# Third-party
import numpy as np
from PIL import Image

# Local
from pib3.config import TrajectoryConfig
from pib3.types import Sketch, Stroke
```

### Naming Conventions

```python
# Modules: lowercase with underscores
trajectory.py
proto_converter.py

# Classes: CamelCase
class RobotBackend:
class TrajectoryConfig:

# Functions/methods: lowercase with underscores
def image_to_sketch():
def get_joint():

# Constants: UPPERCASE with underscores
WEBOTS_OFFSET = 0.0
DEFAULT_TOLERANCE = 0.001

# Private: leading underscore
def _internal_helper():
_cached_result = None
```

## Type Hints

Always include type hints for public functions:

```python
from typing import Dict, List, Optional, Union
import numpy as np

def set_joint(
    self,
    name: str,
    position: float,
    unit: str = "percent",
    async_: bool = True,
) -> bool:
    """Set a joint position."""
    ...

def get_joints(
    self,
    names: Optional[List[str]] = None,
) -> Dict[str, float]:
    """Get multiple joint positions."""
    ...
```

### Common Types

```python
from typing import Callable, Dict, List, Optional, Tuple, Union
from pathlib import Path
import numpy as np
from numpy.typing import NDArray

# Path types
ImageInput = Union[str, Path, np.ndarray, "PIL.Image.Image"]

# Callback types
ProgressCallback = Callable[[int, int], None]
IKProgressCallback = Callable[[int, int, bool], None]

# Array types
JointArray = NDArray[np.float64]  # Shape: (n_joints,)
WaypointArray = NDArray[np.float64]  # Shape: (n_waypoints, n_joints)
```

## Docstrings

Use Google-style docstrings:

```python
def image_to_sketch(
    image: ImageInput,
    config: Optional[ImageConfig] = None,
) -> Sketch:
    """
    Convert an image to a Sketch for trajectory generation.

    Processes the input image to extract contours, simplifies them,
    and returns a normalized Sketch object.

    Args:
        image: Input image. Can be:
            - Path to image file (str or Path)
            - NumPy array (grayscale, RGB, or RGBA)
            - PIL Image object
        config: Optional processing configuration. If None,
            uses default ImageConfig.

    Returns:
        A Sketch object containing normalized strokes.

    Raises:
        FileNotFoundError: If image path doesn't exist.
        ValueError: If image format is not supported.

    Example:
        >>> sketch = image_to_sketch("drawing.png")
        >>> print(f"Extracted {len(sketch)} strokes")
        Extracted 15 strokes

        >>> from pib3 import ImageConfig
        >>> config = ImageConfig(threshold=100)
        >>> sketch = image_to_sketch("light_sketch.jpg", config)
    """
```

### Class Docstrings

```python
class Trajectory:
    """
    Container for robot joint trajectories.

    Stores a sequence of joint positions (waypoints) along with
    metadata about the trajectory source and generation.

    Attributes:
        joint_names: List of joint names in order.
        waypoints: Array of shape (n_waypoints, n_joints) in radians.
        metadata: Dictionary with source info, timestamps, etc.

    Example:
        >>> trajectory = Trajectory.from_json("path.json")
        >>> print(f"Trajectory has {len(trajectory)} waypoints")
        >>> trajectory.to_json("output.json")
    """
```

## Error Handling

### Use Specific Exceptions

```python
# Good: specific exception with context
if not path.exists():
    raise FileNotFoundError(f"Image file not found: {path}")

if threshold < 0 or threshold > 255:
    raise ValueError(f"Threshold must be 0-255, got {threshold}")

# Bad: generic exception
raise Exception("Something went wrong")
```

### Document Exceptions

```python
def connect(self) -> None:
    """
    Connect to the robot.

    Raises:
        ConnectionError: If unable to connect within timeout.
        ValueError: If host address is invalid.
    """
```

## Classes

### Dataclasses for Configuration

```python
from dataclasses import dataclass, field
from typing import Optional

@dataclass
class PaperConfig:
    """Configuration for drawing paper position."""

    center_x: float = 0.15
    center_y: float = 0.15
    height_z: float = 0.74
    size: float = 0.12
    drawing_scale: float = 0.9

    def __post_init__(self):
        if self.size <= 0:
            raise ValueError(f"Paper size must be positive, got {self.size}")
```

### Abstract Base Classes

```python
from abc import ABC, abstractmethod

class RobotBackend(ABC):
    """Abstract base class for robot control backends."""

    @abstractmethod
    def connect(self) -> None:
        """Connect to the robot."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the robot."""
        ...

    # Concrete method with default implementation
    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *args):
        self.disconnect()
```

## Testing

### Test Structure

```python
import pytest
from pib3 import image_to_sketch, Sketch

class TestImageToSketch:
    """Tests for image_to_sketch function."""

    def test_from_file_path(self, tmp_path):
        """Should load image from file path."""
        # Arrange
        image_path = tmp_path / "test.png"
        create_test_image(image_path)

        # Act
        sketch = image_to_sketch(image_path)

        # Assert
        assert isinstance(sketch, Sketch)
        assert len(sketch) > 0

    def test_invalid_path_raises(self):
        """Should raise FileNotFoundError for missing file."""
        with pytest.raises(FileNotFoundError):
            image_to_sketch("nonexistent.png")

    @pytest.mark.parametrize("threshold", [0, 128, 255])
    def test_threshold_values(self, threshold, sample_image):
        """Should accept valid threshold values."""
        config = ImageConfig(threshold=threshold)
        sketch = image_to_sketch(sample_image, config)
        assert isinstance(sketch, Sketch)
```

### Fixtures

```python
import pytest
import numpy as np

@pytest.fixture
def sample_image():
    """Create a simple test image."""
    img = np.zeros((100, 100), dtype=np.uint8)
    img[25:75, 25:75] = 255  # White square
    return img

@pytest.fixture
def sample_sketch():
    """Create a simple test sketch."""
    from pib3 import Sketch, Stroke, Point
    return Sketch([
        Stroke([Point(0.0, 0.0), Point(1.0, 1.0)]),
    ])
```

## Comments

### When to Comment

```python
# Good: clarify complex logic
# Use damped least squares to avoid singularities near joint limits
J_damped = J.T @ np.linalg.inv(J @ J.T + damping * np.eye(6))

# Bad: restating the code
# Subtract 1.0 from radians
webots_pos = radians - 1.0
```

### TODO Comments

```python
# TODO(username): Implement caching for repeated IK solutions
# TODO: Add support for right arm drawing (issue #42)
```

## File Organization

```python
"""
Module docstring explaining purpose.

This module provides functionality for X.
"""

# Imports (grouped and sorted)
import ...

# Constants
CONSTANT = value

# Type aliases
MyType = ...

# Classes
class MyClass:
    ...

# Functions
def my_function():
    ...

# Private helpers
def _helper():
    ...

# Module-level code (if any)
if __name__ == "__main__":
    ...
```
