# Working with Sketches

Manipulate sketches programmatically for advanced drawing control.

## Objectives

By the end of this tutorial, you will:

- Understand the Sketch and Stroke data structures
- Create sketches programmatically
- Modify and combine sketches
- Optimize stroke order
- Save and load sketches

## Prerequisites

- pib3 installed: `pip install git+https://github.com/mamrehn/pib3.git`
- Basic understanding of 2D coordinates

---

## Understanding Sketches

A **Sketch** is a collection of **Strokes**. Each Stroke is a continuous line (pen-down motion) represented as a series of 2D points.

```
Sketch
├── Stroke 1: [(0.1, 0.1), (0.2, 0.1), (0.2, 0.2)]  # L-shape
├── Stroke 2: [(0.5, 0.5), (0.6, 0.5)]               # Short line
└── Stroke 3: [(0.3, 0.3), (0.3, 0.4), (0.4, 0.4), (0.3, 0.3)]  # Triangle (closed)
```

Coordinates are **normalized** to the range [0, 1]:

- `(0, 0)` = top-left corner
- `(1, 1)` = bottom-right corner

---

## Creating Strokes

### From Point Arrays

```python
import numpy as np
from pib3 import Stroke

# Create a stroke from a list of points
points = [
    [0.1, 0.1],
    [0.5, 0.1],
    [0.5, 0.5],
    [0.1, 0.5],
]
stroke = Stroke(points=points, closed=True)  # Closed loop

print(f"Points: {len(stroke)}")
print(f"Length: {stroke.length():.3f}")
print(f"Closed: {stroke.closed}")
print(f"Start: {stroke.start()}")
print(f"End: {stroke.end()}")
```

### Creating Common Shapes

```python
import numpy as np
from pib3 import Stroke

def create_rectangle(x, y, width, height):
    """Create a rectangle stroke."""
    return Stroke(
        points=[
            [x, y],
            [x + width, y],
            [x + width, y + height],
            [x, y + height],
        ],
        closed=True,
    )

def create_circle(cx, cy, radius, num_points=32):
    """Create a circle stroke."""
    angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    points = [[cx + radius * np.cos(a), cy + radius * np.sin(a)] for a in angles]
    return Stroke(points=points, closed=True)

def create_line(x1, y1, x2, y2):
    """Create a line stroke."""
    return Stroke(points=[[x1, y1], [x2, y2]], closed=False)

# Use them
rect = create_rectangle(0.1, 0.1, 0.3, 0.2)
circle = create_circle(0.5, 0.5, 0.2)
line = create_line(0.1, 0.9, 0.9, 0.9)
```

---

## Creating Sketches

### From Strokes

```python
from pib3 import Sketch, Stroke

# Create individual strokes
stroke1 = Stroke(points=[[0.1, 0.1], [0.9, 0.1]])
stroke2 = Stroke(points=[[0.1, 0.5], [0.9, 0.5]])
stroke3 = Stroke(points=[[0.1, 0.9], [0.9, 0.9]])

# Combine into a sketch
sketch = Sketch(strokes=[stroke1, stroke2, stroke3])

print(f"Strokes: {len(sketch)}")
print(f"Total points: {sketch.total_points()}")
print(f"Total length: {sketch.total_length():.3f}")
```

### Adding Strokes Dynamically

```python
from pib3 import Sketch, Stroke

sketch = Sketch()

# Add strokes one by one
sketch.add_stroke(Stroke(points=[[0.2, 0.2], [0.8, 0.2]]))
sketch.add_stroke(Stroke(points=[[0.2, 0.8], [0.8, 0.8]]))

print(f"Now has {len(sketch)} strokes")
```

---

## Inspecting Sketches

### Basic Properties

```python
from pib3 import image_to_sketch

sketch = image_to_sketch("drawing.png")

# Overall statistics
print(f"Number of strokes: {len(sketch)}")
print(f"Total points: {sketch.total_points()}")
print(f"Total path length: {sketch.total_length():.3f}")

# Bounding box (min_u, min_v, max_u, max_v)
# Note: raises ValueError if the sketch has no strokes/points.
if len(sketch) > 0:
    bounds = sketch.bounds()
    print(f"Bounds: {bounds}")

# Source image size (if from image)
if sketch.source_size:
    print(f"Source image: {sketch.source_size[0]}x{sketch.source_size[1]}")
```

### Iterating Over Strokes

```python
# Iterate directly
for stroke in sketch:
    print(f"Stroke: {len(stroke)} points, length={stroke.length():.3f}")

# Access by index
first_stroke = sketch[0]
last_stroke = sketch[-1]

# Get all strokes as list
all_strokes = list(sketch)
```

---

## Modifying Sketches

### Reversing Stroke Direction

```python
from pib3 import Stroke

stroke = Stroke(points=[[0, 0], [1, 0], [1, 1]])
reversed_stroke = stroke.reverse()

print(f"Original end: {stroke.end()}")
print(f"Reversed end: {reversed_stroke.end()}")
```

### Filtering Strokes

```python
from pib3 import Sketch, image_to_sketch

sketch = image_to_sketch("drawing.png")

# Filter out short strokes
min_length = 0.05  # Minimum 5% of drawing area

filtered_strokes = [s for s in sketch if s.length() >= min_length]
filtered_sketch = Sketch(strokes=filtered_strokes, source_size=sketch.source_size)

print(f"Original: {len(sketch)} strokes")
print(f"Filtered: {len(filtered_sketch)} strokes")
```

### Scaling and Transforming

```python
import numpy as np
from pib3 import Stroke, Sketch

def scale_sketch(sketch, scale_x, scale_y):
    """Scale all strokes in a sketch."""
    new_strokes = []
    for stroke in sketch:
        scaled_points = stroke.points.copy()
        scaled_points[:, 0] *= scale_x
        scaled_points[:, 1] *= scale_y
        new_strokes.append(Stroke(points=scaled_points, closed=stroke.closed))
    return Sketch(strokes=new_strokes, source_size=sketch.source_size)

def translate_sketch(sketch, dx, dy):
    """Move all strokes in a sketch."""
    new_strokes = []
    for stroke in sketch:
        translated = stroke.points + np.array([dx, dy])
        new_strokes.append(Stroke(points=translated, closed=stroke.closed))
    return Sketch(strokes=new_strokes, source_size=sketch.source_size)

# Example: Center and scale
sketch = image_to_sketch("drawing.png")
centered = translate_sketch(sketch, 0.1, 0.1)
scaled = scale_sketch(centered, 0.8, 0.8)
```

---

## Combining Sketches

```python
from pib3 import Sketch, image_to_sketch

# Load multiple images
sketch1 = image_to_sketch("part1.png")
sketch2 = image_to_sketch("part2.png")

# Combine all strokes
combined_strokes = list(sketch1) + list(sketch2)
combined = Sketch(strokes=combined_strokes)

print(f"Combined: {len(combined)} strokes")
```

---

## Optimizing Stroke Order

Reorder strokes to minimize pen travel between them:

```python
from pib3 import image_to_sketch, ImageConfig

# Enable path optimization during image processing
config = ImageConfig(
    optimize_path_order=True  # Uses nearest-neighbor algorithm
)
sketch = image_to_sketch("drawing.png", config)
```

### Manual Reordering

```python
from pib3 import Sketch

def optimize_order_greedy(sketch):
    """Reorder strokes using greedy nearest-neighbor."""
    import numpy as np

    if len(sketch) <= 1:
        return sketch

    remaining = list(sketch.strokes)
    ordered = [remaining.pop(0)]

    while remaining:
        current_end = ordered[-1].end()

        # Find nearest stroke
        min_dist = float('inf')
        min_idx = 0
        reverse = False

        for i, stroke in enumerate(remaining):
            # Distance to start
            d_start = np.linalg.norm(stroke.start() - current_end)
            # Distance to end (would reverse)
            d_end = np.linalg.norm(stroke.end() - current_end)

            if d_start < min_dist:
                min_dist = d_start
                min_idx = i
                reverse = False
            if d_end < min_dist:
                min_dist = d_end
                min_idx = i
                reverse = True

        next_stroke = remaining.pop(min_idx)
        if reverse:
            next_stroke = next_stroke.reverse()
        ordered.append(next_stroke)

    return Sketch(strokes=ordered, source_size=sketch.source_size)

# Use it
optimized = optimize_order_greedy(sketch)
```

---

## Saving and Loading Sketches

### To/From Dictionary (JSON-serializable)

```python
import json
from pib3 import Sketch, image_to_sketch

# Convert to dictionary
sketch = image_to_sketch("drawing.png")
data = sketch.to_dict()

# Save to JSON
with open("my_sketch.json", "w") as f:
    json.dump(data, f, indent=2)

# Load from JSON
with open("my_sketch.json") as f:
    loaded_data = json.load(f)
sketch = Sketch.from_dict(loaded_data)
```

### Dictionary Format

```json
{
  "strokes": [
    {
      "points": [[0.1, 0.1], [0.5, 0.1], [0.5, 0.5]],
      "closed": false
    },
    {
      "points": [[0.3, 0.3], [0.7, 0.3], [0.7, 0.7], [0.3, 0.7]],
      "closed": true
    }
  ],
  "source_size": [800, 600]
}
```

---

## Complete Example: Custom Drawing Generator

```python
"""
Generate a custom sketch programmatically and convert to trajectory.
"""
import numpy as np
import pib3
from pib3 import Sketch, Stroke

def create_star(cx, cy, outer_r, inner_r, points=5):
    """Create a star shape."""
    angles = np.linspace(0, 2 * np.pi, points * 2, endpoint=False)
    radii = [outer_r if i % 2 == 0 else inner_r for i in range(points * 2)]
    coords = [
        [cx + r * np.sin(a), cy + r * np.cos(a)]
        for a, r in zip(angles, radii)
    ]
    return Stroke(points=coords, closed=True)

def create_spiral(cx, cy, start_r, end_r, turns=3, points_per_turn=20):
    """Create a spiral shape."""
    total_points = int(turns * points_per_turn)
    angles = np.linspace(0, turns * 2 * np.pi, total_points)
    radii = np.linspace(start_r, end_r, total_points)
    coords = [
        [cx + r * np.cos(a), cy + r * np.sin(a)]
        for a, r in zip(angles, radii)
    ]
    return Stroke(points=coords, closed=False)

# Create a custom sketch
sketch = Sketch()

# Add a star in the center
sketch.add_stroke(create_star(0.5, 0.5, 0.3, 0.15))

# Add spirals in corners
sketch.add_stroke(create_spiral(0.15, 0.15, 0.02, 0.1, turns=2))
sketch.add_stroke(create_spiral(0.85, 0.15, 0.02, 0.1, turns=2))
sketch.add_stroke(create_spiral(0.15, 0.85, 0.02, 0.1, turns=2))
sketch.add_stroke(create_spiral(0.85, 0.85, 0.02, 0.1, turns=2))

# Add border
border = Stroke(
    points=[[0.05, 0.05], [0.95, 0.05], [0.95, 0.95], [0.05, 0.95]],
    closed=True,
)
sketch.add_stroke(border)

print(f"Created sketch with {len(sketch)} strokes")
print(f"Total points: {sketch.total_points()}")

# Convert to trajectory
trajectory = pib3.sketch_to_trajectory(sketch)
trajectory.to_json("custom_drawing.json")
print(f"Saved trajectory with {len(trajectory)} waypoints")
```

---

## Next Steps

- [Image to Trajectory](image-to-trajectory.md) - Convert images to sketches
- [Custom Configurations](custom-configurations.md) - Fine-tune parameters
- [API Reference: Types](../api/types.md) - Full Stroke and Sketch documentation
