# Obstacle Simulation

## Overview

`obstacle_simulation` is a Python package designed to simulate both virtual obstacles and LiDAR readings for robotics applications. It provides tools to model, manage, and analyze obstacle shapes and interactions, with specific support for ship-like objects and LiDAR-based detection systems. The package supports two types of ship geometries: default and custom, allowing flexibility for various use cases.

## Features

- **Ship-Like Obstacles**:  Generate obstacles in the form of polygons to recreate the shape of a ship's hull in 2D.
- **LiDAR Simulation**: Simulate LiDAR sensors to detect obstacles and segments. The ship's perimeter is represented by specialized `ShipSegment` objects, which are used for intersection calculations and tracking detected segments.
- **Collision Detection**:  Identify collisions between an obstacle and an entity defined by a circular footprint.
- **Customizable Transformations**: Rotate, translate, and reposition obstacles dynamically.

## Installation

Clone the repository and install it locally:

```bash
git clone https://github.com/polks93/obstacle_simulation.git
cd obstacle_simulation
pip install .
```

## Usage

### Basic Example

```python
from obstacle_simulation import ShipObstacle

# Create a ship obstacle
ship = ShipObstacle(ship_center=(0, 0))

# Visualize the ship
import matplotlib.pyplot as plt
x, y = zip(*ship.points)
plt.plot(x, y, '-o')
plt.axis('equal')
plt.show()
```

### Simulate LiDAR detection

```python
import numpy as np
from obstacle_simulation import ShipObstacle, lidar

# Create a ship obstacle
ship = ShipObstacle(ship_center=(3, 0))

# Define LiDAR parameters
lidar_params = {
    'max_range': 10,
    'n_beams': 90,
    'FoV': np.pi / 2
}

# Simulate LiDAR from a given pose
pose = np.array([0, 0, np.pi/4])
ranges, angles, seen_segments = lidar(pose, ship, lidar_params)

# Print detected ranges
print(ranges)
```

## Documentation

### Key Classes and Functions

#### Classes
- **`ShipObstacle`**: Represents a ship-like obstacle with customizable geometry and transformations.
  - **Constructor Arguments**:
    - `ship_center: Tuple[float, float]`: The center of the ship in (x, y) coordinates.
    - `Options: Dict = {}`: Additional configuration options for the ship.
    - `inflation_radius: float = 0.5`: The radius of the robot's circular footprint used to control collisions.
    - `use_default_values: bool = True`: Whether to use default ship parameters.
    - `scale: float = 0.9`: Scaling factor for the ship's geometry.
    - `use_custom_ship: bool = False`: Whether to use a custom-defined ship geometry. Set to `False` for a simple geometry or `True` for a more complex geometry.
  - **Methods**:
    - `reset_ship()`: Resets the ship to its original state, including position, orientation, and geometry.
    - `rototranslate(angle: float, vector: tuple)`: Rotates the ship by a specified angle and translates it by a vector.
    - `random_placement(workspace: tuple, safe_distance: float)`: Places the ship randomly within a workspace while maintaining a safe distance.
    - `point_in_ship(point: np.ndarray) -> bool`: Checks if a point is inside the inflated ship geometry.
    - `point_in_custom_ship(point: np.ndarray) -> bool`: Checks if a point is inside the custom ship geometry using the winding number method.

- **`ShipSegment`**: Represents a segment of the ship's perimeter with attributes for start, end, and midpoint coordinates, as well as a unique identifier and visibility status.
  - **Attributes**:
    - `id`: A unique identifier for the segment.
    - `start_point`: The starting coordinate of the segment `(x, y)`.
    - `end_point`: The ending coordinate of the segment `(x, y)`.
    - `mid_point`: The midpoint of the segment `(x, y)`, calculated as the average of the start and end points.
    - `seen`: A boolean indicating whether the segment has been detected by a LiDAR ray, initially set to `False`.
  - **Methods**:
    - `__init__(id: int, start_point: tuple, end_point: tuple)`: Initializes the segment with its attributes.

#### Functions
- **`lidar(pose: np.ndarray, Ship: ShipObstacle, lidar_params: dict) -> Tuple[np.ndarray, np.ndarray, Set[int]]`**: Simulates LiDAR detection for a given pose and obstacle.
  - **Parameters**:
    - `pose`: The sensor's pose `[x, y, theta]`.
    - `Ship`: The `ShipObstacle` to detect.
    - `lidar_params`: Parameters like `max_range`, `n_beams`, and `FoV`.
  - **Returns**:
    - `ranges`: Detected distances for each beam.
    - `angles`: Angles of the beams.
    - `seen_segments`: IDs of the detected obstacle segments.

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository.
2. Create a new branch for your feature or bug fix.
3. Submit a pull request with a clear description of your changes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Support

If you encounter any issues, please open an issue on [GitHub](https://github.com/username/obstacle_simulation/issues).

## Acknowledgments

Special thanks to the open-source community and all contributors for their efforts and support.

