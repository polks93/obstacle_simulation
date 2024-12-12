# Obstacle Simulation

## Overview

`obstacle_simulation` is a Python package designed to simulate both virtual obstacles and LiDAR readings for robotics applications. It provides tools to model, manage, and analyze obstacle shapes and interactions, with specific support for ship-like objects and LiDAR-based detection systems. The package supports two types of ship geometries: default and custom, allowing flexibility for various use cases.

## Features

- **Ship-Like Obstacles**: Model ship obstacles with realistic geometry.
- **Inflation Support**: Generate inflated boundaries for collision avoidance.
- **LiDAR Simulation**: Simulate LiDAR sensors to detect obstacles and segments.
- **Collision Detection**: Identify collisions between obstacles and cirular entities
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
  - **Methods**:
    - `rotate_ship(angle: float)`: Rotates the ship by a specified angle.
    - `translate_ship(vector: tuple)`: Translates the ship by a specified vector.
    - `random_placement(workspace: tuple, safe_distance: float)`: Places the ship randomly within a workspace while maintaining a safe distance.
    - `point_in_ship(point: np.ndarray) -> bool`: Checks if a point is inside the inflated ship geometry.
    - `point_in_custom_ship(point: np.ndarray) -> bool`: Checks if a point is inside the custom ship geometry using the winding number method.

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

