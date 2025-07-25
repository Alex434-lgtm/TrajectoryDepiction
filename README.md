```markdown
# Rocket Flight Path Imaging System

A real-time 3D visualization tool for simulating and displaying rocket flight trajectories with advanced physics modeling and interactive camera controls.

## Overview

This flight imaging system provides a comprehensive visualization environment for rocket dynamics simulation. It features dual-viewport rendering, real-time trajectory tracking, and interactive camera controls to analyze rocket flight paths from multiple perspectives.

## Features

### Core Functionality
- **Real-time Rocket Dynamics Simulation**: Physics-based simulation using Runge-Kutta 4 (RK4) integration
- **Dual Viewport System**: Two synchronized 3D views for comprehensive trajectory analysis
- **Interactive Camera Controls**: Free-roam and follow modes with smooth transitions
- **Trajectory Visualization**: Color-coded flight path markers showing thrust and coast phases
- **Orientation Tracking**: Quaternion-based rocket orientation display with visual triads
- **Time Control**: Real-time playback with elapsed time display
- **Trajectory Scrubbing**: Frame-by-frame navigation through recorded flight path
  - Reverse playback mode with arrow key controls
  - 20-frame stepping for quick navigation

### Physics Simulation
- **Launch Parameters**:
  - Initial mass: 50 kg
  - Thrust force: 7,000 N
  - Burn time: 5 seconds
  - Initial flight path angle: 80°
- **Environmental Factors**:
  - Gravitational acceleration: 9.8066 m/s²
  - Air density: 1.225 kg/m³ (sea level)
  - Drag coefficient: 0.237
  - Cross-sectional area: 0.0314 m²

### Visualization Features
- **XY-plane grid** for spatial reference
- **Oriented coordinate triads** showing rocket orientation
- **Color-coded trajectory markers**:
  - Blue: Powered flight (thrust phase)
  - Green: Coast phase
- **Real-time position and orientation updates**

## System Requirements

### Dependencies
- Qt6 (Core, Widgets, OpenGL)
- OpenGL
- Eigen 3.4.0+ (Linear algebra library)
- CMake 3.16+
- C++17 compatible compiler

### Platform Support
- Windows
- Linux
- macOS

## Building the Project

```bash
# Clone the repository
git clone [repository-url]
cd rocket-flight-imaging

# Create build directory
mkdir build
cd build

# Configure with CMake
cmake ..

# Build
make -j4

# Run
./RocketDynamics
```

### Windows-specific Build
Ensure Eigen is properly configured in CMakeLists.txt:
```cmake
include_directories("C:/Toolbox/eigen-3.4.0")  # Adjust path as needed
```

## Controls

### Camera Movement
- **W/A/S/D**: Move camera forward/left/backward/right
- **I/K**: Pitch camera up/down
- **J/L**: Yaw camera left/right
- **M/N**: Adjust movement speed (faster/slower)
- **Right Mouse + Drag**: Free look

### Playback Controls
- **Checkbox**: Enable/disable reverse playback
- **Arrow Keys**: Step through trajectory (when in reverse mode)
- **ComboBox**: Switch between Free Camera and Follow Rocket modes

### View Controls
- **Button 2/3**: Switch between primary and secondary viewports

## Architecture

### Core Components

1. **MainWindow** (`mainwindow.cpp/h`)
   - Manages the application UI and user interactions
   - Coordinates dual OpenGL widgets
   - Handles camera controls and view switching

2. **RocketOpenGLWidget**
   - Custom OpenGL rendering widget
   - Manages 3D scene rendering
   - Handles camera transformations
   - Draws rocket, trajectory, and grid

3. **Simulation** (`Simulation.cpp/h`)
   - Implements rocket dynamics calculations
   - Uses RK4 integration for accurate physics
   - Calculates thrust, drag, and gravitational forces
   - Returns flight path angle, velocity, and acceleration data

4. **Data Flow**
   ```
   Simulation → Quaternion Orientations → OpenGL Widgets → Visual Output
                     ↓
               Trajectory Data → Marker Positions
   ```

## File Structure

```
rocket-flight-imaging/
├── CMakeLists.txt          # Build configuration
├── main.cpp                # Application entry point
├── mainwindow.cpp          # Main window implementation
├── mainwindow.h            # Main window header
├── mainwindow.ui           # Qt Designer UI file
├── Simulation.cpp          # Physics simulation
└── Simulation.h            # Simulation header
```

## Technical Details

### Coordinate System
- X-axis: Horizontal (East)
- Y-axis: Horizontal (North)
- Z-axis: Vertical (Up)

### Rendering Pipeline
1. Initialize OpenGL context with depth testing
2. Set up perspective projection
3. Apply camera transformations
4. Render scene elements:
   - XY-plane grid
   - Rocket model (sphere)
   - Orientation triads
   - Trajectory markers

### Performance Considerations
- Timer-based updates at 100 FPS (10ms intervals)
- Efficient marker storage using std::vector
- Optimized OpenGL immediate mode rendering

## Customization

### Modifying Simulation Parameters
Edit constants in `Simulation.cpp`:
```cpp
const double m = 50.0;        // Mass (kg)
const double T = 7000.0;      // Thrust (N)
const double burn_time = 5.0; // Burn duration (s)
```

### Adjusting Camera Settings
Modify initial camera positions in `mainwindow.cpp`:
```cpp
cameraX = 5000.0f;
cameraY = 5000.0f;
cameraZ = 0.0f;
```

### Changing Visual Elements
- Grid size: Modify `drawXYPlane(5000.0f, 2000)` parameters
- Marker size: Adjust `drawXMesh(2.0f)` parameter
- Colors: Modify `glColor3f()` calls
## License
This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.
This software uses Qt Community Edition, which is licensed under the GNU General Public License v3.0. 
As a result, this project must also be distributed under GPL v3.0 or later.
## Third-Party Licenses
This software uses Qt Community Edition
- License: GNU GPL v3.0 / GNU LGPL v3.0
- Website: https://www.qt.io/
- License details: https://www.qt.io/licensing/
## Acknowledgments
Built with:
- Qt Community Edition (https://www.qt.io/) - GUI Framework [GPL v3/LGPL v3]
- Eigen (https://eigen.tuxfamily.org/) - Linear Algebra Library [MPL2]
- OpenGL - 3D Graphics API
## Support

For issues, questions, or suggestions, please [create an issue](link-to-issues) in the repository.
```

This README provides a comprehensive overview of the flight imaging system, including its features, build instructions, usage guidelines, and technical details. It's structured to help both users and developers understand and work with the system effectively.
