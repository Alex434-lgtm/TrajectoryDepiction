```markdown
# Rocket Flight Path Imaging System with Live Telemetry

A real-time 3D visualization tool for simulating and displaying rocket flight trajectories with advanced physics modeling, interactive camera controls, and live telemetry support via CCCP (Common Communication Protocol).

## Overview

This flight imaging system provides a comprehensive visualization environment for rocket dynamics, supporting both physics-based simulation and real-time telemetry data. It features dual-viewport rendering, real-time trajectory tracking, interactive camera controls, and live data streaming from rocket hardware via CCCP protocol.

## Features

### Core Functionality
- **Dual Operation Modes**:
  - **Simulation Mode**: Physics-based simulation using Runge-Kutta 4 (RK4) integration
  - **Live Telemetry Mode**: Real-time data streaming via CCCP protocol
- **Real-time Data Visualization**: Live accelerometer and attitude data processing
- **Dual Viewport System**: Two synchronized 3D views for comprehensive trajectory analysis
- **Interactive Camera Controls**: Free-roam and follow modes with smooth transitions
- **Trajectory Visualization**: Color-coded flight path markers showing thrust and coast phases
- **Orientation Tracking**: Quaternion-based rocket orientation display with visual triads
- **Time Control**: Real-time playback with elapsed time display
- **Trajectory Scrubbing**: Frame-by-frame navigation through recorded flight path
- **Connection Status Monitoring**: Real-time CCCP connection status display

### Live Telemetry Features
- **Real-time Accelerometer Data**: 
  - 3-axis acceleration monitoring (X, Y, Z)
  - Live velocity and position integration
  - Visual acceleration indicators
- **Real-time Attitude Data**:
  - Quaternion-based orientation (w, x, y, z)
  - Euler angles display (Roll, Pitch, Yaw)
  - Live 3D orientation visualization
- **Telemetry Display**:
  - Current acceleration values in m/s²
  - Attitude in degrees
  - Quaternion components
  - Connection status indicator
- **Seamless Mode Switching**: Toggle between simulation and live telemetry

### Physics Simulation (Simulation Mode)
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
- **Live telemetry status indicators**

## System Requirements

### Dependencies
- Qt6 (Core, Widgets, OpenGL, Network)
- OpenGL
- Eigen 3.4.0+ (Linear algebra library)
- CCCP Library (Common Communication Protocol)
- CMake 3.16+
- C++17 compatible compiler

### Network Requirements
- UDP port access for CCCP communication
- Network connectivity to rocket telemetry system

### Platform Support
- Windows
- Linux
- macOS

## Building the Project

```bash
# Clone the repository with submodules (including CCCP)
git clone --recursive [repository-url]
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
Ensure Eigen and CCCP are properly configured in CMakeLists.txt:
```cmake
include_directories("C:/Toolbox/eigen-3.4.0")  # Adjust path as needed
# CCCP paths should be configured similarly
```

## Controls

### Camera Movement
- **W/A/S/D**: Move camera forward/left/backward/right
- **I/K**: Pitch camera up/down
- **J/L**: Yaw camera left/right
- **M/N**: Adjust movement speed (faster/slower)
- **Right Mouse + Drag**: Free look

### Operation Mode Controls
- **Live Telemetry Checkbox**: Toggle between simulation and live telemetry mode
  - Checked: Live telemetry mode (connects to CCCP)
  - Unchecked: Simulation mode (physics-based)

### Trajectory Playback Controls
- **Reverse Playback Checkbox**: Enable/disable reverse playback mode
- **Left Arrow**: Scrub backward through trajectory (20 frames/press)
- **Right Arrow**: Scrub forward through trajectory (20 frames/press)

### View Controls
- **Button 2/3**: Switch between primary and secondary viewports
- **ComboBox**: Switch between Free Camera and Follow Rocket modes

## Architecture

### Core Components

1. **MainWindow** (`mainwindow.cpp/h`)
   - Manages the application UI and user interactions
   - Coordinates dual OpenGL widgets
   - Handles camera controls and view switching
   - Manages CCCP integration and telemetry processing

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

4. **CCCPManager**
   - Manages CCCP communication protocol
   - Handles network connectivity
   - Processes incoming telemetry data
   - Emits signals for data updates

5. **Data Flow**
   ```
   Simulation Mode:
   Simulation → Quaternion Orientations → OpenGL Widgets → Visual Output
                     ↓
               Trajectory Data → Marker Positions

   Live Telemetry Mode:
   CCCP Network → CCCPManager → Data Signals → MainWindow → OpenGL Widgets
         ↓              ↓                           ↓
   Accelerometer    Attitude               Position Integration
       Data          Data                  & Visualization Update
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
├── Simulation.h            # Simulation header
├── CCCPManager.cpp         # CCCP communication manager
├── CCCPManager.h           # CCCP manager header
```

## Technical Details

### Coordinate System
- X-axis: Horizontal (East)
- Y-axis: Horizontal (North)
- Z-axis: Vertical (Up)

### Telemetry Data Structure
- **Accelerometer Data**: 3-axis acceleration in m/s²
- **Attitude Data**: Quaternion (w, x, y, z) representing orientation
- **Update Rate**: 100 Hz (10ms intervals)

### CCCP Protocol Integration
- Uses UDP for low-latency communication
- Automatic reconnection handling
- Error detection and reporting
- Thread-safe data processing

### Rendering Pipeline
1. Initialize OpenGL context with depth testing
2. Set up perspective projection
3. Apply camera transformations
4. Render scene elements:
   - XY-plane grid
   - Rocket model (sphere)
   - Orientation triads
   - Trajectory markers
   - Telemetry status indicators

### Performance Considerations
- Timer-based updates at 100 FPS (10ms intervals)
- Efficient marker storage using std::vector
- Optimized OpenGL immediate mode rendering
- Separate thread for CCCP communication
- Minimal latency telemetry processing

## Network Configuration

### CCCP Connection Settings
Configure CCCP connection parameters in `config.h` or through UI:
```cpp
// Default CCCP settings
const QString DEFAULT_HOST = "192.168.1.100";
const int DEFAULT_PORT = 8080;
const int TIMEOUT_MS = 5000;
```

### Firewall Configuration
Ensure the following ports are open:
- UDP port for CCCP communication (default: 8080)
- Allow incoming connections from rocket telemetry system

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

### Configuring Telemetry Display
Customize telemetry labels in `setupCCCP()`:
```cpp
m_telemetryLabel->setStyleSheet("color: white; font-size: 14px;");
m_connectionStatus->setStyleSheet("color: red; font-size: 16px;");
```

### Changing Visual Elements
- Grid size: Modify `drawXYPlane(5000.0f, 2000)` parameters
- Marker size: Adjust `drawXMesh(2.0f)` parameter
- Colors: Modify `glColor3f()` calls

## Troubleshooting

### Connection Issues
- **"Failed to initialize CCCP communication"**: Check network settings and firewall
- **"Disconnected" status**: Verify rocket telemetry system is transmitting
- **No data received**: Ensure correct IP address and port configuration

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

This software uses Qt Community Edition, which is licensed under the GNU General Public License v3.0. 
As a result, this project must also be distributed under GPL v3.0 or later.

## Third-Party Licenses

This software uses:
- **Qt Community Edition**
  - License: GNU GPL v3.0 / GNU LGPL v3.0
  - Website: https://www.qt.io/
  - License details: https://www.qt.io/licensing/
- **Eigen**
  - License: MPL2
  - Website: https://eigen.tuxfamily.org/
## Acknowledgments
Built with:
- Qt Community Edition (https://www.qt.io/) - GUI Framework [GPL v3/LGPL v3]
- Eigen (https://eigen.tuxfamily.org/) - Linear Algebra Library [MPL2]
- OpenGL - 3D Graphics API
- CCCP Protocol - (https://github.com/CTU-Space-Research/CCCP)
## Support

For issues, questions, or suggestions, please [create an issue](link-to-issues) in the repository.
