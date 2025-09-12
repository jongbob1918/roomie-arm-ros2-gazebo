# Roomie 4DOF Robot Arm - ROS2 Gazebo Simulation

A complete ROS2 package for simulating and controlling a 4DOF robot arm with ArUco marker detection capabilities.

## 🚀 Quick Start

### Prerequisites
- ROS2 Jazzy (or compatible version)
- Gazebo
- Python 3.8+

### Clone and Build
```bash
# Clone the repository
git clone <repository-url>
cd roomie-arm-ros2-gazebo

# Install Python dependencies
pip install -r requirements.txt

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### Run the Simulation

#### Option 1: URDF Validation Mode (Development/Debugging)
Fast startup for URDF testing and joint validation:
```bash
ros2 launch arm_bringup urdf_validation.launch.py
```

#### Option 2: Simulation Control Mode (Full Physics)
Complete simulation with physics and real robot control interface:
```bash
ros2 launch arm_bringup simulation_control.launch.py

# Or without GUI for headless operation:
ros2 launch arm_bringup simulation_control.launch.py gui:=false
```

## 📦 Package Structure

- **arm_description**: Robot URDF, meshes, and visual configurations
- **arm_bringup**: Launch files and integration scripts
- **arm_gazebo**: Gazebo-specific configurations and worlds
- **roomie_ac**: Robot control and ArUco detection package
- **roomie_msgs**: Custom ROS2 messages

## 🎮 Manual Control

Send joint commands directly:
```bash
# Move to specific joint positions [joint1, joint2, joint3, joint4]
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.5, 0.3, -0.2, 0.1]"

# Return to home position
ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data: [0.0, 0.0, 0.0, 0.0]"

# Monitor joint states
ros2 topic echo /joint_states
```

## 🔧 Development

### Python Dependencies
All Python dependencies are automatically installed during `colcon build` via the `setup.py` files. No separate `pip install` required.

### Key Dependencies
- **pyserial**: ESP32 communication
- **numpy**: Numerical computations
- **ikpy**: Inverse kinematics
- **opencv-python**: Computer vision and ArUco detection
- **scipy**: Scientific computing

### Testing
```bash
# Run tests
colcon test

# Build specific package
colcon build --packages-select <package_name>
```

## 📚 Documentation

- [Usage Guide](USAGE_GUIDE.md) - Detailed usage instructions
- [Gazebo Configuration](GAZEBO_CONFIGURATION_GUIDE.md) - Gazebo setup and ArUco worlds
- [Development Roadmap](DEVELOPMENT_ROADMAP.md) - Future development plans

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

[Add your license information here]

## 🐛 Troubleshooting

### Common Issues

**CMake Cache Issues**: If you encounter CMake cache errors, clean and rebuild:
```bash
rm -rf build/ install/ log/
colcon build
```

**Missing Dependencies**: Dependencies should install automatically. If issues persist:
```bash
# Check Python environment
python3 -c "import numpy, cv2, ikpy"
```

**Gazebo Clock Warnings**: These are normal and can be ignored. Use `gui:=false` for headless operation.

---

For more detailed information, see the individual documentation files in this repository.