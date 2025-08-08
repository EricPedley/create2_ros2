# Create 2 ROS2 Control Hardware Interface

This package provides a ROS2 Control hardware interface for the iRobot Create 2 based on the Open Interface specification. It allows you to control the Create 2 using standard ROS2 `geometry_msgs/Twist` messages through the diff_drive_controller.

## Features

- Full integration with ROS2 Control framework
- Differential drive controller for twist command handling  
- Real-time encoder feedback for odometry
- Configurable serial communication (115200 or 19200 baud)
- Safety features from Create 2 OI specification
- Standard ROS2 interfaces (cmd_vel, joint_states, odom)

## Hardware Requirements

- iRobot Create 2 robot
- USB to serial cable (like the official iRobot Create USB cable)
- Computer running ROS2 (tested on Humble/Iron/Rolling)

## Installation

1. Clone this repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <this-repo-url> create2_hardware
```

2. Install dependencies:
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the package:
```bash
colcon build --packages-select create2_hardware
source install/setup.bash
```

## Usage

### Basic Setup

1. Connect your Create 2 to your computer via USB/serial cable
2. Find the device path (usually `/dev/ttyUSB0`)
3. Ensure you have permissions to access the serial device:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

### Launch the System

Launch the Create 2 with default settings:
```bash
ros2 launch create2_ros2 create2_bringup.launch.py
```

Or with custom device path:
```bash
ros2 launch create2_ros2 create2_bringup.launch.py device_path:=/dev/ttyUSB0
```

For 19200 baud rate (if needed):
```bash
ros2 launch create2_hardware create2_bringup.launch.py baud_rate:=19200
```

### Control the Robot

Once launched, you can control the robot using standard twist messages:

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
```

Or use keyboard teleoperation:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Monitor Status

View joint states:
```bash
ros2 topic echo /joint_states
```

View odometry:
```bash
ros2 topic echo /create2_base_controller/odom
```

Check controller status:
```bash
ros2 control list_controllers
```

## Configuration

### Hardware Parameters

The hardware interface accepts these parameters in the URDF:

- `device`: Serial device path (default: `/dev/ttyUSB0`)
- `baud_rate`: Communication baud rate - `115200` or `19200` (default: `115200`)

### Controller Configuration

Edit `config/create2_ros2_control.yaml` to modify:

- Update rate (default: 100 Hz)
- Velocity and acceleration limits
- Wheel separation and radius
- PID parameters (if needed)

### Robot Description

The URDF file `urdf/create2.urdf.xacro` contains:

- Physical robot dimensions based on Create 2 specs
- Joint definitions for differential drive
- ROS2 Control configuration
- Visual and collision models

## Troubleshooting

### Serial Connection Issues

1. Check device permissions:
```bash
ls -l /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0  # Temporary fix
```

2. Verify the Create 2 is in the correct mode:
   - Make sure it's powered on
   - Try different baud rates (115200 vs 19200)
   - Check cable connections

### Robot Not Responding

1. Check if the OI is properly initialized:
```bash
ros2 topic echo /rosout | grep -i create2
```

2. Verify controller is loaded:
```bash
ros2 control list_controllers
```

3. Check for error messages:
```bash
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
```

### Performance Issues

1. Reduce update rate in config file
2. Check CPU usage during operation
3. Ensure USB cable supports required data rates

## Development

### Package Structure

```
create2_hardware/
├── config/                  # Controller configurations
│   └── create2_ros2_control.yaml
├── include/create2_hardware/ # Header files
│   └── create2_hardware_interface.hpp
├── launch/                  # Launch files
│   └── create2_bringup.launch.py
├── src/                     # Source files
│   └── create2_hardware_interface.cpp
├── urdf/                    # Robot descriptions
│   └── create2.urdf.xacro
├── CMakeLists.txt
├── package.xml
└── create2_hardware.xml     # Plugin description
```

### Adding New Features

To extend functionality:

1. Add new sensor readings in the `read()` method
2. Implement additional OI commands in hardware interface
3. Create custom controllers for specific behaviors
4. Add safety features and error handling

### Testing

Test with fake hardware for development:
```bash
ros2 launch create2_hardware create2_bringup.launch.py use_fake_hardware:=true
```

## Technical Details

### Create 2 Open Interface

This implementation uses:
- **Opcode 128**: Start OI
- **Opcode 132**: Full mode (disables safety features)  
- **Opcode 145**: Drive Direct command
- **Opcode 149**: Query sensor packets
- **Packets 43/44**: Left/Right encoder counts

### ROS2 Control Integration

- **Hardware Interface**: `SystemInterface` for complete robot control
- **Controllers**: Uses standard `diff_drive_controller`
- **Command Interface**: Velocity control for wheel joints
- **State Interface**: Position and velocity feedback

### Communication Protocol

- Serial communication at 115200 or 19200 baud
- 8N1 format (8 data bits, no parity, 1 stop bit)
- Binary command protocol as per OI specification
- 15ms sensor update cycle matching Create 2's internal rate

## License

This project is licensed under the Apache License 2.0.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## Support

For issues and questions:
- Check the troubleshooting section above
- Review Create 2 Open Interface documentation
- Open an issue on the repository