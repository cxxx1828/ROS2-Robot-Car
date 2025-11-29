# RoboRacer - ROS2 Robotic Car Control System

A ROS2-based control system for robotic cars with joystick input, custom PCB hardware, and real-time motor control.

## System Overview

The system uses three main components:
- **Joystick Node** running on the host PC reads controller input
- **Motor Control Node** translates commands and handles serial communication
- **Arduino Nano** drives the motors directly

Commands flow from the joystick through ROS2 topics, get converted to UART packets, and are executed by the Arduino firmware.

## Architecture

```
Joystick → ROS2 Topics → Motor Control → UART → Arduino → Motors
```

The joystick node publishes to `/cmd_vel` and `/robot_control` topics. The motor control node subscribes to these, formats the data into UART packets, and sends them to the Arduino over serial. The Arduino parses incoming packets and generates PWM signals for the BLDC motor and servo.

## Components

### Joystick Node
Reads USB/Bluetooth joystick input and maps axes to speed/steering commands. Has dead-zone filtering and configurable button mapping.

### Motor Control Node
Handles the bridge between ROS2 and the Arduino. Subscribes to command topics, validates input, and manages serial communication at 115200 baud.

Data packets look like:
```
[START] [SPEED] [DIRECTION] [STEERING] [FLAGS] [CHECKSUM] [END]
```

### Arduino Firmware
Receives UART commands and controls the hardware. Implements a watchdog timer that stops the motors if communication is lost for more than 500ms.

## Hardware

Custom 2-layer PCB designed in KiCad with:
- Arduino Nano socket
- Power regulation (5V/3.3V)
- Screw terminals for motors
- Status LEDs
- Reverse polarity protection

The board connects a BLDC motor for propulsion and a standard servo for steering.

## Getting Started

Flash the Arduino:
```bash
cd firmware/
arduino-cli compile --fqbn arduino:avr:nano main.ino
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano main.ino
```

Build the ROS2 workspace:
```bash
cd ros2_ws/
colcon build
source install/setup.bash
```

Run the system:
```bash
# Terminal 1
ros2 run joystick_node joystick_reader

# Terminal 2
ros2 run motor_control_node motor_controller --port /dev/ttyUSB0
```

Controls:
- Left stick: speed
- Right stick: steering
- Button A: emergency stop

## Project Structure

```
roboracer/
├── ros2_ws/
│   ├── joystick_node/
│   │   ├── joystick_reader.py
│   │   └── config/mapping.yaml
│   └── motor_control_node/
│       ├── motor_controller.py
│       └── protocol.py
├── firmware/
│   ├── main.ino
│   ├── motor_driver.cpp
│   └── servo_control.cpp
├── hardware/
│   ├── pcb/
│   │   ├── roboracer.kicad_pro
│   │   ├── roboracer.kicad_sch
│   │   └── roboracer.kicad_pcb
│   └── bom.csv
└── README.md
```

## Safety Features

The system has several safety mechanisms:
- Watchdog timer in firmware stops motors if no commands received for 500ms
- Command validation in the motor control node
- Emergency stop button
- Rate limiting at 50Hz to prevent command flooding

## Adding New Features

The ROS2 architecture makes it easy to add new capabilities. Want to add a camera? Just run a camera node that publishes to `/camera/image`. Want autonomous navigation? Run a nav2 node that publishes to `/cmd_vel`. The existing nodes don't need any changes.

Examples:
```bash
ros2 run camera_node stream_publisher
ros2 run nav2_node autonomous_driver
ros2 run rviz2 rviz2 -d config/roboracer.rviz
```

## Specs

- Microcontroller: Arduino Nano (ATmega328P)
- Communication: UART @ 115200 baud
- Power: 7.4V LiPo (2S)
- ROS2: Humble Hawksbill
- Control loop: 50Hz
- Latency: <50ms from joystick to motors
- Battery life: ~30 minutes

## Author

Nina Dragićević

## License

MIT License - see LICENSE file for details.
