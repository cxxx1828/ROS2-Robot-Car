# RoboRacer - ROS2 Robotic Car Control System



A complete ROS2-based control system for robotic cars with joystick input, custom PCB hardware, and real-time motor control. Features modular architecture for easy extension with autonomous capabilities.

---

## System Architecture

```
┌─────────────┐        ROS2 Topics         ┌──────────────┐
│  Joystick   │ ───────────────────────────>│ Motor Control│
│    Node     │    /cmd_vel, /robot_control │     Node     │
│ (Host PC)   │                             │  (Host PC)   │
└─────────────┘                             └──────┬───────┘
                                                   │
                                            UART Serial
                                                   │
                                            ┌──────▼───────┐
                                            │   Arduino    │
                                            │     Nano     │
                                            │  (Firmware)  │
                                            └──────┬───────┘
                                                   │
                              ┌────────────────────┴────────────────────┐
                              │                                         │
                        ┌─────▼──────┐                          ┌──────▼──────┐
                        │ BLDC Motor │                          │ Servo Motor │
                        │ (Propulsion)│                          │  (Steering) │
                        └────────────┘                          └─────────────┘
```

---

## Components

### 1. Joystick Node (ROS2 - Host PC)

**Responsibility:** Human interface and command generation

- Continuously reads USB/Bluetooth joystick input
- Maps raw axes and buttons to robot commands:
  - Axes → Speed and steering angle
  - Buttons → Emergency stop, mode switching
- Publishes to ROS2 topics: `/cmd_vel`, `/robot_control`
- Runs on host computer with ROS2 environment

**Key Features:**
- Dead-zone filtering for joystick drift
- Configurable axis mapping
- Real-time input processing

---

### 2. Motor Control Node (ROS2 - Host PC)

**Responsibility:** Protocol translation and communication bridge

- Subscribes to command topics from joystick node
- Translates ROS2 messages → UART data packets
- Manages serial communication with Arduino Nano
- Implements safety features:
  - Command rate limiting
  - Timeout detection
  - Input validation

**Data Packet Format:**
```
[START_BYTE] [SPEED] [DIRECTION] [STEERING] [FLAGS] [CHECKSUM] [END_BYTE]
```

---

### 3. Arduino Nano Firmware

**Responsibility:** Low-level hardware control and execution

- Receives UART commands from motor control node
- Parses data packets and validates checksums
- Generates hardware control signals:
  - **BLDC Motor:** Speed and direction control
  - **Servo Motor:** PWM signal for steering angle
- Safety mechanisms:
  - Watchdog timer (auto-stop on communication loss)
  - Emergency stop handling
  - Safe startup sequence

---

## Data Flow Pipeline

```
User Input → Joystick Node → ROS2 Topics → Motor Control Node → UART → Firmware → Motors
```

**Step-by-step execution:**

1. **User moves joystick** → Analog input detected
2. **Joystick node processes** → Publishes ROS2 message
3. **Motor control node receives** → Formats UART packet
4. **Firmware parses command** → Generates control signals
5. **Motors respond** → Robot moves in real-time

**Typical latency:** <50ms from joystick to motor response

---

## Hardware Integration

### Custom PCB Design (KiCad)

**Features:**
- Arduino Nano socket with pin headers
- Power regulation circuitry (5V/3.3V rails)
- Screw terminals for motor connections:
  - BLDC motor power and control
  - Servo motor signal and power
- Status LEDs for diagnostics
- UART header for programming/debugging
- Reverse polarity protection

**PCB Layers:**
- 2-layer design
- Silkscreen labels for all connectors
- Ground plane for noise reduction

---

## Project Structure

```
roboracer/
├── ros2_ws/
│   ├── joystick_node/           # Joystick input handler
│   │   ├── joystick_reader.py
│   │   └── config/mapping.yaml
│   └── motor_control_node/      # UART communication bridge
│       ├── motor_controller.py
│       └── protocol.py
├── firmware/
│   ├── main.ino                 # Arduino Nano firmware
│   ├── motor_driver.cpp
│   └── servo_control.cpp
├── hardware/
│   ├── pcb/                     # KiCad project files
│   │   ├── roboracer.kicad_pro
│   │   ├── roboracer.kicad_sch
│   │   └── roboracer.kicad_pcb
│   └── bom.csv                  # Bill of materials
└── README.md
```

---

## Quick Start

### 1. Hardware Setup
```bash
# Flash Arduino firmware
cd firmware/
arduino-cli compile --fqbn arduino:avr:nano main.ino
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:nano main.ino
```

### 2. ROS2 Workspace Build
```bash
cd ros2_ws/
colcon build
source install/setup.bash
```

### 3. Launch System
```bash
# Terminal 1: Start joystick node
ros2 run joystick_node joystick_reader

# Terminal 2: Start motor control node
ros2 run motor_control_node motor_controller --port /dev/ttyUSB0
```

### 4. Control the Robot
- Connect joystick via USB/Bluetooth
- Left stick: Forward/backward speed
- Right stick: Steering angle
- Button A: Emergency stop

---

## Safety Features

| Feature | Implementation | Purpose |
|---------|----------------|---------|
| Watchdog Timer | Firmware-level timeout | Auto-stop on communication loss |
| Command Validation | Node-level checks | Prevent invalid motor commands |
| Emergency Stop | Button + software flag | Immediate motor cutoff |
| Rate Limiting | 50Hz max command rate | Prevent command flooding |
| Timeout Detection | 500ms no-command threshold | Safe state on connection loss |

---

## Extending the System

The modular architecture allows easy addition of new capabilities:

### Add Camera Stream
```bash
ros2 run camera_node stream_publisher
# Automatically available to all nodes via /camera/image topic
```

### Add Autonomous Navigation
```bash
ros2 run nav2_node autonomous_driver
# Publishes to same /cmd_vel topic, seamless integration
```

### Add Telemetry Visualization
```bash
ros2 run rviz2 rviz2 -d config/roboracer.rviz
# Subscribe to all topics for real-time monitoring
```

**No modifications needed to existing nodes** - ROS2 publish-subscribe pattern enables loose coupling.

---

## Technical Specifications

| Component | Specification |
|-----------|---------------|
| Microcontroller | Arduino Nano (ATmega328P) |
| Communication | UART @ 115200 baud |
| Motor Driver | BLDC ESC compatible |
| Servo | Standard 50Hz PWM servo |
| Power Supply | 7.4V LiPo (2S) |
| ROS2 Version | Humble Hawksbill |
| Control Frequency | 50Hz command loop |

---

## Performance Metrics

- **Latency:** <50ms joystick-to-motor
- **Command Rate:** 50 Hz sustained
- **Range:** Limited by UART cable (wireless upgrade possible)
- **Battery Life:** ~30 minutes continuous operation

---

## Author

**Nina Dragićević**  


---

## License

MIT License - see [LICENSE](LICENSE) file for details.

---

## Acknowledgments

- ROS2 community for excellent documentation
- KiCad developers for PCB design tools
- Arduino platform for accessible embedded development
