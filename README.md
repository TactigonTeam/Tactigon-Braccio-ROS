# Tactigon Braccio ROS Packages

<img src="models/demo/IMG_4531.jpg" width="300">

---

This repository contains two main ROS 2 packages for controlling an Arduino Braccio robot arm, with or without a Tactigon TSkin device:

- **tactigon_ros**: Integrates a Tactigon TSkin wearable with the Braccio arm for gesture-based control.
- **braccio_ros**: Provides a simplified interface to control the Braccio arm directly, without requiring a Tactigon device.

---

## Table of Contents

* [Features](#features)
* [Prerequisites](#prerequisites)
* [Installation](#installation)
* [Usage](#usage)
  * [tactigon_ros: Tactigon + Braccio Integration](#tactigon_ros-usage)
  * [braccio_ros: Standalone Braccio Control](#braccio_ros-usage)
* [Topics & Messages](#topics--messages)
* [Message Definitions](#message-definitions)
* [Node Details](#node-details)
  * [tactigon_ros Nodes](#tactigon_ros-nodes)
  * [braccio_ros Nodes](#braccio_ros-nodes)
* [License](#license)

---

## Features

* **tactigon_ros**: Real-time Tactigon sensor publishing, gesture-driven Braccio control, custom ROS 2 messages.
* **braccio_ros**: Direct control of the Braccio arm from ROS 2 nodes, no Tactigon required.

---

## Prerequisites

* **Operating System**: Ubuntu 24.04
* **ROS 2 Distribution**: Jazzy Jalisco
* **Python packages**:

  ```bash
  pip install tactigon-gear tactigon-arduino-braccio
  ```
  Note: the option --break-system-packages might be needed to install the libraries globally

---

## Installation

```bash
# Clone into your ROS 2 workspace
cd ~/Tactigon-Braccio-ROS
# or your workspace root
colcon build

# Source the workspace
source install/setup.bash
```
Note: Source the workspace in every new terminal or after making a change, otherwise the message definitions and packages cannot be found.

---

## Usage

### <a name="tactigon_ros-usage"></a>tactigon_ros: Tactigon + Braccio Integration

This mode uses both the Tactigon TSkin and the Braccio arm. Use this if you want gesture-based control.

```bash
# Terminal A: Start the Tactigon data publisher
ros2 run tactigon_ros tactigon_data

# Terminal B: Start the Braccio control node (gesture-based)
ros2 run braccio_ros braccio_control
```

Or launch both together:

```bash
ros2 launch tactigon_ros braccio_control.launch.py
```

### <a name="braccio_ros-usage"></a>braccio_ros: Standalone Braccio Control

This mode allows you to control the Braccio arm directly from ROS 2 nodes, without a Tactigon device. You can write your own publisher to send commands to the Braccio.

```bash
# Terminal: Start the Braccio control node (standalone)
ros2 run braccio_ros braccio_control
```

You can publish commands to the appropriate topics as documented below.

---

## Topics & Messages

| Topic                  | Message Type      | Description                         |
| ---------------------- | ----------------- | ----------------------------------- |
| `/tactigon_state`      | `TSkinState`      | Full Tactigon device state (tactigon_ros) |
| `/braccio_move_result` | `BraccioResponse` | Result of each Braccio move command  |
| `/braccio_command`     | `BraccioCommand`  | Command to set Braccio arm pose (braccio_ros) |

---

## Message Definitions

### TSkinState.msg (tactigon_ros)

```ros
bool     connected
float32  battery        # percentage (0–100)
uint8    selector
bool     selector_valid
Touch    touchpad
bool     touchpad_valid
Angle    angle
bool     angle_valid
Gesture  gesture
bool     gesture_valid

# Selector enum
uint8 BLE_SELECTOR_NONE=0
uint8 BLE_SELECTOR_SENSORS=1
uint8 BLE_SELECTOR_AUDIO=2
```



### BraccioCommand.msg (braccio_ros, tactigon_msgs)

```ros
int16 x           # Target X coordinate
int16 y           # Target Y coordinate
int16 z           # Target Z coordinate
string wrist_state    # "HORIZONTAL" or "VERTICAL"
string gripper_state  # "OPEN" or "CLOSE"
```
### BraccioResponse.msg (both packages)

```ros
bool     success
string   status
float32  move_time      # seconds
```

- **x, y, z**: Target Cartesian coordinates for the Braccio arm.
- **wrist_state**: Set to "HORIZONTAL" or "VERTICAL" to control wrist orientation.
- **gripper_state**: Set to "OPEN" or "CLOSE" to control the gripper.

This message is published to `/braccio_command` (by e.g. `braccio_ui_publisher`) and received by `braccio_control` for direct pose control.

---

## Node Details

### <a name="tactigon_ros-nodes"></a>tactigon_ros Nodes

#### tactigon_data

* **Executable**: `tactigon_data`
* **Publishes**: `/tactigon_state` (`TSkinState`)
* **Functionality**:
  1. Connects via Bluetooth to a Tactigon device using `tactigon_gear.TSkin`.
  2. At 50 Hz, reads battery, selector, touchpad, IMU orientation, and gestures.
  3. Publishes a `TSkinState` message.

#### braccio_control (integration mode)

* **Executable**: `braccio_control`
* **Subscribes to**: `/tactigon_state` (`TSkinState`)
* **Publishes**: `/braccio_move_result` (`BraccioResponse`)
* **Functionality**:
  1. Connects to a Braccio arm via `tactigon_arduino_braccio.Braccio`.
  2. Interprets gestures and touchpad data to control the arm.
  3. Publishes move results.

### <a name="braccio_ros-nodes"></a>braccio_ros Nodes

#### braccio_control 

* **Executable**: `braccio_control`
* **Publishes**: `/braccio_move_result` (`BraccioResponse`)
* **Subscribes to**: `/braccio_command` (custom command message)
* **Functionality**:
  1. Connects to a Braccio arm via `tactigon_arduino_braccio.Braccio`.
  2. Accepts direct commands (e.g., from your own publisher node, CLI, or the `braccio_ui_publisher` node).
  3. Publishes move results.

#### braccio_ui_publisher

* **Executable**: `braccio_ui_publisher`
* **Publishes**: `/braccio_command` (custom command message)
* **Purpose**: 
  - Provides a simple interface to manually publish commands to the Braccio arm for testing and pose adjustment.
  - Useful for verifying Braccio movement without the Tactigon device or for manual calibration.
* **Usage Example**:

```bash
ros2 run braccio_ros braccio_ui_publisher
```

This will start the UI publisher node, allowing you to send manual commands to the Braccio arm. Make sure `braccio_control` is running to receive and execute these commands.

---

## License

Made by the TactigonTeam
