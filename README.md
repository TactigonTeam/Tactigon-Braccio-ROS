# Tactigon Braccio ROS Package

<img src="models/demo/IMG_4531.jpg" width="300">

---
This is a ROS 2 Jazzy package running on Ubuntu 24.04 that integrates a Tactigon TSkin with a Braccio robot arm. It provides two nodes:

1. **tactigon\_data\_publisher**: Connects to the Tactigon device, reads sensor data (battery, selector, touchpad, orientation, gestures), and publishes it as a custom `TSkinState` message.
2. **tactigon\_control\_node**: Subscribes to `TSkinState`, interprets gestures and touch inputs to drive a Braccio arm, and publishes `BraccioResponse` messages with move results.

---

## Table of Contents

* [Features](#features)
* [Prerequisites](#prerequisites)
* [Installation](#installation)
* [Usage](#usage)

  * [Launching Nodes Directly](#launching-nodes-directly)
  * [Launch File](#alternative-launch-file)
* [Topics & Messages](#topics--messages)
* [Message Definitions](#message-definitions)
* [Node Details](#node-details)

  * [tactigon\_data\_publisher](#tactigon_data_publisher)
  * [tactigon\_control\_node](#tactigon_control_node)
* [License](#license)

---

## Features

* **Real-time Tactigon sensor publishing**: Battery level, selector wheel, touchpad, IMU orientation, and recognized gestures.
* **Gesture-driven robot control**: Use up/down, twist, circle, swipe, and tap gestures to command a Braccio arm.
* **Custom ROS 2 messages** for clear data structures and seamless integration.

---

## Prerequisites

* **Operating System**: Ubuntu 24.04
* **ROS 2 Distribution**: Jazzy Jalisco
* **Python packages**:

  ```bash
  pip install tactigon-gear tactigon-arduino-braccio
  ```
  Note: the option --break-system-packages might be needed to install the libraries globally


## Installation

```bash
# Clone into your ROS 2 workspace
clone this repository

# Build the workspace
#Navigate to the workspace. Example:
cd ~/Tactigon-Braccio-Ros
colcon build 

# Source the workspace
source install/setup.bash
```
Note: make sure to source the workspace in every new terminal or after making a change, otherwise the message definitions and packages cannot be found.


---

## Usage

### Launching Nodes Directly

```bash
# Terminal A: Tactigon data publisher
ros2 run tactigon_ros tactigon_data

# Terminal B: Tactigon control node
ros2 run braccio_ros braccio_control
```

### Alternative: Launch File


Launch both together:

```bash
ros2 launch tactigon_ros braccio_control.launch.py
```

---

## Topics & Messages

| Topic                  | Message Type      | Description                         |
| ---------------------- | ----------------- | ----------------------------------- |
| `/tactigon_state`      | `TSkinState`      | Full Tactigon device state          |
| `/braccio_move_result` | `BraccioResponse` | Result of each Braccio move command |

---

## Message Definitions

### TSkinState.msg

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

### BraccioResponse.msg

```ros
bool     success
string   status
float32  move_time      # seconds
```

---

## Node Details

### tactigon\_data

* **Executable**: `tactigon_data`
* **Publishes**: `/tactigon_state` (`TSkinState`)
* **Functionality**:

  1. Connects via Bluetooth to a Tactigon device using `tactigon_gear.TSkin`.
  2. At 50 Hz, reads:

     * **Battery** voltage → percentage
     * **Selector** Selector value
     * **Touchpad** gestures & X/Y positions
     * **IMU** orientation (roll, pitch, yaw)
     * **Gesture recognition** via `GestureConfig`
  3. Populates and publishes a `TSkinState` message.

### braccio\_control

* **Executable**: `braccio_control`
* **Subscribes to**: `/tactigon_state` (`TSkinState`)
* **Publishes**: `/braccio_move_result` (`BraccioResponse`)
* **Functionality**:

  1. Connects to a Braccio arm via `tactigon_arduino_braccio.Braccio`.
  2. Maintains current pose (`x, y, z, wrist, gripper`).
  3. On each incoming `TSkinState`:

     * **Circle gesture**: Shuts down the node.
     * **Up/Down gestures**: Set Z-axis height (0 mm or 150 mm).
     * **Twist gesture**: Toggle wrist orientation (horizontal/vertical).
     * **Single tap**: Toggle gripper open/close.
     * **Live tracking: Swipe Left to toggle**: Continuously map IMU angles to X/Y.
     * **Live tracking alternative: Tap and Hold**: Continuously map IMU angles to X/Y.
  4. When any value changes, calls `Braccio.move(x, y, z, wrist, gripper)` and publishes a `BraccioResponse`.

---

## License

Made by the TactigonTeam
