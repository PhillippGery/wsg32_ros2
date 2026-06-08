# wsg32_ros2

Minimal ROS 2 driver for the **Weiss WSG 32** Ethernet gripper.

No binary protocol parsing. No legacy ROS 1 ports. No Action Servers.  
Just a clean GCL TCP wrapper and a single ROS 2 node — built to work.

Tested on **ROS 2 Jazzy** | Ubuntu 24.04 | WSG 32 firmware 4.x  
Developed as part of the **TwinNexus** dual-arm VLA data collection platform (UR5e + GELLO teleoperation + π₀ training pipeline).

---

## Why this exists

Every WSG gripper driver on GitHub is either:
- ROS 1 only (`catkin_make`, Action Servers, dead since 2019)
- The binary packet protocol (CRC checksums, 2 days of debugging)
- A bare Python class with no ROS integration

This package wraps the **GCL text-based interface** — Weiss's own ASCII command language over TCP — in a proper ROS 2 Python node. The same approach used by Stanford REAL Lab's [UMI](https://github.com/real-stanford/universal_manipulation_interface) bimanual system.

---

## Installation

### Prerequisites

- ROS 2 Jazzy installed on Ubuntu 24.04
- A ROS 2 workspace (create one if needed):
  ```bash
  mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
  colcon build
  source install/setup.bash
  ```
- No extra Python packages required — only the standard library (`socket`, `threading`) and core ROS 2 Python packages (`rclpy`, `std_msgs`, `sensor_msgs`, `std_srvs`)

### Clone and build

```bash
# Clone into your workspace
cd ~/ros2_ws/src
git clone https://github.com/PhilippGery/wsg32_ros2.git

# Install ROS dependencies (resolves package.xml deps automatically)
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select wsg32_driver

# Source
source install/setup.bash
```

Add the source line to your `.bashrc` so you don't have to repeat it:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### Verify the install

```bash
# Check the node is discoverable
ros2 pkg list | grep wsg32

# Run the standalone hardware test (no ROS needed, gripper must be reachable)
python3 ~/ros2_ws/src/wsg32_ros2/wsg32_driver/test_connection.py
```



- Weiss WSG 32 connected via **M8 4-pin D-coded → RJ45** Ethernet cable
- Gripper powered via **M8 A-coded 24V pigtail**
- Static IP assigned to the gripper (default Weiss firmware: `192.168.1.20`; this repo uses `192.168.1.201` for right arm, `192.168.1.202` for left arm)

**One-time setup (do this before anything else):**

Open `http://192.168.1.201` (right) or `http://192.168.1.202` (left) in a browser.  
Go to **Settings → Command Interface → enable "Use text based interface"**.  
This setting persists across reboots. Without it, text commands are silently ignored.

---

## Network setup (enterprise / university networks)

The gripper runs on an isolated static subnet that never touches the enterprise DHCP router.  
If you're on a network with DHCP snooping (which blocks unauthorized static IPs), use IP aliasing:

```bash
# Add a permanent static alias on the same physical interface
# Primary interface keeps DHCP (internet access)
# Alias gives you the isolated 192.168.1.X subnet for the gripper

sudo nmcli connection modify "Wired connection 1" \
  +ipv4.addresses "192.168.1.100/24"

sudo nmcli connection up "Wired connection 1"

# Verify
ip addr show enp128s31f6
# Should show both your DHCP address AND 192.168.1.100
```

Connect gripper + robot controller to a dumb gigabit switch. The switch is invisible to the enterprise router — traffic stays local.

---

## Package structure

```
wsg32_driver/
├── wsg32_driver/
│   ├── wsg_tcp.py        # Pure Python GCL TCP driver (no ROS dependency)
│   └── wsg32_node.py     # ROS 2 node
├── config/
│   └── wsg32_params.yaml # All tunable parameters
├── launch/
│   ├── wsg32.launch.py       # Single gripper
│   └── wsg32_dual.launch.py  # Dual-arm (two grippers, two namespaces)
├── test_connection.py    # Standalone hardware smoke test (no ROS needed)
├── package.xml
└── setup.py
```

---

## Quickstart

### 1. Test hardware connectivity (no ROS needed)

```bash
cd src/wsg32_driver
python3 test_connection.py
# For a second gripper:
python3 test_connection.py --ip 192.168.1.201
```

This connects, clears any fault, runs homing, and moves through three positions.  
**Fix any failures here before touching ROS.**

### 2. Build

```bash
cd ~/ros2_ws
colcon build --packages-select wsg32_driver
source install/setup.bash
```

### 3. Launch

```bash
# Single gripper
ros2 launch wsg32_driver wsg32.launch.py

# Dual-arm
ros2 launch wsg32_driver wsg32_dual.launch.py

# Custom IP
ros2 launch wsg32_driver wsg32.launch.py gripper_ip:=192.168.1.201 gripper_name:=wsg32_right
```

### 4. Send commands

```bash
# Open to 30 mm
ros2 topic pub /left_arm/wsg32_node/cmd_pos std_msgs/Float32 "data: 30.0" --once

# Close
ros2 topic pub /left_arm/wsg32_node/cmd_pos std_msgs/Float32 "data: 0.0" --once

# Watch state
ros2 topic echo /left_arm/wsg32_node/joint_state
```

---

## ROS 2 Interface

### Subscribed topics

| Topic | Type | Description |
|---|---|---|
| `~/cmd_pos` | `std_msgs/Float32` | Target width in mm. 0 = closed, 55 = open. |

### Published topics

| Topic | Type | Description |
|---|---|---|
| `~/joint_state` | `sensor_msgs/JointState` | Last commanded width (meters) + zero velocity/effort. For rosbag2 recording. |

### Services

| Service | Type | Description |
|---|---|---|
| `~/home` | `std_srvs/Trigger` | Run homing sequence (~5 s, blocks). |
| `~/ack_fault` | `std_srvs/Trigger` | Clear a latched fast-stop or fault. |

---

## Parameters

All parameters live in `config/wsg32_params.yaml` and can be overridden at launch.

| Parameter | Default | Description |
|---|---|---|
| `gripper_ip` | `"192.168.1.201"` | Gripper IP on the local subnet. |
| `gripper_port` | `1000` | GCL TCP port (fixed by Weiss firmware). |
| `gripper_name` | `"wsg32_left"` | Joint name in published JointState. |
| `max_pos_mm` | `55.0` | Physical stroke limit. **Verify in web UI: Settings → System Info.** |
| `cmd_deadband_mm` | `0.2` | Suppress moves smaller than this to reduce TCP spam from encoder jitter. |
| `publish_hz` | `50.0` | JointState publish rate. Also sets the GETSTATE poll rate. Match your robot's RTDE rate. |
| `use_feedback` | `true` | `true` = publish measured position/velocity/force from GETSTATE polling. `false` = commanded position only. |

---

## Architecture notes

**Two-layer design:** `wsg_tcp.py` is pure Python with zero ROS dependency. You can import it in any script, test it without a ROS environment, and swap the ROS node layer without touching the protocol code.

**`move_nonblocking()` for teleoperation:** The standard `move()` call blocks until the gripper reports `FIN MOVE` — useful for scripted sequences. `move_nonblocking()` fires the command and returns immediately. This is what the ROS node uses: the GELLO leader streams continuous position targets, and the gripper tracks them without the subscriber callback ever blocking.

**Dual-socket state feedback:** The WSG 32 accepts multiple simultaneous TCP connections on port 1000. The driver opens two: one for commands (`move`, `home`, `ack_fault`) and one dedicated to `GETSTATE()` polling on a background thread. This keeps the command path and the feedback path completely independent — a slow poll never delays a move command. The `state` property returns a thread-safe copy of measured position (mm), velocity (mm/s), and force (N). Set `use_feedback: false` in the yaml to disable this and fall back to commanded position only.

**Staleness detection:** `GripperState.is_fresh(max_age_s)` returns `False` if no feedback has arrived within the threshold. The ROS node checks this before publishing and logs a warning if it falls back to commanded position.

**Deadband filter:** GELLO encoders have noise. Without a deadband, the node would spam the TCP socket with hundreds of identical MOVE commands per second. The `cmd_deadband_mm` parameter filters out moves smaller than the threshold.

---

## Dual-arm usage (TwinNexus)

Each gripper runs as an independent node in its own namespace:

```
/left_arm/wsg32_node/cmd_pos        →  gripper at 192.168.1.202
/right_arm/wsg32_node/cmd_pos       →  gripper at 192.168.1.201
```

```bash
ros2 launch wsg32_driver wsg32_dual.launch.py
```

---

## Limitations / known issues

- `move_nonblocking()` accumulates unread `FIN MOVE` bytes in the command socket buffer over long sessions. This is harmless for teleoperation but means you cannot reliably mix blocking and non-blocking calls in the same session.
- Only tested on WSG 32. Should work on WSG 50 — change `max_pos_mm` to `110.0`.
- If your firmware rejects two simultaneous TCP connections, set `use_feedback: false` in the yaml. The node falls back to publishing commanded position.

---

## Related projects

- [real-stanford/minWSG](https://github.com/real-stanford/minWSG) — minimal Python class this driver is based on
- [real-stanford/universal_manipulation_interface](https://github.com/real-stanford/universal_manipulation_interface) — UMI bimanual teleoperation system (uses the same GCL approach)
- [KITrobotics/weiss_wsg50](https://github.com/KITrobotics/weiss_wsg50) — ROS 1 binary protocol driver (not compatible)

---

## License

MIT — do whatever you want, attribution appreciated.
