# ⌨️ Keyboard Control for VRX Main Boat - Complete Guide

## 🎯 Quick Start (Arrow Keys)

**Easiest method - just arrow keys!**

```bash
cd ~/final_stand/vrx/dashboard
./start_keyboard_control.sh
```

**Controls:**
- **↑** = Forward
- **↓** = Backward
- **←** = Turn Left
- **→** = Turn Right
- **Space** = Stop
- **q** = Quit

---

## 📋 All Available Methods

### **Method 1: Arrow Key Control (RECOMMENDED)** ⭐

**File:** `arrow_key_teleop.py`

**Usage:**
```bash
cd ~/final_stand/vrx/dashboard
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
python3 arrow_key_teleop.py
```

**Or use the helper script:**
```bash
./start_keyboard_control.sh
```

**Features:**
- ✅ Direct arrow key control
- ✅ No dependencies
- ✅ Simple and clean
- ✅ Works immediately

---

### **Method 2: WASD Control (via teleop_twist_keyboard)**

**Step 1: Install package**
```bash
sudo apt install ros-jazzy-teleop-twist-keyboard
```

**Step 2: Start the bridge (Terminal 1)**
```bash
cd ~/final_stand/vrx/dashboard
source /opt/ros/jazzy/setup.bash
source ~/vrx_ws/install/setup.bash
python3 twist_to_thrusters.py
```

**Step 3: Start keyboard teleop (Terminal 2)**
```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- **i** = Forward
- **,** = Backward
- **j** = Turn Left
- **l** = Turn Right
- **k** = Stop
- **q/z** = Increase/decrease max speeds

---

### **Method 3: Direct Command Line Control**

**Quick testing without scripts:**

```bash
# Forward
ros2 topic pub /wamv/thrusters/left/thrust std_msgs/msg/Float64 "data: 800.0" &
ros2 topic pub /wamv/thrusters/right/thrust std_msgs/msg/Float64 "data: 800.0"

# Turn Left
ros2 topic pub /wamv/thrusters/left/thrust std_msgs/msg/Float64 "data: -400.0" &
ros2 topic pub /wamv/thrusters/right/thrust std_msgs/msg/Float64 "data: 400.0"

# Stop
ros2 topic pub --once /wamv/thrusters/left/thrust std_msgs/msg/Float64 "data: 0.0" &
ros2 topic pub --once /wamv/thrusters/right/thrust std_msgs/msg/Float64 "data: 0.0"
```

---

## 🔧 How It Works

### **ROS Topic Structure**

The main boat is controlled via these topics:

| Topic | Message Type | Purpose | Range |
|-------|-------------|---------|-------|
| `/wamv/thrusters/left/thrust` | `std_msgs/msg/Float64` | Left thruster power | -1000.0 to +1000.0 |
| `/wamv/thrusters/right/thrust` | `std_msgs/msg/Float64` | Right thruster power | -1000.0 to +1000.0 |
| `/wamv/thrusters/left/pos` | `std_msgs/msg/Float64` | Left thruster angle | -π to +π radians |
| `/wamv/thrusters/right/pos` | `std_msgs/msg/Float64` | Right thruster angle | -π to +π radians |

### **Control Flow**

```
Keyboard Input
    ↓
arrow_key_teleop.py (or twist_to_thrusters.py)
    ↓
Publishes to /wamv/thrusters/*/thrust
    ↓
Gazebo Thruster Plugin
    ↓
Physics Engine
    ↓
Boat Moves
```

### **Differential Drive Control**

For turning, we use differential thrust:
- **Forward:** Both thrusters forward (same speed)
- **Backward:** Both thrusters reverse (same speed)
- **Turn Left:** Left reverse, right forward
- **Turn Right:** Left forward, right reverse

**Formula:**
```
left_thrust = (linear_x - angular_z) * max_thrust
right_thrust = (linear_x + angular_z) * max_thrust
```

---

## 🎮 Control Parameters

You can adjust these in `arrow_key_teleop.py`:

```python
self.forward_thrust = 600.0  # Forward/backward power
self.turn_thrust = 400.0     # Turning power
```

**Recommended values:**
- **Gentle:** 300-400 (slow, precise)
- **Normal:** 600-800 (balanced)
- **Aggressive:** 900-1000 (fast, less control)

---

## 🐛 Troubleshooting

### **Problem: Arrow keys don't work**

**Solution 1:** Make sure the terminal window is focused (click on it)

**Solution 2:** Check if ROS is running:
```bash
ros2 node list
# Should see nodes listed
```

**Solution 3:** Check if main boat topics exist:
```bash
ros2 topic list | grep wamv
# Should see /wamv/thrusters/left/thrust, etc.
```

### **Problem: Boat doesn't move**

**Check 1:** Verify Gazebo is running and boat is spawned

**Check 2:** Test with direct command:
```bash
ros2 topic pub --once /wamv/thrusters/left/thrust std_msgs/msg/Float64 "data: 500.0"
# Boat should move in Gazebo
```

**Check 3:** Check for errors:
```bash
ros2 topic echo /wamv/thrusters/left/thrust
# Should see messages when you press arrow keys
```

### **Problem: "Permission denied" when running script**

**Solution:**
```bash
chmod +x arrow_key_teleop.py
python3 arrow_key_teleop.py
```

---

## 📚 Technical Details

### **Message Format**

**Thrust Command:**
```python
from std_msgs.msg import Float64

msg = Float64()
msg.data = 600.0  # -1000.0 to +1000.0
publisher.publish(msg)
```

**Thruster Angle:**
```python
msg = Float64()
msg.data = 0.0  # -3.14 to +3.14 radians
publisher.publish(msg)
```

### **Coordinate System**

- **Positive thrust** = Forward (boat moves forward)
- **Negative thrust** = Reverse (boat moves backward)
- **Differential thrust** = Turning (one forward, one reverse)

### **Physics**

The Gazebo thruster plugin:
1. Reads commanded thrust value
2. Applies physics model (drag, inertia, hydrodynamics)
3. Calculates force vector
4. Applies force to boat body
5. Updates boat position/velocity

---

## 🎯 Best Practices

1. **Start slow:** Use lower thrust values (300-400) when learning
2. **Stop before quitting:** Press Space to stop before exiting
3. **One terminal:** Keep the control terminal focused for arrow keys
4. **Check Gazebo:** Watch Gazebo to see boat movement
5. **Monitor topics:** Use `ros2 topic echo` to debug

---

## 📝 Example: Custom Control Script

If you want to create your own control script:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.left_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
    
    def move_forward(self, speed=500.0):
        msg = Float64()
        msg.data = speed
        self.left_pub.publish(msg)
        self.right_pub.publish(msg)

def main():
    rclpy.init()
    node = MyController()
    node.move_forward(speed=800.0)
    rclpy.spin_once(node, timeout_sec=5.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## ✅ Summary

**For main boat control, use:**
```bash
cd ~/final_stand/vrx/dashboard
./start_keyboard_control.sh
```

**That's it!** Arrow keys control the main boat. Simple, direct, no dependencies.

**Files:**
- `arrow_key_teleop.py` - Arrow key control script
- `twist_to_thrusters.py` - WASD control bridge
- `start_keyboard_control.sh` - Quick start script

**Topics (Main Boat Only):**
- `/wamv/thrusters/left/thrust`
- `/wamv/thrusters/right/thrust`
- `/wamv/thrusters/left/pos`
- `/wamv/thrusters/right/pos`

