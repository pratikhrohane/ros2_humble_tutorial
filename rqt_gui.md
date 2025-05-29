# 🖥️ ROS 2 RQT GUI

> 🎛️ RQT is a **GUI-based tool** in ROS 2 that supports various plugins for **visualization**, **robot control**, **monitoring**, and **debugging**.

---

## 📦 What is RQT?

- RQT is a Qt-based framework.
- It allows users to interact with ROS using plugins instead of command-line tools.
- You can **call services**, **plot topics**, **visualize parameters**, **inspect nodes**, and more.

---

## 🛠️ Install RQT (ROS 2 Humble)

Install all common RQT plugins using:
```bash
sudo apt install ~nros-humble-rqt*
````

---

## ▶️ Launch RQT

```bash
rqt
```

> 📝 If it doesn't open, try running `rqt` from a sourced terminal.

---

## 🐢 Try RQT with Turtlesim

1. **Run turtlesim node**

   ```bash
   ros2 run turtlesim turtlesim_node
   ```

2. **Open RQT (if not already)**

   ```bash
   rqt
   ```

3. In RQT, go to:

   ```
   Plugins → Services → Service Caller
   ```

---

## 🎨 Spawn a New Turtle via RQT

1. In **Service Caller** plugin:

   * Under the **Service** dropdown, select:

     ```
     /spawn
     ```
   * Enter custom `x`, `y`, and `theta` values.
   * Press **Call**.

> 🐢 A new turtle (e.g., `turtle2`) will appear in the turtlesim window.

---

## ✏️ Change Turtle Pen Color

1. In the same **Service Caller** plugin:

   * Choose the service:

     ```
     /turtle1/set_pen
     ```
   * Set values like `r: 255`, `g: 0`, `b: 0` to make red color.
   * Press **Call**.

2. Now move the turtle to draw with the new color!

---

## 🔁 Control a New Turtle

By default, `turtle_teleop_key` sends velocity commands to `/turtle1`.

To control `/turtle2`, use ROS 2 remapping:

```bash
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```

---

## 🔌 Other Useful RQT Plugins

| Plugin Name          | Purpose                                           |
| -------------------- | ------------------------------------------------- |
| 🧭 `Node Graph`      | Visualize how nodes, topics, and services connect |
| 📈 `Plot`            | Plot values from topics in real-time              |
| 🛠️ `Service Caller` | Call any available service                        |
| 📋 `Topic Monitor`   | View live messages on selected topics             |
| 🎚️ `Param Editor`   | Inspect and modify ROS parameters live            |
| 📡 `Publisher`       | Manually publish messages to any topic            |

---

## ❓ Think & Reflect

> 💭 Why might a GUI be helpful when debugging or testing ROS-based robots?
> *Hint: Think about real-time feedback vs. terminal commands.*

---

## ✅ What’s Next?

👉 [ROS Node](./ros_nodes.md)

---
