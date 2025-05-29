# 🚀 Running Executables (ROS 2 Nodes)

> 📘 Learn how to install a package and run ROS 2 executables using `ros2 run`.

---

## 🔧 Install Example Package: `turtlesim`

```bash
sudo apt update
sudo apt install ros-humble-turtlesim
````

---

## 📦 Explore Installed Packages

### 🔍 See list of all installed ROS 2 packages

```bash
ros2 pkg list
```

### 📋 See all packages with their executables

```bash
ros2 pkg executables
```

### 🔍 See executables inside a specific package (e.g., `turtlesim`)

```bash
ros2 pkg executables turtlesim
```

---

## 📁 Where Is `turtlesim` Located?

Navigate to installed shared files:

```bash
cd /opt/ros/humble/share
```

You'll find folders for all installed packages here.

---

## ▶️ Running a Node (Executable)

Use the command format:

```bash
ros2 run <package_name> <executable_name>
```

### 🐢 Run the Turtlesim Node

```bash
ros2 run turtlesim turtlesim_node
```

> This opens a GUI window with a turtle in it!

---

## ⌨️ Control the Turtle (in a second terminal)

Open a **new terminal**, and run:

```bash
ros2 run turtlesim turtle_teleop_key
```

> Use arrow keys to move the turtle 🐢

---

## ❓ Think & Reflect

> 💭 What is the difference between a ROS 2 package and an executable?
> *Can a package contain multiple executables?*

---

## ✅ What’s Next?

👉  [ROS2 RQT](./rqt.md)

---
