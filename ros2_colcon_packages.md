# 📦 ROS 2 Package Creation & Build with Colcon

## 🧰 What Is a Package?

A **ROS 2 package** is the basic unit of software organization. It contains all your code, configuration, and metadata needed to build and run ROS 2 programs.

---

## 🧠 What Is a Workspace?

A **workspace** is a directory that contains packages and is used with build tools like `colcon`.

### 🗂️ Workspace Structure:

```
ros2_ws/
├── src/      # Your ROS packages go here
├── build/    # Temporary build files (created after building)
├── install/  # Installed files (used for sourcing)
└── log/      # Build logs
```

---

## 🛠️ Install Colcon

```bash
sudo apt install python3-colcon-common-extensions
```

> ⬇️ Optional (recommended): Enable tab completion

```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
```

---

## 🏗️ Create Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

---

## 🌱 Add Sample Code

```bash
cd ~/ros2_ws/src
git clone https://github.com/ros/ros_tutorials.git -b humble
```

---

## 🔨 Build with Colcon

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

> `--symlink-install`: Makes symbolic links instead of copies (good for development).

---

## 📂 What Are These Folders?

| Folder     | Description                         |
| ---------- | ----------------------------------- |
| `build/`   | Temp files generated during build   |
| `install/` | Final compiled/installable packages |
| `log/`     | Build logs and outputs              |

---

## 🔗 Source the Environment

| Command                             | Purpose                                    |
| ----------------------------------- | ------------------------------------------ |
| `source /opt/ros/humble/setup.bash` | 🟦 Source **underlay** (official packages) |
| `source install/local_setup.bash`   | 🟩 Source **overlay** (your packages)      |

> ℹ️ Sourcing the overlay includes both your packages and underlay automatically.

---

## 🛠️ Modify and Rebuild

1. Change code (e.g., `turtle_frame.cpp` window title).
2. Rebuild:

```bash
colcon build --symlink-install
source install/local_setup.bash
```

3. Run:

```bash
ros2 run turtlesim turtlesim_node
```

---

## 📁 Create New Package

### 📦 CMake Package

```bash
ros2 pkg create --build-type ament_cmake --node-name my_node my_package
```

### 🐍 Python Package

```bash
ros2 pkg create --build-type ament_python --node-name my_node my_package
```

---

## 📤 Package Structure Requirements

### CMake Package

| File/Dir         | Purpose          |
| ---------------- | ---------------- |
| `CMakeLists.txt` | How to build     |
| `package.xml`    | Package metadata |
| `include/<pkg>`  | Header files     |
| `src/`           | Source code      |

---

### Python Package

| File/Dir            | Purpose                     |
| ------------------- | --------------------------- |
| `package.xml`       | Metadata                    |
| `setup.py`          | Install instructions        |
| `setup.cfg`         | Entry point for executables |
| `resource/<pkg>`    | Package marker              |
| `<pkg>/__init__.py` | Module init file            |

---

## 🔁 Build a Custom Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_package
source install/local_setup.bash
```

Run the node:

```bash
ros2 run my_package my_node
```

---

## ❓ FAQ

### 🤔 Why `source install/local_setup.bash`?

To update your environment with the newly built package.

### 🏗️ What is `ament_cmake` vs `ament_python`?

| Build Type     | Use When...               |
| -------------- | ------------------------- |
| `ament_cmake`  | You are writing in C++    |
| `ament_python` | You are writing in Python |

> ✅ Use **CMake** for performance-heavy logic or robot drivers.
> ✅ Use **Python** for scripting, quick prototyping, or simple logic.

---

## ✅ What's Next?

👉 [ROS 2 Publisher and Subscriber Package (C++)](./cpp_pubsub.md)

---
