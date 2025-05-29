## 📦 ROS 2 Humble Installation (Ubuntu)

> 🔗 Official Docs: [ROS 2 Humble Installation - Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

---

### 🛠️ Prerequisites

* ✅ Ubuntu 22.04 (Recommended for ROS 2 Humble)

---

### 📥 Step-by-Step Installation

1. **Setup sources**

   ```bash
   sudo apt update && sudo apt install curl gnupg lsb-release
   ```

2. **Add the ROS 2 GPG key**

   ```bash
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
   ```

3. **Add ROS 2 repository**

   ```bash
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
   sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   ```

4. **Install ROS 2 Humble**

   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop
   ```

---

### ✅ Post Installation: Source the Setup File

> This is required to access ROS 2 tools like `ros2`, `colcon`, etc., in your terminal.

#### ⚙️ Option 1: Command Line (Quick Way)

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### ✍️ Option 2: Manual Edit (Preferred for control)

1. Open `.bashrc` in any editor:

   ```bash
   gedit ~/.bashrc
   ```
2. Add this line at the end:

   ```bash
   source /opt/ros/humble/setup.bash
   ```
3. Save and close the file, then run:

   ```bash
   source ~/.bashrc
   ```

---

### 📌 Bonus Tip

If you're using **zsh**, source the file in `.zshrc` instead:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.zshrc
source ~/.zshrc
```

---

### ❓ Think & Reflect

> 💭 Why do we need to `source` the setup file every time?
> *Hint: Think about how your shell knows where to find ROS 2 commands.*

---

### ✅ What’s Next?

→ [Running Executavle](./running_executable.md)

---
