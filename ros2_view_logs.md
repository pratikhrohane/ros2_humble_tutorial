# 📜 ROS 2 VIEW LOGS

> 🧾 **Logging** in ROS 2 helps you monitor node behavior through logs like:
> - `INFO` – normal messages
> - `WARN` – something might be wrong
> - `ERROR` – something is wrong
> - `DEBUG` – for detailed developer info

You can view logs in **CLI** or with the **RQT Console GUI**.

---

## 🖥️ View Logs with RQT Console

### ✅ Step-by-Step Demo

1. Open the log GUI:
   ```bash
   ros2 run rqt_console rqt_console

2. In another terminal, start turtlesim:

   ```bash
   ros2 run turtlesim turtlesim_node
   ```

3. Open teleop in another terminal:

   ```bash
   ros2 run turtlesim turtle_teleop_key
   ```

4. Crash the turtle into the wall. Observe logs in the RQT Console:

   * **Info**, **Warning**, and **Error** messages will appear.
   * You can filter by log level or node name.

---

## ⚙️ Logging Control in CLI

| 🔧 Command                                                | 📝 Description               | 💡 Use Case                       |
| --------------------------------------------------------- | ---------------------------- | --------------------------------- |
| `ros2 run rqt_console rqt_console`                        | Launch the GUI log viewer    | 🖥️ Monitor logs visually         |
| `ros2 run <pkg> <node>`                                   | Run any node to see logs     | 🐢 Example: `turtlesim_node`      |
| `ros2 run <pkg> <node> --ros-args --log-level <LEVEL>`    | Set logging level at runtime | ⚠️ Filter for warnings only, etc. |
| `--log-level DEBUG` / `INFO` / `WARN` / `ERROR` / `FATAL` | Available log levels         | 📉 Reduce noise or troubleshoot   |

---

### 🧪 Example: Only Show Warnings and Above

```bash
ros2 run turtlesim turtlesim_node --ros-args --log-level WARN
```

✅ Now only `WARN`, `ERROR`, and `FATAL` logs will be visible.

---

## ❓ Think & Reflect

> 💭 Why might filtering logs to show only warnings help during debugging large robots?

---

## ✅ What’s Next?

👉 [ROS 2 Package Creation & Build with Colcon](./ros2_colcon_packages.md)

---
