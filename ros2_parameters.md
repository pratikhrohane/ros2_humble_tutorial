# ⚙️ ROS 2 Parameters

> 🧩 Parameters in ROS 2 are configurable values stored inside a node.
>
> - They let you change a node’s behavior **at runtime or startup**.
> - Useful for tuning settings like colors, speeds, file paths, etc.

---

## 🐢 Try It Out: Turtlesim Example

1. Run turtlesim in one terminal:
   ```bash
   ros2 run turtlesim turtlesim_node


2. (Optional) Run teleop in another:

   ```bash
   ros2 run turtlesim turtle_teleop_key
   ```

---

## 📋 Parameter Commands

| 🧾 Command                                           | 📝 Description                   | 💡 Use Case                         |
| ---------------------------------------------------- | -------------------------------- | ----------------------------------- |
| `ros2 param list`                                    | List all nodes with parameters   | 📋 See which nodes have parameters  |
| `ros2 param get <node> <param>`                      | Get a specific parameter value   | 🔍 Read current setting             |
| `ros2 param set <node> <param> <value>`              | Set a parameter's value          | 🛠️ Modify node behavior at runtime |
| `ros2 param dump <node>`                             | View all parameters of a node    | 📄 Snapshot current config          |
| `ros2 param dump <node> > file.yaml`                 | Save parameters to a YAML file   | 💾 Save reusable config             |
| `ros2 param load <node> <file.yaml>`                 | Load parameters from a YAML file | 🔁 Restore saved state              |
| `ros2 run <pkg> <exe> --ros-args --params-file file` | Load params at node startup      | 🚀 Auto-configure node on launch    |

---

## 🧪 Examples

### ✅ Get a parameter

```bash
ros2 param get /turtlesim background_g
```

### ✅ Set a parameter

```bash
ros2 param set /turtlesim background_g 186
```

Now reset background using:

```bash
ros2 service call /clear std_srvs/srv/Empty
```

### ✅ Save and reuse

```bash
ros2 param dump /turtlesim > turtlesim.yaml
ros2 param load /turtlesim turtlesim.yaml
```

---

## 🚀 Launch Node with Parameters

To start a node with predefined params:

```bash
ros2 run turtlesim turtlesim_node --ros-args --params-file turtlesim.yaml
```

---

## ❓ Think & Reflect

> 💭 Why might it be useful to load parameters from a file instead of setting them manually?

> 💡 Hint: Think about deploying multiple robots or setting default startup configs.

---

## ✅ What’s Next?

👉[ROS2 ACTIONS](./ros2_actions.md)

---
