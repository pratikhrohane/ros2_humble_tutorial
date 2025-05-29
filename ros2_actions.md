# 🎯 ROS 2 Actions

> 🧭 **Actions** let nodes communicate with long-running goals using a client-server pattern.  
> You get:
> - ✅ **Goal** (what to do)
> - 🔄 **Feedback** (how it’s going)
> - 🏁 **Result** (final outcome)

---

## 🐢 Try It Out: Turtlesim Example

1. In one terminal, start turtlesim:
   ```bash
   ros2 run turtlesim turtlesim_node

2. In another terminal, start teleop:

   ```bash
   ros2 run turtlesim turtle_teleop_key
   ```

3. Try sending actions via key press (like `G`, `F`, `V`) in the **teleop terminal** and observe:

   * `G`: Rotate turtle to a goal angle.
   * `F`: Cancel the rotation before completion.
   * `V`: Send another goal before previous one finishes.

---

## 🔍 Inspect Action Clients

Check action clients connected to a node:

```bash
ros2 node info /teleop_turtle
```

---

## 📋 Action Commands Table

| 🔧 Command                                       | 📝 Description                      | 💡 Use Case                            |
| ------------------------------------------------ | ----------------------------------- | -------------------------------------- |
| `ros2 action list`                               | List all active action servers      | 📋 What actions are available?         |
| `ros2 action list -t`                            | List actions with type info         | 🔎 Know their message types            |
| `ros2 action info <action_name>`                 | Get details about a specific action | 🧠 Understand input/output structure   |
| `ros2 interface show <action_type>`              | Show the definition of the action   | 📦 See fields for goal/feedback/result |
| `ros2 action send_goal <action> <type> "<goal>"` | Send a goal to an action server     | 🛠️ Trigger a robot behavior           |

---

## ✅ Example: Rotate the Turtle

Send a rotation goal to the turtle:

```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}"
```

See action structure:

```bash
ros2 interface show turtlesim/action/RotateAbsolute
```

Expected fields:

```text
# Goal
float32 theta
---
# Result
bool success
---
# Feedback
float32 remaining
```

---

## ❓ Think & Reflect

> 💭 When would you prefer **actions** over **topics** or **services**?

> 🧠 Hint: Think about **long-running tasks** (e.g., navigation, docking, arm movement).

---

## ✅ What’s Next?

👉 [ROS2 VIEW LOGS](./ros2_view_logs.md)

---
