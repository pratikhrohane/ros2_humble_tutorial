# 🔄 ROS 2 Topics

> 📡 A **topic** is a one-way communication channel used in ROS 2 to send data between nodes.
>
> - A node can **publish** data to a topic.
> - Another node can **subscribe** to receive that data.
> - Topics follow a **many-to-many** model (many publishers & subscribers).

---

## 🐢 Try It Out: Turtlesim Example

1. In Terminal 1, run the **turtle simulation**:

   ```bash
   ros2 run turtlesim turtlesim_node


2. In Terminal 2, run the **teleop keyboard control**:

   ```bash
   ros2 run turtlesim turtle_teleop_key
   ```

3. See available topics:

   ```bash
   ros2 topic list
   ```

4. See topics with message types:

   ```bash
   ros2 topic list -t
   ```

5. Launch **RQT Graph** to visualize topic-node connections:

   ```bash
   rqt_graph
   ```

   > 💡 Uncheck `Hide dead sinks`, `dead ends`, and `unconnected` to view all topics and connections.

---

## 🧪 Inspecting Topics

| 🧾 Command                     | 📝 Description                               | 💡 Use Case                    |
| ------------------------------ | -------------------------------------------- | ------------------------------ |
| `ros2 topic list`              | List all active topics                       | 📋 See what’s available        |
| `ros2 topic list -t`           | List topics with message types               | 🔍 Get full topic type details |
| `ros2 topic echo <topic_name>` | Print live messages from a topic             | 👀 Debug or visualize data     |
| `ros2 topic info <topic_name>` | Show publishers/subscribers and message type | 🧠 Understand the topic's role |
| `ros2 interface show <type>`   | Show the message structure of a given type   | 📦 Inspect message fields      |

### 🧪 Example

```bash
ros2 topic echo /turtle1/cmd_vel
ros2 topic info /turtle1/cmd_vel
ros2 interface show geometry_msgs/msg/Twist
```

---

## 📨 Publishing to Topics

| 🚀 Command                                   | 📝 Description                         | 💡 Use Case                           |
| -------------------------------------------- | -------------------------------------- | ------------------------------------- |
| `ros2 topic pub <topic> <msg_type> '<args>'` | Publish custom data to a topic         | 🧪 Manual test of functionality       |
| `--once`                                     | Publish once and exit                  | 🧷 One-time test or movement          |
| `--rate <Hz>`                                | Continuously publish at specified rate | 🔁 Simulate ongoing sensor or control |

### 📌 Examples

* **Send one-time velocity command**:

  ```bash
  ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.0}}"
  ```

* **Publish velocity at 1Hz**:

  ```bash
  ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
  ```

---

## 📊 Monitor Topic Frequency

| 📈 Command              | 📝 Description                         | 💡 Use Case                          |
| ----------------------- | -------------------------------------- | ------------------------------------ |
| `ros2 topic hz <topic>` | Check frequency of messages on a topic | ⏱️ Ensure publishing at desired rate |

### 📌 Example

```bash
ros2 topic hz /turtle1/pose
```

> ✅ Confirm if sensor or control loop is consistent.

---

## ❓ Think & Reflect

> 💭 **Why would you want to publish data manually to a topic during development?**

> 💡 *Hint: You might want to simulate sensor input or test control commands without the actual hardware.*

---

## ✅ What’s Next?

👉 [ROS2 SERVICES](./ros_services.md)

---
