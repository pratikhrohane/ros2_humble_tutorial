# 🛠️ ROS 2 Services

> 🔁 **Services** in ROS 2 are a *synchronous communication mechanism* using a **Client-Server model**.
>
> - A **Client Node** sends a **request**.
> - A **Server Node** processes and sends back a **response**.
> - Unlike Topics, Services are **two-way** and occur only when called.

---

## 🐢 Try It Out: Turtlesim Example

1. In Terminal 1, run the turtlesim node:
   ```bash
   ros2 run turtlesim turtlesim_node


2. In Terminal 2, run the teleop key controller:

   ```bash
   ros2 run turtlesim turtle_teleop_key
   ```

---

## 🔍 Exploring Services

| 🧾 Command                   | 📝 Description                            | 💡 Use Case                                  |
| ---------------------------- | ----------------------------------------- | -------------------------------------------- |
| `ros2 service list`          | List all available services               | 📋 View active service endpoints             |
| `ros2 service list -t`       | List services with their message types    | 🔍 Understand service structure              |
| `ros2 service type <name>`   | Get the type of a specific service        | 📦 Identify service interface type           |
| `ros2 service find <type>`   | Find services that use a specific type    | 🔎 Search for services by message definition |
| `ros2 interface show <type>` | Show the fields of a service message type | 🧠 Understand request/response data format   |

### 📌 Examples

```bash
ros2 service list
ros2 service list -t
ros2 service type /clear
ros2 service find std_srvs/srv/Empty
ros2 interface show std_srvs/srv/Empty
```

---

## 📞 Calling Services

| 📡 Command                                 | 📝 Description                          | 💡 Use Case                              |
| ------------------------------------------ | --------------------------------------- | ---------------------------------------- |
| `ros2 service call <name> <type>`          | Call a service (interactive or no args) | 🧪 Manually test behavior                |
| `ros2 service call <name> <type> '<args>'` | Call a service with custom arguments    | 🛠️ Spawn or configure nodes dynamically |

### 📌 Examples

* **Clear background in turtlesim**:

  ```bash
  ros2 service call /clear std_srvs/srv/Empty
  ```

* **Spawn a new turtle**:

  ```bash
  ros2 service call /spawn turtlesim/srv/Spawn "{x: 3.0, y: 3.0, theta: 1.57, name: 'turtle2'}"
  ```

* **Change turtle pen color**:

  ```bash
  ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 5, off: 0}"
  ```

---

## ❓ Think & Reflect

> 💭 When would you choose a **Service** over a **Topic**?

> 💡 Hint: Think about whether the data needs a **response** (confirmation, result, status).

---

## ✅ What’s Next?

👉 [ROS2 PARAMETERS](./ros2_parameters.md)

---
