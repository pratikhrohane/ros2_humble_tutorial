# 🧩 ROS 2 Nodes

> ROS 2 **nodes** are standalone executables that interact with each other through:
>
> - 📨 Topics (for continuous data streams)
> - 🛎️ Services (for request/response tasks)
> - 🏁 Actions (for long-running goals)
> - ⚙️ Parameters (for configuration settings)

---

## 🧠 What is a Node?

- A **Node** is the basic unit of execution in a ROS 2 system.
- Each node handles specific functionality (e.g., sensor reading, controlling motors).
- Nodes can be created in **C++** or **Python**.

---

## 🚀 Node Example

Let’s run a sample node:

```bash
ros2 run turtlesim turtlesim_node
````

Now open another terminal and check running nodes:

```bash
ros2 node list
```

You should see:

```
/turtlesim
```

---

## 📋 Common `ros2 node` Commands

| Command                           | Description                                                    | Use Case                                           |
| --------------------------------- | -------------------------------------------------------------- | -------------------------------------------------- |
| `ros2 node -h`                    | Show help for `ros2 node`                                      | 📘 See available options                           |
| `ros2 node list`                  | List all currently running nodes                               | ✅ Verify what nodes are active                     |
| `ros2 node info <node_name>`      | Show detailed info: publishers, subscribers, services, actions | 🧠 Inspect how a node interacts with the ROS graph |
| `ros2 run <package> <executable>` | Run a node from a package                                      | 🚀 Launch your publisher/subscriber nodes          |
| `ros2 pkg executables <package>`  | See all executables inside a package                           | 🔎 Find what nodes can be launched                 |
| `ros2 node list -a`               | (Optional) List all full node names and namespaces             | 🗂️ See exact node naming in complex systems       |

> Example:
>
> ```bash
> ros2 node info /turtlesim
> ```

---

## 🧵 Types of Nodes

| Type                    | Description                                         | Example                                  |
| ----------------------- | --------------------------------------------------- | ---------------------------------------- |
| 📰 Publisher Node       | Publishes messages to a topic                       | Turtle node publishing position          |
| 📬 Subscriber Node      | Subscribes to topic and processes incoming messages | A node reacting to turtle movement       |
| 🛠️ Service Server Node | Provides a service callable by others               | `/spawn` service in turtlesim            |
| 📞 Service Client Node  | Calls a service provided by another node            | Using RQT to call `/set_pen`             |
| 🎯 Action Server        | Manages a long-running goal                         | Navigation goal tracking                 |
| 🎮 Action Client        | Sends goal to an action server                      | User controlling robot to reach location |
| ⚙️ Parameter Node       | Reads or modifies parameters                        | Dynamic config for simulation settings   |

---

## 💡 Bonus

* Nodes can be grouped in namespaces to avoid conflicts.
* Nodes can publish or subscribe to **multiple topics**.
* Nodes are **processes**—killing a node ends its functionality.

---

## ❓ Think & Reflect

> 💭 Can a single node act as both a publisher and a subscriber?
> *Hint: Think about sensor nodes or autonomous robots.*

---

## ✅ What’s Next?

👉 [ROS2 TOPICS](./ros_topics.md)

---
