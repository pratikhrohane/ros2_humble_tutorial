# 🧱 ROS 2 C++ Publisher & Subscriber

---

### ✅ Step 1: Create the Package

```bash
ros2 pkg create --build-type ament_cmake cpp_pubsub
```

* `ros2 pkg create`: Command to create a new ROS 2 package.
* `--build-type ament_cmake`: Tells ROS 2 to use CMake for building (used for C++ packages).
* `cpp_pubsub`: The name of the package.

---

### 📡 Step 2: Write the Publisher Node

## ✅ PUBLISHER NODE (`publisher_member_function.cpp`)

```cpp
#include <chrono>                // For time-based functions
#include <functional>           // For using std::bind
#include <memory>               // For smart pointers like shared_ptr
#include <string>               // For using std::string
```

* These are standard C++ libraries used to manage time intervals, memory, function callbacks, and strings.

```cpp
#include "rclcpp/rclcpp.hpp"    // Core ROS 2 C++ API
#include "std_msgs/msg/string.hpp"  // Message type used to publish
```

* ROS 2 headers:

  * `rclcpp.hpp` allows us to create nodes, publishers, timers, logs, etc.
  * `std_msgs/msg/string.hpp` gives us access to the `String` message type.

```cpp
using namespace std::chrono_literals;
```

* This allows you to write `500ms` instead of `std::chrono::milliseconds(500)` — cleaner syntax for time durations.

---

### ✅ Define the Publisher Node

```cpp
class MinimalPublisher : public rclcpp::Node
```

* Create a class called `MinimalPublisher` that inherits from `rclcpp::Node`, so it becomes a ROS 2 node.

```cpp
public:
  MinimalPublisher() : Node("minimal_publisher"), count_(0)
```

* The constructor sets the node's name to `"minimal_publisher"`.
* `count_` starts at 0 and will increase every time a message is sent.

```cpp
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
```

* Creates a publisher that sends messages of type `String` on the topic `"topic"` with a queue size of 10.

```cpp
  timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
```

* This sets a timer to call `timer_callback()` every 500 milliseconds.

---

### ✅ Timer Callback

```cpp
private:
  void timer_callback()
```

* Function that gets called on every timer tick.

```cpp
    auto message = std_msgs::msg::String();
```

* Creates an empty String message.

```cpp
    message.data = "Hello, world! " + std::to_string(count_++);
```

* Fills the message with a string + incrementing number (e.g., "Hello, world! 1").

```cpp
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
```

* Prints the message to the terminal (ROS 2 logging system).

```cpp
    publisher_->publish(message);
```

* Sends the message to all subscribers listening on the topic.

---

### ✅ Main Function

```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  // Initialize ROS
  rclcpp::spin(std::make_shared<MinimalPublisher>()); // Run the node
  rclcpp::shutdown(); // Clean shutdown
  return 0;
}
```

---

### 📥 Step 3: Write the Subscriber Node (Conceptual Steps)

## ✅ SUBSCRIBER NODE (`subscriber_member_function.cpp`)

```cpp
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;
```

* Same includes as before. `_1` is a placeholder for the first argument in a callback function (used in `std::bind`).

---

### ✅ Define Subscriber Class

```cpp
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber() : Node("minimal_subscriber")
```

* Constructor creates a node named `"minimal_subscriber"`.

```cpp
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
```

* Subscribes to `"topic"` with queue size 10.
* Whenever a message is received, `topic_callback()` is called with the message as input.

---

### ✅ Callback

```cpp
void topic_callback(const std_msgs::msg::String & msg) const
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
```

* Logs the message received from the publisher.

---

### ✅ Main

```cpp
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

* Same as in the publisher. Initializes ROS, runs the node, and shuts it down.

---

### 📄 Step 4: Update `package.xml`

You must declare dependencies here so ROS knows what packages your code depends on.

Add inside `<package>`:

```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```

This tells ROS 2 that:

* `rclcpp` is required for node creation and ROS 2 functionalities.
* `std_msgs` is required for using `String` message type.

---

### 🛠 Step 5: Update `CMakeLists.txt`

This file tells ROS 2 how to compile your code.
## ✅ `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.5)
project(cpp_pubsub)
```

* Sets the CMake version and project name.

```cmake
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
```

* Use C++14 standard if not specified.

```cmake
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
```

* Adds extra warning flags for better coding practices.

---

### Find ROS 2 Packages

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```

* Required to compile and link against ROS 2 core (`rclcpp`) and the standard message package (`std_msgs`).

---

### Build Executables

```cmake
add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp std_msgs)

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp std_msgs)
```

* Build two executables: `talker` and `listener` from source files.
* Link each with required ROS 2 libraries.

---

### Install Targets

```cmake
install(TARGETS
  talker
  listener
  DESTINATION lib/${PROJECT_NAME})
```

* Installs the binaries into the proper install directory so that `ros2 run` can find them.

---

### Finalize

```cmake
ament_package()
```

* Declares this as a valid ROS 2 package.

---

🔍 Key points:

* `add_executable`: Tells CMake which file to compile and name the resulting binary.
* `ament_target_dependencies`: Links ROS libraries to your binary.
* `install(...)`: Makes your executables accessible with `ros2 run`.

---

### 🧱 Step 6: Build the Package

1. Go to workspace root:

   ```bash
   cd ~/ros2_ws
   ```

2. Install missing dependencies:

   ```bash
   rosdep install -i --from-path src --rosdistro humble -y
   ```

3. Build the package:

   ```bash
   colcon build --packages-select cpp_pubsub
   ```

4. Source the overlay:

   ```bash
   source install/local_setup.bash
   ```

---

### 🚀 Step 7: Run the Nodes

```bash
ros2 run cpp_pubsub talker    # Starts the publisher
ros2 run cpp_pubsub listener  # Starts the subscriber
```

They will start sending and receiving `"Hello, world!"` messages on the topic named `topic`.

---

### 🔚 Why You Source local\_setup.bash

This updates your environment so ROS 2 knows about the packages you just built. Without sourcing, `ros2 run` or `ros2 launch` won’t find your new nodes.

---

### ❓ What Is `ament_cmake` vs `ament_python`

| Build Type     | Language | Use When...                             |
| -------------- | -------- | --------------------------------------- |
| `ament_cmake`  | C++      | You write your ROS 2 nodes in C++       |
| `ament_python` | Python   | You write ROS 2 nodes/scripts in Python |

You choose the build type based on the programming language of your package. Neither is better — just suited to different cases.

---

## Summary

| Element                     | Purpose                                    |
| --------------------------- | ------------------------------------------ |
| `rclcpp::Node`              | Base class for any ROS 2 node              |
| `create_publisher`          | Enables publishing messages on a topic     |
| `create_subscription`       | Listens for messages on a topic            |
| `timer`                     | Automatically call a function periodically |
| `RCLCPP_INFO`               | Print log to the terminal                  |
| `CMakeLists.txt`            | Instructions for how to compile the code   |
| `ament_target_dependencies` | Link ROS 2 packages to your executable     |
| `install(TARGETS...)`       | Make your node available to `ros2 run`     |

---
