# 🗣️ Voice Command Integration with ROS 2

This project demonstrates how to control a **ROS 2**-based `turtlesim` robot using **voice commands** captured via a microphone. It uses a Python-based **speech recognition module** to recognize spoken directions and communicates them over **Serial** to an **Arduino**, which interprets the commands and sends them back to the ROS 2 node to control the turtle’s motion in real time. 🐢

---

## 📦 Prerequisites

* Ubuntu 22.04 (recommended)
* Python 3
* Arduino board (e.g., Uno or Nano)
* USB microphone
* ROS 2 Humble installed
* Python `speech_recognition` and `pyserial` packages

---

## 🛠️ Step 1: Install ROS 2 Humble

> Follow same ROS 2 installation instructions as in your joystick README.

---

## 🧱 Step 2: Create a ROS 2 Workspace

```bash
mkdir -p ~/voice_ws/src
cd ~/voice_ws
colcon build
source install/setup.bash
````

---

## 🧠 Step 3: Arduino Code (Listen via Serial)

**Wiring:**

* No sensor input needed – only USB connection for serial listening.

**Upload this code using Arduino IDE:**

```cpp
// Arduino code: Forward serial command to serial monitor

String command = "";

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      Serial.println(command);  // Echo the command back
      command = "";
    } else {
      command += c;
    }
  }
}
```

---

## 🧠 Step 4: ROS 2 Voice Node (Python)

Inside `voice_ws/src`, create your package:

```bash
cd ~/voice_ws/src
ros2 pkg create voice_controller --build-type ament_python --dependencies rclpy geometry_msgs
```

### 📁 Directory Structure:

```
voice_controller/
├── voice_controller
│   └── voice_node.py
├── package.xml
├── setup.py
└── resource/
    └── voice_controller
```

### ✏️ `voice_node.py`

```python
#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import speech_recognition as sr


class VoiceToTurtle(Node):
    def __init__(self):
        super().__init__("voice_to_turtle")

        self.port_ = "/dev/ttyACM0"
        self.baudrate_ = 115200

        try:
            self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {self.port_}")
        except serial.SerialException:
            self.get_logger().error(f"Could not connect to Arduino on {self.port_}")
            exit(1)

        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        self.recognizer_ = sr.Recognizer()
        self.microphone_ = sr.Microphone()

        self.timer_ = self.create_timer(3.0, self.listen_and_publish)

    def listen_and_publish(self):
        with self.microphone_ as source:
            self.get_logger().info("🎤 Listening...")
            audio = self.recognizer_.listen(source, phrase_time_limit=3)

        try:
            text = self.recognizer_.recognize_google(audio).lower()
            self.get_logger().info(f"✅ Recognized: {text}")

            # Send to Arduino (optional)
            self.arduino_.write((text + "\n").encode())

            twist = Twist()

            if "forward" in text:
                twist.linear.x = 1.0
            elif "backward" in text:
                twist.linear.x = -1.0
            elif "left" in text:
                twist.angular.z = 1.0
            elif "right" in text:
                twist.angular.z = -1.0
            elif "stop" in text:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                self.get_logger().info("⚠️ Unknown command")

            self.cmd_vel_pub_.publish(twist)

        except sr.UnknownValueError:
            self.get_logger().warn("Could not understand audio")
        except Exception as e:
            self.get_logger().error(f"Recognition failed: {e}")


def main():
    rclpy.init()
    node = VoiceToTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

---

### 📄 `package.xml`

```xml
<?xml version="1.0"?>
<package format="3">
  <name>voice_controller</name>
  <version>0.0.1</version>
  <description>Voice-controlled ROS 2 Turtlesim project</description>

  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
</package>
```

---

### 🛠️ `setup.py`

```python
from setuptools import setup

package_name = 'voice_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Voice command interface for ROS 2 Turtlesim',
    license='MIT',
    entry_points={
        'console_scripts': [
            'voice_node = voice_controller.voice_node:main',
        ],
    },
)
```

---

## 🧪 Step 5: Build and Run

```bash
cd ~/voice_ws
colcon build
source install/setup.bash
```

---

## 🐢 Step 6: Launch Everything

### In **Terminal 1** – Launch turtlesim:

```bash
ros2 run turtlesim turtlesim_node
```

### In **Terminal 2** – Run the voice node:

```bash
source ~/voice_ws/install/setup.bash
ros2 run voice_controller voice_node
```

Now speak into your mic: “forward”, “left”, “stop” etc., and see the turtle respond 🐢

---

## 🧰 Useful Terminal Commands

| Command                            | Description                       |
| ---------------------------------- | --------------------------------- |
| `ros2 topic echo /turtle1/cmd_vel` | View movement messages            |
| `ros2 node list`                   | View running ROS nodes            |
| `arecord --list-devices`           | Check if mic is detected          |
| `python3 -m speech_recognition`    | Test voice recognition standalone |

---

## ✅ Extra Tips

* Tune speech recognition timeout and sensitivity.
* Add feedback using LEDs or turtle color changes.
* Test offline with [Vosk](https://alphacephei.com/vosk/) for offline voice recognition.

---

