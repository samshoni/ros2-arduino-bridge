# ROS2-Arduino-Bridge

A simple yet powerful bridge between **ROS 2** and **Arduino**.  
This project demonstrates how to control an Arduino-connected LED from a ROS 2 node over serial communication.  
It serves as a foundation for integrating embedded hardware (sensors, actuators) with ROS 2 robotics middleware.  

---

## 🚀 Features
- ROS 2 node written in Python (`led_logic.py`)  
- Serial communication with Arduino (`/dev/ttyUSB0`)  
- Control a physical LED using ROS 2 messages  
- Modular design (easily extendable for sensors, motors, etc.)  

---

## 🛠 Hardware Setup
- Arduino Mega 2560  
- LED + 220Ω resistor  
- Breadboard + jumper wires  
- USB cable (Arduino ↔ PC)  

**Wiring**  
- LED anode → Arduino Pin 13  
- LED cathode → GND  

---

## 💻 Software Setup

### 1. Arduino
Upload the Arduino sketch (`led.ino`) from the `arduino_code/` folder:

```cpp
void setup() {
  pinMode(13, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') digitalWrite(13, HIGH);
    if (c == '0') digitalWrite(13, LOW);
  }
}

2. ROS 2 Workspace

cd ~/ros2_ws
colcon build
source install/setup.bash

3. Run the ROS 2 Node

ros2 run my_arduino_pkg led_logic

📂 Repository Structure

ros2-arduino-bridge/
├── my_arduino_pkg/        # ROS 2 package
│   ├── package.xml
│   ├── setup.py
│   └── my_arduino_pkg/led_logic.py
├── arduino_code/          # Arduino sketches
│   └── led.ino
└── README.md

🔎 Example Output

[INFO] [led_logic]: Sending 1 (LED ON)
[INFO] [led_logic]: Sending 0 (LED OFF)

And the LED blinks physically on the Arduino board ✨
🌟 Future Extensions

    Add a buzzer controlled by ROS 2 topics

    Publish ultrasonic sensor distance readings to ROS 2

    Expand into motor driver control for a mobile robot

👤 Author

Sam Shoni

    GitHub: samshoni

LinkedIn: Sam Shoni

