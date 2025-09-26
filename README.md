
# Walle ROS2 Robot

Walle is a semi-humanoid intelligent robot powered by **ROS 2**, designed to interact with humans using **voice commands** and AI-driven decision-making. It integrates multiple actuators, including head and hand servos and wheel motors, to perform complex motions. The robot uses **OpenAI's Whisper API** for voice-to-text and **ChatGPT API** for intelligent decision making, providing real-time interactive capabilities.

---

## Features

- **Head Control:** 2 servo actuators for head movements.
- **Wheel/Leg Control:** 3 DC motors for wheel/leg movements.
- **Hand Control:** 4 servo actuators for 2 hands.
- **Voice Interaction:** Real-time voice commands using OpenAI Whisper.
- **Intelligent Responses:** ChatGPT API for AI decision-making.
- **Modular ROS 2 Architecture:** Each component runs as an independent ROS 2 node.

---

## ROS 2 Nodes

| Node | Function |
|------|---------|
| `main_controller_1.py` | Controls head servos and wheel motors. Runs on the first Raspberry Pi. |
| `main_controller_2.py` | Controls hand servos. Runs on the second Raspberry Pi. |
| `voice_listener.py` | Listens to microphone input, converts voice to text using OpenAI Whisper API. |
| `chatgpt_processor.py` | Sends voice text to ChatGPT API and retrieves intelligent commands. |

---

## Hardware Requirements

- 2 × Raspberry Pi 4
- 2 × Servo motors for head
- 4 × Servo motors for hands
- 3 × DC motors for wheels/legs
- Motor drivers compatible with your actuators
- USB microphone
- Speaker (optional)

---

## Software Requirements

- Ubuntu 22.04 (or compatible)
- ROS 2 Humble or later
- Python 3.10+
- Python virtual environment (recommended)
- Python packages:
  - `rclpy`
  - `std_msgs`
  - `numpy<2`
  - `scipy`
  - `sounddevice`
  - `openai`

---

## Setup Instructions

### 1. Clone the Repository

```bash
cd ~/new_ws/src
git clone https://github.com/georgeyaccoup/Walle_ROS2.git
cd Walle_ROS2
````

---

### 2. Set Up Python Virtual Environment

```bash
python3 -m venv .venv
source .venv/bin/activate
```

---

### 3. Install Python Dependencies

```bash
pip install --upgrade pip
pip install "numpy<2" scipy sounddevice rclpy std_msgs openai
```

---

### 4. Set OpenAI API Key

**Important:** Do **not** hardcode your API key. Use environment variables:

```bash
export OPENAI_API_KEY="sk-XXXXXXXXXXXXXXXXXXXX"
```

You can add this line to `~/.bashrc` or `~/.zshrc` to load automatically.

---

### 5. Build the ROS 2 Workspace

```bash
cd ~/new_ws
colcon build
source install/setup.bash
```

---

### 6. Running the Robot Nodes

* **Voice Listener Node:**

```bash
ros2 run walle_ROS2 voice_listener
```

* **ChatGPT Processor Node:**

```bash
ros2 run walle_ROS2 chatgpt_processor
```

* **Main Controller 1 (head + wheels):**

```bash
ros2 run walle_ROS2 main_controller_1
```

* **Main Controller 2 (hands):**

```bash
ros2 run walle_ROS2 main_controller_2
```

> Each node can run on separate Raspberry Pis, communicating via ROS 2 topics.

---

## Recommended Workflow

1. Start both Raspberry Pis.
2. Run `main_controller_1` on Pi 1 and `main_controller_2` on Pi 2.
3. Start `voice_listener.py` to capture voice commands.
4. Run `chatgpt_processor.py` to process commands and send them to controllers.
5. Observe robot responding to commands with motion and AI-driven behavior.

---

## Security Tips

* Never commit your OpenAI API key to GitHub.
* Use `.gitignore` to exclude `.venv/`, `build/`, `install/`, `log/`, and `.env` files.
* Always set your API key via environment variable.

---

## GitHub .gitignore Example

```
# ROS 2 build and install
build/
install/
log/

# Python virtual environment
.venv/

# Secrets
.env
```

---

## References

* [ROS 2 Documentation](https://docs.ros.org/)
* [OpenAI API](https://platform.openai.com/)
* [ChatGPT API](https://platform.openai.com/docs/guides/chat)


