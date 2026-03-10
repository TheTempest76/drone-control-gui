# DroneLink – Network-Controlled Drone Interface

DroneLink is a modular control system that allows a **mobile phone or laptop to control a drone through an onboard computer**. The onboard computer acts as a **command bridge** between the user interface and the flight controller, translating user inputs into MAVLink commands.

The system is designed to work with autopilots such as **ArduPilot** and **PX4**, enabling flexible control architectures for research drones, AI-assisted navigation, and teleoperation.

---

### Core Concept

The control architecture separates the **user interface**, **network communication**, and **flight control** layers.

```

Phone / Laptop (GUI)
│
WiFi / Network
│
Onboard Computer (Jetson / SBC)
│
MAVLink
│
Flight Controller
│
Motors

````

The onboard computer acts as a **mission computer** responsible for command routing, telemetry broadcasting, and safety enforcement.

---

# Features

## 1. Cross-Platform Control Interface

The system supports control from multiple device types.

Supported control clients:

- Mobile phones (Android / iOS browsers)
- Laptops and desktops
- Tablets
- Custom applications

Possible interface types:

- Web-based GUI
- Native mobile app
- Desktop application

User interface components may include:

- Virtual joystick
- Game-style keyboard control
- Button-based commands
- Telemetry dashboard

---

## 2. Game-Style Drone Control

DroneLink enables intuitive control similar to video games.

Possible control mappings:

| Input | Action |
|-----|------|
| W / S | Move forward / backward |
| A / D | Move left / right |
| Mouse | Rotate yaw |
| Space | Increase altitude |
| Ctrl | Decrease altitude |

Control modes:

- Velocity control
- Position control
- Manual stick override
- Autonomous assist

---

## 3. MAVLink Command Translation

The onboard computer translates incoming control commands into MAVLink messages for the flight controller.

Supported MAVLink command types include:

- MANUAL_CONTROL
- SET_POSITION_TARGET_LOCAL_NED
- COMMAND_LONG
- SET_MODE
- ARM / DISARM

Communication is implemented using:

- **pymavlink** for direct MAVLink communication
- MAVLink over UART, UDP, or TCP

---

## 4. Network Control Server

The onboard computer runs a control server that receives commands from client devices.

Supported communication protocols:

- WebSockets
- HTTP API
- UDP
- TCP

Example control message:

```json
{
  "vx": 1.0,
  "vy": 0.0,
  "vz": -0.2,
  "yaw_rate": 0.3
}
````

The server converts incoming messages into MAVLink commands sent to the flight controller.

---

## 5. Real-Time Telemetry Streaming

Telemetry from the flight controller is broadcast back to connected clients.

Telemetry data may include:

* GPS coordinates
* Altitude
* Battery level
* Velocity
* Attitude (roll, pitch, yaw)
* Flight mode
* Armed status

Telemetry distribution methods:

* WebSocket streams
* HTTP polling
* MAVLink forwarding

---

## 6. Multi-Client Support

Multiple control and monitoring clients can connect simultaneously.

Examples:

```
Phone Controller
Laptop Monitoring Dashboard
Ground Control Station
```

Telemetry can be broadcast to multiple endpoints while control authority remains limited to a single operator.

---

## 7. AI Integration Layer

The onboard computer can integrate AI modules that influence or assist flight behavior.

Potential AI features include:

* Object detection
* Person tracking
* Obstacle avoidance
* Target following
* Autonomous search missions

AI modules may override or modify user commands based on safety constraints.

---

## 8. Camera Streaming

Live video feed from onboard cameras can be streamed to clients.

Recommended protocols:

* RTSP
* WebRTC
* GStreamer pipelines

Video streams may include overlays such as:

* object detection bounding boxes
* flight data
* targeting markers

---

## 9. Safety and Fail-Safe System

The onboard computer enforces safety policies to prevent unsafe commands.

Safety features include:

* command rate limiting
* connection timeout detection
* automatic hover on disconnect
* battery monitoring
* geofencing support
* emergency stop command

Example logic:

```
If client connection lost
→ drone enters hover or RTL mode
```

---

## 10. Ground Control Station Compatibility

DroneLink can operate alongside traditional ground control software.

Compatible tools include:

* Mission Planner
* QGroundControl
* MAVProxy

Telemetry can be forwarded to these tools for monitoring and debugging.

---

## 11. Modular Software Architecture

The system is designed as a modular stack.

```
DroneLink Stack

├── Control Server
├── MAVLink Interface
├── Telemetry Broadcaster
├── Command Parser
├── Safety Manager
├── AI Modules
└── User Interface
```

Each module can be extended or replaced depending on project requirements.

---

# Hardware Requirements

Typical hardware configuration:

Onboard Computer

* NVIDIA Jetson
* Raspberry Pi
* Other Linux SBC

Flight Controller

* Pixhawk-compatible autopilot
* ArduPilot or PX4 firmware

Networking

* WiFi
* Ethernet
* LTE / 5G modem

Camera (optional)

* USB camera
* CSI camera

---

# Software Stack

Possible software components:

Backend

* Python
* FastAPI
* WebSocket server
* pymavlink

Frontend

* Web-based UI
* React / Vue
* Mobile-friendly interface

Video

* GStreamer
* WebRTC
* RTSP

---

# Development Goals

The project aims to provide:

* intuitive drone control
* flexible network-based architecture
* AI-assisted flight capabilities
* modular extensibility
* compatibility with existing MAVLink ecosystems

---

# Future Features

Planned or possible future extensions include:

* mission planning interface
* swarm drone control
* VR drone piloting
* AI mission generation
* collaborative multi-operator control
* cloud telemetry dashboards

---

# License

MIT License

```
```
