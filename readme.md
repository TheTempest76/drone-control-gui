# DroneLink – Network-Controlled Drone Interface

DroneLink is a modular control system that allows a **mobile phone or laptop to control a drone through an onboard computer**. The onboard computer acts as a **command bridge** between the user interface and the flight controller, translating user inputs into MAVLink commands.

The system is designed to work with autopilots such as **ArduPilot** and **PX4**, enabling flexible control architectures for research drones, AI-assisted navigation, and teleoperation.

---

### Core Concept

The control architecture separates the **user interface**, **network communication**, and **flight control** layers.


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