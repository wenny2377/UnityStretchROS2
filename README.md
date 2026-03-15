# UnityStretchROS2
### Digital Twin for Stretch Robot: Real-Time Virtual-Physical Synchronization via ROS2

> A cross-site digital twin system that synchronizes a physical Hello Robot Stretch 3 at NYCU (Hsinchu) with a Unity virtual environment at NCKU (Tainan) in real time — enabling remote robot state visualization and bidirectional control over ROS2.

[![Demo Video](https://img.shields.io/badge/Demo-YouTube-red?style=flat-square&logo=youtube)](https://youtu.be/qazeR3iceF4)

![Unity](https://img.shields.io/badge/Unity-2022-000000?style=flat-square&logo=unity&logoColor=white)
![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?style=flat-square&logo=ros&logoColor=white)
![C#](https://img.shields.io/badge/C%23-Scripts-239120?style=flat-square&logo=csharp&logoColor=white)
![ROS Bridge](https://img.shields.io/badge/Bridge-rosbridge__suite-0467DF?style=flat-square)

---

## Demo

[![Cross-site Virtual-Physical Synchronization Demo](https://img.youtube.com/vi/qazeR3iceF4/maxresdefault.jpg)](https://youtu.be/qazeR3iceF4)

*NYCU operator controls the physical Stretch robot → joint states stream over ROS2 → Unity scene at NCKU updates in real time.*

---

## Overview

This project establishes a **real-time digital twin** of the Hello Robot Stretch 3, bridging two geographically separate sites:

| Site | Role |
|------|------|
| **NYCU (Hsinchu)** | Physical Stretch robot — operator controls the real robot |
| **NCKU (Tainan)** | Unity virtual environment — mirrors robot state in real time |

The virtual scene was built from scratch in Unity based on the official Stretch ROS2 repository, with URDF import and joint state synchronization via rosbridge. Cross-site network infrastructure was established in collaboration with a team member at NYCU.

---

## System Architecture

```
┌──────────────────────────────┐         ┌──────────────────────────────┐
│       NYCU (Hsinchu)         │         │       NCKU (Tainan)          │
│                              │         │                              │
│  Physical Stretch Robot      │         │  Unity Virtual Scene         │
│  Hello Robot Stretch 3       │         │  Stretch 3 Digital Twin      │
│                              │         │                              │
│  ROS2 Humble                 │         │  ROS-TCP / rosbridge_suite   │
│  ├─ /joint_states            │────────▶│  ├─ Joint state subscriber   │
│  ├─ /cmd_vel                 │         │  ├─ Transform sync           │
│  └─ /tf                      │         │  └─ Real-time visualization  │
│                              │         │                              │
│  Operator controls robot     │         │  Remote observer / monitor   │
└──────────────────────────────┘         └──────────────────────────────┘
                    Cross-site network (VPN tunnel)
```

---

## Key Features

- **Cross-site synchronization** — physical robot at NYCU, virtual twin at NCKU, connected over a cross-city network
- **Real-time joint state mirroring** — Stretch arm, lift, and base movements reflected in Unity with minimal latency
- **URDF-based virtual model** — Stretch 3 model imported and configured from the official Hello Robot ROS2 repository
- **rosbridge integration** — WebSocket-based ROS2 ↔ Unity communication via rosbridge_suite

---

## Tech Stack

| Layer | Technology |
|-------|-----------|
| Virtual environment | Unity 2022, C# |
| Robot middleware | ROS2 Humble |
| Unity–ROS bridge | rosbridge_suite (WebSocket) |
| Robot platform | Hello Robot Stretch 3 |
| Reference | [hello-robot/stretch_ros2](https://github.com/hello-robot/stretch_ros2) |
| Network | Cross-site VPN (configured by NYCU collaborator) |

---

## Context

This project was developed as part of the **Digital Twin Infrastructure & Autonomous Navigation** research project (May 2024 – Jun 2025), in collaboration with NYCU research teams. The primary goal was to establish a high-fidelity virtual-physical synchronization framework for senior research teams working on home service robot navigation.

The experience directly informed the Unity frontend architecture used in the [vlm-rag-service-robot](https://github.com/wenny2377/vlm-rag-service-robot) thesis project.

---

## Author

**Hui-Hsin Huang**
M.S. Candidate, Computer Science — National Cheng Kung University
Email: wenny2377@gmail.com
