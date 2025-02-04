# Integrating Reinforcement Learning and Virtual Fixtures for Safer Automatic Robotic Surgery

**"RL-VF: Reinforcement Learning Framework for Static and Dynamic Obstacle Avoidance in Robotic Surgery"**.

![Figure 1: abstract graph](abstract.png)

> *Figure 1: Conceptual Diagram of the RL-VF Framework*


## Repository Structure
The code is divided into three main components:

1. **Vision Module**: Responsible for detecting targets and constraints using computer vision techniques.
   - Includes image rectification, segmentation, stereo matching, and point cloud generation.
   - Outputs a sparse 3D point cloud representing safety constraints (e.g., blood vessels) and target positions (e.g., lymph nodes).

2. **RL+VF Algorithm**: Implements reinforcement learning with virtual fixtures to ensure compliance with hard safety constraints.
   - Uses VF for real-time action correction when the agent approaches constraint boundaries.
   - Features safety experience reshaping (SER) for optimizing safe policy training.
   - Supports both static and dynamic constraints.

3. **Simulation Environment**: A Unity3D-based simulation environment for training and testing the framework.
   - Includes tasks such as 2D robot planning, lymph node removal, and human-robot collaboration scenarios.
   - Facilitates Sim-to-Real transfer with domain randomization.

---

## Introduction
The RL-VF framework introduces reinforcement learning with hard safety constraints tailored for robotic manipulator agents. Key features include:
- Integration of static and dynamic obstacle avoidance.
- Applicability in surgical scenarios with constraints modeled by virtual fixtures.
- Comparisons with baseline methods validating improved safety and performance.

---

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/fanke-polimi/RL-VF.git
