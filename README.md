# Real-Time Low-Level Control System for a 12-DoF Quadruped Robot

> **Abstract**: This project provides a modular, real‑time embedded control firmware for a 12 Degree-of-Freedom quadruped robot based on the `STM32F767IGT6` (Cortex‑M7 @216 MHz with single‑precision FPU). It integrates multi-sensor fusion state estimation, Virtual Model Control (VMC), dynamic gait generation, hierarchical scheduling, and structured finite state management. The system is intended as a stable, efficient, and extensible Hardware‑in‑the‑Loop (HIL) research platform for legged locomotion, balance control, and algorithm prototyping.

---
## 1. System Architecture

### 1.1 Hardware Topology
The embedded platform uses a centralized controller with a star-style interconnect to sensing, actuation, and supervisory interfaces over high‑speed buses.

- **Central Control Unit**  
  - `STM32F767IGT6 @ 216 MHz`
- **Sensors & Inputs**  
  - IMU (e.g. MPU6050 / WT901) via UART / SPI  
  - Internal joint position / torque feedback (inside smart actuators)  
  - Remote controller receiver via UART
- **Actuation Subsystem**  
  - 12 smart joint actuators / motor drivers over up to 3 parallel CAN buses
- **Debug & Monitoring**  
  - SWD/JTAG (ST-LINK / J-Link) for flashing & debugging  
  - UART (USB–Serial bridge) for telemetry / logging

### 1.2 Software Layering
A layered, modular architecture promotes high cohesion and low coupling:

| Layer | Purpose | Representative Contents |
|-------|---------|-------------------------|
| Hardware Abstraction (HAL) | Peripheral configuration & low-level access | STM32Cube HAL, clock, GPIO, timers, CAN, UART |
| Driver Layer | Protocol & device data normalization | CAN motor interface, IMU parser, UART RX/TX handlers |
| Core Services | Generic runtime infrastructure | Real‑time scheduler, timing utilities, finite state machine |
| Algorithm Layer | Core control & estimation logic | EKF state estimator, gait planner, kinematics, VMC force mapping |
| Application Layer | Behavior orchestration & command integration | Mode switching, teleoperation mapping, safety logic |

---
## 2. Core Control Strategies

### 2.1 State Estimation
- **Data Sources**: Tri‑axial angular velocity & linear acceleration (IMU), inferred foot kinematics, support phase zero‑velocity constraints.  
- **Filter Model**: Extended Kalman Filter (EKF).  
  - *State Vector (typical)*: `[roll, pitch, yaw, vx, vy, vz, …]`  
  - *Prediction*: Integrate angular velocity & propagate linear velocity/orientation.  
  - *Update*: Apply Zero‑Velocity Update (ZUPT) constraints for feet in firm contact to suppress drift.  
- **Refinements**:  
  - Delayed activation after initial static settling to reduce IMU bias influence.  
  - Dynamic measurement covariance adaptation based on motion classification (static vs dynamic).  

### 2.2 Virtual Model Control (VMC)
VMC abstracts the robot body as a virtual mass requiring regulation of wrench (forces/torques) at the center of mass (CoM). The pipeline:
1. **Virtual Wrench Computation**: PD (optionally + feedforward) on posture & CoM height yields desired virtual force `F_virtual` and torque `T_virtual` (e.g. vertical support, roll/pitch stabilization).  
2. **Ground Reaction Force (GRF) Allocation**: Distribute net desired wrench across the active support legs. Over‑determined allocation can use pseudo‑inverse or quadratic programming if constraints (friction cones, normal force bounds) are enforced.  
3. **Joint Torque Mapping**: For each support leg `i`, map foot force `F_i` into joint torques via Jacobian transpose: `τ_i = J_i^T F_i`.  
4. **Swing Leg Control**: Swing phase legs track Cartesian or joint‑space trajectories (e.g. Bézier / cycloidal foot paths) with smooth lift‑off / touch‑down timing.

### 2.3 Dynamic Gait Planning
- **Phase Representation**: Normalize gait cycle to `[0,1]`; each leg maintains an independent or phase‑offset oscillator to define support vs swing intervals.  
- **Trajectory Generation**:  
  - Swing: Smooth polynomial / Bézier / cycloidal arcs for clearance & impact mitigation.  
  - Support: Foot velocity opposed to body motion to stabilize CoM tracking while minimizing slip.  
- **Online Parameterization**: Step length, step height, duty factor, and cycle period are runtime adjustable for speed scaling or terrain adaptation.  
- **Extension Hooks**: Adaptive foothold selection, terrain classification integration, predictive timing modulation.

---
## 3. Technical Specifications
| Parameter | Specification | Notes |
|-----------|--------------|-------|
| MCU Core | 216 MHz Cortex‑M7 / ~462 DMIPS | Single‑precision FPU enabled |
| Control Loop Period | 1 kHz (primary) | Estimation + VMC force distribution |
| CAN Bus Rate | 1 Mbps (× up to 3 buses) | Parallel segmentation reduces arbitration latency |
| Attitude Estimation Accuracy | <0.5° static / <2° dynamic | Dependent on calibration & mechanical isolation |
| Joint Control Modes | Torque & Position | Mode switches by phase (support/swing) |
| Supported Gaits (baseline) | Stand, Trot, Crawl | Framework extensible (pace, bound, trot-run) |
| On‑board Memory | 512 KB Flash / 320 KB SRAM | Typical static usage leaves headroom for extensions |