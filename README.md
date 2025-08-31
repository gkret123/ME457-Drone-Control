# ME457 Drone Control Simulator

## Overview

This repository contains the complete codebase, simulations, and documentation for the **ME-457 Drone Control Class Project**, developed by **Gabriel Kret** and **Adin Sacho-Tanzer**.

The simulator was originally developed for the Aerosonde UAV and later adapted to model the flight dynamics of a **Cessna 172**. It is based on the structure and methodology of [Beard's MAVSim](https://github.com/byu-magicc/mavsim_public) from BYU MAGICC.

---

## Key Repository Structure Components

```
ME457-Drone-Control/
├── Kret_Sacho-Tanzer_Simulator_Quaternion/
│   └── Aerosonde simulator (quaternion EOMs, PID, Kalman Filter)
│
├── Final_Project_Kret_Sacho-Tanzer_Simulator_Quaternion/
│   └── Cessna 172 simulator (gas engine model, retuned control systems)
│
├── Documentation/
│   └── Technical reports, presentations, references
```

---

## Key Features

- **Quaternion-based equations of motion** for stable 3D rotation modeling
- **Runge-Kutta 4 (RK4)** integration for precise time-domain simulation
- **Linear aerodynamic modeling** with configurable wind effects
- **Successive Loop Closure (SLC)** PID tuning architecture
- **Kalman Filter** for state estimation under noisy sensor conditions
- Support for **electric and gas propulsion systems**

---

## Getting Started

### Clone the repository:
```bash
git clone https://github.com/gkret123/ME457-Drone-Control.git
```

### Navigate to a simulator directory:
```bash
cd ME457-Drone-Control/Kret_Sacho-Tanzer_Simulator_Quaternion
# or
cd ME457-Drone-Control/Final_Project_Kret_Sacho-Tanzer_Simulator_Quaternion
```

### Install requirements:
Dependencies are listed in the respective simulator directories. Typical requirements:
```bash
pip install numpy scipy matplotlib
```

---

## Requirements

- Python 3.x
- NumPy
- SciPy
- Matplotlib

---

## Authors

- **Gabriel Kret:**  gabriel.kret@cooper.edu
- **Adin Sacho-Tanzer:** adin.sachotanzer@cooper.edu

---

## License

This project is open-source. See the `LICENSE` file for usage details.

---

## Contact

For questions, suggestions, or issues, please open a GitHub Issue in this repository.
