# Particle Filter Localisation (MCL) with ROS 2

This repository contains an implementation of the Particle Filter (Monte Carlo Localisation, MCL) algorithm for robot localisation. It is designed to work with the **socspioneer** package and includes features such as motion updates, sensor updates, resampling, and handling the "kidnapped robot" problem.

---

## Getting Started

### Clone this repository:
```bash
git clone https://github.com/catherine2103/pf_localisation.git
cd pf_localisation
```

### Setup

To use the custom Particle Filter implementation:

1. Replace the `pf.py` file in the `socspioneer` package with the provided file from this repository:
   ```bash
   cp pf_localisation/pf.py <path-to-socspioneer-package>/src/

2. Rebuild the socspioneer package:

  ```bash
  cd <path-to-socspioneer-package>
  colcon build
  source install/setup.bash
