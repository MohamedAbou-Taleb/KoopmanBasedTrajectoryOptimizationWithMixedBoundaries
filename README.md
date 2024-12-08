# Koopman Based Trajectory Optimization with Mixed Boundaries

This repository provides the code used for the examples in the paper **"Koopman Based Trajectory Optimization with Mixed Boundaries"** by Mohamed Abou-Taleb, Maximilian Raff, Kathrin Flaßkamp, C. David Remy (https://arxiv.org/abs/2412.03195). The repository includes three examples of periodic trajectory optimization:

1. **The Harmonic Oscillator**
2. **The Pendulum**
3. **The Compass-Gait Walker**

Each example demonstrates how the presented method can be applied to trajectory optimization problems. The repository also includes a visualization for the Compass-Gait Walker in the form of a `.GIF` animation.

---
## Examples

### 1. **Harmonic Oscillator**
   - **Requirements**: MATLAB's Global Optimization Toolbox is needed to use the `multisearch` command.
   - **Main Script**: `bilevel_Optimization_HO.m`

   Run this script to execute the trajectory optimization for the Harmonic Oscillator.

---

### 2. **Pendulum**
   - This example includes two approaches to trajectory optimization:
     1. **Hard Constraints**: `UpperLevel_pendulum_hard.m`
     2. **Soft Constraints**: `UpperLevel_pendulum_soft.m`

   Choose one of the above scripts based on the type of constraint formulation you want to explore.

---

### 3. **Compass-Gait Walker**
   - **Main Script**: `UpperLevel_CGW.m`
   - This example generates a `.GIF` animation of the compass-gait walker. The animation is saved automatically upon running the script.
