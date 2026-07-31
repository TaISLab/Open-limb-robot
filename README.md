# Open-Limb Robot

Open, non-anthropomorphic robot that orbits around a compliant cylindrical surface — such as a human limb or a pipe — with a three-contact grasp: a central traction module with two in-line driven wheels, and two lateral spring-loaded arms with distal wheels.

This repository accompanies the paper:

> L. M. Tobar-Subia-Contento, J. A. Cabrera, A. Mandow, and J. M. Gómez-de-Gabriel, "On-Limb Orbiting Robot: Proprioceptive Diameter Estimation and Orthogonal Grip–Orbit Control," submitted to *Biomimetics* (MDPI), 2026.

The central contribution is an actuation-space decomposition in which the two lateral wheel torques, expressed in a common-mode/differential basis, simultaneously drive the orbital motion and regulate the central normal force — orthogonally and without a dedicated force mechanism. The same compliant arms yield a closed-form estimate of the cylinder diameter from proprioception alone.

## Repository structure

```
Open-limb-robot/
├── LICENSE
├── README.md
├── cad/                # CAD design files (SolidWorks assemblies/parts, STL/STEP exports for 3D printing)
├── simulation/          # MATLAB code for the closed-loop orbital simulation (control architecture, static force model, radius estimator)
└── experiments/          # Data, scripts and figures from the physical prototype tests (grasp retention, orbital rotation, spring calibration)
```

### `cad/`
SolidWorks design files for the Mobile Platform (MP), the central traction module and the lateral grasping arms, plus exported STL/STEP files ready for 3D printing in PETG. Mechanical design parameters are listed in Table 1 of the paper.

### `simulation/`
MATLAB scripts implementing:
- the static force model and actuation-space decomposition (Sections 4–5),
- the closed-loop orbital simulation over a full revolution (Section 8.3, Figure 9),
- the proprioceptive cylinder-radius and contact-geometry estimator with its sensitivity analysis (Sections 4.3 and 8.2, Figure 8).

### `experiments/`
Data and processing scripts from the preliminary prototype tests (Section 8.4):
- spring characteristic and contact-force calibration (Figure 7),
- grasp-retention test across orbital orientations (Figure 10),
- open-loop orbital-rotation tests for different assumed friction coefficients (Figure 11).

## Hardware

- Actuation: DYNAMIXEL XC330-T288-T servo (grasping actuator), DC motors (drive wheels)
- Sensing: rotational springs with angular sensors (central compliant contact), servo encoder (arm angle)
- Chassis: 3D-printed PETG

## Requirements

- MATLAB (developed and tested with MATLAB R2023b or later; no toolboxes beyond base MATLAB required unless noted in individual scripts)
- SolidWorks (for editing CAD source files; STL/STEP exports can be viewed with any standard CAD viewer)

## Citation

If you use this code or design in your research, please cite the paper above. A full citation (with DOI) will be added here once the manuscript is published.

## License

This project is released under the [MIT License](LICENSE).

## Contact

Jesús M. Gómez-de-Gabriel — jesus.gomez@uma.es
Institute for Mechatronics Engineering and Cyber-Physical Systems (IMECH.UMA), Universidad de Málaga
