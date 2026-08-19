# Open-Limb Robot

Open, non-anthropomorphic robot that orbits around a compliant cylindrical surface — such as a human limb or a pipe — with a three-contact grasp: a central traction module with two in-line driven wheels, and two lateral spring-loaded arms with distal wheels.

This repository accompanies the paper:

> L. M. Tobar-Subia-Contento, J. A. Cabrera, A. Mandow, and J. M. Gómez-de-Gabriel, "On-Limb Orbiting Robot: Proprioceptive Diameter Estimation and Orthogonal Grip–Orbit Control," submitted to *Biomimetics* (MDPI), 2026.

The central contribution is an actuation-space decomposition in which the two lateral wheel torques, expressed in a common-mode/differential basis, simultaneously drive the orbital motion and regulate the central normal force — orthogonally and without a dedicated force mechanism. The same compliant arms yield a closed-form estimate of the cylinder diameter from proprioception alone.

## Repository structure

```
Open-limb-robot/
├── CAD/                      # SolidWorks design files and STL/STEP exports
├── experiments/              # Data and processing scripts from the prototype tests
├── simulation/
│   ├── lib/                  # Shared geometry functions
│   └── scripts/              # Runnable analysis scripts
├── On-Limb Orbiting Robot.mp4
├── LICENSE
└── README.md
```

### `CAD/`
SolidWorks design files for the Mobile Platform (MP), the central traction module and the lateral grasping arms, plus exported files ready for 3D printing in PETG. Mechanical design parameters are listed in Table 1 of the paper.

### `simulation/`

`lib/` contains the shared geometry functions used by all scripts:

| Function | Implements |
|---|---|
| `contactDepth.m` | Averaged central contact depth, Eq. (1) |
| `radiusEstimator.m` | Closed-form cylinder-radius estimator, Eq. (3) |
| `lateralContactAngle.m` | Lateral-contact angle γ, Eq. (4) |
| `solveArmAngle.m` | Numerical inversion of Eq. (3) for a known radius |

`scripts/` contains the runnable analyses:

| Script | Reproduces |
|---|---|
| `static_force_model_01.m` | Static force model and actuation-space decomposition (Sections 4–5); numerically verifies that the common-mode/differential basis diagonalises the force map, Eq. (20) |
| `orbital_simulation_02.m` | Closed-loop orbital simulation over a full revolution (Section 8.3, Figure 9, Table 2) |
| `radius_estimator_sensitivity_03.m` | Radius-estimator sensitivity analysis (Sections 4.3 and 8.2, Figure 8) |

Scripts add `../lib` to the path automatically. Run them from the `simulation/scripts/` folder.

**Reproducing the feedforward-only vs. feedforward+PI comparison (Table 2).** In `orbital_simulation_02.m`, set the `has_force_sensor` flag to `false` to disable the PI correction and to `true` for the nominal configuration reported in Figure 9.

### `experiments/`
Data and processing scripts from the preliminary prototype tests (Section 8.4): spring characteristic and contact-force calibration (Figure 7), grasp-retention test across orbital orientations (Figure 10), and open-loop orbital-rotation tests for different assumed friction coefficients (Figure 11).

## Hardware

- Actuation: DYNAMIXEL XC330-T288-T servo (grasping actuator), DC motors (drive wheels)
- Sensing: rotational springs with angular sensors (central compliant contact), servo encoder (arm angle), IMU (orbital angle)
- Chassis: 3D-printed PETG

## Requirements

MATLAB (developed with R2023b or later; no toolboxes beyond base MATLAB are required). The scripts also run under GNU Octave.

For CAD source files, SolidWorks; the exported STL/STEP files can be opened with any standard CAD viewer.

## Citation

If you use this code or design in your research, please cite the paper above. A full citation with DOI will be added once the manuscript is published.

## License

Released under the [MIT License](LICENSE).

## Contact

Jesús M. Gómez-de-Gabriel — jesus.gomez@uma.es
Institute for Mechatronics Engineering and Cyber-Physical Systems (IMECH.UMA), Universidad de Málaga
