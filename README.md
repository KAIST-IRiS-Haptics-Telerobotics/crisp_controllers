<img src="media/crisp_logo.webp" alt="CRISP Controllers Logo"/>

<a href="https://github.com/utiasDSL/crisp_controllers/actions/workflows/ros2_ci.yml"><img src="https://github.com/utiasDSL/crisp_controllers/actions/workflows/ros2_ci.yml/badge.svg"/></a>
<a href="https://danielsanjosepro.github.io/crisp_controllers/"><img alt="Static Badge" src="https://img.shields.io/badge/docs-passing-blue?style=flat&link=https%3A%2F%2Fdanielsanjosepro.github.io%2Fcrisp_controllers%2F"></a>
<a href="https://ieeexplore.ieee.org/document/11505755"><img alt="Static Badge" src="https://img.shields.io/badge/IEEE-RAP%202026-blue?style=flat&link=https%3A%2F%2Fieeexplore.ieee.org%2Fabstract%2Fdocument%2F11505755"></a>

CRISP is a collection of real-time, C++ controllers for compliant torque-based control for manipulators compatible with `ros2_control`, including **Cartesian Impedance Control** and **Operational Space Control**. Developed for deploying high-level learning-based policies (VLA, Diffusion, ...) and teleoperation on your manipulator. It is robot-agnostic and compatible with any manipulator offering and effort interface. Check the [project website](https://utiasdsl.github.io/crisp_controllers/) for guides, getting started, demos and more! 

## IRiS private fork

This repository is the IRiS lab's private, API-compatible fork of
[`utiasDSL/crisp_controllers`](https://github.com/utiasDSL/crisp_controllers). The ROS package and
plugin names intentionally remain `crisp_controllers`, so existing robot descriptions and
controller YAML files continue to work.

Additional features maintained by IRiS over upstream are:

- leader-side active assistance from a compensated local F/T measurement, with an explicit sensor
  sign, independent force/torque scaling, low-pass filtering, activation-local zero capture, and
  stale-measurement fail-safe behavior;
- one-sample dynamics compensation in wrench space, with configurable gain and filtering;
- tests and a staged hardware evaluation protocol for using both paths independently or together
  in bilateral FR3 teleoperation.

The implementation and commissioning procedure are documented in
[`docs/inertia_shaping.md`](docs/inertia_shaping.md). Features unique to this fork are opt-in and
disabled by default unless an IRiS controller profile enables them.


> [!NOTE]
> To reduce maintenance overhead, all ROS 2 distributions are supported from a single `main` branch. The code uses compile-time macros to handle version-specific differences.

## Features
- 🐍 **Python interface** to move your ROS2 robot around without having to think about topics, spinning, and more ROS2 concepts but without loosing the powerful ROS2 API. Check [crisp_py](https://github.com/utiasDSL/crisp_py) for more information and examples.
- 🔁 **Gymnasium environment** with utilities to deploy learning-based policies and record trajectories in LeRobotFormat. Check [crisp_gym](https://github.com/utiasDSL/crisp_gym).
- ❓ **Demos** showcasing how to use the controller with FR3 of Franka Emika in single and bimanual setup. Check the [crisp_controller_demos](https://github.com/utiasDSL/crisp_controllers_demos).
- ⚙️ Dynamically and highly parametrizable: powered by the [`generate_parameter_library`](https://github.com/PickNikRobotics/generate_parameter_library) you can modify stiffness and more during operation.  
- 🤖 Operational Space Controller as well as Cartesian Impedance Controller for torque-based control.  
- 🖐️ Optional, filtered leader-side measured-wrench assistance and one-sample dynamics compensation, with stale-data fallback and an [evaluation protocol](docs/inertia_shaping.md).
- 🚫 No MoveIt or complicated path-planning, just a simple C++ `ros2_controller`. Ready to use.  


______

### For Contributors

##### Updating the website

We use [zensical](https://www.zensical.org/) to generate the website from markdown. You can modify it within `docs/` in particular the `index.md`.
You can run the website locally with:
```bash
pixi run zensical serve
```
The website is automatically generated and deployed on GitHub pages with the CI. You can check the [project website](https://utiasdsl.github.io/crisp_controllers/) for guides, getting started, demos and more!
