<p align="center">
  <a>
    <img src="ext/image/logo.png" alt="logo" width="300">
  </a>
</p>

---

# mc_rtc RL-QP Controller for HRP5P

This repository provides an adaptation of the **mc_rtc RL-QP Controller Template** for the **HRP5P** humanoid robot.

The goal of this repository is to help users deploy reinforcement learning (RL) policies within a Quadratic Programming (QP) framework augmented with Control Barrier Functions (CBFs). This combination enforces physical and safety constraints, including:

* Joint position limits
* Joint velocity limits
* Torque limits
* Self-collision avoidance

Further details are available in:

> *Safe Execution of RL Policies via Acceleration-based CBF-QP Constraint Enforcement for Real-World Robotic Deployments*
> https://hal.science/hal-05362571

It comes with:

- a CMake project that can build a controller in [mc_rtc], the project can be put within the [mc_rtc] source tree for easier updates;
- clang-format files;
- automated GitHub Actions builds on three major platforms.

Currently only ONNX format policies are supported.

## Documentation

The complete documentation, including the controller architecture, observation system, policy configuration, adaptation guide and API reference, is available at:

**https://alhuuin.github.io/rl-qp-controller.github.io/**

The documentation also includes practical guides for adapting the controller to new robots and examples.

The HRP5P repository follows the architecture described there and extends it with robot-specific startup logic, control-mode management and contact handling.

## Repository-specific features

Compared to the generic template, this repository provides:

- HRP5P robot integration;
- dedicated Initial and RL states;
- runtime control-mode management;
- force-sensor-based contact handling;
- force-sensor-based external force estimation;
- HRP5P joint groups and conventions;
- standing and walking example policies.

## HRP5P-specific implementation

Unlike the template, the HRP5P controller introduces an initial state before entering RL execution.

### Two-state architecture

The Initial state is responsible for:

- position-control startup;
- posture stabilization;
- contact initialization;
- preparing the robot for RL execution.

The RL state then:

- switches to torque control;
- starts policy inference;
- updates the torque task targets.

This repository therefore provides a complete example of a controller requiring an explicit transition from initialization to RL.

### Observer pipeline

The HRP5P implementation extends the template with several robot-specific observers.

In particular, it provides:

- encoder velocity observations;
- floating-base estimation with acceleration;
- force-sensor-based external force estimation.

Unlike the H1 implementation, the external force observer is configured for robots equipped with dedicated force/torque sensors and uses motor torque measurements.

### Contact handling

The HRP5P implementation illustrates a force-sensor-based contact pipeline.

Foot contacts are inferred from the left and right foot force sensors using independent Schmitt triggers before updating the controller contact state.

This repository therefore serves as an example of integrating contact-aware control on robots equipped with force sensors.

### Runtime control modes

Unlike the template, the HRP5P controller explicitly manages runtime control-mode transitions.

It starts in position control during initialization before switching to torque control for RL execution. The controller also adapts the solver feedback model according to the active control mode.

### Policies

The template example policies have been replaced by HRP5P policies.

Policy configuration follows exactly the format described in the documentation.

### Configuration

This implementation introduces several additional configuration parameters for the initial state, including:

- `is_initial_posture_rl` : defines if the posture of the initial state;
- `high_kp`/`high_kd` : PD gains used in initial state;
## Building

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build . -j
sudo cmake --install .
```

After installation, enable `HRP5pRLQPController` in **mc_rtc**.

---

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/
