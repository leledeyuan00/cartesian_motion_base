---
title: "CMB: A C++ Library for Real-Time Cartesian Control Architectures of Robotic Manipulation in ROS 2"
tags:
  - C++
  - ROS2
  - manipulator
  - robotics
  - real-time architecture
authors:
  - name: Dayuan Chen
    orcid: 0000-0001-9644-7584
    corresponding: true
    affiliation: 1
  - name: Yasuhisa Hirata
    orcid: 0000-0002-5931-0471
    affiliation: 1
affiliations:
  - name: Department of Robotics, Tohoku University, Sendai, Miyagi, 980-8579
    index: 1
date: 20 Feb 2026
bibliography: paper.bib
---

# Summary

Controlling robotic manipulators in the real world often requires translating intuitive Cartesian commands (e.g., moving a tool in 3D space) into complex joint angle trajectories. While achieving motion safety with the environment requires real-time compliance feedback, which is notoriously difficult to achieve on robotics systems, especially industrial robots, that are limited to standard position or velocity control interfaces.

A typical robotic manipulation task consists of a sequence of dynamic sub-tasks. For example, a contact-rich "pick-and-place" operation might involve a rapid, rough approach to a target, followed by precise, visually-guided alignment, and finally, a compliant grasping motion. Ensuring uninterrupted, real-time control across these transitioning phases is technically challenging. Therefore, developing a control architecture that provides a real-time, sequential, and flexible task-execution paradigm is crucial for advanced robotics research.

Cartesian Motion Base (CMB) is an extensible C++ library for the Robot Operating System 2 (ROS 2) that addresses this challenge. CMB provides a robust Finite State Machine (FSM) architecture that manages sequential manipulation sub-tasks while maintaining real-time response. By seamlessly integrating with existing ROS 2 `cartesian_controllers`, which generate joint commands from Cartesian targets and force/torque feedback (utilizing wrist-mounted sensors and compliance controllers), CMB allows users to intuitively program single- or multi-arm robot systems. Researchers and engineers can easily inherit from the CMB base class to rapidly design, deploy, and scale their own custom, complex manipulation sequences.

# Statement of need

<!-- For which audience needs to use -->
To handle general Cartesian planning, CMB builds upon the robust ROS 2 `cartesian_controllers` package [@FDCC]. This dependency uses an iterative forward-virtual-dynamics kinematics approach—employing $H_q J^\top$ instead of the traditional $J^{-1}$—to map Cartesian-space commands to joint space. Here, $J$ represents the real-time updated Jacobian matrix and $H$ denotes the positive definite joint space inertia matrix. This underlying formulation ensures stable operation even near singularities and joint limits.

However, utilizing these low-level controllers directly in complex, multi-stage real-world applications is highly intricate. CMB abstracts this complexity by introducing a dynamically initialized Finite State Machine (FSM) mechanism to realize a unified manipulation control system. To facilitate seamless real-world deployment, CMB significantly extends the baseline `cartesian_controllers` with the following practical features:

- Unified State Machine Architecture: A standardized command and state interface that allows users to rapidly design, extend, and deploy sequential tasks.
- Enhanced Service Interfaces: Exposing joint trajectory execution as an easily accessible ROS 2 service.
- Real-time Synchronization & Monitoring: Publishing the current wrench (referenced to the base frame) and the current target Cartesian commands to synchronize multi-process nodes.
- Hardware Safety Mechanisms: Implementing configurable force emergency thresholds to ensure safe operation during physical human-robot or robot-environment interactions.
- Gravity Compensation: Seamless integration of gravity compensation models [@GravityCompensation] to decouple the mass of the end-effector from the payload of force/torque readings.

Ultimately, CMB provides the missing "glue" between mathematical Cartesian control and practical, sequence-driven robotic programming, significantly lowering the barrier to entry for advanced manipulation research.

# State of the field

<!-- What's the relationship with current packages -->
In the ROS 2 ecosystem, several established frameworks address robotic manipulation and task sequencing, but they typically focus on global motion planning or on executing generated logic, leaving a gap for real-time Cartesian-based compliance tasks.

<!-- 1. Compared with MoveIt -->
The most prominent framework in manipulator control is MoveIt 2 and its extension, MoveIt Task Constructor (MTC) [@MoveItTaskConstructor]. While MTC excels at sequencing complex tasks involving collision avoidance and global trajectory generation, it primarily relies on a "plan-then-execute" paradigm. This approach is highly effective for collision-free motion but is often insufficient for contact-rich or servo tasks (e.g., peg-in-hole insertion, surface wiping, visual servoing, or compliant grasping), which require millisecond-level, closed-loop reactions. CMB complements the ecosystem by focusing entirely on the real-time execution and compliance phase rather than global planning.

<!-- 2. Compared with Behavior Tree -->
For task execution and logic sequencing, general-purpose frameworks like BehaviorTree.CPP [@BehaviorTreesInAction] or state-state machine libraries [@YASMIN] are widely used in ROS 2. They provide roubust methods for realizing logical task sequences and offer clear visualization of state transitions and execution statuses. However, they are designed for general-purpose systems rather than specifically for the manipulators, researchers have to write significant amounts of boilerplate code to bridge the gap between high-level logic and low-level Cartesian kinematics, wrench signals, and controller states. CMB is specifically tailored for robotic manipulators, directly integrating Cartesian controllers to seamlessly execute user-defined, real-time tasks.

<!-- 3. Build vs. Contribute -->
To provide a robust kinematic foundation, CMB relies heavily on the `cartesian_controllers` package. Rather than merging CMB directly into that repository, CMB is architected as a separate, higher-level task execution package. This maintains a clear separation of concerns: `cartesian_controllers` operates at the ros2_control layer for high-frequency mathmatical calculations and hardware communication, whereas CMB operates at the user application node layer. Furthermore, to ensure a synergistic relationship, we have actively contributed several general-purpose features back upstream to the `cartesian_controllers` repository, including joint trajectory execution interfaces and gravity compensation models to decouple the end-effector and payload masses.

# Software design

<!-- Architecture -->

![The CMB architecture with ROS2 system](https://github.com/user-attachments/assets/b3149861-9ca2-418f-80c7-6e1c20ea0f6b){#fig:architecture width="70%"}

To maximize code reusability and ensure deterministic real-time execution, CMB is architected around a continuous Finite State Machine (FSM) running within a primary, non-blocking outer loop.

When users inherit from the CMB base class, the system automatically abstracts the boilerplate ROS 2 communication with the underlying `cartesian_controllers`. As demonstrated in our single- and multi-robot arm configuration interfaces, CMB seamlessly manages Cartesian position commands, wrench feedback, and joint trajectory services for single or multiple manipulators. This allows researchers to focus entirely on implementing their real-time task logic. Because all tasks are evaluated at a fixed, user-defined frequency within the main loop, developers do not need to rely on thread-blocking functions like `sleep()` to realize continuous execution. By default, tasks follow a sequential queue; however, users can easily implement dynamic conditional state transitions by querying task IDs and using the `goto_specific_task(TASK_ID)` method.

As illustrated in Fig. \ref{fig:architecture}, each user-defined task is strictly divided into three phases: Init, Execute, and Finish.

- Init: Executed exactly once upon entering a new task. During this phase, CMB automatically caches essential environmental states, such as `start_time`, `start_pose`, and `start_wrench`.
- Execute: The core real-time loop where the custom manipulation logic is evaluated at every tick. Users can effortlessly retrieve the cached initial states via built-in getters (e.g., `get_system_state().start_time` or `get_start_pose(NAME)`).
- Finish: Handles cleanup and securely schedules the transition to the next state.

## Design of Trade-offs
A critical design trade-off in CMB is the intentional exclusion of built-in global collision avoidance. To maintain a lightweight design and ensure real-time performance at high frequencies, CMB assumes that global, obstacle-free motion planning is handled upstream by the user. However, to compensate for this and ensure hardware safety during contact-rich operations, CMB actively monitors real-time force/torque feedback (when a wrist-mounted sensor is available) and triggers an emergency stop if predefined wrench thresholds are exceeded.

## Example of usage
To demonstrate the simplicity and expressiveness of the CMB API, the following C++ snippet illustrates how a user can define a custom contact-rich task (e.g, safely approaching a surface until a contact force is detected). Users only need to inherit from `CartesianMotionBase` and use `task_pushback()` to register their real-time logic.

```cpp
class PickAndPlaceTask : public cartesian_motion_base::CartesianMotionBase {
public:
  // Inherit constructor
  PickAndPlaceTask(
    const std::string & node_name,
    std::vector<RobotConfig> robot_configs,
    uint16_t rate)
  : CartesianMotionBase(node_name, robot_configs, rate) {} 

  void tasks_init() override {
    // Register a new state machine task named "Approach"
    task_pushback(TaskPtr("Approach", 
      // Init Phase: Runs only once when entering this task
      [this]() {
        RCLCPP_INFO(this->get_logger(), "Starting to approach the target...");
      },
      // Execute Phase: Runs continuously at the defined real-time rate
      [this]() {
      auto current_pose = get_current_pose("ur");
      auto current_wrench = get_current_wrench("ur");
      auto target_pose = current_pose;

      // Move the end-effector downwards
      target_pose.position.z = current_pose.position.z - 0.001; 

      // Set the target pose
      PoseMap pose_map;
      pose_map["ur"] = target_pose;
      set_target_pose("ur", current_pose);

      // Transition to the next task if contact force exceeds 10 N
      if (current_wrench.force.z > 10.0) {
        RCLCPP_INFO(this->get_logger(), "Contact detected. Task finished.");
        // Finish Phase will runnig into built-in base class.
        set_task_finished();
      }
    }));
    
    // Additional tasks (e.g., "Grasp", "Lift") can be pushed here...
  }
};

```

# Research impact statement

<!-- Applications -->
To demonstrate its practical utility and reliability, CMB has been successfully deployed as the core control architecture in several manipulation research projects.

First, CMB was utilized to develop a human-guided manipulation framework [@HumanGuided]. In this application, CMB's Finite State Machine (FSM) seamlessly managed transitions between an autonomous task (covering a flat board with a t-shirt) and an interactive human-guidance mode. If the environment changed dynamically (e.g., the target board was moved or a new configuration product line was introduced), CMB's real-time interaction monitoring enabled the human operator to safely intervene. The FSM smoothly transitioned the robot into a compliant, lead-through dragging mode, enabling the operator to physically guide the manipulator to the new target pose before resuming the autonomous sequence.

Furthermore, CMB's architecture is highly interoperable with high-level Python-based planning nodes via customized ROS 2 interfaces. This capability was leveraged to deploy a Reinforcement Learning (RL) policy for contact-rich manipulation [@ContactRL]. The Python-based RL node generated Cartesian target positions and seamlessly transmitted the commands to an incremental impedance controller, which was effectively wrapped as aservice-triggered task within the CMB framework [@Yukuan-incre]. Additionally, CMB's stable force-feedback capabilities facilitated a human-guided residual learning approach designed to bridge the sim-to-real gap during physical human-robot interaction [@AdaptiveSim].

Finally, CMB served as the real-time execution engine for the immersive teleoperation system. By integrating a Meta Quest 3 VR headset via a ROS 2 interface, researchers used CMB to collect high-quality human demonstration datasets. This stable data-collection pipeline culminated in the successful training and deployment of a force-conditioned imitation learning policy for a highly complex, deformable-object manipulation task (cloth hanging) [@FACT].

# AI usage disclosure
During the development of this project and the preparation of this manuscript, generative AI tools (specifically, Large Language Models) were utilized to assisted with several tasks.

**Software Development & Documentation:** AI was used as a pair-programming assistant to troubleshoot C++ compilation errors, resolves ROS 2 GTest Cmake configuration issues, debug GitHub Actions CI workflows, and generate formatting for the `README.md` documentation.

**Paper Authoring:** AI was used to refine the English grammar, enhance the flow of the text, and ensure the phrasing met acedamic standards for the JOSS manuscript.

**Verification:** All AI-generated code snippets and configuration files wre manually reviewed, compiled, and rigorously validated against the project's test suite (GTest) and CI pipeline. All AI-assisted text in this paper was thoroughly reviewed and edited by the human author to ensure scientific accuracy and correctly refect the original research intent.

# Acknowledgements

This work was partially supported by the Innovation and Technology Commission of the HKSAR Government under the InnoHK initiative.

# References