# Hardware Experiments

---

## Dobot Magician (Hardware Used)  
*(The equiped suction end effector with servo control for axis rotation isn't connected to the external suction module)*

<div>
  <img src="https://github.com/user-attachments/assets/fd9398d0-eeab-4eae-9146-3130e191118b" alt="Dobot Magician hardware" height="800" width="800"/>
</div>


- Started by setting up the [`pydobot`](https://github.com/luismesas/pydobot) package — a lightweight USB serial communication library — to interface the local system with the Dobot Magician hardware.
- Conducted experiments using the package to test various PTP (Point-To-Point) motion modes for robot control. Two experiments done on two major(IK) modes are shown below. One is the MOVJ_XYZ and the other is MOVL_XYZ; both being task-space based control with 'L' being the PTP motion will be in a straight path whereas 'J' will make the PTP motion take a non-linear path as you can see briefly in the gifs given below where the end-effector (x-y) tracjectory is plotted online during the motion (PS: the end-effector was only given planner 2-D motion to properly test the PTP motion differences.

    <div>
     <img src="" alt="MOVJ_XYZ_MOTION"/>
    </div>
      
    <div>
     <img src="" alt="MOVL_XYZ_MOTION"/>
    </div
  
- Here is the Dobot Magician Hardware [user guide](https://www.generationrobots.com/media/Dobot-Magician-User-Manual-V1.2.4.pdf?srsltid=AfmBOorvQr62dOwXR68afS7-bREsKwmtFq0SUyWa29sUH7gtJNatVLZY)
- **Note:** The joint sensor values extracted by `pydobot` provide:
  - The end-effector (EEF) pose in millimeters within the robot’s workspace,
  - Joint positions in degrees,
  - An `r` parameter representing the end-effector rotation angle in degrees, which is meaningful only if an end-effector is physically attached.

 - Software stack used: ROS2-Humble(would definitely work in Jazzy and Kilted as long as the Ubuntu 24.04 is used for those distros), Tested on Ubuntu 22.04, Python3.10.12, PyDobot

---

## Understanding Dobot Magician Default Pose Feedback

### 1. Joint Position Estimation via Step Counting

The Dobot Magician **does not use high-resolution absolute encoders** on all joints. Instead, its position feedback relies on:

- **Stepper motors** (instead of servo motors),
- **Step counting implemented in firmware** to keep track of motor positions,
- **Mechanical limit switches** to establish the home (zero) position reference.

#### What does this mean?

- Upon performing a **homing operation**, the robot establishes a known zero reference point using the physical limit switches.
- From this zero position, it **counts every motor step** to estimate the current joint angles.
- The Cartesian position `(x, y, z, r)` is calculated by the robot’s firmware through **forward kinematics** based on these joint angles.

Thus, when querying the robot’s pose, you are **not receiving direct sensor measurements** but rather the **internal model maintained by the firmware**, derived from commanded and tracked stepper motor movements.

#### Important note — what if the robot is bumped or moved manually?

- Because this is an **open-loop system after homing**, if you **physically move the robot arm by hand**, the firmware **cannot detect this disturbance**.
- The system assumes that no steps are missed or skipped during operation.
- If the arm is bumped, overloaded, or experiences mechanical slips causing lost steps, the reported pose can become **inaccurate** until the robot is homed again.

---

> Understanding this behavior is crucial when interpreting pose feedback and designing experiments or applications involving the Dobot Magician.


## Dobot Magician Hardware ROS2 Interfacing (Made using PyDobot module so the pkg is in rclpy for now)
 - Interface dobot magician to ros2 by extracting joint states and eef states and publishing them to ros2 topics at fixed rates (10Hz) --> the dobot hardware feedback update rate is around 2.5Hz(in async mode) and 1.4Hz (sync Mode). Here async means the consecutive injected motions commands aren't blocked by the ongoing motion command which reduced the delay in extracting the feedback from the dobot sensors too.
 - Home Pose Figure given below (all joint pos at 0 rads(degs))
    <div>
     <img src="" alt="home configuration"/>
    </div>
- Verified the ros2 state interface package on RViz using a digital twin examples the states are matching (need to fine tune a bit more after control pkg is integrated) ---> shown below
  <div>
      <img src="" alt="joint states matching alright"/>
  </div>

- Control functionality via ros2 to be done. So the entire hardware was sucessfully interfaced with ROS2 with both c-space as well as task-space control functionality. [codebase here](https://github.com/Labeeb1234/Manipulator-Experiments/tree/main/dobot_state_interface/dobot_rclpy_interface)
- From the videos below there is clearly an undesireable offset in the digital twin model (the URDF in RViz) the URDF was taken from the official docs of the dobot but for some reason there was already an issue with the joint  offsets but the joint axis is clearly matched up properly with the hardware joint axes. Will need to fix the URDF for it.
- Another issue that is visible is the update rate of the feedback from the hardware is close to 2.5Hz hence the RViz update is a little on the choppy side. (any suggestions to reduce the choppyness if it is even possible is welcomed)
- The current ros2 interface architecture for the dobot hardware is written in rclpy with pub/sub, but I feel like creating an action-server for it may make it more robust (something to work on later on)
- Video Demos Given below for both joint as well as end effector control


  <div>
    <img src="" alt="joint space control mode"/>
  </div>
  <div>
    <img src="" alt="end effector control mode"/>
  </div>

- So for now the digital twin part is complete as a rough prototype.
- **Note** May build a custom pkg for Dobot Communication in CPP later on after testing out Moveit2 and maybe even some VLA/RL algo on this hardware.


- Gesture Control
  <div>
  <img src="" alt="Temu Gesture Control"/>
  </div>

### =========================================================================  





# Software Experiments [Franka Emika Panda Arm]

### =======================================================================
- **Note**: Currently using RL_Games framework with PPO RL algo for the training process (for reach to pose).

### IsaacLab RL based Pick And Place Task

- Model Selected from default assets
- Custom GymEnv and MDP Package created
- First Test Demo (3 envs)
<div align="center"> 
  <img src="https://github.com/user-attachments/assets/c2783e19-e49e-4d3f-9680-fe541ffb5d2c" alt="Custom Env Testing" />
</div>

### IsaacLab RL Based Reach To Goal Pose Task

- First Training Demo
  
  - After training 5 envs for more than 500 episodes (less than 1000-episodes)
  - [video_1](https://github.com/user-attachments/assets/be928c7b-d279-4377-a167-dd227b47ba0e)



  -  After training 5 envs for more than 2k episodes (less than 4k-episodes)
  - [video_2](https://github.com/user-attachments/assets/67f04259-4701-4d9d-ad26-e301ff86a27f)
    
- Second Training Demo

  - Trained on an improved reward model (better than the first check the IsaacLab MDP pkg for reference. The results were far from satisfactory on rl_games PP0 model even after training for 18k episodes. The rewards peaked for a range of 20k episodes after around 8kth episode the max reward from 0-18k episodes was around -1.42(net per episode). Allthough on the bright side the jerky motion of joints stopped which is a huge improvement in terms of motion. As of now the motion is JointPositionBased Control without any kinematics maybe I have to train more or improve the reward model for better reach to pose accuracy.

  - After training 5 envs for more for 10400 episodes
  - [Video-1](https://github.com/user-attachments/assets/46c86c45-6aeb-4bf7-bf85-bcde8fcec558)


- Next Phase (moving to end-effector/task space based control via IDK of the model)

  - During the initial test the action space control was purely joint control (c-space control) which since there were like 8-states to control, with no kinematic or dynamic constraints mapping to the desired actions was very hard and plus clearly don't have too much time to train the model to fit this complex of a model. So in order to get better control and response plus faster convergence to the required solution the kinematic constraints was introduced and by using IDK the action control will be in the t-space (in global frame of ref/env frame of ref).
  - Just a small side track: teleop policy integration demo video (down here) --> (will use to collect data/demonstration for imitation learning for more advanced taks)

### =========================================================================







### Acknowledgements
- [Dobot-Magician Python SDK](https://github.com/luismesas/pydobot)
