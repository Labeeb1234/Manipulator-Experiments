# Hardware Experiments

---

## Dobot Magician  
*(Currently equipped with a mock/fake end effector, effectively functioning as a 3R arm system)*

<div>
  <img src="" alt="Dobot Magician hardware" />
</div>

- Started by setting up the [`pydobot`](https://github.com/luismesas/pydobot) package — a lightweight USB serial communication library — to interface the local system with the Dobot Magician hardware.
- Conducted experiments using the package to test various PTP (Point-To-Point) motion modes for robot control.
- **Note:** The joint sensor values extracted by `pydobot` provide:
  - The end-effector (EEF) pose in millimeters within the robot’s workspace,
  - Joint positions in degrees,
  - An `r` parameter representing the end-effector rotation angle in degrees, which is meaningful only if an end-effector is physically attached.

---

## Understanding Dobot Magician Pose Feedback

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




## Understanding Dobot Magician Pose Feedback

### 1. Joint Position Estimation via Step Counting

The Dobot Magician **does not use high-resolution absolute encoders** on all its joints. Instead, it relies on:

- **Stepper motors** (rather than servos)
- **Step counting in the firmware** to track motor positions
- **Mechanical limit switches** to define the home (zero) position

#### What does this mean?

- After performing a **homing operation**, the robot sets a known zero reference using the physical limit switches.
- From this reference, it **counts every motor step** to estimate the current joint angles.
- The Cartesian position `(x, y, z, r)` is then calculated by the robot’s firmware using **forward kinematics** based on these joint angles.

Thus, when you query the robot’s pose, you are **not getting direct sensor measurements**, but rather the **firmware’s internal model** based on commanded and tracked stepper motor movements.

#### Important note — what if the robot is bumped?

- Since this system is essentially **open-loop after homing**, if you **physically move the robot arm by hand**, the firmware **has no way of detecting** this external disturbance.
- The robot assumes that no steps are lost or skipped during operation.
- Therefore, if the arm is bumped or overloaded, causing missed steps or mechanical slips, the reported pose may become **inaccurate** until the robot is homed again.

---

This behavior is important to understand when interpreting pose feedback and designing experiments or applications involving the Dobot Magician.



  

# Software Experiments

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






- Gesture control

### Acknowledgements
- [Dobot-Magician Python SDK](https://github.com/luismesas/pydobot)
