# Software Experiments

### =======================================================================
- **Note**: Currently using RL_Games framework with PPO RL algo for the training process (for reach to pose).

### IsaacLab RL based Pick And Place

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




# Hardware Experiments

### =========================================================================
## Dobot Magician (No End Effector Attached as of now hence a 3R system)

- Gesture control

### Acknowledgements
- [Dobot-Magician Python SDK](https://github.com/luismesas/pydobot)
