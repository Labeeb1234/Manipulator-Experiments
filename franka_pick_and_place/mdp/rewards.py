from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers.manager_base import ManagerTermBase
from isaaclab.managers.manager_term_cfg import RewardTermCfg
from isaaclab.sensors import ContactSensor, RayCaster

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv

import isaaclab_tasks.manager_based.manipulation.franka_pick_and_place.mdp as cmdp


def goal_reward(
    env: ManagerBasedRLEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    obj_cfg: SceneEntityCfg = SceneEntityCfg("obj"),
    dis_scale: float = 20.0,
    orient_scale: float = 10.0,
    dis_threshold: float = 0.03, # in [m]
    orient_threshold: float = 0.1, # in [rad]
    success_bonus_val: float = 200.0, 
)->torch.Tensor:

    '''L2 Kernel Based Pose Tracking Error based penalty'''
    robot: Articulation = env.scene[robot_cfg.name]
    obj: RigidObject = env.scene[obj_cfg.name]

    rel_obj_pose = cmdp.obj_eef_relative_pose(env=env, robot_cfg=robot_cfg, obj_cfg=obj_cfg)
    p_err, q_err = rel_obj_pose[:, :3], rel_obj_pose[:, 3:7]
    
    d_err = torch.norm(p_err[:, :3], dim=-1)
    d_rew = torch.exp(-dis_scale*d_err**2)

    orient_err = torch.acos(torch.clamp(torch.abs(q_err[:, 0]), -1.0, 1.0))
    orient_rew_term = torch.exp(-orient_scale*orient_err**2)

    is_pos_reached = d_err < dis_threshold
    orient_rew = torch.where(is_pos_reached, orient_rew_term, torch.zeros_like(orient_rew_term))

    # task sucess bonus orientation reward
    is_successful = (d_err < dis_threshold) & (orient_err < orient_threshold)
    success_bonus = torch.where(is_successful, success_bonus_val, torch.zeros_like(d_err))

    total_term = d_rew + orient_rew + success_bonus

    return total_term

def action_penalty(env: ManagerBasedRLEnv)->torch.Tensor:
    '''L-2 Kernel Penalty for actions'''
    # Penalize the magnitude of the actions taken by the robot.
    # This encourages the agent to find solutions with minimal movement, preventing jerky or excessive actions.
    # `env.actions` typically holds the last actions applied (e.g., joint velocities or position targets).
    # We sum the squared actions across the action dimension and multiply by a weight.
    action_penalty = torch.sum(torch.square(env.action_manager.action), dim=1)
    return action_penalty