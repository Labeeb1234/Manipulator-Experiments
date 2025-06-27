from __future__ import annotations

import torch
from typing import TYPE_CHECKING

from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.sensors import ContactSensor

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedRLEnv
    from isaaclab.managers.command_manager import CommandTerm

import isaaclab_tasks.manager_based.manipulation.franka_pick_and_place.mdp as cmdp


def terminate_after_goal(
    env: ManagerBasedRLEnv, 
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"),
    obj_cfg: SceneEntityCfg = SceneEntityCfg("obj"),
    dis_tol: float = 0.01 # in [m]
):
    ''' currently only focusing on the distance to goal tolerance '''
    rel_obj_pose = cmdp.obj_eef_relative_pose(env=env, robot_cfg=robot_cfg, obj_cfg=obj_cfg)
    p_err, q_err = rel_obj_pose[:, :3], rel_obj_pose[:, 3:7]
    d_err = torch.norm(p_err[:, :3], dim=-1)
    return d_err <= dis_tol
    