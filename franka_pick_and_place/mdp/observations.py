from __future__ import annotations

import torch
from typing import TYPE_CHECKING

import isaaclab.utils.math as math_utils
from isaaclab.assets import Articulation, RigidObject
from isaaclab.managers import SceneEntityCfg
from isaaclab.managers.manager_base import ManagerTermBase
from isaaclab.managers.manager_term_cfg import ObservationTermCfg
from isaaclab.sensors import Camera, Imu, RayCaster, RayCasterCamera, TiledCamera, ContactSensor

if TYPE_CHECKING:
    from isaaclab.envs import ManagerBasedEnv, ManagerBasedRLEnv


def contact_sensor_readings(
    env: ManagerBasedEnv, 
    asset_cfg: SceneEntityCfg
)-> torch.Tensor:
    
    contact_sensor: ContactSensor = env.scene[asset_cfg.name]
    
    contact_sensor_pos = contact_sensor.data.pos_w # wrt world frame
    contact_sensor_quat = torch.squeeze(contact_sensor.data.quat_w, dim=1)  # type: ignore # wrt world frame
    contact_sensor_rot = math_utils.euler_xyz_from_quat(contact_sensor_quat)

    return torch.tensor(0.0)

def robot_eef_state_env(env: ManagerBasedEnv, asset_cfg: SceneEntityCfg=SceneEntityCfg("robot"))->torch.Tensor:
    robot: Articulation = env.scene[asset_cfg.name]
    eef_idx = robot.data.body_names.index("panda_leftfinger") 
    eef_pos, eef_q = robot.data.body_pos_w[: , eef_idx], robot.data.body_quat_w[:, eef_idx] # wrt world frame
    eef_pos = eef_pos-env.scene.env_origins

    return torch.cat((eef_pos, eef_q), dim=1)

def obj_state_w(
    env: ManagerBasedEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("obj")
):
    obj: RigidObject = env.scene[asset_cfg.name]
    obj_world_state = obj.data.root_state_w

    return obj_world_state

def obj_arm_relative_pose(
    env: ManagerBasedEnv,
    asset_cfg: SceneEntityCfg = SceneEntityCfg("obj"),
):
    obj: RigidObject = env.scene[asset_cfg.name]
    obj_pos, obj_q = obj.data.root_pos_w-env.scene.env_origins, obj.data.root_quat_w
    return torch.cat((obj_pos, obj_q), dim=1)

def obj_eef_relative_pose(
    env: ManagerBasedEnv,
    robot_cfg: SceneEntityCfg = SceneEntityCfg("robot"), 
    obj_cfg: SceneEntityCfg = SceneEntityCfg("obj")
)-> torch.Tensor:
    
    robot: Articulation = env.scene[robot_cfg.name]
    obj: RigidObject = env.scene[obj_cfg.name]
    

    robot_pos, robot_q = robot.data.root_pos_w, robot.data.root_quat_w # world frame arm base pose
    # robot pose wrt env scene origins --> quats remain the same in this frame too
    robot_pos = robot_pos-env.scene.env_origins # ideal they concide since the env is centred around the arm

    # for now we consider only one digit pose 
    eef_idx = robot.data.body_names.index("panda_leftfinger") 
    eef_pos, eef_q = robot.data.body_pos_w[:, eef_idx], robot.data.body_quat_w[:, eef_idx] # wrt world frame
    eef_pos = eef_pos-env.scene.env_origins # env_origins wrt eef_pos

    # obj_pos wrt env_origins
    obj_pos, obj_q = obj.data.root_pos_w-env.scene.env_origins, obj.data.root_quat_w 
    # calculating the relative pose of obj wrt end effector
    relative_obj_pos, relative_obj_q = math_utils.subtract_frame_transforms(
        t01=eef_pos,
        q01=eef_q,
        t02=obj_pos,
        q02=obj_q
    )
    
    return torch.cat((relative_obj_pos, relative_obj_q), dim=1)
    