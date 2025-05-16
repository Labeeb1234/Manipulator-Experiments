import isaaclab.sim as sim_utils
from isaaclab.utils import configclass


import isaaclab.envs.mdp as mdp
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.managers import RewardTermCfg as RewTerm
from isaaclab.managers.action_manager import ActionTermCfg as ActionTerm

from isaaclab.managers.curriculum_manager import CurriculumTermCfg

from isaaclab.managers import SceneEntityCfg
from isaaclab.assets import AssetBaseCfg, ArticulationCfg
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG


@configclass
class MyCustomSceneCfg(InteractiveSceneCfg):
    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane", 
        spawn=sim_utils.GroundPlaneCfg()
    )

    # lights
    lights = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0,)
    )

    robot: ArticulationCfg = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot") #type:ignore


@configclass
class ActionsCfg:
    # for now using Abinitio based controlling 
    joint_actions = mdp.JointEffortActionCfg(
        asset_name="robot",
        joint_names=["panda_joint[1-7]"],
        scale=1.0, # raw action scaler
        offset=0.0 # action offsetter
    )

    gripper_actions = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["panda_finger_joint.*"],
        scale=1.0,
        offset=0.04
    )

@configclass
class ObservationsCfg:

    # creating a policy grp to keep track of the feedback from the env
    class PolicyCfg(ObsGroup):
        # joint positions
        joint_pos = ObsTerm(
            func=mdp.joint_pos,
            params={
                "asset_cfg": SceneEntityCfg("robot"),
            }
        ) # in rad

        # joint velocities
        joint_vel = ObsTerm(
            func=mdp.joint_vel,
            params={
                "asset_cfg": SceneEntityCfg("robot"),
            }
        ) # in rad/s

        # previous actions
        previous_actions = ObsTerm(
            func=mdp.last_action,
            params={
                "action_name": ["joint_actions","gripper_actions"]
            }
        )


        def __post_init__(self):
            self.enable_corruption = True
            self.concatenate_terms = False

@configclass
class EventCfg:
    # ---------------------------- events on reset ---------------------------
    scene_state = EventTerm(
        func=mdp.reset_scene_to_default
    )
    # ------------------------------------------------------------------------


@configclass
class TerminationCfg:
    # reset on timeout
    time_out = DoneTerm(func=mdp.time_out)

    # joint pos limit timeout
    joint_pos_limit = DoneTerm(
        func=mdp.joint_pos_limits,
        params={
            "asset_cfg": SceneEntityCfg("robot")
        }
    )


    # reset on null space encounter (from custom mdp) (need to add)

@configclass
class RewardCfg:
    # penalty for just existing
    alive_penalty = RewTerm(
        func=mdp.is_alive,
        weight=-1.0
    )

    # custom reward model (from cmdp) (need to addd)


@configclass
class PandaEnvCfg(ManagerBasedRLEnvCfg):
    # scene settings and setup
    scene: MyCustomSceneCfg = MyCustomSceneCfg(num_envs=3, env_spacing=2.5)
    # Basic Settings
    actions: ActionsCfg = ActionsCfg()
    observations: ObservationsCfg = ObservationsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardCfg = RewardCfg()
    terminations: TerminationCfg = TerminationCfg()
    curriculum: CurriculumTermCfg = CurriculumTermCfg(func=mdp.modify_reward_weight,
        params={
            "weight": 1.0,
            "num_steps": 10000
        }
    )

    def __post_init__(self):
        """Post initialization."""

        '''

        general info:
            In Isaac Lab, the use of substeps has been replaced by a combination of the simulation dt and the decimation parameters. 
            For example, in IsaacGymEnvs, having dt=1/60 and substeps=2 is equivalent to taking 2 simulation steps with dt=1/120, but running the task step at 1/60 seconds. 
            The decimation parameter is a task parameter that controls the number of simulation steps to take for each task (or RL) step, 
            replacing the controlFrequencyInv parameter in IsaacGymEnvs. Thus, the same setup in Isaac Lab will become dt=1/120 and decimation=2.
        
        '''
        # general settings
        self.decimation = 2
        self.episode_length_s = 12.0
        # viewer settings
        self.viewer.eye = (3.5, 3.5, 3.5)
        # simulation settings
        self.sim.dt = 1.0/60.0
        self.sim.render_interval = self.decimation




