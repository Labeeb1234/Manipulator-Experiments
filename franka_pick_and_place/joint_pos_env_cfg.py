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
from isaaclab.managers.curriculum_manager import CurriculumTermCfg as CurrTerm
from isaaclab.managers.command_manager import CommandTermCfg as CommandTerm

from isaaclab.managers import SceneEntityCfg
from isaaclab.assets import AssetBaseCfg, ArticulationCfg, RigidObjectCfg
from isaaclab.sensors import ContactSensorCfg, TiledCameraCfg
from isaaclab_assets.robots.franka import FRANKA_PANDA_CFG


import isaaclab_tasks.manager_based.manipulation.franka_pick_and_place.mdp as cmdp

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

    # robot articulation (system)
    robot: ArticulationCfg = FRANKA_PANDA_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot") #type:ignore

    # rgb-camera sensor
    tiled_camera: TiledCameraCfg = TiledCameraCfg(
        prim_path="{ENV_REGEX_NS}/Robot/panda_hand/Camera",
        offset=TiledCameraCfg.OffsetCfg(pos=(0.08647, 0.03819, -0.04451), rot=(-0.0148355, 0.8732992, -0.1098778, 0.4744), convention="world"),
        data_types=["rgb"],
        spawn=sim_utils.PinholeCameraCfg(
            focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 20.0)
        ),
        width=100,
        height=100,
    )

    # contact sensor on the grippers
    contact_forces_gripper: ContactSensorCfg = ContactSensorCfg(
        prim_path="{ENV_REGEX_NS}/Robot/panda_leftfinger",
        update_period=0.0,
        history_length=6,
        debug_vis=True,
        track_pose=True,
        track_air_time=True
    )

    # cube object for pick and place
    obj: RigidObjectCfg = RigidObjectCfg(
        prim_path="{ENV_REGEX_NS}/Cube",
        spawn=sim_utils.CuboidCfg(
            size=(0.1, 0.1, 0.1),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=10.0),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            physics_material=sim_utils.RigidBodyMaterialCfg(static_friction=1.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0), metallic=0.2),
        ),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0.0, 0.5, 0.05)),
    )


@configclass
class ActionsCfg:
    # for now using Abinitio based controlling 
    joint_actions = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["panda_joint[1-7]"],
        scale=1.0, # raw action scaler
        offset=0.0 # action offsetter
    )

    # binary joint position based control (dependent joint actions)
    gripper_actions = mdp.BinaryJointPositionActionCfg(
        asset_name="robot",
        joint_names=["panda_finger.*"],
        open_command_expr={"panda_finger_.*": 0.04},
        close_command_expr={"panda_finger_.*": 0.0},
    )


@configclass
class ObservationsCfg:
    # creating a policy grp to keep track of the feedback from the env
    @configclass
    class PolicyCfg(ObsGroup):
        # object pos wrt end-effector
        obj_eef_rel_pose = ObsTerm(
            func=cmdp.obj_eef_relative_pose,
            params={
                "robot_cfg": SceneEntityCfg("robot"),
                "obj_cfg": SceneEntityCfg("obj")
            }
        )

        # joint positions
        joint_pos = ObsTerm(
            func=mdp.joint_pos,
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "degree": False
            }
        ) # in rad

        # joint velocities
        joint_vel = ObsTerm(
            func=mdp.joint_vel,
            params={
                "asset_cfg": SceneEntityCfg("robot"),
            }
        ) # in rad/s

        # # previous actions
        previous_actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True
    
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    # ---------------------------- events on reset ---------------------------
    scene_state = EventTerm(
        func=mdp.reset_scene_to_default,
        mode="reset"
    )
    # ------------------------------------------------------------------------


@configclass
class TerminationCfg:
    # reset on timeout
    time_out = DoneTerm(func=mdp.time_out)
    # reset on reaching the goal tolerance (pos)
    after_goal_timeout = DoneTerm(
        func=cmdp.terminate_after_goal,
        params={
            "robot_cfg": SceneEntityCfg("robot"),
            "obj_cfg": SceneEntityCfg("obj"),
            "dis_tol": 0.01
        }
    )
    # reset on null space encounter (from custom mdp) (need to add)


@configclass
class RewardCfg:
    # penalty for just existing
    # alive_penalty = RewTerm(
    #     func=mdp.is_alive,
    #     weight=-0.5
    # )

    # penalty for termination other than timeout
    # termination_pen = RewTerm(
    #     func=mdp.is_terminated,
    #     weight=-0.5
    # )

    # custom reward model (from cmdp) (need to addd)
    goal_pen = RewTerm(
        func=cmdp.goal_reward,
        weight=1.0,
        params={"robot_cfg": SceneEntityCfg("robot"), "obj_cfg": SceneEntityCfg("obj")}
    )

    # action penalty to learn optimized and less jittery actions --> in this scene setup its joint position action
    action_pen = RewTerm(
        func=cmdp.action_penalty,
        weight=-0.01,
    )

    # action rate of change penalty 
    action_rate_pen = RewTerm(
        func=mdp.action_rate_l2,
        weight=-0.0001
    )

    # joint velocity penalty to control joint velocities
    joint_vel = RewTerm(
        func=mdp.joint_vel_l2,
        weight=-0.0001,
        params={"asset_cfg": SceneEntityCfg("robot")},
    )

    
# optional ones
@configclass
class CurriculumCfg:
    joint_rate = CurrTerm(
        func=mdp.modify_reward_weight,
        params={
            "term_name": "joint_vel",
            "weight": -0.005, 
            "num_steps": 10000
        }
    )


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
    curriculum: CurriculumCfg = CurriculumCfg()

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
        self.episode_length_s = 12.0 # [sec]
        # viewer settings
        self.viewer.eye = (3.5, 3.5, 3.5)
        # simulation settings
        self.sim.dt = 1.0/60.0
        self.sim.render_interval = self.decimation




