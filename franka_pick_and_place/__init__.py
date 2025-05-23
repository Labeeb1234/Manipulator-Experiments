import gymnasium as gym


##
# Register Gym environments.
##

gym.register(
    id="Isaac-Pandas-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.panda_env_cfg:PandaEnvCfg",
    },
)

gym.register(
    id="Isaac-Pandas-v1",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.joint_pos_env_cfg:PandaEnvCfg",
    },
)
