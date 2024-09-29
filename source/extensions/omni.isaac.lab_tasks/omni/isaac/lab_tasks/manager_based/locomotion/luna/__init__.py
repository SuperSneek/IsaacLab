import gymnasium as gym

from . import agents, luna_env_cfg
##
# Register Gym environments.
##

gym.register(
    id="Isaac-Bipedal-Flat-Luna-v0",
    entry_point="omni.isaac.lab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": luna_env_cfg.LunaFlatEnvCfg,
        "rsl_rl_cfg_entry_point": agents.rsl_rl_cfg.LunaFlatPPORunnerCfg
    },
)