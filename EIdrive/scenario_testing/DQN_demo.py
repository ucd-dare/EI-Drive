"""
The script trains DQN for vehicle lane keeping.
"""

from EIdrive.core.basic.EIDriveEnv import EIDriveEnv
from stable_baselines3 import SAC, DQN
import wandb
from stable_baselines3.common.callbacks import CallbackList, CheckpointCallback, BaseCallback


class TensorBoardCallback(BaseCallback):
    def __init__(self, verbose=0):
        super().__init__(verbose)

    def _on_step(self) -> bool:
        return True

    def _on_rollout_end(self) -> bool:
        self.logger.record("rewards/episode_return", self.training_env.get_attr("episode_reward")[0])
        self.logger.record("rewards/reward_progress", self.training_env.get_attr("reward_prog")[0])
        # self.logger.record("rewards/penalty_lane", self.training_env.get_attr("penalty_lane")[0])
        self.logger.record("rewards/average_speed", self.training_env.get_attr("average_speed")[0])
        return True


def run_scenario(config_yaml):
    train = True
    wb = False
    file_name = config_yaml["RL_environment"]["agent_name"]
    env = EIDriveEnv(config_yaml, True)
    # check_env(env)

    trained = False

    if wb:
        run = wandb.init(
            # set the wandb project where this run will be logged
            project="lane-keeping",
            sync_tensorboard=True
        )
    try:
        if train:

            model = SAC("MlpPolicy", env, verbose=1, learning_starts=5000, learning_rate=0.0003, ent_coef="0.1",
                        gradient_steps=1, buffer_size=250000, device='auto', tensorboard_log="./tensorboard_logs/")

            # model = DQN("MlpPolicy", env, learning_starts=100, verbose=1, train_freq=1, batch_size=256,tau=1.0, learning_rate=0.0005, gamma=0.99, exploration_initial_eps=0.4, exploration_fraction=0.75, device='cuda', tensorboard_log="./tensorboard_logs/")

            # API with multipel datasets
            # datasets with different agents
            # mixed dataset eventually

            if wb:
                checkpoint_callback = CheckpointCallback(save_freq=100000, save_path='./checkpoints/',
                                                         name_prefix='rl_model')
                callback = CallbackList([checkpoint_callback, TensorBoardCallback()])
                model.learn(total_timesteps=1000000, log_interval=1, callback=callback)
            else:
                model.learn(total_timesteps=1000000, log_interval=1)
            # env.save_fig()
            model.save(file_name)
            print("Training complete. Saved Model")
            trained = True
        else:
            model = SAC.load(file_name, env=env)
            for i in range(100):
                r = 0
                obs = env.reset()
                steps = 0
                done = False
                while not done:
                    action, _states = model.predict(obs, deterministic=True)
                    obs, rewards, done, info = env.step(action)
                    # print("steps: "+str(steps) + "reward: " + str(r))
                    r += rewards
                    steps += 1
                    # time.sleep(0.05)
                print("score: " + str(r))

    finally:
        if not trained and train:
            model.save(file_name + "_crashed")
            print("Crashed! Saved Model")
        print('Done')
        env.close()
