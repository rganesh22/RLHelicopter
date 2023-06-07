import gym
from mlagents_envs.environment import UnityEnvironment
from gym_unity.envs import UnityToGymWrapper
import numpy as np
# from stable_baselines3.common.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
import os
import time
import pickle

if __name__ == "__main__":
    first_unity_env = UnityEnvironment("Build/FirstEnv", no_graphics=True, worker_id=2)
    first_env = UnityToGymWrapper(first_unity_env, uint8_visual=False) 

    reward_func = "Step1_reward_just_fly"
    lr = 1e-2
    bs = 512
    gamma = 0.99
    log_dir = f"stable_results/ppo/{reward_func}/lr{lr}_batchsize{bs}/"
    os.makedirs(log_dir, exist_ok=True)
    env = Monitor(env, log_dir, allow_early_resets=True)
    env = DummyVecEnv([lambda: env])  # The algorithms require a vectorized environment to run
    model = PPO("MlpPolicy", env, learning_rate=lr, batch_size=bs, verbose=1, tensorboard_log=log_dir)
    model.save(log_dir + "/model")
    model.save("Step1_reward_just_fly")


    model2 = PPO.load('Step1_reward_just_fly')
    model2.set_env(env)
    
    # model = PPO("MlpPolicy", env, n_steps=2048, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=2048, learning_rate=1e-2, gamma=0.95, clip_range=0.5, verbose=1, tensorboard_log=log_dir)

    # model = PPO("MlpPolicy", env, n_steps=500, verbose=1, tensorboard_log=log_dir)    
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-2, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-2, gamma=0.95, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-2, gamma=0.95, clip_range=0.5, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-2, batch_size=32, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-2, batch_size=32, clip_range=0.5, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-2, batch_size=16, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-1, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-1, gamma=0.95, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-1, batch_size=32, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, learning_rate=1e-1, batch_size=16, verbose=1, tensorboard_log=log_dir)
    # model = PPO("MlpPolicy", env, n_steps=500, batch_size=32, verbose=1, tensorboard_log=log_dir)

    # model = PPO("CnnPolicy", env, n_steps=500, verbose=1, tensorboard_log=log_dir)

    # model = DDPG("MlpPolicy", env, n_steps=500, verbose=1, tensorboard_log=log_dir)
    # model = DDPG("CnnPolicy", env, n_steps=500, verbose=1, tensorboard_log=log_dir)

    # model = DQN("MlpPolicy", env, n_steps=500, verbose=1, tensorboard_log=log_dir)
    # model = DQN("CnnPolicy", env, n_steps=500, verbose=1, tensorboard_log=log_dir)
    
    # model.learn(total_timesteps=100000)
    # model.learn(total_timesteps=200000)
    model.learn(total_timesteps=70000)
    
    model.save(log_dir+"/model")
    model.save("latest_model")

    env.close()

    #evaluate agent
    eval_unity_env = UnityEnvironment("Eval_Build/ArcadeJetFlightExample", worker_id=3)
    eval_env = UnityToGymWrapper(eval_unity_env, uint8_visual=False) 

    episodes = 100
    ep_r = []
    ep_l = []
    for e in range(episodes):
        obs = eval_env.reset()
        total_r = 0.
        total_l = 0.
        while True:
            action, _states = model.predict(obs)
            obs, reward, done, info = eval_env.step(action)
            # if e < 20:
            #     print(f'Observation: {obs} \n')
            #     print(f'Action: {action} \n\n')
            total_l += 1.
            total_r += reward
            if done:
                break
        ep_r.append(total_r)
        ep_l.append(total_l)
    print("episode mean reward: {:0.3f} mean length: {:0.3f}".format(np.mean(ep_r), np.mean(ep_l)))
    with open('{}_eval.pkl'.format(log_dir), 'wb') as f:
        pickle.dump(ep_r, f)
        pickle.dump(ep_l, f)

    eval_env.close()
    # model.save(log_dir+"/model")
    # model.save("latest_model")