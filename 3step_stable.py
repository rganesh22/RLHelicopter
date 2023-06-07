import gym
from mlagents_envs.environment import UnityEnvironment
from gym_unity.envs import UnityToGymWrapper
import numpy as np
# from stable_baselines3.common.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor
import os
import pickle
import time
from pathlib import Path

if __name__ == "__main__":
    first_unity_env = UnityEnvironment("Build/FirstEnv", no_graphics=True, worker_id=2)
    first_basic_env = UnityToGymWrapper(first_unity_env, uint8_visual=False) 

    reward_func = "Step1_reward_just_fly"
    lr = 1e-2
    bs = 512
    gamma = 0.99
    log_dir = f"stable_results/ppo/{reward_func}/lr{lr}_batchsize{bs}/"
    os.makedirs(log_dir, exist_ok=True)
    first_env = Monitor(first_basic_env, log_dir, allow_early_resets=True)
    first_env = DummyVecEnv([lambda: first_env])  # The algorithms require a vectorized environment to run
    model = PPO("MlpPolicy", first_env, learning_rate=lr, batch_size=bs, verbose=1, tensorboard_log=log_dir)
    model.learn(total_timesteps=70000)
    model.save(log_dir + "/model")
    model.save(reward_func)
    first_env.close()

    first_eval_unity_env = UnityEnvironment("Eval_Build/FirstEnv", worker_id=3)
    first_basic_eval_env = UnityToGymWrapper(first_eval_unity_env, uint8_visual=False) 

    episodes = 100
    success_counter = 0
    ep_r = []
    ep_l = []
    longfile = Path(f"{log_dir}/longlogs.txt")
    longfile.touch(exist_ok=True)
    logfile = Path(f"{log_dir}/readlogs.txt")
    logfile.touch(exist_ok=True)
    print(logfile)
    print(longfile)
    for e in range(episodes):
        obs = first_basic_eval_env.reset()
        total_r = 0.
        total_l = 0.
        while True:
            action, _states = model.predict(obs)
            obs, reward, done, info = first_basic_eval_env.step(action)
            total_l += 1.
            if reward == -1:
                total_r = -1
            elif reward == 1:
                total_r = 1
                success_counter += 1
            else:
                total_r += reward
            if done:
                break
            with open(longfile, 'a') as file1:
                file1.write(f"Episode: {e}\n Observation:\n {obs}\n Action:\n {action}\n Action Reward: {reward}, Total Reward: {total_r}, Current Total Length: {total_l} \n\n")
        ep_r.append(total_r)
        ep_l.append(total_l)
        with open(logfile, 'a') as file2:
            file2.write(f"Episode: {e}, Total Reward: {total_r}, Total Length: {total_l} \n")
            if e == episodes - 1:
                file2.write("Final Episode Success Rate: {:0.3f}".format(success_counter/episodes))
    print("Episode Mean Reward: {:0.3f}, Mean Length: {:0.3f}, Success Rate: {:0.3f}".format(np.mean(ep_r), np.mean(ep_l), (success_counter/episodes)))
    with open('{}_eval.pkl'.format(log_dir), 'wb') as f:
        pickle.dump(ep_r, f)
        pickle.dump(ep_l, f)

    first_basic_eval_env.close()

    print("10 seconds to reset, then moving into 2nd learning step")
    time.sleep(10)

    second_unity_env = UnityEnvironment("Build/SecondEnv", no_graphics=True, worker_id=2)
    second_basic_env = UnityToGymWrapper(second_unity_env, uint8_visual=False) 

    reward_func = "Step2_reward_inplace_target"
    log_dir = f"stable_results/ppo/{reward_func}/lr{lr}_batchsize{bs}/"
    os.makedirs(log_dir, exist_ok=True)
    second_env = Monitor(second_basic_env, log_dir, allow_early_resets=True)
    second_env = DummyVecEnv([lambda: second_env])  # The algorithms require a vectorized environment to run
    model2 = PPO.load("Step1_reward_just_fly")
    model2.learn(total_timesteps=70000)
    model2.save(log_dir + "/model")
    model2.save(reward_func)
    second_env.close()

    second_eval_unity_env = UnityEnvironment("Eval_Build/FirstEnv", worker_id=3)
    second_basic_eval_env = UnityToGymWrapper(second_eval_unity_env, uint8_visual=False) 

    episodes = 100
    success_counter = 0
    ep_r = []
    ep_l = []
    longfile = Path(f"{log_dir}/longlogs.txt")
    longfile.touch(exist_ok=True)
    logfile = Path(f"{log_dir}/readlogs.txt")
    logfile.touch(exist_ok=True)
    print(logfile)
    print(longfile)
    for e in range(episodes):
        obs = second_basic_eval_env.reset()
        total_r = 0.
        total_l = 0.
        while True:
            action, _states = model2.predict(obs)
            obs, reward, done, info = second_basic_eval_env.step(action)
            total_l += 1.
            if reward == -1:
                total_r = -1
            elif reward == 1:
                total_r = 1
                success_counter += 1
            else:
                total_r += reward
            if done:
                break
            with open(longfile, 'a') as file1:
                file1.write(f"Episode: {e}\n Observation:\n {obs}\n Action:\n {action}\n Action Reward: {reward}, Total Reward: {total_r}, Current Total Length: {total_l} \n\n")
        ep_r.append(total_r)
        ep_l.append(total_l)
        with open(logfile, 'a') as file2:
            file2.write(f"Episode: {e}, Total Reward: {total_r}, Total Length: {total_l} \n")
            if e == episodes - 1:
                file2.write("Final Episode Success Rate: {:0.3f}".format(success_counter/episodes))
    print("Episode Mean Reward: {:0.3f}, Mean Length: {:0.3f}, Success Rate: {:0.3f}".format(np.mean(ep_r), np.mean(ep_l), (success_counter/episodes)))
    with open('{}_eval.pkl'.format(log_dir), 'wb') as f:
        pickle.dump(ep_r, f)
        pickle.dump(ep_l, f)

    second_basic_eval_env.close()

    print("10 seconds to reset, then moving into 3rd learning step")
    time.sleep(10)

    third_unity_env = UnityEnvironment("Build/ThirdEnv", no_graphics=True, worker_id=2)
    third_basic_env = UnityToGymWrapper(third_unity_env, uint8_visual=False) 

    reward_func = "Step3_reward_moving_target"
    log_dir = f"stable_results/ppo/{reward_func}/lr{lr}_batchsize{bs}/"
    os.makedirs(log_dir, exist_ok=True)
    third_env = Monitor(third_basic_env, log_dir, allow_early_resets=True)
    third_env = DummyVecEnv([lambda: third_env])  # The algorithms require a vectorized environment to run
    model3 = PPO.load("Step2_reward_inplace_target")
    model3.learn(total_timesteps=70000)
    model3.save(log_dir + "/model")
    model3.save(reward_func)
    third_env.close()

    third_eval_unity_env = UnityEnvironment("Eval_Build/FirstEnv", worker_id=3)
    third_basic_eval_env = UnityToGymWrapper(third_eval_unity_env, uint8_visual=False) 

    episodes = 100
    success_counter = 0
    ep_r = []
    ep_l = []
    longfile = Path(f"{log_dir}/longlogs.txt")
    longfile.touch(exist_ok=True)
    logfile = Path(f"{log_dir}/readlogs.txt")
    logfile.touch(exist_ok=True)
    print(logfile)
    print(longfile)
    for e in range(episodes):
        obs = third_basic_eval_env.reset()
        total_r = 0.
        total_l = 0.
        while True:
            action, _states = model3.predict(obs)
            obs, reward, done, info = third_basic_eval_env.step(action)
            total_l += 1.
            if reward == -1:
                total_r = -1
            elif reward == 1:
                total_r = 1
                success_counter += 1
            else:
                total_r += reward
            if done:
                break
            with open(longfile, 'a') as file1:
                file1.write(f"Episode: {e}\n Observation:\n {obs}\n Action:\n {action}\n Action Reward: {reward}, Total Reward: {total_r}, Current Total Length: {total_l} \n\n")
        ep_r.append(total_r)
        ep_l.append(total_l)
        with open(logfile, 'a') as file2:
            file2.write(f"Episode: {e}, Total Reward: {total_r}, Total Length: {total_l} \n")
            if e == episodes - 1:
                file2.write("Final Episode Success Rate: {:0.3f}".format(success_counter/episodes))
    print("Episode Mean Reward: {:0.3f}, Mean Length: {:0.3f}, Success Rate: {:0.3f}".format(np.mean(ep_r), np.mean(ep_l), (success_counter/episodes)))
    with open('{}_eval.pkl'.format(log_dir), 'wb') as f:
        pickle.dump(ep_r, f)
        pickle.dump(ep_l, f)

    third_basic_eval_env.close()


