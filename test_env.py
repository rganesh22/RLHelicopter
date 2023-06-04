import gym

from mlagents_envs.environment import UnityEnvironment
from gym_unity.envs import UnityToGymWrapper

unity_env = UnityEnvironment("Build/RLHelicopter", worker_id=3)
env = UnityToGymWrapper(unity_env, uint8_visual=False) 

observation = env.reset()

for episode in range(100000):
    initial_observation = env.reset()
    done = False
    episode_rewards = 0
    while not done:
        action = env.action_space.sample()
        observation, reward, done, info = env.step(action)
        print('observation', observation)
        print('action', action)
        episode_rewards += reward
    print("Total reward this episode: {}".format(episode_rewards))

env.close()