import stable_baselines3 as sb3
from warehouse_robot_env import WarehouseRobotEnv  # Your custom environment

# Create the environment
env = WarehouseRobotEnv()

# Train a PPO agent
model = sb3.PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=10000)

# Save the model
model.save("ppo_warehouse_robot")

# Test the trained model
obs = env.reset()
done = False
while not done:
    action, _ = model.predict(obs)
    obs, reward, done, info = env.step(action)
    print(f"Action: {action}, Reward: {reward}")
