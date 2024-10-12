from queue import Empty
import gym
from gym import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class WarehouseRobotEnv(gym.Env):
    def __init__(self):
        super(WarehouseRobotEnv, self).__init__()

        # Initialize ROS2 node
        rclpy.init(args=None)  # Initialization for ROS 2
        self.node = rclpy.create_node('warehouse_robot_env')

        # Define action and observation space
        # Actions: forward/backward, turn left/right
        self.action_space = spaces.Discrete(3)  # 0: forward, 1: left, 2: right

        # Observations: laser scan data (e.g., distances to obstacles)
        self.observation_space = spaces.Box(low=0, high=10, shape=(10,), dtype=np.float32)

        # Velocity publisher (to send velocity commands to the robot)
        self.vel_pub = self.node.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to the laser scan data
        self.scan_sub = self.node.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Store the laser scan data
        self.laser_data = np.zeros(10)

        # Define robot speed and turn speed
        self.linear_speed = 0.5
        self.angular_speed = 1.0

        # Store the current reward
        self.current_reward = 0

    def scan_callback(self, data):
        """Callback to process the laser scan data."""
        # Take a subset of the laser scan ranges (e.g., 10 values) for simplicity
        self.laser_data = np.array(data.ranges[0:10])

    def step(self, action):
        """Executes the robot action."""
        twist = Twist()
        if action == 0:  # Move forward
            twist.linear.x = self.linear_speed
            twist.angular.z = 0
        elif action == 1:  # Turn left
            twist.linear.x = 0
            twist.angular.z = self.angular_speed
        elif action == 2:  # Turn right
            twist.linear.x = 0
            twist.angular.z = -self.angular_speed

        # Publish the command to the robot
        self.vel_pub.publish(twist)

        # Calculate reward
        reward = self._get_reward()

        # Check if episode is done (e.g., robot collides or completes the task)
        done = self._is_done()

        # Return observation, reward, done, and additional info
        return self.laser_data, reward, done, {}

    def reset(self):
        """Reset the environment to its initial state."""
        # Reset the robot position (e.g., use ROS service for resetting Gazebo world)
        # Since rospy.ServiceProxy is not available in ROS 2, you can simulate a reset action
        # Reset reward and laser scan data
        self.current_reward = 0
        self.laser_data = np.zeros(10)

        # Return initial observation (laser scan data)
        return self.laser_data

    def render(self, mode='human'):
        """Render the environment (Gazebo already handles visualization)."""
        pass

    def close(self):
        """Cleanup when closing the environment."""
        self.node.destroy_node()
        rclpy.shutdown()

    def _get_reward(self):
        """Calculate the reward for the current step."""
        # Example reward logic: negative reward for close obstacles, positive for moving forward
        if np.min(self.laser_data) < 0.5:  # Obstacle very close
            reward = -1.0  # Penalize for being too close to an obstacle
        else:
            reward = 1.0  # Reward for safe navigation

        self.current_reward += reward
        return reward

    def _is_done(self):
        """Determine if the episode is done (e.g., collision or task completion)."""
        if np.min(self.laser_data) < 0.2:  # Collision threshold
            return True
        # Could also add more conditions like reaching a goal or time limits
        return False
