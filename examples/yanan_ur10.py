
#test self made ur10 with robotiq 85 gripper
#lots of issues there does not work
#6 Feb 2023

from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.arms.ur10 import UR10
from pyrep.robots.end_effectors.robotiq85_gripper import Robotiq85Gripper
from pyrep.objects import VisionSensor, Shape
import torch
import numpy as np


pr = PyRep()
# Launch the application with a scene file in headless mode
SCENE_FILE = join(dirname(abspath(__file__)),'ur10_test_v3.ttt')
pr.launch(SCENE_FILE, headless=False)
pr.start()  # Start the simulation

agent = UR10()
gripper = Robotiq85Gripper()
camera = VisionSensor('Vision_sensor')
target = Shape('banana')


# Do some stuff
training = True
while training:
    target.set_position(np.random.uniform(-1.0, 1.0, size=3))
    episode_done = False
    while not episode_done:
         # Capture observations from the vision sensor
         rgb_obs = camera.capture_rgb()
         depth_obs = camera.capture_depth()
         action = agent.act([rgb_obs, depth_obs]) # Neural network predicting actions
         agent.set_joint_target_velocities(action)
         pr.step() # Step the physics simulation
         # Check if the agent has reached the target
         episode_done = target.get_position() == agent.get_tip().get_position()

pr.stop()  # Stop the simulation
pr.shutdown()  # Close the application
