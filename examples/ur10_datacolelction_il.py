
#test self made ur10 with robotiq 85 gripper
#lots of issues there does not work (partially because of the coppliasim version,
# better with 4.1.0)
#6 Feb 2023

from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.arms.ur10 import UR10
from pyrep.robots.end_effectors.robotiq85_gripper import Robotiq85Gripper
from pyrep.objects import VisionSensor, Shape
import torch
import numpy as np
import math
from pyrep.errors import ConfigurationPathError
from pyrep.const import PrimitiveShape

print(torch.cuda.is_available())

pr = PyRep()
# Launch the application with a scene file in headless mode
SCENE_FILE = join(dirname(abspath(__file__)),'ur10_test_v4.ttt')
pr.launch(SCENE_FILE, headless=False)
pr.start()  # Start the simulation

agent = UR10()
gripper = Robotiq85Gripper()
# camera = VisionSensor('Vision_sensor')
# target = Shape('banana')
target = Shape.create(type=PrimitiveShape.SPHERE,
                      size=[0.05, 0.05, 0.05],
                      color=[1.0, 0.1, 0.1],
                      static=True, respondable=False)

images = []
depths = []
joint_data = []

# position_min, position_max = [0.8, -0.2, 1.0], [1.0, 0.2, 1.4]
# starting_joint_positions = agent.get_joint_positions()

LOOPS = 50
position_min, position_max = [-0.5, 0.3, 0.1], [0.5, 1.0, 0.5]

starting_joint_positions = agent.get_joint_positions()

for i in range(LOOPS):

    # Reset the arm at the start of each 'episode'
    agent.set_joint_positions(starting_joint_positions)

    # Get a random position within a cuboid and set the target position
    pos = list(np.random.uniform(position_min, position_max))
    target.set_position(pos)

    # Get a path to the target (rotate so z points down)
    try:
        path = agent.get_path(
            position=pos, euler=[0, math.radians(180), 0])
    except ConfigurationPathError as e:
        print('Could not find path')
        continue

    # Step the simulation and advance the agent along the path
    done = False
    while not done:
        done = path.step()
        pr.step()

    print('Reached target %d!' % i)

pr.stop()
pr.shutdown()

#
# LOOPS = 10
#
# for i in range(LOOPS):
#
#     # Reset the arm at the start of each 'episode'
#     agent.set_joint_positions(starting_joint_positions)
#     # Get a random position within a cuboid and set the target position
#     target_position = target.get_position()
#
#     # Get a path to the target (rotate so z points down)
#     try:
#         path = agent.get_path(
#             position=target_position, euler=[0, math.radians(180), 0])
#     except ConfigurationPathError as e:
#         print('Could not find path')
#         continue
#
#     # Step the simulation and advance the agent along the path
#     done = False
#     while not done:
#         done = path.step()
#         pr.step()
#
#     print('Reached target %d!' % i)
#
# # Do some stuff
# for i in range(1000):
#     target.set_position(np.random.uniform(-1.0, 1.0, size=3))
#     target_position = target.get_position()
#     ee_position = agent.get_tip().get_position()
#
#     # Open the gripper
#     gripper.release()
#     pr.step() # Step the physics simulation
#
#     print(i)
#     print(target_position)

    # episode_done = False
    # while not episode_done:
    #      # Capture observations from the vision sensor
    #      rgb_obs = camera.capture_rgb()
    #      depth_obs = camera.capture_depth()
    #      action = agent.act([rgb_obs, depth_obs]) # Neural network predicting actions
    #      agent.set_joint_target_velocities(action)
    #      pr.step() # Step the physics simulation
    #      # Check if the agent has reached the target
    #      episode_done = target.get_position() == agent.get_tip().get_position()

pr.stop()  # Stop the simulation
pr.shutdown()  # Close the application
