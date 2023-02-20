
#test self made ur10 with robotiq 85 gripper
#lots of issues there does not work (partially because of the coppliasim version,
# better with 4.1.0)
#6 Feb 2023

from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.arms.ur5 import UR5
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
SCENE_FILE = join(dirname(abspath(__file__)),'ur5_grasp.ttt')
pr.launch(SCENE_FILE, headless=False)
pr.start()  # Start the simulation

agent = UR5()
gripper = Robotiq85Gripper()
# camera = VisionSensor('Vision_sensor')
target = Shape('target')
# target = Shape.create(type=PrimitiveShape.SPHERE,
#                       size=[0.05, 0.05, 0.05],
#                       color=[1.0, 0.1, 0.1],
#                       static=True, respondable=False)

images = []
depths = []
joint_data = []

LOOPS = 100
position_min, position_max = [-0.5, 0.3, 0.1], [0.5, 1.0, 0.5]

starting_joint_positions = agent.get_joint_positions()
joint_plot = []
for i in range(LOOPS):

    # Reset the arm at the start of each 'episode'
    agent.set_joint_positions(starting_joint_positions)

    # Get a random position within a cuboid and set the target position
    pos = list(np.random.uniform(position_min, position_max))
    target.set_position(pos)
    pose = target.get_pose()

    # Get a path to the target (rotate so z points down)
    try:
        path = agent.get_path(position=pos, euler=pose)
    except ConfigurationPathError as e:
        print('Could not find path')
        continue

    # Step the simulation and advance the agent along the path
    done = False
    joints = []
    while not done:
        done = path.step()
        pr.step()
        # print(agent.get_joint_positions())
        joints.append(agent.get_joint_positions())
    joint_plot = joints
    print('Reached target %d!' % i)

print(joint_plot[6])
pr.stop()  # Stop the simulation
pr.shutdown()  # Close the application
