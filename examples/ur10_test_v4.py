
import pyrep
import numpy as np
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


# Start the simulation
pr = pyrep.PyRep()
pr.launch('path_planning_UR10_gripper.ttt', headless=False)

# Load the robot and the target object
pr = PyRep()
# Launch the application with a scene file in headless mode
SCENE_FILE = join(dirname(abspath(__file__)),'ur10_test_v3.ttt')
pr.launch(SCENE_FILE, headless=False)
pr.start()  # Start the simulation

agent = UR10()
gripper = Robotiq85Gripper()
camera = VisionSensor('Vision_sensor')

# Get the initial position of the robot and target
start_position = agent.get_position()
target_position = target.get_position()

# Compute the vector pointing from the start to the target
path_vector = target_position - start_position

# Plan a straight line path from the start to the target
path = agent.get_linear_path(start_position, path_vector, 1000, 0.01)
agent.get_linear_path()
# Open the gripper
gripper.actuate(0, 0)

# Execute the path
agent.follow_path(path)

# Close the gripper
gripper.actuate(1, 1)

# Stop the simulation
pr.stop()
pr.shutdown()

