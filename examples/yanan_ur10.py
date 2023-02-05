from os.path import dirname, join, abspath
from pyrep import PyRep
from pyrep.robots.arms.ur10 import UR10
from pyrep.robots.end_effectors.robotiq85_gripper import Robotiq85Gripper
from pyrep.objects import VisionSensor, Shape
import torch
import numpy as np

print(torch.cuda.is_available())

pr = PyRep()
# Launch the application with a scene file in headless mode
SCENE_FILE = join(dirname(abspath(__file__)),'ur10_test_v2.ttt')
pr.launch(SCENE_FILE, headless=False)
pr.start()  # Start the simulation

agent = UR10()
gripper = Robotiq85Gripper()
# camera = VisionSensor('kinect')
# target = Shape('target')

# print(camera)

# Do some stuff

pr.stop()  # Stop the simulation
pr.shutdown()  # Close the application
