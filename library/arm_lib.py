from library import kine_lib2 as kl
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import time
import numpy as np

# copelia sim here-----------------------------
client = RemoteAPIClient()

# all simIK.* type of functions and constants
simIK = client.getObject('simIK')
sim = client.getObject('sim')
simBase = sim.getObject('/base_dyn')
simTip = sim.getObject('/base_dyn/tip')
# simTarget = sim.getObject('/base_dyn/target')
# sphere = sim.getObject('/base_dyn/manipSphere')
g_joint1 = sim.getObjectHandle('/base_dyn/gripper_joint1')
g_joint2 = sim.getObjectHandle('/base_dyn/gripper_joint2')
# cup_0 = sim.getObjectHandle('/Cup[0]/visible_transparent')
# cup_1 = sim.getObjectHandle('/Cup[1]/visible_transparent')
joint1 = sim.getObjectHandle('/base_dyn/joint1')
joint2 = sim.getObjectHandle('/base_dyn/joint2')
joint3 = sim.getObjectHandle('/base_dyn/joint3')
joint4 = sim.getObjectHandle('/base_dyn/joint4')

# to find the homegeneous matrix
simBase = sim.getObject('/base_dyn')


def move_arm(initial_guess, desired_position):
    desiredJointAngle, optimizedJoint = kl.angleTraj(
        initial_guess, desired_position)
    print(optimizedJoint)

    for i in range(50):
        sim.setJointTargetPosition(
            joint1, np.deg2rad(desiredJointAngle[i, 0]))
        sim.setJointTargetPosition(
            joint2, np.deg2rad(desiredJointAngle[i, 1]))
        sim.setJointTargetPosition(
            joint3, np.deg2rad(desiredJointAngle[i, 2]))
        sim.setJointTargetPosition(
            joint4, np.deg2rad(desiredJointAngle[i, 3]))
        time.sleep(0.1)

    return optimizedJoint.flatten()


def get_angle():
    a = sim.getJointTargetPosition(joint1)
    b = sim.getJointTargetPosition(joint2)
    c = sim.getJointTargetPosition(joint3)
    d = sim.getJointTargetPosition(joint4)
    print(a, b, c, d)


def rel_cup():
    sim.setJointTargetPosition(g_joint1, 10*3.14/180)
    sim.setJointTargetPosition(g_joint2, -10*3.14/180)
    time.sleep(2)


def grab_cup():
    sim.setJointTargetPosition(g_joint1, 30*3.14/180)
    sim.setJointTargetPosition(g_joint2, -30*3.14/180)
    time.sleep(2)


def to_disp(initial_guess):
    initial_guess = move_arm(initial_guess, np.array([1.1098, -0.00807, 0.1]))
    return initial_guess


def to_client(initial_guess):
    initial_guess = move_arm(initial_guess, np.array([0.7437,  0.92385, 0.06]))
    return initial_guess
