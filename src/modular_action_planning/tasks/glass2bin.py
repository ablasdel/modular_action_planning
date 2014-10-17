from utils.startToTree import StartToTree
from taskUtils import setupTableEnv

import nodes.meta
import nodes.robot

from tableNodes.graspGlass import GraspGlass
from tableNodes.placeItemInBin import PlaceItemInBin

import numpy

def add_arguments(parser):
    pass

def init_env(common):
    env = common['execEnv']
    robot = common['execRobot']

    table = setupTableEnv.add_table(env, robot)
    setupTableEnv.set_robot_pose(env, robot, table)
    bin = setupTableEnv.add_bin(env)

    x = .5 + 2 / (2.0*3)
    y = .6 + 2 / (2.0*3)
    glass = setupTableEnv.place_glass_on_table(env, table, x, y)

def reset_env(env, robot):
    pass

def get_plan(common):
    for body in common['env'].GetBodies():
        if body.GetName().startswith('glass'):
            glass = body
            break
    arm = common['robot'].left_arm

    home_config = common['robot'].configurations.get_configuration('home')[1][0:len(arm.GetIndices())]

    return nodes.meta.Exec(
        nodes.meta.NormalizedSeq([
            GraspGlass(glass.GetName(), arm, **common),
            PlaceItemInBin(glass.GetName(), arm, **common),
            nodes.robot.PlanArmToConfig(config=home_config, armName=arm.GetName(), **common),
        ]))
