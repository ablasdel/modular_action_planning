from utils.startToTree import StartToTree
from taskUtils import setupTableEnv

import nodes.meta

from tableNodes.item2bin import Item2Bin
from taskUtils import tableManip

import numpy
import random

def add_arguments(parser):
    pass

def init_env(common):
    env = common['execEnv']
    robot = common['execRobot']

    table = setupTableEnv.add_table(env, robot)
    table.SetName('table')
    setupTableEnv.set_robot_pose(env, robot, table)
    bin = setupTableEnv.add_bin(env)

def reset_env(env, robot):
    tableManip.reset_env(env, robot, N=4)

def get_plan(common):
    arm = common['robot'].left_arm
    env = common['env']

    return nodes.meta.Exec(Item2Bin(arm, **common))


