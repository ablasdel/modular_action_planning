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
    tableManip.reset_env(env, robot, N=8)

def get_plan(common):
    arm = common['robot'].left_arm

    subnodes = []
    numToMove = 0
    env = common['env']
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('glass')])
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('bowl')])
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('plate')])

    for _ in xrange(numToMove):
        subnodes.append(nodes.meta.Checkpoint(Item2Bin(arm, **common)))

    return nodes.meta.Exec(nodes.meta.NormalizedSeq(subnodes))

