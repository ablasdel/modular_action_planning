from utils.startToTree import StartToTree
from examples import setupTableEnv
from components import coreNodeComponents as components

from nodes import metaNodes
from nodes import robotNodes

from tableActions.DynamicToBin import DynamicToBin
from utils import tableManip

import numpy
import random

def init_env(env, robot):

    table = setupTableEnv.add_table(env, robot)
    table.SetName('table')
    setupTableEnv.set_robot_pose(env, robot, table)
    bin = setupTableEnv.add_bin(env)
    tableManip.reset_env(env, robot, N=8)

def reset_env(env, robot):
    pass
    #tableManip.reset_env(env, robot)

def get_plan(common):
    if common['args'].startToTree:
        if 'startToTree' not in common:
            common['startToTree'] = StartToTree()
    else:
        common['startToTree'] = None#StartToTree()
    arm = common['robot'].left_arm

    subnodes = []
    numToMove = 0
    env = common['env']
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('glass')])
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('bowl')])
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('plate')])

    #common['planType'] = 'RRTConnect'
    common['numStarts'] = common['args'].numStarts

    for _ in xrange(numToMove):
        subnodes.append(metaNodes.CheckpointNode(DynamicToBin(arm, **common)))

    node = metaNodes.PrioritizedSeqNode(subnodes)
    return node
