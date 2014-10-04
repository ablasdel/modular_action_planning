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
    #tableManip.reset_env(env, robot, N=2, includePlate=False, includeBowls=False)
    setupTableEnv.place_bowl_on_table(env, table, .7, .8)
    setupTableEnv.place_glass_on_table(env, table, .6, .7)
    #import IPython; IPython.embed()

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

    #robot = common['robot']
    #home_config = robot.configurations.get_configuration('home')[1][0:7]
    #relaxed_home_config = robot.configurations.get_configuration('relaxed_home')[1][0:7]
    #subnodes = [
    #    robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
    #    robotNodes.PlanArmToConfigNode(config=relaxed_home_config, armName=arm.GetName(), **common),
    #    robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
    #    robotNodes.PlanArmToConfigNode(config=relaxed_home_config, armName=arm.GetName(), **common),
    #    robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
    #]

    node = metaNodes.PrioritizedSeqNode(subnodes)
    return metaNodes.ExecNode(node)

