from utils.startToTree import StartToTree
from examples import setupTableEnv

from nodes import metaNodes
from nodes import robotNodes

from tableActions.DynamicReconfigToBin import DynamicReconfigToBin

from tableActions import symbolicDomain

import numpy

import random

def init_env(env, robot):

    table = setupTableEnv.add_table(env, robot)
    setupTableEnv.set_robot_pose(env, robot, table)
    bin = setupTableEnv.add_bin(env)

    for x_ in xrange(1,3):
        for y_ in xrange(1,3):
            
            x = .5 + x_ / (2.0*3)
            y = .6 + y_ / (2.0*3)
            glass = setupTableEnv.place_glass_on_table(env, table, x, y)

def reset_env(env, robot):
    pass
    #setupTableEnv.place_obj_on_table(execEnv, table, glass, locs[0][0], locs[0][1])

"""
    FSM:
        searching
            exec
                plan to location from distribution
            execCheck
                go to fetch | update distributions, go to search
        fetching
            grasp, bring to table
"""

def get_plan(common):
    if 'startToTree' not in common:
        common['startToTree'] = StartToTree()
    env = common['env']
    robot = common['robot']
    objects, actionsByName, predicatesByName = symbolicDomain.get(env, robot, common)
    fsm = [
        (SearchNode(obj), functools.bind(found, obj)),
    ]
    #return robotNodes.PDDLNode(objects, actionsByName.values(), predicatesByName.values(), predicatesByName['arm_home'](robot.left_arm.GetName()), **common);

