from utils.startToTree import StartToTree
from taskUtils import setupTableEnv

import nodes.meta
import nodes.robot

from tableNodes import symbolicDomain

import numpy

import random

def add_arguments(parser):
    pass

def init_env(common):
    env = common['execEnv']
    robot = common['execRobot']

    table = setupTableEnv.add_table(env, robot)
    setupTableEnv.set_robot_pose(env, robot, table)
    bin = setupTableEnv.add_bin(env)

    for x_ in xrange(1,3):
        for y_ in xrange(2,3):
            
            x = .5 + x_ / (2.0*3)
            y = .6 + y_ / (2.0*3)
            glass = setupTableEnv.place_glass_on_table(env, table, x, y)

def reset_env(env, robot):
    pass
    #setupTableEnv.place_obj_on_table(execEnv, table, glass, locs[0][0], locs[0][1])

def get_plan(common):
    #if 'startToTree' not in common:
    #    common['startToTree'] = StartToTree()
    env = common['env']
    robot = common['robot']
    objects, actionsByName, predicatesByName = symbolicDomain.get(env, robot, common)
    goal =  predicatesByName['robot_arm_home'](robot.left_arm.GetName()) 
    print objects
    goal = goal & predicatesByName['obj_posx_of_obj']('glass_3', 'glass_4')
        
    return nodes.meta.Exec(nodes.robot.PDDL(objects, actionsByName.values(), predicatesByName.values(), goal, **common))

