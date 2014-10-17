from utils.startToTree import StartToTree
from taskUtils import setupTableEnv

import nodes.meta
import nodes.robot
import nodes.robotChoice
from utils import utils
from utils import choice

from tableNodes.graspGlass import GraspGlass
from tableNodes.placeItemInBin import PlaceItemInBin
from tableNodes.moveNearObj import MoveNearObj

import numpy
import math
import random

hiddenGlassTable = {}
tables = []
glassContainer = {}

def add_arguments(parser):
    pass

def init_env(common):
    env = common['execEnv']
    robot = common['execRobot']

    for x in xrange(-3, 5, 3):
        for y in xrange(-2, 3, 2):
            if y != 0:
                tables.append(setupTableEnv.add_table(env, robot, x=x, y=y))

    setupTableEnv.set_robot_pose(env, robot, tables[0])
    bin = setupTableEnv.add_bin(env, x=0, y=0)

    hiddenGlassTable['val'] = random.choice(tables)
    glass = setupTableEnv.place_glass_on_table(env, hiddenGlassTable['val'], .5, 9.0/10)
    glassContainer['val'] = glass

def reset_env(env, robot):
    pass
    #setupTableEnv.place_obj_on_table(execEnv, table, glass, locs[0][0], locs[0][1])

def ssd(A, B):
    return sum((A-B)**2)

def detect_glass(robot, env):
    bestTable = None
    bestTableDist = float('inf')
    for table in tables:
        dist = ssd(table.GetTransform()[:3,3], robot.GetTransform()[:3,3])
        if dist < bestTableDist:
            bestTableDist = dist
            bestTable = table
    if bestTableDist < 4 and bestTable == hiddenGlassTable['val']:
        return bestTable, glassContainer['val']
    return bestTable, None

def get_plan(common):
    env = common['env']
    robot = common['robot']
    arm = robot.left_arm

    toBin = []
    bodyNames = [ b.GetName() for b in env.GetBodies() if b.GetName().startswith('glass') ]
    random.shuffle(bodyNames)
    toBin = bodyNames[0:1]

    glassLocs = tables

    glassLocPriors = {}
    for loc in glassLocs:
        glassLocPriors[loc] = 1

    def startFn(state, fsmState, fsm):
        utils.restoreEnv(env, robot, state)
        home_config = robot.configurations.get_configuration('home')[1][0:len(arm.GetIndices())]

        loc, glass = detect_glass(common['execRobot'], common['execEnv'])

        if not glassContainer['val'].IsEnabled():
            return None, None

        if glass == None:
            #NOTE if we were more confident we could set to 0
            glassLocPriors[loc] /= 2

            scoreSum = sum(glassLocPriors.values())
            scoreSelection = random.random()*scoreSum
            for loc, score in glassLocPriors.items():
                scoreSelection -= score
                if scoreSelection <= 0:
                    fsm.disableState(state)
                    return nodes.meta.Exec(MoveNearObj(loc, **common)), startFn
        else:
            fsm.disableState(state)
            return nodes.meta.Exec(nodes.meta.NormalizedSeq([
                MoveNearObj(glass, **common),
                GraspGlass(glass.GetName(), arm, **common),
                nodes.robot.PlanArmToConfig(config=home_config, armName=arm.GetName(), **common),
                MoveNearObj(env.GetKinBody('bin'), **common),
                PlaceItemInBin(glass.GetName(), arm, **common),
                nodes.robot.PlanArmToConfig(config=home_config, armName=arm.GetName(), **common),
            ])), startFn

    return nodes.meta.NormalizedFSM(startFn)
