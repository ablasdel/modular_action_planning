from components import coreNodeComponents as components
from utils.startToTree import StartToTree
from examples import setupTableEnv

from nodes import metaNodes
from nodes import robotNodes
from nodes import robotChoiceNodes
from utils import utils
from utils import choice

from tableActions.DynamicReconfigToBin import DynamicReconfigToBin

from tableActions.GrabGlass2 import GrabGlass
from tableActions.GraspBowlNode import GrabBowl
from tableActions.PlaceObjOnObj import PlaceObjOnObj
from tableActions.PlaceObjInBinNode import PlaceObjInBin

import numpy
import math
import random

hiddenGlassTable = {}
tables = []
glassContainer = {}

def init_env(env, robot):

    for x in xrange(-3, 5, 3):
        for y in xrange(-2, 3, 2):
            if y != 0:
                tables.append(setupTableEnv.add_table(env, robot, x=x, y=y))

    setupTableEnv.set_robot_pose(env, robot, tables[0])
    bin = setupTableEnv.add_bin(env, x=0, y=0)

    hiddenGlassTable['val'] = random.choice(tables)
    glass = setupTableEnv.place_glass_on_table(env, hiddenGlassTable['val'], .5, 9.0/10)
    glassContainer['val'] = glass
    raw_input('%')

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
    if 'startToTree' not in common:
        common['startToTree'] = StartToTree()
    arm = common['robot'].left_arm

    env = common['env']
    robot = common['robot']

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
        home_config = robot.configurations.get_configuration('home')[1][0:7]
        arm = robot.left_arm

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
                    return metaNodes.ExecNode(MoveNearObj(loc, **common)), startFn
        else:
            fsm.disableState(state)
            return metaNodes.ExecNode(metaNodes.PrioritizedSeqNode([
                MoveNearObj(glass, **common),
                GrabGlass(glass.GetName(), arm, **common),
                robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
                MoveNearObj(env.GetKinBody('bin'), **common),
                PlaceObjInBin(glass.GetName(), arm, **common),
                robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
            ])), startFn

    return metaNodes.PrioritizedFSMNode(startFn)

class MoveNearObj:
    def __init__(self, obj, *args, **kwargs):
        self.obj = obj
        robotNodes.setupWrappedRobotNodeHelper(self, kwargs)
        components.extend(self, [
            ('node', robotChoiceNodes.PlanBaseToXYThetaGoalGenChoiceNode(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToXYTheta ])
        ])
    def getStartGoalGenChoice(self, start):
        return choice.Choices({
            "side": choice.Choose(['x+', 'x-', 'y+', 'y-']),
            "p": choice.Uniform(-1, 1),
            "offset": choice.Uniform(0, .25),
            "theta": choice.Uniform(-math.pi, math.pi),
        })
    def startGoalGenChoiceToXYTheta(self, start, choice):
        obj_aabb = self.obj.ComputeAABB()
        robot_aabb = self.robot.ComputeAABB()

        if choice['side'] == 'x+':
            x = obj_aabb.pos()[0] + obj_aabb.extents()[0] + robot_aabb.extents()[0] + choice['offset']
            y = obj_aabb.pos()[1] + choice['p']*obj_aabb.extents()[1] + robot_aabb.extents()[1]
        elif choice['side'] == 'x-':
            x = obj_aabb.pos()[0] - obj_aabb.extents()[0] - robot_aabb.extents()[0] - choice['offset']
            y = obj_aabb.pos()[1] + choice['p']*obj_aabb.extents()[1] + robot_aabb.extents()[1]
        elif choice['side'] == 'y+':
            y = obj_aabb.pos()[1] + obj_aabb.extents()[1] + robot_aabb.extents()[1] + choice['offset']
            x = obj_aabb.pos()[0] + choice['p']*obj_aabb.extents()[0] + robot_aabb.extents()[0]
        elif choice['side'] == 'y-':
            y = obj_aabb.pos()[1] - obj_aabb.extents()[1] - robot_aabb.extents()[1] - choice['offset']
            x = obj_aabb.pos()[0] + choice['p']*obj_aabb.extents()[0] + robot_aabb.extents()[0]
        else:
            raise Exception('bad choice ' + str(choice['side']))

        return x, y, choice['theta']
    def extendTo(self, other):
        self.components.node.extendTo(other)
