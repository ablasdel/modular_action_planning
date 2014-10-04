from components import coreNodeComponents as components

from nodes import robotNodes
from nodes import metaNodes
from utils import choice

from tableActions.GrabGlass2 import GrabGlass
from tableActions.GraspBowlNode import GrabBowl
from tableActions.GraspPlateNode import GrabPlate
from tableActions.PlaceNode import Place
from tableActions.SlidePlateNode import SlidePlate
from tableActions.SlideAndGraspPlateNode import SlideAndGraspPlate

import numpy
import itertools

from utils import utils

def tableDynToBin(startToTree, arm, env, robot, execEnv, execRobot):
    common = {
        "robot": robot,
        "env": env,
        "execRobot": execRobot,
        "execEnv": execEnv,
    }

    subnodes = []
    numToMove = 0
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('glass')])
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('bowl')])
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('plate')])

    for _ in xrange(numToMove):
        subnodes.append(metaNodes.CheckpointNode(DynamicToBin(arm, startToTree, **common)))

    node = metaNodes.PrioritizedSeqNode(subnodes)
    return node

class DynamicToBin:
    def __init__(self, arm, startToTree, **common):
        robotNodes.setupWrappedRobotNodeHelper(self, common)
        components.extend(self, [
            ('choice', metaNodes.prioritizedComponents.PrioritizedSingleStartGoalGenChoiceGenSubnode(), 
                                                                            [ self.startGoalGenChoiceToSubnode,
                                                                              self.getStartGoalGenChoice ]),
        ])
        self.common = common
        self.startToTree = startToTree
        self.arm = arm
    def startGoalGenChoiceToSubnode(self, start, c):
        home_config = numpy.copy(self.robot.configurations.get_configuration('home')[1][0:7])
        if c['objname'].startswith('glass'):
            return metaNodes.PrioritizedSeqNode([
                GrabGlass(c['objname'], self.startToTree, self.arm, **self.common),
                Place(c['objname'], self.startToTree, self.arm, **self.common),
                robotNodes.PlanArmToConfigNode(config=home_config, startToTree=self.startToTree, armName=self.arm.GetName(), **self.common),
            ])
        elif c['objname'].startswith('bowl'):
            return metaNodes.PrioritizedSeqNode([
                GrabBowl(c['objname'], self.startToTree, self.arm, **self.common),
                Place(c['objname'], self.startToTree, self.arm, **self.common),

                robotNodes.PlanArmToConfigNode(config=home_config, startToTree=self.startToTree, armName=self.arm.GetName(), **self.common),
            ])
        elif c['objname'].startswith('plate'):
            return metaNodes.PrioritizedSeqNode([
                SlideAndGraspPlate(c['objname'], self.startToTree, self.arm, **self.common),
                #SlidePlate(c['objname'], self.startToTree, self.arm, **self.common),
                #GrabPlate(c['objname'], self.startToTree, self.arm, **self.common),
                Place(c['objname'], self.startToTree, self.arm, **self.common),

                robotNodes.PlanArmToConfigNode(config=home_config, startToTree=self.startToTree, armName=self.arm.GetName(), **self.common),
            ])
        else:
            raise Exception('unknown obj')
    def getStartGoalGenChoice(self, start):
        utils.restoreEnv(self.env, self.robot, start)
        objnames = [ b.GetName() for b in self.env.GetBodies() if (b.GetName().startswith('glass') or b.GetName().startswith('bowl') or b.GetName().startswith('plate')) and b.IsEnabled() ]
        return choice.Choices({
            "objname": choice.Choose(objnames)
        })
    def extendTo(self, other):
        self.components.choice.extendTo(other)

