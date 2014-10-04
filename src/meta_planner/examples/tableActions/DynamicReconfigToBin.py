from components import coreNodeComponents as components
from nodes import robotNodes
from nodes import metaNodes
from utils import choice
from utils import utils

from tableActions.GrabGlass2 import GrabGlass
from tableActions.GraspBowlNode import GrabBowl
from tableActions.SlideAndGraspPlateNode import SlideAndGraspPlate
from tableActions.PlaceObjInBinNode import PlaceObjInBin
from tableActions.PlaceObjOnObj import PlaceObjOnObj

import numpy

#class DynamicReconfigToBin:
#    def __init__(self, arm, toBin, **common):
#        robotNodes.setupWrappedRobotNodeHelper(self, common)
#        components.extend(self, [
#            ('choice', metaNodes.prioritizedComponents.PrioritizedSingleStartGenSubnode(), 
#                                                                            [ self.startToSubnode ]),
#        ])
#        self.toBin = common['toBin']
#        self.common = common
#        self.arm = arm
#    def startToSubnode(self):
#        return DynamicReconfigToBinChain(self.arm, self.toBin, **self.common)
#    def extendTo(self, other):
#        self.components.choice.extendTo(other)

class DynamicReconfigToBin:
    def __init__(self, arm, toBin, **common):
        robotNodes.setupWrappedRobotNodeHelper(self, common)
        components.extend(self, [
            ('seq', metaNodes.PrioritizedGrowingSeqNode(), [ self.genSubnode, self.isGoal ]),
        ])
        self.toBin = toBin
        self.common = common
        self.arm = arm
    def isGoal(self, state):
        utils.restoreEnv(self.env, self.robot, state)
        for itemName in self.toBin:
            if self.env.GetKinBody(itemName).IsEnabled():
                return False
        return True
    def genSubnode(self):
        return DynamicReconfigItemToBin(self.arm, self.toBin, **self.common)

class DynamicReconfigItemToBin:
    def __init__(self, arm, toBin, **common):
        robotNodes.setupWrappedRobotNodeHelper(self, common)
        components.extend(self, [
            ('choice', metaNodes.prioritizedComponents.PrioritizedSingleStartGoalGenChoiceGenSubnode(), 
                                                                            [ self.startGoalGenChoiceToSubnode,
                                                                              self.getStartGoalGenChoice ]),
        ])
        self.common = common
        self.toBin = toBin
        self.arm = arm
    def startGoalGenChoiceToSubnode(self, start, c):
        home_config = numpy.copy(self.robot.configurations.get_configuration('home')[1][0:7])

        if c['objname'] in self.toBin:
            if c['objname'].startswith('glass'):
                return metaNodes.PrioritizedSeqNode([
                    GrabGlass(c['objname'], self.arm, **self.common),
                    PlaceObjInBin(c['objname'], self.arm, **self.common),
                    robotNodes.PlanArmToConfigNode(config=home_config, armName=self.arm.GetName(), **self.common),
                ])
            elif c['objname'].startswith('bowl'):
                return metaNodes.PrioritizedSeqNode([
                    GrabBowl(c['objname'], self.arm, **self.common),
                    PlaceObjInBin(c['objname'], self.arm, **self.common),

                    robotNodes.PlanArmToConfigNode(config=home_config, armName=self.arm.GetName(), **self.common),
                ])
            elif c['objname'].startswith('plate'):
                return metaNodes.PrioritizedSeqNode([
                    SlideAndGraspPlate(c['objname'], self.arm, **self.common),
                    #SlidePlate(c['objname'], self.arm, **self.common),
                    #GrabPlate(c['objname'], self.arm, **self.common),
                    PlaceObjInBin(c['objname'], self.arm, **self.common),

                    robotNodes.PlanArmToConfigNode(config=home_config, armName=self.arm.GetName(), **self.common),
                ])
            else:
                raise Exception('unknown obj')
        else:
            if c['objname'].startswith('glass'):
                return metaNodes.PrioritizedSeqNode([
                    GrabGlass(c['objname'], self.arm, **self.common),
                    PlaceObjOnObj(c['objname'], 'table', self.arm, **self.common),
                    robotNodes.PlanArmToConfigNode(config=home_config, armName=self.arm.GetName(), **self.common),
                ])
            elif c['objname'].startswith('bowl'):
                return metaNodes.PrioritizedSeqNode([
                    GrabBowl(c['objname'], self.arm, **self.common),
                    PlaceObjOnObj(c['objname'], 'table', self.arm, **self.common),
                    robotNodes.PlanArmToConfigNode(config=home_config, armName=self.arm.GetName(), **self.common),
                ])
            elif c['objname'].startswith('plate'):
                return metaNodes.PrioritizedSeqNode([
                    SlideAndGraspPlate(c['objname'], self.arm, **self.common),
                    #SlidePlate(c['objname'], self.arm, **self.common),
                    #GrabPlate(c['objname'], self.arm, **self.common),
                    PlaceObjOnObj(c['objname'], 'table', self.arm, **self.common),
                    robotNodes.PlanArmToConfigNode(config=home_config, armName=self.arm.GetName(), **self.common),
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
