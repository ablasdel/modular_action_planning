from components import coreNodeComponents as components
from nodes import robotNodes
from nodes import metaNodes
from utils import choice
from utils import utils

from tableActions.GrabGlass2 import GrabGlass
from tableActions.GraspBowlNode import GrabBowl
from tableActions.SlideAndGraspPlateNode import SlideAndGraspPlate
from tableActions.PlaceObjInBinNode import PlaceObjInBin

from tableActions.GrabGlass2 import MoveToGlass
from tableActions.GraspBowlNode import MoveToBowl

import numpy
import openravepy
import random

class DynamicToBin:
    def __init__(self, arm, **common):
        robotNodes.setupWrappedRobotNodeHelper(self, common)
        components.extend(self, [
            #('choice', metaNodes.prioritizedComponents.PrioritizedSingleStartGoalGenChoiceGenSubnode(), 
            #                                                                [ self.startGoalGenChoiceToSubnode,
            #                                                                  self.getStartGoalGenChoice ]),
            ('choice', metaNodes.prioritizedComponents.PrioritizedSingleStartGenSubnode(), 
                                                                            [ self.startToSubnode ]),
        ])
        self.common = common
        self.arm = arm
    def startToSubnode(self, start):
        utils.restoreEnv(self.env, self.robot, start)
        if self.robot.GetName() == 'TIM':
            home_config = numpy.copy(self.robot.configurations.get_configuration('home')[1][0:6])
        else:
            home_config = numpy.copy(self.robot.configurations.get_configuration('home')[1][0:7])

        objnames = [ b.GetName() for b in self.env.GetBodies() if (b.GetName().startswith('glass') or b.GetName().startswith('tag_glass') or b.GetName().startswith('bowl') or b.GetName().startswith('plate')) and b.IsEnabled() ]

        objScores = {}
        for objname in objnames:
            objScores[objname] = 1

        #for objname in objnames:
        #    if objname.startswith('glass'):
        #        poseGen = MoveToGlass(objname, armName=self.arm.GetName(), **self.common)
        #    elif objname.startswith('bowl'):
        #        poseGen = MoveToBowl(objname, armName=self.arm.GetName(), **self.common)
        #    elif objname.startswith('plate'):
        #        poseGen = SlideAndGraspPlate(objname, self.arm, **self.common)
        #    else:
        #        raise Exception('unknown obj')

        #    for i in xrange(10):
        #        if objname.startswith('plate'):
        #            Choice = poseGen.getStartGoalGenChoice(start)
        #            choiceVector = choice.randomChoiceVector(Choice.getChoiceVector())
        #            c = Choice.processChoiceVector(list(choiceVector))
        #            try:
        #                subnode = poseGen.startGoalGenChoiceToSubnode(start, c)
        #                pose = subnode.nodes[1].kwargs['pose']
        #            except:
        #                continue
        #            continue
        #        else:
        #            Choice = poseGen.getStartGoalGenChoice(start)
        #            choiceVector = choice.randomChoiceVector(Choice.getChoiceVector())
        #            c = Choice.processChoiceVector(list(choiceVector))
        #            pose = poseGen.startGoalGenChoiceToPose(start, c)
        #        ik = self.arm.FindIKSolution(pose, openravepy.IkFilterOptions.CheckEnvCollisions)
        #        if ik != None:
        #            objScores[objname] += 1
        #print objScores
        scoreSum = sum(objScores.values())
        scoreSelection = random.random()*scoreSum
        for objname, score in objScores.items():
            scoreSelection -= score
            if scoreSelection < 0:
                if objname.startswith('glass') or objname.startswith('tag_glass'):
                    return metaNodes.PrioritizedSeqNode([
                        GrabGlass(objname, self.arm, **self.common),
                        PlaceObjInBin(objname, self.arm, **self.common),
                        robotNodes.PlanArmToConfigNode(config=home_config, armName=self.arm.GetName(), **self.common),
                    ])
                elif objname.startswith('bowl'):
                    return metaNodes.PrioritizedSeqNode([
                        GrabBowl(objname, self.arm, **self.common),
                        PlaceObjInBin(objname, self.arm, **self.common),

                        robotNodes.PlanArmToConfigNode(config=home_config, armName=self.arm.GetName(), **self.common),
                    ])
                elif objname.startswith('plate'):
                    return metaNodes.PrioritizedSeqNode([
                        SlideAndGraspPlate(objname, self.arm, **self.common),
                        #SlidePlate(c['objname'], self.arm, **self.common),
                        #GrabPlate(c['objname'], self.arm, **self.common),
                        PlaceObjInBin(objname, self.arm, **self.common),

                        robotNodes.PlanArmToConfigNode(config=home_config, armName=self.arm.GetName(), **self.common),
                    ])
                else:
                    raise Exception('unknown obj')


    #def startGoalGenChoiceToSubnode(self, start, c):
    #    home_config = numpy.copy(self.robot.configurations.get_configuration('home')[1][0:7])
    #    if c['objname'].startswith('glass'):
    #        return metaNodes.PrioritizedSeqNode([
    #            GrabGlass(c['objname'], self.arm, **self.common),
    #            PlaceObjInBin(c['objname'], self.arm, **self.common),
    #            robotNodes.PlanArmToConfigNode(config=home_config, armName=self.arm.GetName(), **self.common),
    #        ])
    #    elif c['objname'].startswith('bowl'):
    #        return metaNodes.PrioritizedSeqNode([
    #            GrabBowl(c['objname'], self.arm, **self.common),
    #            PlaceObjInBin(c['objname'], self.arm, **self.common),

    #            robotNodes.PlanArmToConfigNode(config=home_config, armName=self.arm.GetName(), **self.common),
    #        ])
    #    elif c['objname'].startswith('plate'):
    #        return metaNodes.PrioritizedSeqNode([
    #            SlideAndGraspPlate(c['objname'], self.arm, **self.common),
    #            #SlidePlate(c['objname'], self.arm, **self.common),
    #            #GrabPlate(c['objname'], self.arm, **self.common),
    #            PlaceObjInBin(c['objname'], self.arm, **self.common),

    #            robotNodes.PlanArmToConfigNode(config=home_config, armName=self.arm.GetName(), **self.common),
    #        ])
    #    else:
    #        raise Exception('unknown obj')
    #def getStartGoalGenChoice(self, start):
    #    utils.restoreEnv(self.env, self.robot, start)
    #    objnames = [ b.GetName() for b in self.env.GetBodies() if (b.GetName().startswith('glass') or b.GetName().startswith('bowl') or b.GetName().startswith('plate')) and b.IsEnabled() ]
    #    return choice.Choices({
    #        "objname": choice.Choose(objnames)
    #    })
    def extendTo(self, other):
        self.components.choice.extendTo(other)
