import components.core
import components.normalized
import nodes.meta
import nodes.robot
import nodes.robotChoice

from utils import utils

from graspGlass import GraspGlass
from graspBowl import GraspBowl
from slideAndGraspPlate import SlideAndGraspPlate
from placeItemInBin import PlaceItemInBin

import numpy
import openravepy
import random

class Item2Bin:
    def __init__(self, arm, **common):
        nodes.robot.setupWrappedRobotNodeHelper(self, common)
        components.core.extend(self, [
            ('choice', components.normalized.NormalizedSingleStartGenSubnode(), [ self.startToSubnode ]),
        ])
        self.common = common
        self.arm = arm
    def startToSubnode(self, start):
        utils.restoreEnv(self.env, self.robot, start)

        home_config = self.robot.configurations.get_configuration('home')[1][0:len(self.arm.GetIndices())]

        objnames = [ b.GetName() for b in self.env.GetBodies() if (b.GetName().startswith('glass') or b.GetName().startswith('tag_glass') or b.GetName().startswith('bowl') or b.GetName().startswith('plate')) and b.IsEnabled() ]

        objScores = {}
        for objname in objnames:
            objScores[objname] = 1
        scoreSum = sum(objScores.values())
        scoreSelection = random.random()*scoreSum
        for objname, score in objScores.items():
            scoreSelection -= score
            if scoreSelection < 0:
                if objname.startswith('glass') or objname.startswith('tag_glass'):
                    return nodes.meta.NormalizedSeq([
                        GraspGlass(objname, self.arm, **self.common),
                        PlaceItemInBin(objname, self.arm, **self.common),
                        nodes.robot.PlanArmToConfig(config=home_config, armName=self.arm.GetName(), **self.common),
                    ])
                elif objname.startswith('bowl'):
                    return nodes.meta.NormalizedSeq([
                        GraspBowl(objname, self.arm, **self.common),
                        PlaceItemInBin(objname, self.arm, **self.common),
                        nodes.robot.PlanArmToConfig(config=home_config, armName=self.arm.GetName(), **self.common),
                    ])
                elif objname.startswith('plate'):
                    return nodes.meta.NormalizedSeq([
                        SlideAndGraspPlate(objname, self.arm, **self.common),
                        #SlidePlate(c['objname'], self.arm, **self.common),
                        #GrabPlate(c['objname'], self.arm, **self.common),
                        PlaceItemInBin(objname, self.arm, **self.common),
                        nodes.robot.PlanArmToConfig(config=home_config, armName=self.arm.GetName(), **self.common),
                    ])
                else:
                    raise Exception('unknown obj')
    def extendTo(self, other):
        self.components.choice.extendTo(other)
