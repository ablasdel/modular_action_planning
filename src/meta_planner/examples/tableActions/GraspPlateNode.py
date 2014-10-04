from components import coreNodeComponents as components
from components import metaNodeComponents as metaComponents
from components import prioritizedNodeComponents as prioritizedComponents
from nodes import robotNodes
from nodes import metaNodes
from nodes import robotChoiceNodes
from utils import choice
from utils import utils
from utils import plateUtils

import math
import random
import numpy as np

def GrabPlate(plateName, startToTree, arm, **common):
    armName = arm.GetName()
    node = GraspPlateChoiceNode(plateName, startToTree, armName, **common)
    return node

class GraspPlateChoiceNode:
    def __init__(self, plateName, startToTree, armName, **common):
        robotNodes.setupWrappedRobotNodeHelper(self, common)
        components.extend(self, [
            ('choice', metaNodes.prioritizedComponents.PrioritizedSingleStartGoalGenChoiceGenSubnode(), 
                                                                            [ self.startGoalGenChoiceToSubnode ]),
        ])
        self.setGetStartGoalGenChoice(self.getStartGoalGenChoice)
        self.startToTree = startToTree
        self.plateName = plateName
        self.armName = armName
        self.common = common
    def onDone(self, node, onDone):
        self.addDisabledStates(node.getConnectedStarts())
        onDone()
    def startGoalGenChoiceToSubnode(self, start, c):
        utils.restoreEnv(self.env, self.robot, start)
        with self.env:
            plate = self.env.GetKinBody(self.plateName)
            plateRadius = plate.ComputeAABB().extents()[1]
            arm = self.robot.GetManipulator(self.armName)
        pose_near_plate, tableDir, tableOffset = plateUtils.get_pose_near_plate(self.env, c, arm, plate)

        node = metaNodes.PrioritizedSeqNode([
            robotNodes.MoveHandToNode(f1=.5, f2=.5, f3=.5, spread=0, handName=arm.hand.GetName(), **self.common),
            robotNodes.PlanArmToPoseNode(pose=pose_near_plate, startToTree=self.startToTree, armName=self.armName, **self.common),
            robotNodes.MoveHandToNode(f1=1.5, f2=1.5, f3=0, spread=.5, disable=[self.plateName, 'table'], handName=arm.hand.GetName(), **self.common),
            robotNodes.PlanArmToEndEffectorOffsetNode(moveDir=tableDir, moveDist=tableOffset, disable=[self.plateName, 'table'], armName=self.armName, **self.common),
            robotNodes.PlanArmToEndEffectorOffsetNode(moveDir=-tableDir, moveDist=.01, disable=[self.plateName, 'table'], armName=self.armName, **self.common),
            robotNodes.MoveHandToNode(f1=1.5, f2=1.5, f3=3, spread=.5, disable=[self.plateName, 'table'], handName=arm.hand.GetName(), **self.common),
            robotNodes.MoveHandToNode(f1=3, f2=3, f3=3, spread=.5, disable=[self.plateName, 'table'], handName=arm.hand.GetName(), **self.common),

            robotNodes.GrabNode(objname=self.plateName, armName=arm.GetName(), **self.common),
            robotNodes.PlanArmToEndEffectorOffsetNode(moveDir=-tableDir, moveDist=.025, disable=[self.plateName, 'table'], armName=self.armName, **self.common),
            robotNodes.ReleaseNode(objname=self.plateName, armName=arm.GetName(), **self.common),

            robotNodes.MoveHandToNode(f1=1.4, f2=1.4, f3=1.2, spread=.5, disable=[self.plateName, 'table'], handName=arm.hand.GetName(), **self.common),
            robotNodes.PlanArmToEndEffectorOffsetNode(moveDir=tableDir, moveDist=.015, disable=[self.plateName, 'table'], armName=self.armName, **self.common),
            robotNodes.MoveHandToNode(f1=1.4, f2=1.4, f3=3, spread=.5, disable=[self.plateName, 'table'], handName=arm.hand.GetName(), **self.common),
            robotNodes.MoveHandToNode(f1=3, f2=3, f3=3, spread=.5, disable=[self.plateName, 'table'], handName=arm.hand.GetName(), **self.common),

            robotNodes.GrabNode(objname=self.plateName, armName=arm.GetName(), **self.common),
            robotNodes.PlanArmToEndEffectorOffsetNode(moveDir=[0,0,1], moveDist=c['endDistanceAboveTable'], disable=[self.plateName, 'table'], armName=self.armName, **self.common),
        ])
        return node
    def getStartGoalGenChoice(self, start):
        with self.env:
            plate = self.env.GetKinBody(self.plateName)
            radius = plate.ComputeAABB().extents()[1]
            finger_length = .1
            return choice.Choices({
                "startDistanceFromPlate": choice.Uniform(.025, .1),
                "endDistanceAboveTable": choice.Uniform(.05, .15),
            })