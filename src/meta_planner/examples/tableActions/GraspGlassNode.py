from components import coreNodeComponents as components
from nodes import robotNodes
from nodes import robotChoiceNodes
from nodes import metaNodes
from utils import choice
from utils import utils

import math
import numpy as np

def GrabGlass(glassName, startToTree, arm, **common):
    armName = arm.GetName()
    node = metaNodes.PrioritizedSeqNode([ 
        robotNodes.MoveHandToNode(f1=.5, f2=.5, f3=.5, spread=0, handName=arm.hand.GetName(), **common),
        MoveToGrabGlass(glassName, armName=armName, numStarts=12, startToTree=startToTree, **common),
        MoveTowardsGlass(glassName, armName=armName, disable=[glassName], **common),
        robotNodes.MoveHandToNode(f1=1.5, f2=1.5, f3=1.5, spread=0, handName=arm.hand.GetName(), disable=[glassName], **common),
        robotNodes.GrabNode(objname=glassName, armName=armName, **common),
        robotNodes.PlanArmToEndEffectorOffsetNode(moveDir=[0,0,1], moveDist=.1, armName=armName, disable=[glassName], **common)
    ])
    return node

class MoveToGrabGlass:
    def __init__(self, glassName, *args, **kwargs):
        components.extend(self, [
            #('node', robotChoiceNodes.PlanArmToPoseGoalGenChoiceNode(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToPose ])
            ('node', robotChoiceNodes.PlanArmToPosesGoalGenChoicesNode(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToPose ])
        ])
        robotNodes.extendRobotNode(self, self.components.node)
        self.glassName = glassName
        self.armName = kwargs['armName']
    def getStartGoalGenChoice(self, start):
        with self.env:
            glass = self.env.GetKinBody(self.glassName)
            radius = glass.ComputeAABB().extents()[1]
        return choice.Choices({
            "graspAngle": choice.Uniform(-math.pi, math.pi),
            "distanceFromGlass": choice.Uniform(radius+.05, radius+.1)
        })
    def startGoalGenChoiceToPose(self, start, c):
        with self.env:
            glass = self.env.GetKinBody(self.glassName)
            pose_near_glass = glass.GetTransform()
        pose_near_glass[:3,:3] = utils.zrot(c['graspAngle'])
        pose_near_glass[:3,3] += np.dot(pose_near_glass[:3,:3], [0, c['distanceFromGlass'], 0])
        pose_near_glass[2,3] += .1

        pose_near_glass[:3,:3] = np.dot(pose_near_glass[:3,:3], utils.xrot(math.pi/2))

        pose_near_glass[:3,3] += np.dot(pose_near_glass[:3,:3], [0, 0, -.25])
        return pose_near_glass

class MoveTowardsGlass:
    def __init__(self, glassName, *args, **kwargs):
        components.extend(self, [
            ('node', robotChoiceNodes.PlanArmToEndEffectorOffsetGoalGenChoiceNode(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToEndEffectorOffset ])
        ])
        robotNodes.extendRobotNode(self, self.components.node)

        self.glassName = glassName
        self.armName = kwargs['armName']
    def getStartGoalGenChoice(self, start):
        return choice.Choices({ })
    def startGoalGenChoiceToEndEffectorOffset(self, start, c):
        with self.env:
            glass = self.env.GetKinBody(self.glassName)
            glassRadius = glass.ComputeAABB().extents()[1]
            arm = self.robot.GetManipulator(self.armName)
            moveDir = arm.GetEndEffectorTransform()[0:3,2]
            xyArm = arm.GetEndEffectorTransform()[0:2,3]
            xyGlass = glass.GetTransform()[0:2,3]
        moveDist = np.linalg.norm(xyArm-xyGlass) - .15 - glassRadius
        return moveDir, moveDist

