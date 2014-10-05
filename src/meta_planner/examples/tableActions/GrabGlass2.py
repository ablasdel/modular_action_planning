from components import coreNodeComponents as components
from nodes import robotNodes
from nodes import robotChoiceNodes
from nodes import metaNodes
from utils import choice
from utils import utils

import math
import numpy as np

def GrabGlass(glassName, arm, **common):
    armName = arm.GetName()
    node = metaNodes.PrioritizedSeqNode([ 
        robotNodes.MoveHandToNode(f1=.5, f2=.5, f3=.5, spread=0, handName=arm.hand.GetName(), **common),
        MoveToGlass(glassName, armName=armName, **common),
        robotNodes.MoveHandToNode(f1=1.8, f2=1.8, f3=1.8, spread=0, handName=arm.hand.GetName(), disable=[glassName], **common),
        robotNodes.GrabNode(objname=glassName, armName=armName, **common),
        #robotNodes.PlanArmToOffsetPoseNode(moveDir=[0,0,1], moveDist=.1, armName=armName, disable=[glassName], **common)
    ])
    return node

class MoveToGlass:
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
        })
    def startGoalGenChoiceToPose(self, start, c):
        with self.env:
            glass = self.env.GetKinBody(self.glassName)
            pose_near_glass = glass.GetTransform()
        pose_near_glass[:3,:3] = utils.zrot(c['graspAngle'])
        #pose_near_glass[:3,3] += np.dot(pose_near_glass[:3,:3], [0, c['distanceFromGlass'], 0])
        pose_near_glass[2,3] += .1

        pose_near_glass[:3,:3] = np.dot(pose_near_glass[:3,:3], utils.xrot(math.pi/2))

        pose_near_glass[:3,3] += np.dot(pose_near_glass[:3,:3], [0, 0, -.21])

        #import openravepy
        #ik = self.robot.left_arm.FindIKSolution(pose_near_glass, openravepy.IkFilterOptions.CheckEnvCollisions)
        #if ik != None:
        #    self.robot.left_arm.SetDOFValues(ik)
        #    raw_input('')
        return pose_near_glass

