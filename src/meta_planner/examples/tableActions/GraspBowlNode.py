from components import coreNodeComponents as components
from nodes import robotNodes
from nodes import robotChoiceNodes
from nodes import metaNodes
from utils import choice
from utils import utils

import math
import numpy as np

def GrabBowl(bowlName, startToTree, arm, **common):
    armName = arm.GetName()
    node = metaNodes.PrioritizedSeqNode([ 
        robotNodes.MoveHandToNode(f1=.5, f2=.5, f3=.5, spread=0, handName=arm.hand.GetName(), **common),
        MoveToGrabBowl(bowlName, armName=armName, numStarts=12, startToTree=startToTree, **common),
        MoveTowardsBowl(bowlName, armName=armName, disable=[bowlName], **common),
        robotNodes.MoveHandToNode(f1=1.5, f2=1.5, f3=1.5, spread=0, handName=arm.hand.GetName(), disable=[bowlName], **common),
        robotNodes.GrabNode(objname=bowlName, armName=armName, **common),
        robotNodes.PlanArmToEndEffectorOffsetNode(moveDir=[0,0,1], moveDist=.1, armName=armName, disable=[bowlName], **common)
    ])
    return node

class MoveToGrabBowl:
    def __init__(self, bowlName, *args, **kwargs):
        components.extend(self, [
            #('node', robotChoiceNodes.PlanArmToPoseGoalGenChoiceNode(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToPose ])
            ('node', robotChoiceNodes.PlanArmToPosesGoalGenChoicesNode(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToPose ])
        ])
        robotNodes.extendRobotNode(self, self.components.node)
        self.bowlName = bowlName 
        self.armName = kwargs['armName']
    def getStartGoalGenChoice(self, start):
        with self.env:
            bowl = self.env.GetKinBody(self.bowlName)
            height = bowl.ComputeAABB().extents()[2]*2
        return choice.Choices({
            "graspAngle": choice.Uniform(-math.pi, math.pi),
            "distanceAboveTable": choice.Uniform(height+.05, height+.1),
            #"graspAngle": choice.Uniform(2.019, 2.219),
            #"distanceAboveTable": choice.Uniform(.021, .221),
        })
    def startGoalGenChoiceToPose(self, start, c):
        with self.env:
            bowl = self.env.GetKinBody(self.bowlName)
            pose_above_bowl = bowl.GetTransform()
            bowl_radius = bowl.ComputeAABB().extents()[1]
        pose_above_bowl[:3,:3] = utils.zrot(c['graspAngle'])
        pose_above_bowl[:3,3] += np.dot(pose_above_bowl[:3,:3], [0, bowl_radius, c['distanceAboveTable']])
        pose_above_bowl[:3,:3] = np.dot(pose_above_bowl[:3,:3], utils.yrot(math.pi))
        pose_above_bowl[:3,:3] = np.dot(pose_above_bowl[:3,:3], utils.zrot(math.pi/2))
        pose_above_bowl[:3,3] += np.dot(pose_above_bowl[:3,:3], [0, 0, -.25])

        return pose_above_bowl

class MoveTowardsBowl:
    def __init__(self,bowlName, *args, **kwargs):
        components.extend(self, [
            ('node', robotChoiceNodes.PlanArmToEndEffectorOffsetGoalGenChoiceNode(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToEndEffectorOffset ])
        ])
        robotNodes.extendRobotNode(self, self.components.node)
        self.bowlName = bowlName
        self.armName = kwargs['armName']
    def getStartGoalGenChoice(self, start):
        return choice.Choices({ })
    def startGoalGenChoiceToEndEffectorOffset(self, start, c):
        with self.env:
            bowl = self.env.GetKinBody(self.bowlName)
            bowlHeight = bowl.ComputeAABB().extents()[2]*2
            arm = self.robot.GetManipulator(self.armName)
            moveDir = arm.GetEndEffectorTransform()[0:3,2]
            zArm = arm.GetEndEffectorTransform()[2,3]
            zBowl = bowl.GetTransform()[2,3]
        return moveDir, np.linalg.norm(zArm-zBowl) -.2 - bowlHeight

