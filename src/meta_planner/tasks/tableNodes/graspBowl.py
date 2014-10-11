import components.core
import nodes.meta
import nodes.robot
import nodes.robotChoice
from utils import choice
from utils import utils

import math
import numpy as np

def GraspBowl(bowlName, arm, **common):
    armName = arm.GetName()
    node = nodes.meta.NormalizedSeq([ 
        nodes.robot.MoveHandTo(f1=.75, f2=.75, f3=.75, spread=0, handName=arm.hand.GetName(), **common),
        MoveToBowl(bowlName, armName=armName, **common),
        nodes.robot.MoveHandTo(f1=2.5, f2=2.5, f3=2.5, spread=0, handName=arm.hand.GetName(), disable=[bowlName], **common),
        nodes.robot.Grab(objname=bowlName, armName=armName, **common),
    ])
    return node

class MoveToBowl:
    def __init__(self, bowlName, *args, **kwargs):
        components.core.extend(self, [
            ('node', nodes.robotChoice.PlanArmToPosesGoalGenChoices(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToPose ])
        ])
        nodes.robot.extendRobotNode(self, self.components.node)
        self.bowlName = bowlName 
        self.armName = kwargs['armName']
    def getStartGoalGenChoice(self, start):
        with self.env:
            bowl = self.env.GetKinBody(self.bowlName)
            height = bowl.ComputeAABB().extents()[2]*2
        return choice.Choices({
            "graspAngle": choice.Uniform(-math.pi, math.pi),
        })
    def startGoalGenChoiceToPose(self, start, c):
        with self.env:
            bowl = self.env.GetKinBody(self.bowlName)
            pose_above_bowl = bowl.GetTransform()
            bowl_radius = bowl.ComputeAABB().extents()[1]
        pose_above_bowl[:3,:3] = utils.zrot(c['graspAngle'])
        pose_above_bowl[:3,3] += np.dot(pose_above_bowl[:3,:3], [0, bowl_radius, 0.01])
        pose_above_bowl[:3,:3] = np.dot(pose_above_bowl[:3,:3], utils.yrot(math.pi))
        pose_above_bowl[:3,:3] = np.dot(pose_above_bowl[:3,:3], utils.zrot(math.pi/2))
        pose_above_bowl[:3,3] += np.dot(pose_above_bowl[:3,:3], [0, 0, -.25])

        #import openravepy
        #config = self.robot.left_arm.FindIKSolution(pose_above_bowl, openravepy.IkFilterOptions.CheckEnvCollisions)
        #if config != None:
        #    orig_config = self.robot.left_arm.GetDOFValues()
        #    self.robot.left_arm.SetDOFValues(config)
        #    raw_input('%')
        #    self.robot.left_arm.SetDOFValues(orig_config)

        return pose_above_bowl
