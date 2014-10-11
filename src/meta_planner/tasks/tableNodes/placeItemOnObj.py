import components.core
import nodes.meta
import nodes.robot
import nodes.robotChoice
from utils import choice
from utils import utils

import math
import numpy as np

from prpy.tsr import *
from prpy.tsr.rodrigues import *
from prpy.tsr.tsr import *

def PlaceItemOnObj(objname, onObjname, arm, **common):
    armName = arm.GetName()
    subnodes = [
        MoveObjOnObj(objname, onObjname, armName=armName, **common),
        nodes.robot.MoveHandTo(f1=.5, f2=.5, f3=.5, spread=0, handName=arm.hand.GetName(), **common),
        nodes.robot.Release(objname=objname, armName=arm.GetName(), **common),
    ]
    if 'disableObj' in common:
        subnodes.append(nodes.robot.Enable(objname=objname, enabled=False, **common))
    #subnodes.append(MoveAwayFromObj(objname, armName=armName, **common))
    return nodes.meta.NormalizedSeq(subnodes)

class MoveObjOnObj:
    def __init__(self, objname, onObjname, *args, **kwargs):
        nodes.robot.setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('node', nodes.robotChoice.PlanArmToPosesGoalGenChoices(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToPose ])
        ])
        self.armName = kwargs['armName']
        self.objname = objname
        self.onObjname = onObjname
    def getStartGoalGenChoice(self, start):
        return choice.Choices({
            "x": choice.Uniform(-1, 1),
            "y": choice.Uniform(-1, 1),
            "theta": choice.Uniform(0, 2*math.pi),
        })
    def startGoalGenChoiceToPose(self, start, c):
        arm = self.robot.GetManipulator(self.armName)
        obj = self.env.GetKinBody(self.objname)
        onObj = self.env.GetKinBody(self.onObjname)
        pose = arm.GetEndEffectorTransform()
        onObjAABB = onObj.ComputeAABB()
        fxyz = [
            onObjAABB.pos()[0] + onObjAABB.extents()[0]*c['x'],
            onObjAABB.pos()[1] + onObjAABB.extents()[1]*c['y'],
            onObjAABB.pos()[2] + onObjAABB.extents()[2] + .01,
        ]
        pose[0:3,3] += fxyz - obj.GetTransform()[0:3,3]

        onPose = np.eye(4)
        onPose[0:3,3] = fxyz

        #A = openravepy.misc.DrawAxes(self.env, pose)
        #B = openravepy.misc.DrawAxes(self.env, onPose)
        #A.SetShow(True)
        #B.SetShow(True)
        #raw_input('')
        #A.SetShow(False)
        #B.SetShow(False)

        return pose
