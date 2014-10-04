from components import coreNodeComponents as components
from nodes import robotNodes
from nodes import robotChoiceNodes
from nodes import metaNodes
from utils import choice
from utils import utils

import math
import numpy as np

from prpy.tsr import *
from prpy.tsr.rodrigues import *
from prpy.tsr.tsr import *

def PlaceObjOnObj(objname, onObjname, arm, **common):
    armName = arm.GetName()
    subnodes = [
        MoveObjOnObj(objname, onObjname, armName=armName, **common),
        robotNodes.MoveHandToNode(f1=.5, f2=.5, f3=.5, spread=0, handName=arm.hand.GetName(), **common),
        robotNodes.ReleaseNode(objname=objname, armName=arm.GetName(), **common),
    ]
    if 'disableObj' in common:
        subnodes.append(robotNodes.EnableNode(objname=objname, enabled=False, **common))
    #subnodes.append(MoveAwayFromObj(objname, armName=armName, **common))
    return metaNodes.PrioritizedSeqNode(subnodes)

class MoveObjOnObj:
    def __init__(self, objname, onObjname, *args, **kwargs):
        robotNodes.setupWrappedRobotNodeHelper(self, kwargs)
        components.extend(self, [
            ('node', robotChoiceNodes.PlanArmToPosesGoalGenChoicesNode(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToPose ])
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

#class MoveAwayFromObj:
#    def __init__(self, objname, *args, **kwargs):
#        components.extend(self, [
#            ('node', robotChoiceNodes.PlanArmToPosesGoalGenChoicesNode(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToPose ])
#        ])
#        robotNodes.extendRobotNode(self, self.components.node)
#        self.armName = kwargs['armName']
#    def getStartGoalGenChoice(self, start):
#        return choice.Choices({
#            "d": choice.Uniform(0, .5),
#            "theta": choice.Uniform(0, 2*math.pi),
#        })
#    def startGoalGenChoiceToPose(self, start, c):
#        with self.env:
#            bin = self.env.GetKinBody('bin')
#            arm = self.robot.GetManipulator(self.armName)
#            pose = arm.GetEndEffectorTransform()
#            pose[2,3] = getPoseAboveBin(bin, c['x'], c['y'], c['z'])[2,3]
#            pose[:3,:3] = [ [-1, 0, 0], [0, 0, 1], [0, 1, 0] ] 
#        return pose
