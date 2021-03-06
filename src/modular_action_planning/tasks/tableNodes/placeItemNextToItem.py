import components.core
import nodes.meta
import nodes.robot
import nodes.robotChoice
from utils import choice
from utils import utils

import math
import numpy as np

def PlaceItemNextToItem(objname, refObjname, onObjname, placeParameters, arm, **common):
    armName = arm.GetName()
    subnodes = [
        MoveObjNextToObj(objname, refObjname, onObjname, placeParameters, armName=arm.GetName(), **common),
        nodes.robot.MoveHandTo(f1=.5, f2=.5, f3=.5, spread=0, handName=arm.hand.GetName(), **common),
        nodes.robot.Release(objname=objname, armName=arm.GetName(), **common),
    ]
    if 'disableObj' in common:
        subnodes.append(nodes.rboot.Enable(objname=objname, enabled=False, **common))
    return nodes.meta.NormalizedSeq(subnodes)

class MoveObjNextToObj:
    def __init__(self, objname, refObjname, onObjname, placeParameters,  *args, **kwargs):
        nodes.robot.setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('node', nodes.robotChoice.PlanArmToPosesGoalGenChoices(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToPose ])
        ])
        self.armName = kwargs['armName']
        self.objname = objname
        self.onObjname = onObjname
        self.refObjname = refObjname
        self.placeParameters = placeParameters
    def getStartGoalGenChoice(self, start):
        return choice.Choices({
            "x": choice.Uniform(self.placeParameters[0] - self.placeParameters[2], 
                                self.placeParameters[0] + self.placeParameters[2]),
            "y": choice.Uniform(self.placeParameters[1] - self.placeParameters[3], 
                                self.placeParameters[1] + self.placeParameters[3]),
            "theta": choice.Uniform(0, 2*math.pi),
        })
    def startGoalGenChoiceToPose(self, start, c):
        arm = self.robot.GetManipulator(self.armName)
        obj = self.env.GetKinBody(self.objname)
        onObj = self.env.GetKinBody(self.onObjname)
        refObj = self.env.GetKinBody(self.refObjname)
        pose = arm.GetEndEffectorTransform()
        onObjAABB = onObj.ComputeAABB()
        refObjAABB = refObj.ComputeAABB()
        fxyz = [
            refObjAABB.pos()[0] + c['x'],
            refObjAABB.pos()[1] + c['y'],
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

