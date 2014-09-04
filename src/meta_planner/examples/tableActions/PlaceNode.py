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

def Place(objname, startToTree, arm, **common):
    armName = arm.GetName()
    node = metaNodes.PrioritizedSeqNode([ 
        MoveObjAboveBin(objname, startToTree=startToTree, numStarts=12, armName=armName, **common),
        MoveObjTowardsBin(objname, disable=['bin'], armName=armName,**common),
        robotNodes.MoveHandToNode(f1=.5, f2=.5, f3=.5, spread=0, disable=['bin'], handName=arm.hand.GetName(), **common),
        robotNodes.ReleaseNode(objname=objname, armName=arm.GetName(), **common),
        robotNodes.EnableNode(objname=objname, enabled=False, **common),
        MoveAboveBin(trajDisable=['bin'], numStarts=12, startToTree=startToTree, armName=armName, **common),
    ])
    return node

#class MoveObjAboveBin(robotComputeNode.PlanArmToTSR):
#    def initPlanArmToTSRNode(self, objname, binName):
#        self.objname = objname
#        self.binName = binName
#    def startChoiceToTSR(self, start, c):
#        rotation, extraRotation, offset = getObjSpecificProps(self.objname)
#        abovePose, aboveIk, tsr_chain_place, tsr_chain_constrain = placeAboveBin(self.env, self.robot, c, self.objname, rotation, offset, extraRotation)
#        return [tsr_chain_place, tsr_chain_constrain]
#    def getInternalChoice(self, env, robot, start):
#        return choice.Choices({
#            "x": choice.Uniform(-.25, .25),
#            "y": choice.Uniform(-.5, .5),
#            "z": choice.Uniform(-.2, .2),
#        })

class MoveObjAboveBin:
    def __init__(self, objname, *args, **kwargs):
        components.extend(self, [
            #('node', robotChoiceNodes.PlanArmToPoseGoalGenChoiceNode(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToPose ])
            ('node', robotChoiceNodes.PlanArmToPosesGoalGenChoicesNode(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToPose ])
        ])
        robotNodes.extendRobotNode(self, self.components.node)
        self.objname = objname
    def getStartGoalGenChoice(self, start):
        return choice.Choices({
            "x": choice.Uniform(-.25, .25),
            "y": choice.Uniform(-.5, .5),
            "z": choice.Uniform(-.2, .2),
            #"x": choice.Uniform(.1782, .1982),
            #"y": choice.Uniform(.216, .236),
            #"z": choice.Uniform(.089, .109),
        })
    def startGoalGenChoiceToPose(self, start, c):
        rotation, extraRotation, offset = getObjSpecificProps(self.objname)
        abovePose, aboveIk, tsr_chain_place, tsr_chain_constrain = placeAboveBin(self.env, self.robot, c, self.objname, rotation, offset, extraRotation)
        return abovePose

class MoveObjTowardsBin:
    def __init__(self, objname, *args, **kwargs):
        components.extend(self, [
            ('node', robotChoiceNodes.PlanArmToEndEffectorOffsetGoalGenChoiceNode(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToEndEffectorOffset ])
        ])
        robotNodes.extendRobotNode(self, self.components.node)
        self.objname = objname
    def getStartGoalGenChoice(self, start):
        return choice.Choices({ })
    def startGoalGenChoiceToEndEffectorOffset(self, start, c):
        with self.env:
            bin = self.env.GetKinBody('bin')
            bin_aabb = bin.ComputeAABB()
            obj = self.env.GetKinBody(self.objname)
            dist = obj.GetTransform()[2,3] - (bin.GetTransform()[2,3] - bin_aabb.extents()[2])
        return [0, 0, -1], dist

class MoveAboveBin:
    def __init__(self, *args, **kwargs):
        components.extend(self, [
            #('node', robotChoiceNodes.PlanArmToPoseGoalGenChoiceNode(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToPose ])
            ('node', robotChoiceNodes.PlanArmToPosesGoalGenChoicesNode(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToPose ])
        ])
        robotNodes.extendRobotNode(self, self.components.node)
        self.armName = kwargs['armName']
    def getStartGoalGenChoice(self, start):
        return choice.Choices({
            "x": choice.Uniform(-1, 1),
            "y": choice.Uniform(-1, 1),
            "z": choice.Uniform(.2, .4),
            #"x": choice.Uniform(.41, .61),
            #"y": choice.Uniform(-.13, -.33),
            #"z": choice.Uniform(.18, .38),
        })
    def startGoalGenChoiceToPose(self, start, c):
        with self.env:
            bin = self.env.GetKinBody('bin')
            arm = self.robot.GetManipulator(self.armName)
            pose = arm.GetEndEffectorTransform()
            pose[2,3] = getPoseAboveBin(bin, c['x'], c['y'], c['z'])[2,3]
            pose[:3,:3] = [ [-1, 0, 0], [0, 0, 1], [0, 1, 0] ] 
        return pose

def getObjSpecificProps(objname):
    if objname.startswith('plate'):
        rotation = np.dot(np.dot(utils.zrot(-math.pi/2), utils.yrot(math.pi/2)), utils.zrot(math.pi))
        extraRotation = utils.yrot(math.pi/8)
        offset = [ 0, 0, -.25 ]
    elif objname.startswith('bowl'):
        rotation = utils.zrot(math.pi*1/2) 
        extraRotation = None
        offset = [ .25, 0, 0 ]
    elif objname.startswith('glass'):
        rotation = np.dot(utils.xrot(math.pi/2), utils.zrot(math.pi))
        extraRotation = None
        offset = [ 0, 0, -.25 ] 
    return rotation, extraRotation, offset

def placeAboveBin(env, robot, c, objname, rotation, offset, extraRotation=None):
    with env:
        startArmConfig = robot.GetDOFValues()
        bin = env.GetKinBody('bin')
        place_obj = env.GetKinBody(objname)
    abovePose = getAboveBinPose(env, robot, objname, c['x'], c['y'], c['z'], rotation, offset)

    with env:
        arm = utils.getArmGrabbingObject(robot, place_obj)
        arm.SetActive()
        manip_idx = robot.GetActiveManipulatorIndex()

        ee_in_obj = np.dot(np.linalg.inv(place_obj.GetTransform()), arm.GetEndEffectorTransform())
        Bw = np.array([[0,0], [0,0], [0,0], [0,0], [0,0], [-math.pi, math.pi]])
        tsr_place = TSR(T0_w=abovePose, Bw=Bw, manip=manip_idx)
        tsr_chain_place = TSRChain(TSR=tsr_place,  sample_start=False, constrain=False, sample_goal=True)

        r = math.pi/6
        cx = r
        cy = r
        cz = math.pi
        Bw_constrain = np.array([  
                [-100,100],
                [-100,100],
                [-100,100],
                [-cx,cx  ],
                [-cy,cy  ],
                [-cz,cz  ],
                ])

        tsr_constrain = TSR(T0_w=place_obj.GetTransform(), Tw_e=ee_in_obj, Bw=Bw_constrain, manip=manip_idx)
        tsr_chain_constrain = TSRChain(TSR=tsr_constrain, constrain=True, sample_start=False, sample_goal=False)

    #with env:
    #    arm = utils.getArmGrabbingObject(robot, place_obj)
    #    aboveIk = arm.FindIKSolution(abovePose, openravepy.IkFilterOptions.CheckEnvCollisions)
    #if aboveIk != None:
    #    oldIk = robot.GetDOFValues()
    #    arm.SetDOFValues(aboveIk)
    #    A = openravepy.misc.DrawAxes(env, abovePose)
    #    print abovePose
    #    A.SetShow(True)
    #    raw_input()
    #    arm.SetDOFValues(oldIk)
    
    return abovePose, None, tsr_chain_place, tsr_chain_constrain

def getPoseAboveBin(bin, x, y, z):
    bin_aabb = bin.ComputeAABB()
    abovePose = np.eye(4)
    abovePose[0,3] = bin_aabb.pos()[0] + x*bin_aabb.extents()[0]
    abovePose[1,3] = bin_aabb.pos()[1] + y*bin_aabb.extents()[1]
    abovePose[2,3] = bin_aabb.pos()[2] + bin_aabb.extents()[2] + z
    abovePose[:3,:3] = bin.GetTransform()[:3,:3]
    abovePose[:3,:3] = np.dot(abovePose[:3,:3], utils.yrot(math.pi))
    #abovePose[:3,:3] = np.dot(abovePose[:3,:3], zrot(math.pi))
    #[ [0, -1, 0], [0, 0, 1], [-1, 0, 0] ] 
    return abovePose

def getAboveBinPose(env, robot, objname, xperc, yperc, z, rotation, offset):
    with env:
        bin = env.GetKinBody('bin')
        place_obj = env.GetKinBody(objname)
    with env:
        bin_aabb = bin.ComputeAABB()
        arm = utils.getArmGrabbingObject(robot, place_obj)

    with env:
        place_obj_aabb = place_obj.ComputeAABB()
        abovePose = getPoseAboveBin(bin, 0, 0, .2)

    abovePose[:3,:3] = np.dot(abovePose[:3,:3], rotation)

#            A = openravepy.misc.DrawAxes(env, abovePose)
#            A.SetShow(True)
#            raw_input()
    with env:
        abovePose[:3,3] += np.dot(bin.GetTransform()[:3,:3], [ xperc*bin_aabb.extents()[1], yperc*bin_aabb.extents()[0], z] )
    #    A = openravepy.misc.DrawAxes(env, abovePose)
    #    A.SetShow(True)
    #    raw_input()
    abovePose[:3,3] += np.dot(abovePose[:3,:3], offset)
    #    A = openravepy.misc.DrawAxes(env, abovePose)
    #    A.SetShow(True)
    #    raw_input()

    return abovePose
