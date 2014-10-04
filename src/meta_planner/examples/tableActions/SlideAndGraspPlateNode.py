from components import coreNodeComponents as components
from components import metaNodeComponents as metaComponents
from components import prioritizedNodeComponents as prioritizedComponents
from nodes import robotNodes
from nodes import metaNodes
from utils import choice
from utils import utils
from utils import plateUtils

import numpy as np
import random
import openravepy

def SlideAndGraspPlate(plateName, arm, **common):
    armName = arm.GetName()
    node = SlideAndGraspPlateChoiceNode(plateName, armName, **common)
    return node

class SlideAndGraspPlateChoiceNode:
    def __init__(self, plateName, armName, **common):
        robotNodes.setupWrappedRobotNodeHelper(self, common)
        components.extend(self, [
            ('choice', metaNodes.prioritizedComponents.PrioritizedSingleStartGoalGenChoiceGenSubnode(), 
                                                                            [ self.startGoalGenChoiceToSubnode ]),
        ])

        self.setGetStartGoalGenChoice(self.getStartGoalGenChoice)
        self.plateName = plateName
        self.armName = armName
        self.common = common
    def startGoalGenChoiceToSubnode(self, start, c):
        utils.restoreEnv(self.env, self.robot, start)
        with self.env:
            plate = self.env.GetKinBody(self.plateName)
            plateRadius = plate.ComputeAABB().extents()[1]
            arm = self.robot.GetManipulator(self.armName)
        pose_above_plate, end_plate_pose, moveDir, moveLen = plateUtils.get_plate_poses(self.env, self.robot, c, arm, plate)
        with self.env:
            original_pose = plate.GetTransform()
            plate.SetTransform(end_plate_pose)
        pose_near_plate, tableDir, tableOffset = plateUtils.get_pose_near_plate(self.env, c, arm, plate)
        with self.env:
            plate.SetTransform(original_pose)
        config = arm.FindIKSolution(pose_near_plate, openravepy.IkFilterOptions.CheckEnvCollisions)
        if config == None:
            raise Exception('no ik')

        node = metaNodes.PrioritizedSeqNode([
            robotNodes.MoveHandToNode(f1=.5, f2=.5, f3=.5, spread=0, handName=arm.hand.GetName(), **self.common),
            robotNodes.PlanArmToPoseNode(pose=pose_above_plate, armName=self.armName, **self.common),
            robotNodes.MoveHandToNode(f1=.5, f2=.5, f3=0, spread=.5, disable=[self.plateName], handName=arm.hand.GetName(), **self.common),
            robotNodes.PlanArmToEndEffectorOffsetNode(moveDir=[0, 0, -1], moveDist=c['startDistanceAboveTable'], armName=self.armName, **self.common),
            robotNodes.GrabNode(objname=self.plateName, armName=arm.GetName(), **self.common),
            robotNodes.PlanArmToEndEffectorOffsetNode(moveDir=moveDir, moveDist=moveLen, armName=self.armName, **self.common),
            robotNodes.ReleaseNode(objname=self.plateName, armName=self.armName, **self.common),
            robotNodes.PlanArmToEndEffectorOffsetNode(moveDir=[0,0,1], moveDist=.1, armName=self.armName, **self.common),

            robotNodes.MoveHandToNode(f1=.5, f2=.5, f3=.5, spread=0, handName=arm.hand.GetName(), **self.common),
            robotNodes.PlanArmToPoseNode(pose=pose_near_plate, armName=self.armName, **self.common),
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
            #robotNodes.PlanArmToEndEffectorOffsetNode(moveDir=[0,0,1], moveDist=c['endDistanceAboveTable'], disable=[self.plateName, 'table'], armName=self.armName, **self.common),
        ])
        return node
    def getStartGoalGenChoice(self, start):
        with self.env:
            plate = self.env.GetKinBody(self.plateName)
            radius = plate.ComputeAABB().extents()[1]
            finger_length = .1
            return choice.Choices({
                "slideTargetOffset": choice.Uniform(-.5, .5),
                "tableEdge": choice.Choose([ 1, ]),
                #"tableEdge": choice.Choose([ 0, 1, 2, 3 ]), TODO 0,2, untested
                "startPlateOffset": choice.Uniform( radius + .03 - finger_length, 
                                                    radius + .1 - finger_length),
                "startDistanceAboveTable": choice.Uniform(.05, .15),
                "startDistanceFromPlate": choice.Uniform(.025, .1),
                #"endDistanceAboveTable": choice.Uniform(.05, .15),
            })

