from components import coreNodeComponents as components
from nodes import robotNodes
from nodes import metaNodes
from utils import utils

import functools
import openravepy

class PlanArmToEndEffectorOffsetGoalGenChoiceNode:
    def __init__(self, *args, **kwargs):
        robotNodes.setupWrappedRobotNodeHelper(self, kwargs)
        components.extend(self, [
            ('choice', metaNodes.prioritizedComponents.PrioritizedSingleStartGoalGenChoiceGenSubnode(), 
                                                                            [ self.startGoalGenChoiceToSubnode ]),
        ])
    def setStartGoalGenChoiceToEndEffectorOffset(self, startGoalGenChoiceToEndEffectorOffset):
        self.startGoalGenChoiceToEndEffectorOffset = startGoalGenChoiceToEndEffectorOffset
    def startGoalGenChoiceToSubnode(self, start, c):
        utils.restoreEnv(self.env, self.robot, start)
        moveDir, moveDist = self.startGoalGenChoiceToEndEffectorOffset(start, c)
        #import IPython; IPython.embed()
        return robotNodes.PlanArmToEndEffectorOffsetNode(moveDir=moveDir, moveDist=moveDist, **self.kwargs)
    def execute(self, (start, goal, (node, execData))):
        self.disableNode()
        node.execute((start, goal, execData))
    def extendTo(self, other):
        self.components.choice.extendTo(other)
        other.execute = self.execute

class PlanBaseToXYThetaGoalGenChoiceNode:
    def __init__(self, *args, **kwargs):
        robotNodes.setupWrappedRobotNodeHelper(self, kwargs)
        components.extend(self, [
            ('choice', metaNodes.prioritizedComponents.PrioritizedSingleStartGoalGenChoiceGenSubnode(), 
                                                                [ self.startGoalGenChoiceToSubnode ]),
        ])
    def setStartGoalGenChoiceToXYTheta(self, startGoalGenChoiceToXYTheta):
        self.startGoalGenChoiceToXYTheta = startGoalGenChoiceToXYTheta
    def startGoalGenChoiceToSubnode(self, start, c):
        utils.restoreEnv(self.env, self.robot, start)
        x,y,theta = self.startGoalGenChoiceToXYTheta(start, c)
        return robotNodes.PlanBaseToXYThetaNode(x, y, theta, **self.kwargs)
    def execute(self, (start, goal, (node, execData))):
        self.disableNode()
        node.execute((start, goal, execData))
    def extendTo(self, other):
        self.components.choice.extendTo(other)
        other.execute = self.execute

#============================================================================================================

class PlanArmToPoseGoalGenChoiceNode:
    def __init__(self, *args, **kwargs):
        robotNodes.setupWrappedRobotNodeHelper(self, kwargs)
        kwargs['numStarts'] = 1
        components.extend(self, [
            ('choice', PlanArmToPosesGoalGenChoicesNode(**kwargs), [ ]),
        ])

class PlanArmToPosesGoalGenChoicesNode:
    def __init__(self, *args, **kwargs):
        robotNodes.setupWrappedRobotNodeHelper(self, kwargs)
        components.extend(self, [
            ('choice', metaNodes.prioritizedComponents.PrioritizedMultiStartsGoalGenChoicesGenSubnode(**kwargs), 
                                                                            [ self.startsGoalGenChoicesToSubnode ]),
        ])
    def setStartGoalGenChoiceToPose(self, startGoalGenChoiceToPose):
        self.startGoalGenChoiceToPose = startGoalGenChoiceToPose
    def startsGoalGenChoicesToSubnode(self, starts, choices):
        poses = []
        for start, c in zip(starts, choices):
            utils.restoreEnv(self.env, self.robot, start)
            pose = self.startGoalGenChoiceToPose(start, c)
            poses.append(pose)
        return robotNodes.PlanArmToPosesNode(poses=poses, **self.kwargs)
    def execute(self, (start, goal, (node, execData))):
        self.disableNode()
        node.execute((start, goal, execData))
    def extendTo(self, other):
        self.components.choice.extendTo(other)
        other.execute = self.execute
