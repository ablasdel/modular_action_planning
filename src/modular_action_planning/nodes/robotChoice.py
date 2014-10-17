import components.core
import components.normalized
import nodes.robot
import nodes.meta
from utils import utils

import functools
import openravepy

class PlanArmToEndEffectorOffsetGoalGenChoice:
    def __init__(self, *args, **kwargs):
        nodes.rboot.setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('choice', components.normalized.NormalizedSingleStartGoalGenChoiceGenSubnode(), 
                                                                            [ self.startGoalGenChoiceToSubnode ]),
        ])
    def setStartGoalGenChoiceToEndEffectorOffset(self, startGoalGenChoiceToEndEffectorOffset):
        self.startGoalGenChoiceToEndEffectorOffset = startGoalGenChoiceToEndEffectorOffset
    def startGoalGenChoiceToSubnode(self, start, c):
        utils.restoreEnv(self.env, self.robot, start)
        moveDir, moveDist = self.startGoalGenChoiceToEndEffectorOffset(start, c)
        #import IPython; IPython.embed()
        return nodes.robot.PlanArmToEndEffectorOffset(moveDir=moveDir, moveDist=moveDist, **self.kwargs)
    def execute(self, (start, goal, (node, execData))):
        self.disableNode()
        node.execute((start, goal, execData))
    def extendTo(self, other):
        self.components.choice.extendTo(other)
        other.execute = self.execute

class PlanBaseToXYThetaGoalGenChoice:
    def __init__(self, *args, **kwargs):
        nodes.robot.setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('choice', components.normalized.NormalizedSingleStartGoalGenChoiceGenSubnode(), 
                                                                [ self.startGoalGenChoiceToSubnode ]),
        ])
    def setStartGoalGenChoiceToXYTheta(self, startGoalGenChoiceToXYTheta):
        self.startGoalGenChoiceToXYTheta = startGoalGenChoiceToXYTheta
    def startGoalGenChoiceToSubnode(self, start, c):
        utils.restoreEnv(self.env, self.robot, start)
        x,y,theta = self.startGoalGenChoiceToXYTheta(start, c)
        return nodes.robot.PlanBaseToXYTheta(x, y, theta, **self.kwargs)
    def execute(self, (start, goal, (node, execData))):
        self.disableNode()
        node.execute((start, goal, execData))
    def extendTo(self, other):
        self.components.choice.extendTo(other)
        other.execute = self.execute

#============================================================================================================

class PlanArmToPoseGoalGenChoice:
    def __init__(self, *args, **kwargs):
        nodes.robot.setupWrappedRobotNodeHelper(self, kwargs)
        kwargs['numStarts'] = 1
        components.core.extend(self, [
            ('choice', PlanArmToPosesGoalGenChoicesNode(**kwargs), [ ]),
        ])

class PlanArmToPosesGoalGenChoices:
    def __init__(self, *args, **kwargs):
        nodes.robot.setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('choice', components.normalized.NormalizedMultiStartsGoalGenChoicesGenSubnode(**kwargs), 
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
        return nodes.robot.PlanArmToPoses(poses=poses, **self.kwargs)
    def execute(self, (start, goal, (node, execData))):
        self.disableNode()
        node.execute((start, goal, execData))
    def extendTo(self, other):
        self.components.choice.extendTo(other)
        other.execute = self.execute
