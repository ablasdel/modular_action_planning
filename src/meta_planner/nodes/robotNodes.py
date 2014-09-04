from components import coreNodeComponents as components
from components import metaNodeComponents as metaComponents
from components import prioritizedNodeComponents as prioritizedComponents
import metaNodes
from utils import utils
from utils import choice
from utils import planArmNodeHelper

import numpy as np
import random
import openravepy
import copy
import functools

import prpy.rave

#============================================================================================================

def setupWrappedRobotNodeHelper(node, kwargs):
    node.kwargs = kwargs
    node.env = kwargs['env']
    node.robot = kwargs['robot']
    node.execEnv = kwargs['execEnv']
    node.execRobot = kwargs['execRobot']
    node.disable = kwargs.get('disable', [])
    node.disablePadding = kwargs.get('disablePadding', [])

#============================================================================================================


class PlanArmToPoseNode:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.extend(self, [
            ('subnode', metaNodes.prioritizedComponents.PrioritizedSingleStartGenSubnode(), 
                                                            [ ('startToSubnode', self.chooseSubnode) ]),
        ])
    def chooseSubnode(self, start):
        pose = self.kwargs['pose']
        import hashlib
        poseHash = hashlib.sha1(pose.view(np.uint8)).hexdigest()

        trajDisable = self.kwargs.pop('trajDisable', [])
        with self.env:
            arm = self.robot.GetManipulator(self.kwargs['armName'])
            with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
                config = arm.FindIKSolution(pose, openravepy.IkFilterOptions.CheckEnvCollisions)
                if config == None:
                    raise Exception('no ik')
        disable = self.kwargs.pop('disable', [])
        return PlanArmToConfigNode(config=config, disable=disable+trajDisable, **self.kwargs)
    def execute(self, (start, goal, (node, execData))):
        self.disableNode()
        node.execute((start, goal, execData))
    def extendTo(self, other):
        self.components.subnode.extendTo(other)
        other.execute = self.execute

class PlanArmToConfigNode:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.extend(self, [
            ('prioritizedSingleStart', metaNodes.prioritizedComponents.PrioritizedSingleStart(), [ self.runWithStart ])
        ])
        self.startToTree = kwargs['startToTree']
        self.armName = kwargs['armName']
        self.config = kwargs['config']
        self.startGoalToTraj = {}
    def runWithStart(self, start):
        utils.restoreEnv(self.env, self.robot, start)
        with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
            if self.robot.CheckSelfCollision() or self.env.CheckCollision(self.robot):
                self.disableState(start)
                return
        with self.env:
            arm = self.robot.GetManipulator(self.armName)
            #startGoalPairs = planArmNodeHelper.planArmRRTConnect(
            #                                        self.env, self.robot, self.armName, 
            #                                        start, self.config, 
            #                                        self.startToTree, 
            #                                        self.disable, self.disablePadding)
            #startGoalPairs = planArmNodeHelper.planArmAnytimePRM(
            #                                        self.env, self.robot, self.armName, 
            #                                        start, self.config, 
            #                                        self.startToTree, 
            #                                        self.disable, self.disablePadding)
            startGoalPairs = planArmNodeHelper.planArmReuseMultiPRM(
                                                    self.env, self.robot, self.armName, 
                                                    [start], [self.config], 
                                                    self.startToTree, 
                                                    self.disable, self.disablePadding)
        for (start, goal, traj) in startGoalPairs:
            self.addConnectedGoal(goal)
            self.startGoalToTraj[(start, goal)] = traj
            self.addConnectedStartGoalPair((start, goal, None))
    def execute(self, (start, goal, _)):
        self.disableNode()
        traj = self.startGoalToTraj[(start, goal)]

        utils.restoreEnv(self.env, self.robot, start)
        with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
            self.simplifier = openravepy.RaveCreatePlanner(self.env, 'OMPLSimplifier')
            params = openravepy.Planner.PlannerParameters()
            params.SetExtraParameters('<time_limit>{:f}</time_limit>'.format(3))
            self.simplifier.InitPlan(self.robot, params)
            self.simplifier.PlanPath(traj)

        with utils.DisableWrapper(self.execEnv, self.disable, self.disablePadding):
            try:
                self.execRobot.ExecuteTrajectory(traj)
            except Exception as e:
                print e
                import IPython; IPython.embed()
                raise
    def extendTo(self, other):
        self.components.prioritizedSingleStart.extendTo(other)
        other.execute = self.execute

class PlanArmToEndEffectorOffsetNode:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.extend(self, [
            ('prioritizedSingleStart', metaNodes.prioritizedComponents.PrioritizedSingleStart(), [ self.runWithStart])
        ])
        self.armName = kwargs['armName']
        self.moveDir = kwargs['moveDir']
        self.moveDist = kwargs['moveDist']
        self.startGoalToTraj = {}
    def runWithStart(self, start):
        self.disableState(start)
        utils.restoreEnv(self.env, self.robot, start)
        with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
            if self.robot.CheckSelfCollision() or self.env.CheckCollision(self.robot):
                self.addDisabledStates(self.getConnectedStarts())
                return
        with self.env:
            arm = self.robot.GetManipulator(self.armName)
            finalPose = arm.GetEndEffectorTransform()
            finalPose[:3,3] += np.transpose(np.array(self.moveDir)*self.moveDist)
            with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
                finalPoseIk = arm.FindIKSolution(finalPose, openravepy.IkFilterOptions.CheckEnvCollisions)
                if finalPoseIk == None:
                    raise Exception('no ik')
        startGoalPairs = planArmNodeHelper.planEEOffsetCBiRRT(self.env, self.robot, self.armName, start, self.moveDir, self.moveDist, self.disable, self.disablePadding)
        for (start, goal, traj) in startGoalPairs:
            self.addConnectedGoal(goal)
            self.startGoalToTraj[(start, goal)] = traj
            self.addConnectedStartGoalPair((start, goal, None))
    def execute(self, (start, goal, _)):
        self.disableNode()
        traj = self.startGoalToTraj[(start, goal)]
        with utils.DisableWrapper(self.execEnv, self.disable, self.disablePadding):
            self.execRobot.ExecuteTrajectory(traj)
    def extendTo(self, other):
        self.components.prioritizedSingleStart.extendTo(other)
        other.execute = self.execute

class MoveHandToNode:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.extend(self, [
            ('prioritizedSingleStart', metaNodes.prioritizedComponents.PrioritizedSingleStart(), [ self.runWithStart ])
        ])
        self.handName = kwargs['handName']
        self.handKwargs = {}
        for e in [ 'f1', 'f2', 'f3', 'spread' ]:
            self.handKwargs[e] = kwargs[e]
        self.startGoalToTraj = {}
    def runWithStart(self, start):
        self.disableState(start)
        utils.restoreEnv(self.env, self.robot, start)
        hand = getHand(self.robot, self.handName)
        with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
            hand.MoveHand(**self.handKwargs)
        goal = utils.saveEnv(self.env, self.robot)
        self.addConnectedGoal(goal)
        self.addConnectedStartGoalPair((start, goal, None))
    def execute(self, (start, goal, _)):
        self.disableNode()
        hand = getHand(self.execRobot, self.handName)
        with utils.DisableWrapper(self.execEnv, self.disable, self.disablePadding):
            hand.MoveHand(**self.handKwargs)
    def extendTo(self, other):
        self.components.prioritizedSingleStart.extendTo(other)
        other.execute = execute

class GrabNode:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.extend(self, [
            ('prioritizedSingleStart', metaNodes.prioritizedComponents.PrioritizedSingleStart(), [ self.runWithStart ])
        ])
        self.objname = kwargs['objname']
        self.armName = kwargs['armName']
        self.startGoalToTraj = {}
    def runWithStart(self, start):
        self.disableState(start)
        utils.restoreEnv(self.env, self.robot, start)
        self.grab(self.robot, self.env)
        goal = utils.saveEnv(self.env, self.robot)
        self.addConnectedGoal(goal)
        self.addConnectedStartGoalPair((start, goal, None))
    def execute(self, (start, goal, _)):
        self.disableNode()
        self.grab(self.execRobot, self.execEnv)
    def grab(self, robot, env):
        with env:
            arm = robot.GetManipulator(self.armName)
            arm.SetActive()
            robot.Grab(env.GetKinBody(self.objname))
    def extendTo(self, other):
        self.components.prioritizedSingleStart.extendTo(other)
        other.execute = execute

class ReleaseNode:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.extend(self, [
            ('prioritizedSingleStart', metaNodes.prioritizedComponents.PrioritizedSingleStart(), [ self.runWithStart ])
        ])
        self.objname = kwargs['objname']
        self.startGoalToTraj = {}
    def runWithStart(self, start):
        self.disableState(start)
        utils.restoreEnv(self.env, self.robot, start)
        self.release(self.robot, self.env)
        goal = utils.saveEnv(self.env, self.robot)
        self.addConnectedGoal(goal)
        self.addConnectedStartGoalPair((start, goal, None))
    def execute(self, (start, goal, _)):
        self.disableNode()
        self.release(self.execRobot, self.execEnv)
    def release(self, robot, env):
        with env:
            robot.Release(env.GetKinBody(self.objname))
    def extendTo(self, other):
        self.components.prioritizedSingleStart.extendTo(other)
        other.execute = execute

class EnableNode:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.extend(self, [
            ('prioritizedSingleStart', metaNodes.prioritizedComponents.PrioritizedSingleStart(), [ self.runWithStart ])
        ])
        self.objname = kwargs['objname']
        self.enabled = kwargs['enabled']
        self.startGoalToTraj = {}
    def runWithStart(self, start):
        self.disableState(start)
        utils.restoreEnv(self.env, self.robot, start)
        self.setEnabled(self.env)
        goal = utils.saveEnv(self.env, self.robot)
        self.addConnectedGoal(goal)
        self.addConnectedStartGoalPair((start, goal, None))
    def execute(self, (start, goal, _)):
        self.disableNode()
        self.setEnabled(self.execEnv)
    def setEnabled(self, env):
        with env:
            obj = env.GetKinBody(self.objname)
            obj.Enable(self.enabled)
    def extendTo(self, other):
        self.components.prioritizedSingleStart.extendTo(other)
        other.execute = execute

#============================================================================================================

class PlanArmToPosesNode:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.extend(self, [
            ('subnode', metaNodes.prioritizedComponents.PrioritizedMultiStartsGenSubnode(**kwargs), 
                                                            [ ('startsToSubnode', self.chooseSubnode) ]),
        ])
    def chooseSubnode(self, starts):
        trajDisable = self.kwargs.pop('trajDisable', [])
        poses = self.kwargs['poses']
        configs = []
        goals = set()
        with self.env:
            arm = self.robot.GetManipulator(self.kwargs['armName'])
            for pose in poses:
                with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
                    config = arm.FindIKSolution(pose, openravepy.IkFilterOptions.CheckEnvCollisions)
                    if config == None:
                        continue
                    arm.SetDOFValues(config)
                goal = utils.saveEnv(self.env, self.robot)
                goals.add(goal)
                configs.append(config)
        if len(configs) == 0:
            raise Exception('no iks')
        disable = self.kwargs.pop('disable', [])
        return PlanArmToConfigsNode(configs=configs, disable=disable+trajDisable, **self.kwargs)
    def execute(self, (start, goal, (node, execData))):
        self.disableNode()
        node.execute((start, goal, execData))
    def extendTo(self, other):
        self.components.subnode.extendTo(other)
        other.execute = self.execute

class PlanArmToConfigsNode:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        #setupWrappedRobotNode(self, kwargs)
        components.extend(self, [
            ('choice', metaNodes.prioritizedComponents.PrioritizedMultiStarts(**kwargs), [ self.runWithStarts ])
        ])
        self.startToTree = kwargs['startToTree']
        self.armName = kwargs['armName']
        self.configs = kwargs['configs']
        self.startGoalToTraj = {}
    def runWithStarts(self, starts):
        for start in starts:
            utils.restoreEnv(self.env, self.robot, start)
            with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
                if self.robot.CheckSelfCollision() or self.env.CheckCollision(self.robot):
                    self.disableState(start)
                    starts.remove(start)
        if len(starts) == 0:
            raise Exception('no valid starts')
        with self.env:
            arm = self.robot.GetManipulator(self.armName)
            startGoalPairs = planArmNodeHelper.planArmReuseMultiPRM(
                                                    self.env, self.robot, self.armName, 
                                                    starts, self.configs, 
                                                    self.startToTree, 
                                                    self.disable, self.disablePadding)
        for (start, goal, traj) in startGoalPairs:
            self.addConnectedGoal(goal)
            self.startGoalToTraj[(start, goal)] = traj
            self.addConnectedStartGoalPair((start, goal, None))
    def execute(self, (start, goal, _)):
        self.disableNode()
        traj = self.startGoalToTraj[(start, goal)]

        utils.restoreEnv(self.env, self.robot, start)
        with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
            self.simplifier = openravepy.RaveCreatePlanner(self.env, 'OMPLSimplifier')
            params = openravepy.Planner.PlannerParameters()
            params.SetExtraParameters('<time_limit>{:f}</time_limit>'.format(3))
            self.simplifier.InitPlan(self.robot, params)
            self.simplifier.PlanPath(traj)

        with utils.DisableWrapper(self.execEnv, self.disable, self.disablePadding):
            try:
                self.execRobot.ExecuteTrajectory(traj)
            except Exception as e:
                print e
                import IPython; IPython.embed()
                raise
    def extendTo(self, other):
        self.components.choice.extendTo(other)
        other.execute = self.execute


#============================================================================================================

def extendRobotNode(node, robotNode):
    node.env = robotNode.env
    node.robot = robotNode.robot

def getHand(robot, handName):
    if handName == '/left/wam7':
        return robot.left_hand
    elif handName == '/right/wam7':
        return robot.right_hand
    else:
        raise Exception('unknown hand name: ' + str(handName))
