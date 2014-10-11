import components.core
import components.meta
import components.normalized
import components.robot
import nodes.meta
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

class PlanArmToEndEffectorOffset:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('prioritizedSingleStart', nodes.meta.components.normalized.NormalizedSingleStart(), [ self.runWithStart])
        ])
        self.armName = kwargs['armName']
        self.moveDir = kwargs['moveDir']
        self.moveDist = kwargs['moveDist']
        self.startGoalToTraj = {}
    def runWithStart(self, start):
        self.disableState(start)
        utils.restoreEnv(self.env, self.robot, start)
        with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
            with self.env:
                if self.robot.CheckSelfCollision() or self.env.CheckCollision(self.robot):
                    self.addDisabledStates(self.getConnectedStarts())
                    return
        #print(self.robot.left_arm.GetEndEffectorTransform())
        #_input = raw_input('EE PLAN')
        #if _input == 'a':
        #    import IPython; IPython.embed()
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

class PlanBaseToXYTheta:
    def __init__(self, x, y, theta, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('prioritizedSingleStart', nodes.meta.components.normalized.NormalizedSingleStart(), [ self.runWithStart])
        ])
        self.x = x
        self.y = y
        self.theta = theta
        self.startGoalToTraj = {}
    def runWithStart(self, start):
        with self.env:
            pose = self.robot.GetTransform()
            original_pose = self.robot.GetTransform()
            pose[0,3] = self.x
            pose[1,3] = self.y
            pose[:3,:3] = utils.zrot(self.theta)
            self.robot.SetTransform(pose)
            if self.env.CheckCollision(self.robot):
                return
            self.robot.SetTransform(original_pose)
            traj = self.robot.sbpl_planner.PlanToBasePose(self.robot, pose, timelimit=3.0, return_first=False)
            self.robot.SetTransform(pose)
            goal = utils.saveEnv(self.env, self.robot)
            if traj != None:
                self.addConnectedGoal(goal)
                self.addConnectedStartGoalPair((start, goal, None))
                self.startGoalToTraj[(start, goal)] = traj
    def execute(self, (start, goal, _)):
        sim_traj = self.startGoalToTraj[(start, goal)]

        traj = openravepy.RaveCreateTrajectory(self.execEnv, '')
        traj.deserialize(sim_traj.serialize())
        with utils.DisableWrapper(self.execEnv, self.disable, self.disablePadding):
            self.execRobot.ExecuteTrajectory(traj)
        utils.restoreEnv(self.execEnv, self.execRobot, goal)
        #self.execRobot.SetTransform(sim_traj)
    def extendTo(self, other):
        self.components.prioritizedSingleStart.extendTo(other)
        other.execute = self.execute

class PlanArmToOffsetPose:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('subnode', nodes.meta.components.normalized.NormalizedSingleStartGenSubnode(), 
                                                            [ ('startToSubnode', self.chooseSubnode) ]),
        ])
        self.armName = kwargs['armName']
        self.moveDir = kwargs['moveDir']
        self.moveDist = kwargs['moveDist']
    def chooseSubnode(self, start):
        utils.restoreEnv(self.env, self.robot, start)
        arm = self.robot.GetManipulator(self.armName)
        pose = arm.GetEndEffectorTransform()
        pose[:3,3] += np.transpose(np.array(self.moveDir)*self.moveDist)
        return PlanArmToPose(pose=pose, **self.kwargs)
    def execute(self, (start, goal, (node, execData))):
        self.disableNode()
        node.execute((start, goal, execData))
    def extendTo(self, other):
        self.components.subnode.extendTo(other)
        other.execute = self.execute

class MoveHandTo:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('node', components.normalized.NormalizedAllStartsGoalGen(), [ self.startGoalGen ]),
            #('prioritizedSingleStart', nodes.meta.components.normalized.NormalizedSingleStart(), [ self.runWithStart ])
        ])
        self.handName = kwargs['handName']
        self.handKwargs = {}
        for e in [ 'f1', 'f2', 'f3', 'spread' ]:
            self.handKwargs[e] = kwargs[e]
        self.startGoalToTraj = {}
    def startGoalGen(self, start):
        #self.disableState(start)
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
        #import IPython; IPython.embed()
        with utils.DisableWrapper(self.execEnv, self.disable, self.disablePadding):
            hand.MoveHand(timeout=2, **self.handKwargs)
    def extendTo(self, other):
        #self.components.prioritizedSingleStart.extendTo(other)
        self.components.node.extendTo(other)
        other.execute = execute

class Grab:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('node', components.normalized.NormalizedAllStartsGoalGen(), [ self.startGoalGen ]),
            #('prioritizedSingleStart', nodes.meta.components.normalized.NormalizedSingleStart(), [ self.runWithStart ])
        ])
        self.objname = kwargs['objname']
        self.armName = kwargs['armName']
        self.startGoalToTraj = {}
    def startGoalGen(self, start):
        #self.disableState(start)
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
        #self.components.prioritizedSingleStart.extendTo(other)
        self.components.node.extendTo(other)
        other.execute = execute

class Release:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('node', components.normalized.NormalizedAllStartsGoalGen(), [ self.startGoalGen ]),
            #('prioritizedSingleStart', nodes.meta.components.normalized.NormalizedSingleStart(), [ self.runWithStart ])
        ])
        self.objname = kwargs['objname']
        self.startGoalToTraj = {}
    def startGoalGen(self, start):
        #self.disableState(start)
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
        #self.components.prioritizedSingleStart.extendTo(other)
        self.components.node.extendTo(other)
        other.execute = execute

class Enable:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('node', components.normalized.NormalizedAllStartsGoalGen(), [ self.startGoalGen ]),
            #('prioritizedSingleStart', nodes.meta.components.normalized.NormalizedSingleStart(), [ self.runWithStart ])
        ])
        self.objname = kwargs['objname']
        self.enabled = kwargs['enabled']
        self.startGoalToTraj = {}
    def startGoalGen(self, start):
        #self.disableState(start)
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
        #self.components.prioritizedSingleStart.extendTo(other)
        self.components.node.extendTo(other)
        other.execute = execute

#============================================================================================================

class PlanArmToConfig:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        kwargs['numStarts'] = 1
        kwargs['configs'] = [kwargs['config']]
        components.core.extend(self, [
            ('subnode', PlanArmToConfigs(**kwargs), []),
        ])
    def extendTo(self, other):
        self.components.subnode.extendTo(other)

class PlanArmToPose:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        kwargs['numStarts'] = 1
        kwargs['poses'] = [kwargs['pose']]
        components.core.extend(self, [
            ('subnode', PlanArmToPoses(**kwargs), []),
        ])
    def extendTo(self, other):
        self.components.subnode.extendTo(other)

class PlanArmToPoses:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('subnode', nodes.meta.components.normalized.NormalizedMultiStartsGenSubnode(**kwargs), 
                                                            [ ('startsToSubnode', self.chooseSubnode) ]),
        ])
    def chooseSubnode(self, starts):
        trajDisable = self.kwargs.pop('trajDisable', [])
        poses = self.kwargs['poses']
        configs = []
        goals = set()
        with self.env:
            arm = self.robot.GetManipulator(self.kwargs['armName'])
            Zs = []
            for pose in poses:
                #Zs.append(openravepy.misc.DrawAxes(self.env, pose))
                with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
                    config = arm.FindIKSolution(pose, openravepy.IkFilterOptions.CheckEnvCollisions)
                    if config == None:
                        continue
                    arm.SetDOFValues(config)
                goal = utils.saveEnv(self.env, self.robot)
                goals.add(goal)
                configs.append(config)
        #for Z in Zs:
        #    Z.SetShow(True)
        #raw_input('%')
        #for Z in Zs:
        #    Z.SetShow(False)
        if len(configs) == 0:
            raise Exception('no iks')
        disable = self.kwargs.pop('disable', [])
        return PlanArmToConfigs(configs=configs, disable=disable+trajDisable, **self.kwargs)
    def execute(self, (start, goal, (node, execData))):
        self.disableNode()
        node.execute((start, goal, execData))
    def extendTo(self, other):
        self.components.subnode.extendTo(other)
        other.execute = self.execute

class PlanArmToConfigs:
    def __init__(self, *args, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('choice', components.normalized.NormalizedMultiStarts(**kwargs), [ self.runWithStarts ]),
            ('execTraj', components.robot.ExecChompdTraj(
                            components.robot.ExecSmoothedTraj(
                                components.robot.ExecTraj(), **kwargs), **kwargs), []),
            #('execTraj', robotComponents.ExecSmoothedTraj(robotComponents.ExecTraj(), **kwargs), []),
            ('traj', components.robot.Traj(), [])
        ])
        self.components.execTraj.setTrajNode(self.components.traj)
        self.components.execTraj.setExecTraj(self.execTraj)
        self.startToTree = kwargs.get('startToTree', None)
        self.armName = kwargs['armName']
        self.configs = kwargs['configs']
    def runWithStarts(self, starts):
        for start in starts:
            utils.restoreEnv(self.env, self.robot, start)
            with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
                with self.env:
                    if self.robot.CheckSelfCollision() or self.env.CheckCollision(self.robot):
                        self.disableState(start)
                        starts.remove(start)
        if len(starts) == 0:
            raise Exception('no valid starts')
        with self.env:
            arm = self.robot.GetManipulator(self.armName)
        planType = self.kwargs.get('planType', 'ReuseMultiPRM')
        if planType == 'ReuseMultiPRM':
            startGoalPairs = planArmNodeHelper.planArmReuseMultiPRM(
                                                    self.env, self.robot, self.armName, 
                                                    starts, self.configs, 
                                                    self.startToTree, 
                                                    self.disable, self.disablePadding)
        elif planType == 'RRTConnect':
            if len(starts) > 1 or len(self.configs) > 1:
                print('warning, only planning with one start and one goal')
            startGoalPairs = planArmNodeHelper.planArmRRTConnect(
                                                    self.env, self.robot, self.armName, 
                                                    starts[0], self.configs[0], 
                                                    self.startToTree, 
                                                    self.disable, self.disablePadding)
        else:
            raise Exception('invalid plan type')
        for (start, goal, traj) in startGoalPairs:
            self.addConnectedGoal(goal)
            self.setTraj(start, goal, traj)
            #self.startGoalToTraj[(start, goal)] = traj
            self.addConnectedStartGoalPair((start, goal, None))
    def execTraj(self, start, sim_traj):
        traj = openravepy.RaveCreateTrajectory(self.execEnv, '')
        traj.deserialize(sim_traj.serialize())
        #TODO file bug for needing to Retime
        openravepy.planningutils.RetimeTrajectory(traj)
        with utils.DisableWrapper(self.execEnv, self.disable, self.disablePadding):
            #import IPython; IPython.embed()
            self.execRobot.ExecuteTrajectory(traj)
    #def execute(self, (start, goal, _)):
    #    self.disableNode()
    #    #TODO TODO TODO
    #    traj = self.startGoalToTraj[(start, goal)]

    #    utils.restoreEnv(self.env, self.robot, start)
    #    with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
    #        self.simplifier = openravepy.RaveCreatePlanner(self.env, 'OMPLSimplifier')
    #        params = openravepy.Planner.PlannerParameters()
    #        params.SetExtraParameters('<time_limit>{:f}</time_limit>'.format(3))
    #        self.simplifier.InitPlan(self.robot, params)
    #        self.simplifier.PlanPath(traj)

    #    with utils.DisableWrapper(self.execEnv, self.disable, self.disablePadding):
    #        try:
    #            self.execRobot.ExecuteTrajectory(traj)
    #        except Exception as e:
    #            print e
    #            import IPython; IPython.embed()
    #            raise
    def extendTo(self, other):
        self.components.choice.extendTo(other)
        self.components.execTraj.extendTo(other)


#============================================================================================================

import pddlpy

class PDDL:
    def __init__(self, objects, actions, predicates, goal, **kwargs):
        setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('node', components.normalized.NormalizedSingleStartGenSubnode(), [ self.startToSubnode ])
        ])
        self.objects = objects
        self.actions = actions
        self.predicates = predicates
        self.goal = goal
    def startToSubnode(self, start):
        utils.restoreEnv(self.env, self.robot, start)

        domain = pddlpy.Domain(self.predicates, self.actions);
        problem = pddlpy.Problem(domain, self.objects, self.goal)
        problem.populate_state()
        plan = problem.solve()

        seq = []
        for fn, args in plan:
            seq.append(fn(args))
        return nodes.meta.NormalizedSeq(seq)
    def extendTo(self, other):
        self.components.node.extendTo(other)

#============================================================================================================

def extendRobotNode(node, robotNode):
    node.env = robotNode.env
    node.robot = robotNode.robot

def getHand(robot, handName):
    if handName == '/left/wam7':
        return robot.left_hand
    elif handName == '/right/wam7':
        return robot.right_hand
    if handName == 'TEMASim/ARM_l/wam7':
        return robot.left_hand
    if handName == 'TEMASim/ARM_r/wam7':
        return robot.right_hand
    else:
        raise Exception('unknown hand name: ' + str(handName))
