from utils.startToTree import StartToTree
from taskUtils import setupTableEnv

import nodes.meta
import nodes.robot
from utils import utils
from utils import choice

import openravepy

from taskUtils import tableManip

from tableNodes.graspGlass import GraspGlass
from tableNodes.graspGlass import MoveToGlass
from tableNodes.graspBowl import GraspBowl
from tableNodes.graspBowl import MoveToBowl
from tableNodes.placeItemInBin import PlaceItemInBin
from tableNodes.placeItemOnObj import PlaceItemOnObj

import numpy

import random
import copy
import math

def add_arguments(parser):
    parser.add_argument('--heuristic', action='store_true', default=False)

def init_env(common):
    env = common['execEnv']
    robot = common['execRobot']

    table = setupTableEnv.add_table(env, robot)
    table.SetName('table')
    setupTableEnv.set_robot_pose(env, robot, table)
    bin = setupTableEnv.add_bin(env)

def reset_env(env, robot):
    tableManip.reset_env(env, robot, includePlate=False)

def get_plan(common):
    arm = common['robot'].left_arm

    env = common['env']
    robot = common['robot']

    toBin = []
    objnames = [ b.GetName() for b in env.GetBodies() if (b.GetName().startswith('glass') or b.GetName().startswith('bowl') or b.GetName().startswith('plate')) and b.IsEnabled() ]
    random.shuffle(objnames)
    toBin = objnames[0:3]

    stateMap = {}

    def ssd(A, B):
        return sum((A-B)**2)

    def startFn(state, fsmState, fsm):
        utils.restoreEnv(env, robot, state)
        arm = robot.left_arm
        home_config = robot.configurations.get_configuration('home')[1][0:len(arm.GetIndices())]

        objnames = [ b.GetName() for b in env.GetBodies() if (b.GetName().startswith('glass') or b.GetName().startswith('bowl') or b.GetName().startswith('plate')) and b.IsEnabled() ]

        toBinSet = set(toBin)
        objnamesSet = set(objnames)
        remainingToMove = toBinSet & objnamesSet
        canReconfig = objnamesSet - toBinSet
        if len(remainingToMove) == 0:
            return None, None

        activeDOFs = arm.GetArmIndices()
        partialState = copy.deepcopy(state)
        partialState['bodies'][robot.GetName()]['configuration'] = numpy.delete(partialState['bodies'][robot.GetName()]['configuration'], activeDOFs)
        utils.calcHashSavedState(partialState)

        glassBinStateMap = {}
        bowlBinStateMap = {}
        glassReconfigStateMap = {}
        bowlReconfigStateMap = {}

        reuse = True

        def scoreTargetObjects(objnames):
            
            toScore = set(objnames)
            targetObjects = set(toScore)
            scoredObjnames = set()
            objectScores = {}
            collisionObjectScores = {}
            targetObjectScores = {}

            while len(toScore) > 0:
                iks = {}
                iksInCollision = {}
                for objname in toScore:
                    iks[objname] = list()
                    iksInCollision[objname] = list()

                    if objname.startswith('glass'):
                        poseGen = MoveToGlass(objname, armName=arm.GetName(), **common)
                    elif objname.startswith('bowl'):
                        poseGen = MoveToBowl(objname, armName=arm.GetName(), **common)
                    else:
                        raise Exception('unsupported obj ' + str(objname))

                    for i in xrange(10):
                        Choice = poseGen.getStartGoalGenChoice(state)
                        choiceVector = choice.randomChoiceVector(Choice.getChoiceVector())
                        c = Choice.processChoiceVector(list(choiceVector))
                        pose = poseGen.startGoalGenChoiceToPose(state, c)
                        ik = arm.FindIKSolution(pose, openravepy.IkFilterOptions.CheckEnvCollisions)
                        if ik != None:
                            iks[objname].append(ik)
                        else:
                            ik = arm.FindIKSolution(pose, 0)
                            if ik != None:
                                iksInCollision[objname].append(ik)
                toScore = set()
                for objname in iks:
                    if objname in targetObjects:
                        if objname not in targetObjectScores:
                            targetObjectScores[objname] = 0
                        targetObjectScores[objname] += len(iks[objname])
                    else:
                        if objname not in objectScores:
                            objectScores[objname] = 0
                        objectScores[objname] += len(iks[objname])
                    scoredObjnames.add(objname)
                for objname in iksInCollision:
                    for ik in iksInCollision[objname]:
                        arm.SetDOFValues(ik)
                        collidedObjnames = set()
                        for objname in objnamesSet:
                            if env.CheckCollision(env.GetKinBody(objname)):
                                if objname not in collisionObjectScores:
                                    collisionObjectScores[objname] = 0
                                collisionObjectScores[objname] += 1
                                collidedObjnames.add(objname)
                        for objname in collidedObjnames:
                            if objname not in scoredObjnames:
                                toScore.add(objname)
            for objname in collisionObjectScores:
                if objname not in objectScores:
                    objectScores[objname] = 0
                objectScores[objname] = math.sqrt(collisionObjectScores[objname] * objectScores[objname])
            for objname in targetObjectScores:
                if objname not in objectScores:
                    objectScores[objname] = 0
                objectScores[objname] += 10*targetObjectScores[objname]
            return objectScores
                
        if common['args'].heuristic:
            objectScores = scoreTargetObjects(remainingToMove)
        else:
            objectScores = {}
        for objname in objnamesSet:
            if objname not in objectScores:
                objectScores[objname] = 0
            objectScores[objname] += 1
        print objectScores, remainingToMove
        #import IPython; IPython.embed()

        scoreSum = sum(objectScores.values())
        scoreSelection = random.random()*scoreSum
        for objname, score in objectScores.items():
            scoreSelection -= score
            if scoreSelection < 0:
                print objname
                #raw_input('')
                if objname in remainingToMove:
                    if objname.startswith('glass'):
                        if partialState in glassBinStateMap and reuse:
                            return glassBinStateMap[partialState]
                        out = nodes.meta.NormalizedSeq([
                            GraspGlass(objname, arm, **common),
                            PlaceItemInBin(objname, arm, **common),
                            nodes.robot.PlanArmToConfig(config=home_config, armName=arm.GetName(), **common),
                        ]), startFn
                        glassBinStateMap[partialState] = out
                        return out
                    elif objname.startswith('bowl'):
                        if partialState in bowlBinStateMap and reuse:
                            return bowlBinStateMap[partialState]
                        out = nodes.meta.NormalizedSeq([
                            GraspBowl(objname, arm, **common),
                            PlaceItemInBin(objname, arm, **common),
                            nodes.robot.PlanArmToConfig(config=home_config, armName=arm.GetName(), **common),
                        ]), startFn
                        bowlBinStateMap[partialState] = out
                        return out
                    else:
                        raise Exception('unsupported obj ' + str(objname))
                else:
                    if objname.startswith('glass'):
                        if partialState in glassReconfigStateMap and reuse:
                            return glassReconfigStateMap[partialState]
                        out = nodes.meta.NormalizedSeq([
                            GraspGlass(objname, arm, **common),
                            PlaceItemOnObj(objname, 'table', arm, **common),
                            nodes.robot.PlanArmToConfig(config=home_config, armName=arm.GetName(), **common),
                        ]), startFn
                        glassReconfigStateMap[partialState] = out
                        return out
                    elif objname.startswith('bowl'):
                        if partialState in bowlReconfigStateMap and reuse:
                            return bowlReconfigStateMap[partialState]
                        out = nodes.meta.NormalizedSeq([
                            GraspBowl(objname, arm, **common),
                            PlaceItemOnObj(objname, 'table', arm, **common),
                            nodes.robot.PlanArmToConfig(config=home_config, armName=arm.GetName(), **common),
                        ]), startFn
                        bowlReconfigStateMap[partialState] = out 
                        return out 
                    else:
                        raise Exception('unsupported obj ' + str(objname))

    return  nodes.meta.NormalizedFSM(startFn)
