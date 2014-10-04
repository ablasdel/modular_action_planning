from utils.startToTree import StartToTree
from examples import setupTableEnv

from nodes import metaNodes
from nodes import robotNodes
from utils import utils
from utils import choice

from tableActions.DynamicReconfigToBin import DynamicReconfigToBin
import openravepy

from utils import tableManip

from tableActions.GrabGlass2 import GrabGlass
from tableActions.GrabGlass2 import MoveToGlass
from tableActions.GraspBowlNode import GrabBowl
from tableActions.GraspBowlNode import MoveToBowl
from tableActions.PlaceObjOnObj import PlaceObjOnObj
from tableActions.PlaceObjInBinNode import PlaceObjInBin

import numpy

import random
import copy
import math

def init_env(env, robot):

    table = setupTableEnv.add_table(env, robot)
    table.SetName('table')
    setupTableEnv.set_robot_pose(env, robot, table)
    bin = setupTableEnv.add_bin(env)
    tableManip.reset_env(env, robot, includePlate=False)


def reset_env(env, robot):
    pass

def get_plan(common):
    if common['args'].startToTree:
        if 'startToTree' not in common:
            common['startToTree'] = StartToTree()
    else:
        common['startToTree'] = None#StartToTree()
    arm = common['robot'].left_arm
    common['numStarts'] = common['args'].numStarts

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
        home_config = robot.configurations.get_configuration('home')[1][0:7]
        arm = robot.left_arm

        objnames = [ b.GetName() for b in env.GetBodies() if (b.GetName().startswith('glass') or b.GetName().startswith('bowl') or b.GetName().startswith('plate')) and b.IsEnabled() ]

        toBinSet = set(toBin)
        objnamesSet = set(objnames)
        remainingToMove = toBinSet & objnamesSet
        canReconfig = objnamesSet - toBinSet
        if len(remainingToMove) == 0:
            return None, None

        activeDOFs = arm.GetArmIndices()
        partialState = copy.deepcopy(state)
        partialState['bodies']['herb']['configuration'] = numpy.delete(partialState['bodies']['herb']['configuration'], activeDOFs)
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
                        out = metaNodes.PrioritizedSeqNode([
                            GrabGlass(objname, arm, **common),
                            PlaceObjInBin(objname, arm, **common),
                            robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
                        ]), startFn
                        glassBinStateMap[partialState] = out
                        return out
                    elif objname.startswith('bowl'):
                        if partialState in bowlBinStateMap and reuse:
                            return bowlBinStateMap[partialState]
                        out = metaNodes.PrioritizedSeqNode([
                            GrabBowl(objname, arm, **common),
                            PlaceObjInBin(objname, arm, **common),
                            robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
                        ]), startFn
                        bowlBinStateMap[partialState] = out
                        return out
                    else:
                        raise Exception('unsupported obj ' + str(objname))
                else:
                    if objname.startswith('glass'):
                        if partialState in glassReconfigStateMap and reuse:
                            return glassReconfigStateMap[partialState]
                        out = metaNodes.PrioritizedSeqNode([
                            GrabGlass(objname, arm, **common),
                            PlaceObjOnObj(objname, 'table', arm, **common),
                            robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
                        ]), startFn
                        glassReconfigStateMap[partialState] = out
                        return out
                    elif objname.startswith('bowl'):
                        if partialState in bowlReconfigStateMap and reuse:
                            return bowlReconfigStateMap[partialState]
                        out = metaNodes.PrioritizedSeqNode([
                            GrabBowl(objname, arm, **common),
                            PlaceObjOnObj(objname, 'table', arm, **common),
                            robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
                        ]), startFn
                        bowlReconfigStateMap[partialState] = out 
                        return out 
                    else:
                        raise Exception('unsupported obj ' + str(objname))



        #for objname in toBinIKs:
        #    if len(toBinIKsInCollision[objname]) > 0:
        #        for ik in toBinIKsInCollision[objname]:
        #            robot.left_arm.SetDOFValues(ik)
        #            #TODO get objects in collision with, add to object scores

        """
        for all bin objects:
            compute number of ik solutions
            compute number of ik solutions (w/out) collision
        """


        #r = random.random()
        #if r < .5:
        #    objname = random.choice(list(remainingToMove))
        #    return metaNodes.PrioritizedSeqNode([
        #        GrabGlass(objname, arm, **common),
        #        PlaceObjInBin(objname, arm, **common),
        #        robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
        #    ]), startFn
        #else:
        #    objname = random.choice(list(canReconfig))
        #    return metaNodes.PrioritizedSeqNode([
        #        GrabGlass(objname, arm, **common),
        #        PlaceObjOnObj(objname, 'table', arm, **common),
        #        robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
        #    ]), startFn

    return  metaNodes.PrioritizedFSMNode(startFn)
