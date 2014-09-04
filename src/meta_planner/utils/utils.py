import numpy as np
import math
import openravepy
import prpy.rave

import hashlib

import Queue

class Object(object):
    pass

def getArmGrabbingObject(robot, obj):
  if robot.right_arm.IsGrabbing(obj):
    return robot.right_arm
  if robot.left_arm.IsGrabbing(obj):
    return robot.left_arm
  return None

class DisableWrapper:
    def __init__(self, env, disable, disablePadding):
        self.env = env
        self.disable = disable
        self.disablePadding = disablePadding
        with env:
            disableObjs = [ self.env.GetKinBody(name) for name in self.disable ]
            disablePaddingObjs = [ self.env.GetKinBody(name) for name in self.disablePadding ]
        self.disabled = prpy.rave.AllDisabled(self.env, disableObjs, padding_only=False)
        self.disabledPadding = prpy.rave.AllDisabled(self.env, disablePaddingObjs, padding_only=True)
    def __enter__(self):
        self.disabled.__enter__()
        self.disabledPadding.__enter__()
    def __exit__(self, type, value, traceback):
        self.disabled.__exit__(type, value, traceback)
        self.disabledPadding.__exit__(type, value, traceback)

class CostlyPlanSaver:
    def __init__(self, env, robot):
        self.plans = Queue.PriorityQueue()
        self.env = env
        self.robot = robot
    def addPlan(self, fn, effort):
        self.plans.put((effort, (saveEnv(self.env, self.robot), fn)))
    def run(self):
        doneState = saveEnv(self.env, self.robot)
        while not self.plans.empty():
            (effort, (state, fn)) = self.plans.get()
            restoreEnv(self.env, self.robot, state)
            fn()
        restoreEnv(self.env, self.robot, doneState)

def zrot(theta):
    return [ [ math.cos(theta), -math.sin(theta), 0 ],
             [ math.sin(theta), math.cos(theta), 0 ],
             [ 0, 0, 1 ] ]

def yrot(theta):
    return [ [ math.cos(theta), 0, math.sin(theta) ],
             [ 0, 1, 0 ],
             [ -math.sin(theta), 0, math.cos(theta) ], ]

def xrot(theta):
    return [ [ 1, 0, 0 ],
             [ 0, math.cos(theta), -math.sin(theta) ],
             [ 0, math.sin(theta), math.cos(theta) ], ]

def PlanArmToConfiguration(isExec, env, arm, config, costlyPlanSaver=None):
    if isExec:
        arm.PlanToConfiguration(config, execute=True, timelimit=20, n_iter=100)
    else:
        fullCheck = lambda: arm.PlanToConfiguration(config, execute=False, timelimit=5, n_iter=100, smoothingitrs=1)
        if costlyPlanSaver == None:
            fullCheck()
        else:
            costlyPlanSaver.addPlan(fullCheck, 5)
        with env:
            arm.SetDOFValues(config)

def PlanArmToTSR(isExec, env, robot, tsrList=None):
    if isExec:
        robot.PlanToTSR(tsrList, timelimit=60)
    else:
        robot.PlanToTSR(tsrList, timelimit=30)

def PlanToEndEffectorOffset(isExec, arm, direction, distance, costlyPlanSaver=None, runFullCheck=True):
    step_size = min(.005, distance/10)
    if isExec:
        arm.PlanToEndEffectorOffset(direction=direction, 
            distance=distance, 
            max_distance=distance,
            step_size=step_size, 
            execute=True, 
            timelimit=5.0)
    else:
        with arm.GetRobot().GetEnv():
            finalPose = arm.GetEndEffectorTransform()
            finalPose[:3,3] += np.transpose(np.array(direction)*distance)
            finalPoseIk = arm.FindIKSolution(finalPose, openravepy.IkFilterOptions.CheckEnvCollisions)
            if finalPoseIk == None:
                raise Exception('no ik')
        fullCheck = lambda: arm.PlanToEndEffectorOffset(direction=direction, \
                                            distance=distance, \
                                            max_distance=distance, \
                                            step_size=step_size, \
                                            execute=False, \
                                            timelimit=1.0)
        if costlyPlanSaver == None:
            if runFullCheck:
                fullCheck()
        else:
            costlyPlanSaver.addPlan(fullCheck, 1)
        with arm.GetRobot().GetEnv():
            arm.SetDOFValues(finalPoseIk)

class ManuallyHashedDict(dict):
    def __hash__(self):
        return hash(self.hashValue)
    def __eq__(self, other):
        #TODO bad bad bad - hack to deal with numpy array equality
        return hash(self) == hash(other)

def saveEnv(env, robot):
  with env:
    savedState = ManuallyHashedDict()
    savedState['bodies'] = {}
    for body in env.GetBodies():
      name = body.GetName()
      savedState['bodies'][name] = {}
      savedState['bodies'][name]['xmlPath'] = body.GetXMLFilename()
      savedState['bodies'][name]['configuration'] = body.GetDOFValues()
      savedState['bodies'][name]['transform'] = body.GetTransform()
      savedState['bodies'][name]['enabled'] = body.IsEnabled()
    savedState['grabbed'] = []
    for obj in robot.GetGrabbed():
      arm = getArmGrabbingObject(robot, obj)
      if arm == robot.left_arm:
        armName = 'left'
      elif arm == robot.right_arm:
        armName = 'right'
      else:
        raise Exception('unknown arm name ' + str(armName))
      name = obj.GetName()
      savedState['grabbed'].append((name, armName))
    calcHashSavedState(savedState)
    return savedState

def calcHashSavedState(savedState):
    hashValue = hashlib.md5()
    for name in savedState['bodies']:
        hashValue.update(name)
        for i in xrange(len(savedState['bodies'][name]['configuration'])):
            if savedState['bodies'][name]['configuration'][i] == -0:
                savedState['bodies'][name]['configuration'][i] = 0
        for i in xrange(len(savedState['bodies'][name]['transform'])):
            for j in xrange(len(savedState['bodies'][name]['transform'][i])):
                if savedState['bodies'][name]['transform'][i][j] == -0:
                    savedState['bodies'][name]['transform'][i][j] = 0
        hashValue.update(savedState['bodies'][name]['configuration'].view(np.uint8))
        hashValue.update(savedState['bodies'][name]['transform'].view(np.uint8))
        hashValue.update(str(savedState['bodies'][name]['enabled']))
    for (name, armName) in savedState['grabbed']:
      hashValue.update('(' + name + ',' + armName + ')')
    savedState.hashValue = hashValue.hexdigest()

def envToVector(env, robot):
    with env:
        v = []
        c = robot.GetTransform()[:3,3]
        for body in env.GetBodies():
            t = body.GetTransform()[:3,3]
            v = v + list((t - c).reshape(-1))
        return v

def getArmGrabbingObject(robot, obj):
  if robot.right_arm.IsGrabbing(obj):
    return robot.right_arm
  if robot.left_arm.IsGrabbing(obj):
    return robot.left_arm
  return None

def restoreEnv(env, robot, savedState):
  with env:
    for obj in robot.GetGrabbed():
      robot.Release(obj)
    for name in savedState['bodies']:
      body = env.GetKinBody(name)
      if body is None:
        body = env.ReadKinBodyXMLFile(savedState['bodies'][name]['xmlPath'])
        body.SetName(name)
        env.AddKinBody(body)
      if len(savedState['bodies'][name]['configuration']) > 0:
        body.SetDOFValues(savedState['bodies'][name]['configuration'])
      body.SetTransform(savedState['bodies'][name]['transform'])
      body.Enable(savedState['bodies'][name]['enabled'])
    for body in env.GetBodies():
      if body.GetName() not in savedState['bodies']:
        env.RemoveKinBody(body)

    for objname, armName in savedState['grabbed']:
      obj = env.GetKinBody(objname)
      if armName == 'left':
        robot.left_arm.SetActive()
      elif armName == 'right':
        robot.right_arm.SetActive()
      else:
        raise Exception('unknown arm name ' + str(armName))
      robot.Grab(obj)
