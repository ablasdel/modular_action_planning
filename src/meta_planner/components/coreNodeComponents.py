import threading
from utils import utils

statesContainer = { 'val': 0 }

class Run:
    def __init__(self):
        self.paused = False
        self._runOnce = False
        self.wakeBlock = None
    def run(self, onDone=None):
        if onDone == None:
            onDone = lambda: None
        self.activeThread = threading.Thread(target=self.onRun, args=([onDone]))
        self.activeThread.start()
    def runFor(self, N):
        self.wakeBlock = threading.Event()
        self.run(self.wakeEvent)
        self.wakeBlock.wait(N)
        self.wakeBlock.clear()
        self.pause();
        self.wakeBlock.clear()
    def runOnce(self):
        self.run()
        self._runOnce = True
        self.waitTillDone()
        self._runOnce = False
    def waitTillDone(self):
        self.activeThread.join()
    def wakeEvent(self):
        if self.wakeBlock != None:
            self.wakeBlock.set()
    def setRun(self, onRun):
        self.onRun = onRun 
    def pause(self):
        self.paused = True
        self.waitTillDone()
        self.paused = False
    def isPaused(self):
        return self.paused
    def isRunOnce(self):
        return self._runOnce
    def extendTo(self, other):
        other.run = self.run
        other.runFor = self.runFor
        other.runOnce = self.runOnce
        other.waitTillDone = self.waitTillDone
        other.setRun = self.setRun
        other.pause = self.pause
        other.isPaused = self.isPaused
        other.isRunOnce = self.isRunOnce

class DisableState:
    def __init__(self):
        self.disabledStates = set()
    def enableState(self, state):
        self.disabledStates.remove(state)
    def disableState(self, state):
        self.disabledStates.add(state)
    def isDisabled(self, state):
        return state in self.disabledStates
    def getDisabledStates(self):
        return self.disabledStates
    def addDisabledStates(self, states):
        self.disabledStates |= states
    def removeDisabledStates(self, states):
        self.disabledStates -= states
    def extendTo(self, other):
        other.isDisabled = self.isDisabled
        other.disableState = self.disableState
        other.enableState = self.enableState
        other.getDisabledStates = self.getDisabledStates
        other.addDisabledStates = self.addDisabledStates
        other.removeDisabledStates = self.removeDisabledStates

class DisableNode:
    def __init__(self):
        self.disabled = False
    def isNodeDisabled(self):
        return self.disabled
    def disableNode(self):
        self.disabled = True
    def enableNode(self):
        self.disabled = False
    def extendTo(self, other):
        other.isNodeDisabled = self.isNodeDisabled
        other.disableNode = self.disableNode
        other.enableNode = self.enableNode

class ConnectedStartGoalPairs:
    def __init__(self):
        self.pairs = set()
    def addConnectedStartGoalPair(self, pair):
        self.pairs.add(pair)
    def addConnectedStartGoalPairs(self, pairs):
        self.pairs |= pairs
    def removeConnectedStartGoalPair(self, pair):
        self.pairs.remove(pair)
    def removeConnectedStartGoalPairs(self, pairs):
        self.pairs -= pairs
    def getConnectedStartGoalPairs(self):
        return self.pairs
    def extendTo(self, other):
        other.addConnectedStartGoalPair = self.addConnectedStartGoalPair
        other.addConnectedStartGoalPairs = self.addConnectedStartGoalPairs
        other.removeConnectedStartGoalPair = self.removeConnectedStartGoalPair
        other.removeConnectedStartGoalPairs = self.removeConnectedStartGoalPairs
        other.getConnectedStartGoalPairs = self.getConnectedStartGoalPairs

class MultiStartGoal:
    def __init__(self):
        self.connectedStarts = set()
        self.connectedGoals = set()
        self.potentialStarts = set()
        self.potentialGoals = set()

        self.onChangedStarts = lambda: None
        self.onChangedGoals = lambda: None
    def getPotentialGoals(self):
        return self.potentialGoals
    def addPotentialGoal(self, goal):
        self.potentialGoals.add(goal)
    def removePotentialGoal(self, goal):
        self.potentialGoals.remove(goal)
    def addPotentialGoals(self, goals):
        self.potentialGoals |= goals
    def removePotentialGoals(self, goals):
        self.potentialGoals -= goals

    def getConnectedGoals(self):
        return self.connectedGoals
    def addConnectedGoal(self, goal, internal=False):
        statesContainer['val'] -= len(self.connectedGoals)
        self.connectedGoals.add(goal)
        statesContainer['val'] += len(self.connectedGoals)
        if not internal:
            self.onChangedGoals()
    def removeConnectedGoal(self, goal, internal=False):
        self.connectedGoals.remove(goal)
        if not internal:
            self.onChangedGoals()
    def addConnectedGoals(self, goals, internal=False):
        statesContainer['val'] -= len(self.connectedGoals)
        self.connectedGoals |= goals
        statesContainer['val'] += len(self.connectedGoals)
        if not internal:
            self.onChangedGoals()
    def removeConnectedGoals(self, goals, internal=False):
        self.connectedGoals -= goals
        if not internal:
            self.onChangedGoals()

    def getPotentialStarts(self):
        return self.potentialStarts
    def addPotentialStart(self, start):
        self.potentialStarts.add(start)
    def removePotentialStart(self, start):
        self.potentialStarts.remove(start)
    def addPotentialStarts(self, starts):
        self.potentialStarts |= starts 
    def removePotentialStarts(self, starts):
        self.potentialStarts -= starts

    def getConnectedStarts(self):
        return self.connectedStarts
    def addConnectedStart(self, start, internal=False):
        statesContainer['val'] -= len(self.connectedStarts)
        self.connectedStarts.add(start)
        statesContainer['val'] += len(self.connectedStarts)
        if not internal:
            self.onChangedStarts()
    def removeConnectedStart(self, start, internal=False):
        self.connectedStarts.remove(start)
        if not internal:
            self.onChangedStarts()
    def addConnectedStarts(self, starts, internal=False):
        statesContainer['val'] -= len(self.connectedStarts)
        self.connectedStarts |= starts 
        statesContainer['val'] += len(self.connectedStarts)
        if not internal:
            self.onChangedStarts()
    def removeConnectedStarts(self, starts, internal=False):
        self.connectedStarts -= starts 
        if not internal:
            self.onChangedStarts()

    def setOnChangedStarts(self, fn):
        self.onChangedStarts = fn
    def setOnChangedGoals(self, fn):
        self.onChangedGoals = fn

    def extendTo(self, other):
        other.getPotentialGoals = self.getPotentialGoals
        other.addPotentialGoal = self.addPotentialGoal
        other.removePotentialGoal = self.removePotentialGoal
        other.getConnectedGoals = self.getConnectedGoals
        other.addConnectedGoal = self.addConnectedGoal
        other.removeConnectedGoal = self.removeConnectedGoal
        other.getPotentialStarts = self.getPotentialStarts
        other.addPotentialStart = self.addPotentialStart
        other.removePotentialStart = self.removePotentialStart
        other.getConnectedStarts = self.getConnectedStarts
        other.addConnectedStart = self.addConnectedStart
        other.removeConnectedStart = self.removeConnectedStart

        other.addConnectedStarts = self.addConnectedStarts
        other.removeConnectedStarts = self.removeConnectedStarts
        other.addPotentialStarts = self.addPotentialStarts
        other.removePotentialStarts = self.removePotentialStarts
        other.addConnectedGoals = self.addConnectedGoals
        other.removeConnectedGoals = self.removeConnectedGoals
        other.addPotentialGoals = self.addPotentialGoals
        other.removePotentialGoals = self.removePotentialGoals

def applyComponents(node):
    for componentName in dir(node.components):
        component = getattr(node.components, componentName)
        if componentName[0] != '_' and hasattr(component, 'extendTo'):
            component.extendTo(node)

def extend(node, components):
    node.components = utils.Object()
    lowerCaseFirst = lambda s: s[0].lower() + s[1:]
    for name, component, fnsToSet in components:
        component.parent = node
        setattr(node.components, name, component)
        fnsToSetNames = {}
        for fn in fnsToSet:
            if isinstance(fn, tuple):
                fnsToSetNames[fn[0]] = fn[1]
            else:
                fnsToSetNames[fn.__name__] = fn
        setFnNames = [s for s in dir(component) if 'set' in s and s.index('set') == 0]
        for setFnName in setFnNames:
            fnName = lowerCaseFirst(setFnName[3:])
            setFn = getattr(component, setFnName)
            if fnName in fnsToSetNames.keys():
                setFn(fnsToSetNames[fnName])
            else:
                setattr(node, setFn.__name__, setFn)
    applyComponents(node)
