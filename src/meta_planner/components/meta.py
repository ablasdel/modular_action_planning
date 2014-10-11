import core
from utils import utils

import functools
import sys
import traceback
from utils import choice
import networkx as nx

import time

nodeSequence = []

class Subnode:
    def __init__(self):
        core.extend(self, [
            ('run', core.Run(), [ ('run', self.chooseAndRun) ])
        ])
    def chooseAndRun(self, onDone):
        try:
            node = self.chooseSubnode()
        except Exception as e:
            if str(e) == 'no ik':
                print 'no ik'
            elif str(e) == 'no iks':
                print 'no iks'
            else:
                traceback.print_exception(*sys.exc_info())
            onDone()
            return
        if node != None:
            if hasattr(self, 'onAboutToRun'):
                try:
                    self.onAboutToRun(node)
                except Exception as e:
                    traceback.print_exception(*sys.exc_info())
                    onDone()
                    return
            nodeSequence.append((node, time.time()))
            node.runOnce()
            if hasattr(self, 'onDone'):
                try:
                    self.onDone(node, onDone)
                except Exception as e:
                    traceback.print_exception(*sys.exc_info())
                    onDone()
                    return
            else:
                onDone()
        else:
            onDone()
    def setChooseSubnode(self, chooseSubnode):
        self.chooseSubnode = chooseSubnode
    def setOnAboutToRun(self, onAboutToRun):
        self.onAboutToRun = onAboutToRun
    def setOnDone(self, onDone):
        self.onDone = onDone
    def extendTo(self, other):
        self.components.run.extendTo(other)

class ChooseSubnode:
    def __init__(self):
        core.extend(self, [
            ('subnode', Subnode(), [ self.chooseSubnode ]),
        ])
        self.subnodes = []
    def chooseSubnode(self):
        subnode = self._chooseSubnode()
        if subnode == None:
            subnode = self.genSubnode()
            if subnode != None:
                self.subnodes.append(subnode)
        return subnode
    def setChooseSubnode(self, _chooseSubnode):
        self._chooseSubnode = _chooseSubnode
    def setGenSubnode(self, genSubnode):
        self.genSubnode = genSubnode
    def extendTo(self, other):
        self.components.subnode.extendTo(other)
        other.subnodes = self.subnodes

#===============================================================

allEnvs = {}

class Seq:
    def __init__(self, nodes):
        core.extend(self, [
            ('subnode', Subnode(), [ self.onDone ]),
            ('multi', core.MultiStartGoal(), [ ('onChangedStarts', functools.partial(self.connectIdx, 0)),
                                                     ('onChangedGoals', functools.partial(self.connectIdx, len(nodes))), ]),
            ('connectedPairs', core.ConnectedStartGoalPairs(), []),
            ('disableNode', core.DisableNode(), []),
        ])
        self.nodes = nodes
        self.connectivityGraph = nx.DiGraph()

        self.connectivityGraph.add_node('start')
        self.connectivityGraph.add_node('end')

        for i in xrange(0,len(self.nodes)):
            self.nodes[i].pnode = (i, self)
        for i in xrange(0,len(self.nodes)+1):
            self.connectIdx(i)
    def onDone(self, node, onDone):
        nodeidx = self.nodes.index(node)
        
        for (start, goal, execData) in node.getConnectedStartGoalPairs():
            self.connectivityGraph.add_node((nodeidx, start))
            self.connectivityGraph.add_node((nodeidx+1, goal))
            self.connectivityGraph.add_edge((nodeidx, start), 
                                            (nodeidx+1, goal), execData=execData)
            #print 'ADD', node, hash(start), hash(goal)
            if node == self.nodes[0]:
                #print 'N0', node
                self.connectivityGraph.add_edge('start', (nodeidx, start))
            if node == self.nodes[-1]:
                #print 'N1', node
                self.connectivityGraph.add_edge((nodeidx+1, goal), 'end')
        if nx.has_path(self.connectivityGraph, 'start', 'end'):
            for path in nx.all_shortest_paths(self.connectivityGraph, 'start', 'end'):
                #import IPython; IPython.embed()
                self.addConnectedStartGoalPair((path[1][1], path[-2][1], None))

        self.connectIdx(nodeidx)
        self.connectIdx(nodeidx+1)
        if self.isPaused() or self.isRunOnce():
            onDone()
            return
        if len(self.getConnectedStartGoalPairs()) == 0:
            self.components.subnode.chooseAndRun(onDone)
        else:
            onDone()
            return
    def connectIdx(self, nodeidx):
        def add(idx):
            def buildFamily(l, p):
                while True:
                    if hasattr(p, 'pnode'):
                        pidx, pnode = p.pnode
                        l.append(pidx)
                        buildFamily(l, pnode)
                        break
                    if hasattr(p, 'parent'):
                        p = p.parent
                    else:
                        break
            pidx = []
            buildFamily(pidx, self)

            startPos = list(pidx)
            startPos.append(idx)
            goalPos = list(pidx)
            goalPos.append(idx+1)

            startPos = '.'.join([str(x) for x in startPos])
            goalPos = '.'.join([str(x) for x in goalPos])

            for start in self.nodes[idx].getConnectedStarts():
                allEnvs[start] = (startPos, False)
            for goal in self.nodes[idx].getConnectedGoals():
                allEnvs[goal] = (goalPos, False)
        if nodeidx == 0:
            add(0)
            self.nodes[0].addConnectedStarts(self.getConnectedStarts())
            self.addConnectedStarts(self.nodes[0].getConnectedStarts(), internal=True)
        elif nodeidx == len(self.nodes):
            add(nodeidx-1)
            self.nodes[len(self.nodes)-1].addPotentialGoals(self.getPotentialGoals())
            self.addConnectedGoals(self.nodes[len(self.nodes)-1].getConnectedGoals(), internal=True)
        else:
            add(nodeidx-1)
            self.connectNodes(self.nodes[nodeidx-1], self.nodes[nodeidx])
    def connectNodes(self, node, successor):
        node.addPotentialGoals(successor.getConnectedStarts())
        successor.addConnectedStarts(node.getConnectedGoals())
    def execute(self, (start, goal, _)):
        self.disableNode()
        path = nx.shortest_path(self.connectivityGraph, (0, start), (len(self.nodes), goal))
        startNodes = path[0:-1]
        goalNodes = path[1:]
        #import IPython; IPython.embed()
        for (node, (sidx, start), (gidx, goal)) in zip(self.nodes, startNodes, goalNodes):
            execData = self.connectivityGraph.get_edge_data((sidx, start), (gidx, goal))['execData']
            allEnvs[start] = (allEnvs[start][0], True)
            allEnvs[goal] = (allEnvs[goal][0], True)
            print 'EXECUTE', node
            node.execute((start, goal, execData))
    def extendTo(self, other):
        self.components.subnode.extendTo(other)
        self.components.multi.extendTo(other)
        self.components.connectedPairs.extendTo(other)
        self.components.disableNode.extendTo(other)
        other.execute = self.execute
        other.nodes = self.nodes

class Par:
    def __init__(self, nodes):
        core.extend(self, [
            ('subnode', Subnode(), [ self.onDone ]),
            ('multi', core.MultiStartGoal(), [ ('onChangedStarts', self.connectAll),
                                                     ('onChangedGoals', self.connectAll), ]),
        ])
        self.nodes = nodes
        self.connectAll()
    def onDone(self, node, onDone):
        nodeidx = self.nodes.index(node)
        self.connectIdx(nodeidx)
        if self.isPaused() or self.isRunOnce():
            onDone()
            return
        if len(self.getConnectedGoals()) == 0:
            self.components.subnode.chooseAndRun(onDone)
        else:
            onDone()
            return
    def connectAll(self):
        for i in xrange(len(self.nodes)):
            self.connectIdx(i)
    def connectIdx(self, nodeidx):
        self.nodes[nodeidx].addConnectedStarts(self.getConnectedStarts())
        self.addConnectedStarts(self.nodes[nodeidx].getConnectedStarts(), internal=True)
        self.nodes[nodeidx].addPotentialGoals(self.getPotentialGoals())
        self.addConnectedGoals(self.nodes[nodeidx].getConnectedGoals(), internal=True)
    def extendTo(self, other):
        self.components.subnode.extendTo(other)
        self.components.multi.extendTo(other)
        other.nodes = self.nodes

class FSM:
    def __init__(self, startFn):
        core.extend(self, [
            ('subnode', ChooseSubnode(), [ self.onDone, self.genSubnode ]),
            ('multi', core.MultiStartGoal(), [ ('onChangedStarts', self.updateStarts), ]),
            ('connectedPairs', core.ConnectedStartGoalPairs(), []),
            ('disableState', core.DisableState(), []),
        ])
        self.nodeToTransitionFn = {}
        self.nodeToTransitionFn[None] = startFn
        self.stateToSrcNode = {}
        self.addedStates = set()
        self.nodes = set()
        self.state = {}
        self.startGoalToExecData = {}
        self.srcStateOptions = set()
    def updateStarts(self):
        statesToAdd = self.getConnectedStarts() - self.addedStates
        for state in statesToAdd:
            self.addStateOption(state)
    def genSubnode(self):
        srcNode, state = self.chooseStateForGenSubnode()
        return self.addState(state, srcNode)
    def setChooseStateForGenSubnode(self, chooseStateForGenSubnode):
        self.chooseStateForGenSubnode = chooseStateForGenSubnode
    def addState(self, state, node=None):
        self.srcStateOptions.add((node, state))
        self.addedStates.add(state)
        self.stateToSrcNode[state] = node
        fn = self.nodeToTransitionFn[node]
        newNode, newFn = fn(state, self.state, self)
        if newNode == None or newFn == None:
            self.addConnectedGoal(state)
            self.buildConnectedStartGoalPair(state)
        else:
            self.nodeToTransitionFn[newNode] = newFn
            newNode.addConnectedStart(state)
            self.nodes.add(newNode)
        return newNode
    def addStateOption(self, state, node=None):
        self.srcStateOptions.add((node, state))
    def onDone(self, node, onDone):
        statesToAdd = node.getConnectedGoals() - self.addedStates
        for state in statesToAdd:
            self.addStateOption(state, node)
        if self.isPaused() or self.isRunOnce():
            onDone()
            return
        self.components.subnode.components.subnode.chooseAndRun(onDone)
    def buildConnectedStartGoalPair(self, goal):
        nodes = []
        states = []
        state = goal
        execDatas = []
        states.insert(0, state)
        while self.stateToSrcNode[state] != None:
            node = self.stateToSrcNode[state]
            nodes.insert(0, node)
            for (start, goal, execData) in node.getConnectedStartGoalPairs():
                if goal == state:
                    state = start
                    execDatas.insert(0, execData)
            states.insert(0, state)
        startStates = states[0:-1]
        goalStates = states[1:]
        self.startGoalToExecData[(startStates[0], goalStates[-1])] = zip(startStates, goalStates, execDatas, nodes)
        self.addConnectedStartGoalPair((startStates[0], goalStates[-1], None))
    def execute(self, (start, goal, _)):
        seq = self.startGoalToExecData[(start, goal)]
        for start, goal, execData, node in seq:
            node.execute((start, goal, execData))
    def extendTo(self, other):
        self.components.subnode.extendTo(other)
        self.components.multi.extendTo(other)
        self.components.connectedPairs.extendTo(other)
        self.components.disableState.extendTo(other)
        other.execute = self.execute
        other.nodes = self.nodes

#===============================================================

class MultiStarts:
    def __init__(self, *args, **kwargs):
        core.extend(self, [
            ('multi', core.MultiStartGoal(), []),
            ('run', core.Run(), [ ('run', self.chooseStartsAndRun) ]),
            ('disabled', core.DisableState(), []),
            ('connectedPairs', core.ConnectedStartGoalPairs(), []),
            ('disableNode', core.DisableNode(), []),
        ])
    def chooseStartsAndRun(self, onDone):
        starts = self.getStarts()
        try:
            return self.runWithStarts(starts)
        except Exception as e:
            if str(e) == 'no ik':
                print 'no ik'
            elif str(e) == 'no iks':
                print 'no iks'
            else:
                traceback.print_exception(*sys.exc_info())
        finally:
            onDone()
    def setRunWithStarts(self, runWithStarts):
        self.runWithStarts = runWithStarts
    def setGetStarts(self, getStarts):
        self.getStarts = getStarts
    def extendTo(self, other):
        self.components.multi.extendTo(other)
        self.components.run.extendTo(other)
        self.components.disabled.extendTo(other)
        self.components.connectedPairs.extendTo(other)
        self.components.disableNode.extendTo(other)

#===============================================================

class MultiStartsGenSubnode:
    def __init__(self):
        core.extend(self, [
            ('subnode', MultiStarts(), [ self.runWithStarts ]),
            ('choose', ChooseSubnode(), [ self.genSubnode, self.onDone ])
        ])
    def onDone(self, node, onDone):
        self.addConnectedGoals(node.getConnectedGoals())
        for (start, goal, execData) in node.getConnectedStartGoalPairs():
            self.addConnectedStartGoalPair((start, goal, (node, execData)))
            #import IPython; IPython.embed()
        if 'getDisabledStates' in dir(node):
            self.addDisabledStates(node.getDisabledStates())
        onDone()
    def genSubnode(self):
        return self.components.subnode.chooseStartsAndRun(lambda: None)
    def runWithStarts(self, starts):
        subnode = self.startsToSubnode(starts)
        subnode.addConnectedStarts(set(starts))
        return subnode
    def execute(self, (start, goal, (node, execData))):
        node.execute((start, goal, execData))
    def setStartsToSubnode(self, startsToSubnode):
        self.startsToSubnode = startsToSubnode
    def extendTo(self, other):
        self.components.subnode.extendTo(other)
        self.components.choose.extendTo(other)
        other.execute = self.execute

#===============================================================

class MultiStartsGoalGenChoicesGenSubnode:
    def __init__(self):
        core.extend(self, [
            ('subnode', MultiStartsGenSubnode(), [ self.startsToSubnode ]),
        ])
    def startsToSubnode(self, starts):
        choices = []
        for start in starts:
            Choice = self.getStartGoalGenChoice(start)
            choiceVector = choice.randomChoiceVector(Choice.getChoiceVector())
            c = Choice.processChoiceVector(list(choiceVector))
            choices.append(c)
        subnode = self.startsGoalGenChoicesToSubnode(starts, choices)
        subnode.addConnectedStarts(set(starts))
        return subnode
    def setGetStartGoalGenChoice(self, getStartGoalGenChoice):
        self.getStartGoalGenChoice = getStartGoalGenChoice
    def setStartsGoalGenChoicesToSubnode(self, startsGoalGenChoicesToSubnode):
        self.startsGoalGenChoicesToSubnode = startsGoalGenChoicesToSubnode
    def extendTo(self, other):
        self.components.subnode.extendTo(other)

#===============================================================

class AllStartsGoalGen:
    def __init__(self, *args, **kwargs):
        core.extend(self, [
            ('multi', core.MultiStartGoal(), [ ('onChangedStarts', self.updateStarts) ]),
            ('run', core.Run(), [ ('run', self.chooseStartsAndRun) ]),
            ('connectedPairs', core.ConnectedStartGoalPairs(), []),
            ('disableNode', core.DisableNode(), []),
        ])
        self.unranStarts = set()
        self.ranStarts = set()
    def chooseStartsAndRun(self, onDone):
        for start in self.unranStarts:
            try:
                self.startGoalGen(start)
            except Exception as e:
                if str(e) == 'no ik':
                    print 'no ik'
                elif str(e) == 'no iks':
                    print 'no iks'
                else:
                    traceback.print_exception(*sys.exc_info())
            finally:
                self.ranStarts |= self.unranStarts
                onDone()
    def updateStarts(self):
        self.unranStarts = self.getConnectedStarts() - self.ranStarts
    def setStartGoalGen(self, startGoalGen):
        self.startGoalGen = startGoalGen
    def extendTo(self, other):
        self.components.multi.extendTo(other)
        self.components.run.extendTo(other)
        self.components.connectedPairs.extendTo(other)
        self.components.disableNode.extendTo(other)

#===============================================================
