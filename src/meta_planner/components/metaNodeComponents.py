import coreNodeComponents as components
from utils import utils

import functools
import sys
import traceback
from utils import choice
import networkx as nx

class Subnode:
    def __init__(self):
        components.extend(self, [
            ('run', components.Run(), [ ('run', self.chooseAndRun) ])
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
    def setOnDone(self, onDone):
        self.onDone = onDone
    def extendTo(self, other):
        self.components.run.extendTo(other)

class Seq:
    def __init__(self, nodes):
        components.extend(self, [
            ('subnode', Subnode(), [ self.onDone ]),
            ('multi', components.MultiStartGoal(), [ ('onChangedStarts', functools.partial(self.connectIdx, 0)),
                                                     ('onChangedGoals', functools.partial(self.connectIdx, len(nodes))), ]),
            ('connectedPairs', components.ConnectedStartGoalPairs(), []),
            ('disableNode', components.DisableNode(), []),
        ])
        self.nodes = nodes
        self.connectivityGraph = nx.DiGraph()

        self.connectivityGraph.add_node('start')
        self.connectivityGraph.add_node('end')

        for i in xrange(0,len(self.nodes)+1):
            self.connectIdx(i)
    def onDone(self, node, onDone):
        nodeidx = self.nodes.index(node)
        
        for (start, goal, execData) in node.getConnectedStartGoalPairs():
            self.connectivityGraph.add_node(start)
            self.connectivityGraph.add_node(goal)
            self.connectivityGraph.add_edge(start, goal, execData=execData)
            if node == self.nodes[0]:
                self.connectivityGraph.add_edge('start', start)
            if node == self.nodes[-1]:
                self.connectivityGraph.add_edge(goal, 'end')
        if nx.has_path(self.connectivityGraph, 'start', 'end'):
            for path in nx.all_shortest_paths(self.connectivityGraph, 'start', 'end'):
                self.addConnectedStartGoalPair((path[1], path[-2], None))

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
        if nodeidx == 0:
            self.nodes[0].addConnectedStarts(self.getConnectedStarts())
            self.addConnectedStarts(self.nodes[0].getConnectedStarts(), internal=True)
        elif nodeidx == len(self.nodes):
            self.nodes[len(self.nodes)-1].addPotentialGoals(self.getPotentialGoals())
            self.addConnectedGoals(self.nodes[len(self.nodes)-1].getConnectedGoals(), internal=True)
        else:
            self.connectNodes(self.nodes[nodeidx-1], self.nodes[nodeidx])
    def connectNodes(self, node, successor):
        node.addPotentialGoals(successor.getConnectedStarts())
        successor.addConnectedStarts(node.getConnectedGoals())
    def execute(self, (start, goal, _)):
        self.disableNode()
        path = nx.shortest_path(self.connectivityGraph, start, goal)
        startNodes = path[0:-1]
        goalNodes = path[1:]
        for (node, start, goal) in zip(self.nodes, startNodes, goalNodes):
            execData = self.connectivityGraph.get_edge_data(start, goal)['execData']
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
        components.extend(self, [
            ('subnode', Subnode(), [ self.onDone ]),
            ('multi', components.MultiStartGoal(), [ ('onChangedStarts', self.connectAll),
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

class ChooseSubnode:
    def __init__(self):
        components.extend(self, [
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

class MultiStarts:
    def __init__(self, *args, **kwargs):
        components.extend(self, [
            ('multi', components.MultiStartGoal(), []),
            ('run', components.Run(), [ ('run', self.chooseStartsAndRun) ]),
            ('disabled', components.DisableState(), []),
            ('connectedPairs', components.ConnectedStartGoalPairs(), []),
            ('disableNode', components.DisableNode(), []),
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
        components.extend(self, [
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
    def setStartsToSubnode(self, startsToSubnode):
        self.startsToSubnode = startsToSubnode
    def extendTo(self, other):
        self.components.subnode.extendTo(other)
        self.components.choose.extendTo(other)

#===============================================================

class MultiStartsGoalGenChoicesGenSubnode:
    def __init__(self):
        components.extend(self, [
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
