from components import coreNodeComponents as components
from components import metaNodeComponents as metaComponents
from components import prioritizedNodeComponents as prioritizedComponents
from utils import choice
from utils import utils

import random

class RandomSeqNode:
    def __init__(self, nodes):
        components.extend(self, [
            ('seq', metaComponents.Seq(nodes), [ ('chooseSubnode', self.chooseNode) ])
        ])
    def chooseNode(self):
        node = random.choice(self.nodes)
        if len(node.getConnectedStarts()) == 0:
            return self.chooseNode()
        return node
    def extendTo(self, other):
        self.components.seq.extendTo(other)

class PrioritizedSeqNode:
    def __init__(self, nodes):
        components.extend(self, [
            ('seq', metaComponents.Seq(nodes), [ ('chooseSubnode', self.chooseNode) ])
        ])
    def chooseNode(self):
        node = prioritizedComponents.choosePrioritizedSubnode(self.nodes, 0)
        nodeidx = self.nodes.index(node)
        return node
    def getNormalizedScore(self):
        return prioritizedComponents.combineNodesForPrioritizedScore(self.nodes)
    def extendTo(self, other):
        self.components.seq.extendTo(other)
        other.getNormalizedScore = self.getNormalizedScore

class PrioritizedGrowingSeqNode:
    def __init__(self):
        components.extend(self, [
            ('seq', metaComponents.GrowingSeq(), [ ('chooseExistingSubnode', self.chooseNode) ])
        ])
    def chooseNode(self):
        node = prioritizedComponents.choosePrioritizedSubnode(self.subnodes, 0)
        nodeidx = self.subnodes.index(node)
        return node
    def getNormalizedScore(self):
        return prioritizedComponents.combineNodesForPrioritizedScore(self.subnodes)
    def extendTo(self, other):
        self.components.seq.extendTo(other)
        other.getNormalizedScore = self.getNormalizedScore

class PrioritizedParNode:
    def __init__(self, nodes):
        components.extend(self, [
            ('par', metaComponents.Par(nodes), [ ('chooseSubnode', self.chooseNode) ])
        ])
    def chooseNode(self):
        return prioritizedComponents.choosePrioritizedSubnode(self.nodes, 0)
    def getNormalizedScore(self):
        return prioritizedComponents.combineNodesForPrioritizedScore(self.nodes)

class PrioritizedFSMNode:
    def __init__(self, startFn):
        components.extend(self, [
            ('fsm', metaComponents.FSM(startFn), [ self.chooseSubnode, self.chooseStateForGenSubnode ]),
        ])
        self.sampleWeight = float('inf')
        self.srcStateOptionNs = {}
    def chooseSubnode(self):
        return prioritizedComponents.choosePrioritizedSubnode(self.nodes, self.sampleWeight)
    def chooseStateForGenSubnode(self):
        optionScores = []
        for srcState in self.components.fsm.srcStateOptions:
            if self.isDisabled(srcState):
                continue
            n = 0
            if srcState in self.srcStateOptionNs.keys():
                n = self.srcStateOptionNs[srcState]
            if n == 0:
                optionScores.append((srcState, float('inf')))
            else:
                optionScores.append((srcState, 1.0/n))
        choices = prioritizedComponents.choosePrioritizedOptions(optionScores, 1, isDisabled=lambda x: False, replace=False)
        srcState = choices[0]
        if srcState not in self.srcStateOptionNs:
            self.srcStateOptionNs[srcState] = 0
        self.srcStateOptionNs[srcState] += 1
        self.sampleWeight = sum([ 1.0/n for n in self.srcStateOptionNs.values() if n > 0 ]) + \
                            sum([ float('inf') for n in self.srcStateOptionNs.values() if n == 0 ])
        return srcState
    def getNormalizedScore(self):
        N = len(self.nodes) + len(self.srcStateOptionNs)
        score = combineNodesForPrioritizedScore(self.nodes)*len(self.nodes)/N + \
               self.sampleWeight*len(self.srcStateOptionNs)/N
        return score
    def extendTo(self, other):
        self.components.fsm.extendTo(other)
        other.getNormalizedScore = self.getNormalizedScore

#============================================================================================================

import threading
class ExecNode:
    ExecLock = threading.Event()
    def __init__(self, subnode):
        self.subnode = subnode
        components.extend(self, [
            ('node', self.subnode, []),
            ('subnode', metaComponents.Subnode(), [ self.chooseSubnode, self.onDone ]),
            ('disableNode', components.DisableNode(), []),
        ])
    def chooseSubnode(self):
        return self.subnode
    def onDone(self, node, onDone):
        if len(node.getConnectedStartGoalPairs()) > 0:
            pair = list(node.getConnectedStartGoalPairs())[0]
            self.disableNode()
            #onDone()
            #ExecNode.ExecLock.set()
            #TODO GIL makes this not parallel...
            node.execute(pair)
            #ExecNode.ExecLock.clear()
            onDone()
        elif self.isPaused() or self.isRunOnce():
            onDone()
        else:
            self.components.subnode.chooseAndRun(onDone)

class CheckpointNode:
    def __init__(self, subnode):
        self.subnode = subnode
        components.extend(self, [
            ('node', self.subnode, []),
            ('subnode', metaComponents.Subnode(), [ self.chooseSubnode, self.onDone ]),
            ('disableNode', components.DisableNode(), []),
        ])
    def chooseSubnode(self):
        return self.subnode
    def onDone(self, node, onDone):
        if len(node.getConnectedStartGoalPairs()) > 0:
            pair = list(node.getConnectedStartGoalPairs())[0]
            self.disableNode()
            onDone()
        elif self.isPaused() or self.isRunOnce():
            onDone()
        else:
            self.components.subnode.chooseAndRun(onDone)
