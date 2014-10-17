import components.core
import components.meta
import components.normalized
from utils import choice
from utils import utils

import random

class RandomSeq:
    def __init__(self, nodes):
        components.core.extend(self, [
            ('seq', components.meta.Seq(nodes), [ ('chooseSubnode', self.chooseNode) ])
        ])
    def chooseNode(self):
        node = random.choice(self.nodes)
        if len(node.getConnectedStarts()) == 0:
            return self.chooseNode()
        return node
    def extendTo(self, other):
        self.components.seq.extendTo(other)

class NormalizedSeq:
    def __init__(self, nodes):
        components.core.extend(self, [
            ('seq', components.meta.Seq(nodes), [ ('chooseSubnode', self.chooseNode) ])
        ])
    def chooseNode(self):
        node = components.normalized.chooseNormalizedSubnode(self.nodes, 0)
        nodeidx = self.nodes.index(node)
        return node
    def getNormalizedScore(self):
        return components.normalized.combineNodesForNormalizedScore(self.nodes)
    def extendTo(self, other):
        self.components.seq.extendTo(other)
        other.getNormalizedScore = self.getNormalizedScore

class NormalizedGrowingSeq:
    def __init__(self):
        components.core.extend(self, [
            ('seq', components.meta.GrowingSeq(), [ ('chooseExistingSubnode', self.chooseNode) ])
        ])
    def chooseNode(self):
        node = components.normalized.chooseNormalizedSubnode(self.subnodes, 0)
        nodeidx = self.subnodes.index(node)
        return node
    def getNormalizedScore(self):
        return components.normalized.combineNodesForNormalizedScore(self.subnodes)
    def extendTo(self, other):
        self.components.seq.extendTo(other)
        other.getNormalizedScore = self.getNormalizedScore

class NormalizedPar:
    def __init__(self, nodes):
        components.core.extend(self, [
            ('par', components.meta.Par(nodes), [ ('chooseSubnode', self.chooseNode) ])
        ])
    def chooseNode(self):
        return components.normalized.chooseNormalizedSubnode(self.nodes, 0)
    def getNormalizedScore(self):
        return components.normalized.combineNodesForNormalizedScore(self.nodes)

class NormalizedFSM:
    def __init__(self, startFn):
        components.core.extend(self, [
            ('fsm', components.meta.FSM(startFn), [ self.chooseSubnode, self.chooseStateForGenSubnode ]),
        ])
        self.sampleWeight = float('inf')
        self.srcStateOptionNs = {}
    def chooseSubnode(self):
        return components.normalized.chooseNormalizedSubnode(self.nodes, self.sampleWeight)
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
        choices = components.normalized.chooseNormalizedOptions(optionScores, 1, isDisabled=lambda x: False, replace=False)
        srcState = choices[0]
        if srcState not in self.srcStateOptionNs:
            self.srcStateOptionNs[srcState] = 0
        self.srcStateOptionNs[srcState] += 1
        self.sampleWeight = sum([ 1.0/n for n in self.srcStateOptionNs.values() if n > 0 ]) + \
                            sum([ float('inf') for n in self.srcStateOptionNs.values() if n == 0 ])
        return srcState
    def getNormalizedScore(self):
        N = len(self.nodes) + len(self.srcStateOptionNs)
        score = combineNodesForNormalizedScore(self.nodes)*len(self.nodes)/N + \
               self.sampleWeight*len(self.srcStateOptionNs)/N
        return score
    def extendTo(self, other):
        self.components.fsm.extendTo(other)
        other.getNormalizedScore = self.getNormalizedScore

#============================================================================================================

class Exec:
    def __init__(self, subnode):
        self.subnode = subnode
        components.core.extend(self, [
            ('node', self.subnode, []),
            ('subnode', components.meta.Subnode(), [ self.chooseSubnode, self.onDone ]),
            ('disableNode', components.core.DisableNode(), []),
        ])
    def chooseSubnode(self):
        return self.subnode
    def onDone(self, node, onDone):
        if len(node.getConnectedStartGoalPairs()) > 0:
            pair = list(node.getConnectedStartGoalPairs())[0]
            self.disableNode()
            node.execute(pair)
            onDone()
        elif self.isPaused() or self.isRunOnce():
            onDone()
        else:
            self.components.subnode.chooseAndRun(onDone)

class Checkpoint:
    def __init__(self, subnode):
        self.subnode = subnode
        components.core.extend(self, [
            ('node', self.subnode, []),
            ('subnode', components.meta.Subnode(), [ self.chooseSubnode, self.onDone ]),
            ('disableNode', components.core.DisableNode(), []),
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
