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
        print 'CHOOSE', nodeidx, node
        #import IPython; IPython.embed()
        return node
    def getNormalizedScore(self):
        return prioritizedComponents.combineNodesForPrioritizedScore(self.nodes)
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

#============================================================================================================

class ExecNode:
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
            node.execute(pair)
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

