import coreNodeComponents as components
import metaNodeComponents as metaComponents
from utils import choice
from utils import utils

import random

import traceback
import sys
import math

def combineNodesForPrioritizedScore(nodes):
    if len(nodes) == 0:
        return 0
    scores = []
    for node in nodes:
        if not node.isNodeDisabled():
            score = node.getNormalizedScore()
            if score == float('inf'):
                return score
            scores.append(score)
    return sum(scores)/len(scores)

def choosePrioritizedSubnode(nodes, sampleWeight):
    optionScores = []
    optionScores.append((None, sampleWeight))
    for node in nodes:
        if not node.isNodeDisabled():
            optionScores.append((node, node.getNormalizedScore()))
    optionScores.reverse()
    subnodes = choosePrioritizedOptions(optionScores, 1)
    if len(subnodes) > 0:
        return subnodes[0]
    else:
        return None

def choosePrioritizedOptions(optionScores, N, isDisabled=lambda x: False, replace=False):
    scores = []
    options = []
    choices = []

    for option, score in optionScores:
        if isDisabled(option) or score == 0:
            continue
        if score == float('inf') and len(choices) < N:
            choices.append(option)
            continue
        options.append(option)
        scores.append(score)

    if replace and len(choices) > 0:
        choices = choices*int(math.ceil(N*1.0/len(choices)))
        choices = choices[0:N]
    
    for i in xrange(N):
        if len(choices) >= N:
            break
        choiceScore = random.random()*sum(scores)
        partialScore = 0
        for i, score in enumerate(scores):
            partialScore += score
            if partialScore >= choiceScore:
                option = options[i]
                #print 'choose', i, score, sum(scores)
                #raw_input('')
                choices.append(option)
                if replace == False:
                    options.pop(i)
                    scores.pop(i)
                break
    return choices

def choosePrioritizedStarts(self, N, replace=False):
    optionScores = []
    for option, score in self.startToWeight.items():
        optionScores.append((option, score))
    starts = choosePrioritizedOptions(optionScores, N, self.isDisabled, replace)
    for start in starts:
        if start not in self.startToN:
            self.startToN[start] = 0
        self.startToN[start] += 1
        self.startToWeight[start] = 1.0/self.startToN[start]

        nonDisabledValues = [ score for option, score in self.startToWeight.items() if not self.isDisabled(option) ]

        self.sampleWeight = sum(nonDisabledValues)/len(nonDisabledValues)

    return starts

#===============================================================

class PrioritizedMultiStartsComponent:
    def __init__(self, **kwargs):
        components.extend(self, [
            ('subnode', kwargs['parent'], [ self.getStarts, self.onChangedStarts ])
        ])
        self.numStarts = kwargs.get('numStarts', 12)
        self.startToN = {}
        self.startToWeight = {}
    def onChangedStarts(self):
        for start in self.getConnectedStarts():
            if start not in self.startToN:
                self.startToN[start] = 0
                self.startToWeight[start] = float('inf')
    def getStarts(self):
        return choosePrioritizedStarts(self, self.numStarts, replace=True)
    def getNormalizedScore(self):
        values = [ value for start, value in self.startToWeight.items() if not self.isDisabled(start) ]
        if len(values) == 0:
            return 0
        return sum(values)/len(values)
    def extendTo(self, other):
        self.components.subnode.extendTo(other)
        other.getNormalizedScore = self.getNormalizedScore

class PrioritizedMultiStartsGenSubnodeComponent:
    def __init__(self, **kwargs):
        components.extend(self, [
            ('subnode', kwargs['parent'], [ self.getStarts, self.onChangedStarts, self.chooseSubnode ])
        ])
        self.numStarts = kwargs.get('numStarts', 12)
        self.startToN = {}
        self.startToWeight = {}
        self.sampleWeight = 0
    def chooseSubnode(self):
        return choosePrioritizedSubnode(self.subnodes, self.sampleWeight)
    def onChangedStarts(self):
        for start in self.getConnectedStarts():
            if start not in self.startToN:
                self.startToN[start] = 0
                self.startToWeight[start] = float('inf')
                self.sampleWeight = float('inf')
    def getStarts(self):
        return choosePrioritizedStarts(self, self.numStarts, replace=True)
    def getNormalizedScore(self):
        return combineNodesForPrioritizedScore(self.subnodes)*len(self.subnodes)/(len(self.subnodes)+1) + self.sampleWeight/(len(self.subnodes)+1)
    def extendTo(self, other):
        self.components.subnode.extendTo(other)
        other.getNormalizedScore = self.getNormalizedScore

#===============================================================

class PrioritizedMultiStarts:
    def __init__(self, *args, **kwargs):
        parent = metaComponents.MultiStarts()
        components.extend(self, [
            ('subnode', PrioritizedMultiStartsComponent(parent=parent, **kwargs), [])
        ])
    def extendTo(self, other):
        self.components.subnode.extendTo(other)

class PrioritizedSingleStart:
    def __init__(self):
        kwargs = {}
        kwargs['numStarts'] = 1
        components.extend(self, [
            ('subnode', PrioritizedMultiStarts(**kwargs), [ self.runWithStarts ]),
        ])
    def runWithStarts(self, starts):
        self.runWithStart(starts[0])
    def setRunWithStart(self, runWithStart):
        self.runWithStart = runWithStart
    def extendTo(self, other):
        self.components.subnode.extendTo(other)

#===============================================================

class PrioritizedMultiStartsGoalGenChoicesGenSubnode:
    def __init__(self, **kwargs):
        parent = metaComponents.MultiStartsGoalGenChoicesGenSubnode()
        components.extend(self, [
            ('subnode', PrioritizedMultiStartsGenSubnodeComponent(parent=parent, **kwargs), [])
        ])
    def extendTo(self, other):
        self.components.subnode.extendTo(other)

class PrioritizedSingleStartGoalGenChoiceGenSubnode:
    def __init__(self):
        kwargs = {}
        kwargs['numStarts'] = 1
        components.extend(self, [
            ('subnode', PrioritizedMultiStartsGoalGenChoicesGenSubnode(**kwargs), [ self.startsGoalGenChoicesToSubnode ]),
        ])
    def startsGoalGenChoicesToSubnode(self, starts, choices):
        return self.startGoalGenChoiceToSubnode(starts[0], choices[0])
    def setStartGoalGenChoiceToSubnode(self, startGoalGenChoiceToSubnode):
        self.startGoalGenChoiceToSubnode = startGoalGenChoiceToSubnode
    def extendTo(self, other):
        self.components.subnode.extendTo(other)

#===============================================================

class PrioritizedMultiStartsGenSubnode:
    def __init__(self, **kwargs):
        parent = metaComponents.MultiStartsGenSubnode()
        components.extend(self, [
            ('subnode', PrioritizedMultiStartsGenSubnodeComponent(parent=parent, **kwargs), [])
        ])
    def extendTo(self, other):
        self.components.subnode.extendTo(other)

class PrioritizedSingleStartGenSubnode:
    def __init__(self):
        kwargs = {}
        kwargs['numStarts'] = 1
        components.extend(self, [
            ('subnode', PrioritizedMultiStartsGenSubnode(**kwargs), [ self.startsToSubnode ]),
        ])
    def startsToSubnode(self, starts):
        return self.startToSubnode(starts[0])
    def setStartToSubnode(self, startToSubnode):
        self.startToSubnode = startToSubnode
    def extendTo(self, other):
        self.components.subnode.extendTo(other)

#===============================================================

class PrioritizedAllStartsGoalGen:
    def __init__(self):
        components.extend(self, [
            ('subnode', metaComponents.AllStartsGoalGen(), [])
        ])
    def getNormalizedScore(self):
        if len(self.components.subnode.unranStarts) > 0:
            return float('inf')
        else:
            return 0
    def extendTo(self, other):
        self.components.subnode.extendTo(other)
        other.getNormalizedScore = self.getNormalizedScore

#===============================================================
