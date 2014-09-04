import numpy as np
import copy
import utils

class StartToTree:
    def __init__(self):
        self.startToConfigPlanner = {}
    def _startOnlyEnabled(self, start, robot, activeDOFs):
        startOnlyEnabled = copy.deepcopy(start)
        startOnlyEnabled['bodies']['herb']['configuration'] = np.delete(startOnlyEnabled['bodies']['herb']['configuration'], activeDOFs)
        for name in start['bodies']:
            if not startOnlyEnabled['bodies'][name]['enabled']:
                del startOnlyEnabled['bodies'][name]
            else:
                startOnlyEnabled['bodies'][name]['configuration'] = np.around(startOnlyEnabled['bodies'][name]['configuration'], 4)
                startOnlyEnabled['bodies'][name]['transform'] = np.around(startOnlyEnabled['bodies'][name]['transform'], 4)
        for (name, armName) in startOnlyEnabled['grabbed']:
            arm = robot.GetManipulator(armName)
            objInEE = np.dot(np.linalg.inv(arm.GetEndEffectorTransform()), startOnlyEnabled['bodies'][name]['transform'])
            #NOTE we assume rotation invariance for these objects
            #TODO restrict rotational invariance to z axis
            if name.startswith('glass') or \
               name.startswith('bowl') or \
               name.startswith('plate'):
                objInEE[:3,:3] = np.eye(3)
            objInEE = np.around(objInEE, 3)
            #print objInEE
            startOnlyEnabled['bodies'][name]['transform'] = objInEE
        utils.calcHashSavedState(startOnlyEnabled)
        return startOnlyEnabled
    def putTree(self, start, planner, robot, activeDOFs):
        self.startToConfigPlanner[self._startOnlyEnabled(start, robot, activeDOFs)] = planner
    def getTree(self, start, robot, activeDOFs):
        startOnlyEnabled = self._startOnlyEnabled(start, robot, activeDOFs)
        if startOnlyEnabled in self.startToConfigPlanner:
           return self.startToConfigPlanner[startOnlyEnabled]
    def getCompatibleTree(self, start, robot, activeDOFs):
        startOnlyEnabled = self._startOnlyEnabled(start, robot, activeDOFs)
        bestCompatiblePlanner = None
        bestMatches = 10000000
        for otherStart, planner in self.startToConfigPlanner.items():
            if len(otherStart['bodies']) < bestMatches:
                mismatch = False
                if startOnlyEnabled['grabbed'] != otherStart['grabbed']:
                    mismatch = True
                if not mismatch:
                    for name in startOnlyEnabled['bodies']:
                        if name not in otherStart['bodies']:
                            mismatch = True
                            break
                        if not np.array_equal(startOnlyEnabled['bodies'][name]['configuration'], otherStart['bodies'][name]['configuration']):
                            mismatch = True
                            break
                        if not np.array_equal(startOnlyEnabled['bodies'][name]['transform'], otherStart['bodies'][name]['transform']):
                            mismatch = True
                            break
                if not mismatch:
                    bestCompatiblePlanner = planner
                    bestMatches = len(otherStart['bodies'])
        return bestCompatiblePlanner
