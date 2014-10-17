import random

class Choice:
    def getChoiceVector(self):
        raise Exception("implement in subclass")
    def processChoiceVector(self):
        raise Exception("implement in subclass")

class Choices(Choice):
    def __init__(self, dct):
        self.dct = dct 
    def getChoiceVector(self):
        cv = []
        for val in self.dct.values():
            cv += val.getChoiceVector()
        return cv
    def processChoiceVector(self, cv):
        choice = {}
        for key, val in self.dct.items():
            choice[key] = val.processChoiceVector(cv)
        return choice

class Uniform(Choice):
    def __init__(self, low, high):
        self.low = low
        self.high = high
    def getChoiceVector(self):
        return ['c']
    def processChoiceVector(self, cv):
        return self.low + cv.pop(0)*(self.high - self.low)
   
class Choose(Choice):
    def __init__(self, items):
        self.items = items
    def getChoiceVector(self):
        return [ len(self.items) ]
    def processChoiceVector(self, cv):
        return self.items[cv.pop(0)]

def randomChoiceVector(cv):
    out = []
    for c in cv:
        if c == 'c':
            out.append(random.random())
        else:
            out.append(random.choice(range(c)))
    return out
