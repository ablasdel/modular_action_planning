import components.core
import nodes.robotChoice
import nodes.robot

from utils import choice

import math

class MoveNearObj:
    def __init__(self, obj, *args, **kwargs):
        self.obj = obj
        nodes.robot.setupWrappedRobotNodeHelper(self, kwargs)
        components.core.extend(self, [
            ('node', nodes.robotChoice.PlanBaseToXYThetaGoalGenChoice(*args, **kwargs), [ self.getStartGoalGenChoice, self.startGoalGenChoiceToXYTheta ])
        ])
    def getStartGoalGenChoice(self, start):
        return choice.Choices({
            "side": choice.Choose(['x+', 'x-', 'y+', 'y-']),
            "p": choice.Uniform(-1, 1),
            "offset": choice.Uniform(0, .25),
            "theta": choice.Uniform(-math.pi, math.pi),
        })
    def startGoalGenChoiceToXYTheta(self, start, choice):
        obj_aabb = self.obj.ComputeAABB()
        robot_aabb = self.robot.ComputeAABB()

        if choice['side'] == 'x+':
            x = obj_aabb.pos()[0] + obj_aabb.extents()[0] + robot_aabb.extents()[0] + choice['offset']
            y = obj_aabb.pos()[1] + choice['p']*obj_aabb.extents()[1] + robot_aabb.extents()[1]
        elif choice['side'] == 'x-':
            x = obj_aabb.pos()[0] - obj_aabb.extents()[0] - robot_aabb.extents()[0] - choice['offset']
            y = obj_aabb.pos()[1] + choice['p']*obj_aabb.extents()[1] + robot_aabb.extents()[1]
        elif choice['side'] == 'y+':
            y = obj_aabb.pos()[1] + obj_aabb.extents()[1] + robot_aabb.extents()[1] + choice['offset']
            x = obj_aabb.pos()[0] + choice['p']*obj_aabb.extents()[0] + robot_aabb.extents()[0]
        elif choice['side'] == 'y-':
            y = obj_aabb.pos()[1] - obj_aabb.extents()[1] - robot_aabb.extents()[1] - choice['offset']
            x = obj_aabb.pos()[0] + choice['p']*obj_aabb.extents()[0] + robot_aabb.extents()[0]
        else:
            raise Exception('bad choice ' + str(choice['side']))

        return x, y, choice['theta']
    def extendTo(self, other):
        self.components.node.extendTo(other)

