import core
import openravepy
from utils import utils

class ExecTraj:
    def __init__(self):
        pass
    def setTrajNode(self, trajNode):
        self.trajNode = trajNode
    def setExecTraj(self, execTraj):
        self.execTraj = execTraj
    def execute(self, (start, goal, _)):
        traj = self.trajNode.getTraj(start, goal)
        print 'EXEC TRAJ', traj
        self.execTraj(start, traj)
    def extendTo(self, other):
        other.execute = self.execute

class Traj:
    def __init__(self):
        self.trajs = {}
    def getTraj(self, start, goal):
        return self.trajs[(start, goal)]
    def setTraj(self, start, goal, traj):
        self.trajs[(start, goal)] = traj
    def extendTo(self, other):
        other.getTraj = self.getTraj
        other.setTraj = self.setTraj

class ExecChompdTraj:
    def __init__(self, execTraj, **kwargs):
        core.extend(self, [
            ('execTraj', execTraj, [ self.execTraj ]),
        ])
        self.env = kwargs['env']
        self.robot = kwargs['robot']
        self.disable = kwargs.get('disable', [])
        self.disablePadding = kwargs.get('disablePadding', [])
    def setExecTraj(self, _execTraj):
        self._execTraj = _execTraj
    def execTraj(self, start, traj):
        utils.restoreEnv(self.env, self.robot, start)
        with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
            try:
                traj = self.robot.chomp_planner.OptimizeTrajectory(self.robot, traj)
            except Exception as e:
                print 'CHOMP EXCEPTION', e
        self._execTraj(start, traj)
    def extendTo(self, other):
        self.components.execTraj.extendTo(other)

class ExecSmoothedTraj:
    def __init__(self, execTraj, **kwargs):
        core.extend(self, [
            ('execTraj', execTraj, [ self.execTraj ]),
        ])
        self.env = kwargs['env']
        self.robot = kwargs['robot']
    def setExecTraj(self, _execTraj):
        self._execTraj = _execTraj
    def execTraj(self, start, traj):
        utils.restoreEnv(self.env, self.robot, start)
        simplifier = openravepy.RaveCreatePlanner(self.env, 'omplsimplifier')
        simplifier_params = openravepy.Planner.PlannerParameters()
        simplifier_params.SetExtraParameters('<time_limit>{:f}</time_limit>'.format(8))
        simplifier.InitPlan(self.robot, simplifier_params)
        simplifier.PlanPath(traj)
        self._execTraj(start, traj)
    def extendTo(self, other):
        self.components.execTraj.extendTo(other)
