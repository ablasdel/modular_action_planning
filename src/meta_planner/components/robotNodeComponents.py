from components import coreNodeComponents as components
import openravepy
from utils import utils

class ExecTrajNode:
    def __init__(self):
        pass
    def setTrajNode(self, trajNode):
        self.trajNode = trajNode
    def setExecTraj(self, execTraj):
        self.execTraj = execTraj
    def execute(self, (start, goal, _)):
        traj = self.trajNode.getTraj(start, goal)
        print 'EXEC TRAJ', traj
        self.execTraj(traj)
    def extendTo(self, other):
        other.execute = self.execute

class TrajNode:
    def __init__(self):
        self.trajs = {}
    def getTraj(self, start, goal):
        return self.trajs[(start, goal)]
    def setTraj(self, start, goal, traj):
        self.trajs[(start, goal)] = traj
    def extendTo(self, other):
        other.getTraj = self.getTraj
        other.setTraj = self.setTraj

class ExecChompdTrajNode:
    def __init__(self, execTraj, **kwargs):
        components.extend(self, [
            ('execTraj', execTraj, [ self.execTraj ]),
        ])
        self.env = kwargs['env']
        self.robot = kwargs['robot']
        self.disable = kwargs.get('disable', [])
        self.disablePadding = kwargs.get('disablePadding', [])
    def setExecTraj(self, _execTraj):
        self._execTraj = _execTraj
    def execTraj(self, traj):

        from orcdchomp import orcdchomp
        chomp = openravepy.RaveCreateModule(self.env, 'orcdchomp')
        orcdchomp.bind(chomp)

        self.computeDistanceField(chomp)


        #traj = chomp.runchomp(robot=self.robot, adofgoal=traj.GetWaypoint(-1),
        #                            lambda_=100, n_iter=50,
        #                            releasegil=True)
    
        #self._execTraj(traj)
        #return

        cspec = traj.GetConfigurationSpecification()
        cspec.AddDeltaTimeGroup()
        openravepy.planningutils.ConvertTrajectorySpecification(traj, cspec)
        for i in xrange(traj.GetNumWaypoints()):
            waypoint = traj.GetWaypoint(i)
            cspec.InsertDeltaTime(waypoint, .1)
            traj.Insert(i, waypoint, True)
        
        commonArgs = { }
        createArgs = { 'robot': self.robot }
        createArgs['starttraj'] = traj
        createArgs['lambda_'] = 100
        #createArgs['adofgoal'] = traj.GetWaypoint(-1)

        iterateArgs = { 'n_iter': 50 }
        destroyArgs = {}
        
        try:
            with utils.DisableWrapper(self.env, self.disable, self.disablePadding):
                run = chomp.create(**(dict(commonArgs, **createArgs)))
                iterateArgs['run'] = run
                destroyArgs['run'] = run

                chomp.iterate(**(dict(commonArgs, **iterateArgs)))
                chomp.destroy(**(dict(commonArgs, **destroyArgs)))
        except Exception as e:
            print e
        finally:
            self._execTraj(traj)
    def computeDistanceField(self, chomp):
        # Clone the live environment into the planning environment.
        live_robot = self.robot
        live_env = live_robot.GetEnv()

        env = openravepy.Environment()
        with live_env:
            from openravepy import CloningOptions
            env.Clone(live_env, CloningOptions.Bodies)
            robot = env.GetRobot(live_robot.GetName())

        # We can't use a with statement here because it is overriden on cloned
        # environments.
        env.Lock()
        try:
            # Disable everything.
            for body in env.GetBodies():
                body.Enable(False)

            # Compute the distance field for the non-spherized parts of HERB. This
            # includes everything that isn't attached to an arm. Otherwise the
            # initial arm will be incorrectly added to the distance field.
            robot.Enable(True)
            print("Creating the robot's distance field.")
            proximal_joints = [ manip.GetArmIndices()[0] for manip in robot.GetManipulators() ]
            for link in robot.GetLinks():
                for proximal_joint in proximal_joints:
                    if robot.DoesAffect(proximal_joint, link.GetIndex()):
                        link.Enable(False)

            cache_path = self.GetCachePath(robot)
            chomp.computedistancefield(robot, cache_filename=cache_path, releasegil=True)
            robot.Enable(False)

            # Compute separate distance fields for all other objects.
            bodyGeometryHashes = set()
            for body in env.GetBodies():
                bodyGeometryHash = body.GetKinematicsGeometryHash()
                if body != robot and bodyGeometryHash not in bodyGeometryHashes:
                    print("Creating distance field for '{0:s}'.".format(body.GetName()))
                    body.Enable(True)
                    cache_path = self.GetCachePath(body)
                    chomp.computedistancefield(body, cache_filename=cache_path, releasegil=True)
                    body.Enable(False)
                    bodyGeometryHashes.add(bodyGeometryHash)

                print('done with body %s', body.GetName())
        finally:
            env.Unlock()

        self.initialized = True 
    def GetCachePath(self, body):
        import os, rospkg
        cache_dir = rospkg.get_ros_home()
        cache_name = '{0:s}.chomp'.format(body.GetKinematicsGeometryHash())
        return os.path.join(cache_dir, cache_name)
    def extendTo(self, other):
        self.components.execTraj.extendTo(other)

class ExecSmoothedTrajNode:
    def __init__(self, execTraj, **kwargs):
        components.extend(self, [
            ('execTraj', execTraj, [ self.execTraj ]),
        ])
        self.env = kwargs['env']
        self.robot = kwargs['robot']
    def setExecTraj(self, _execTraj):
        self._execTraj = _execTraj
    def execTraj(self, traj):
        simplifier = openravepy.RaveCreatePlanner(self.env, 'omplsimplifier')
        simplifier_params = openravepy.Planner.PlannerParameters()
        simplifier_params.SetExtraParameters('<time_limit>{:f}</time_limit>'.format(3))
        simplifier.InitPlan(self.robot, simplifier_params)
        simplifier.PlanPath(traj)
        self._execTraj(traj)
    def extendTo(self, other):
        self.components.execTraj.extendTo(other)
