import utils
import openravepy

import prpy.kin, prpy.tsr
from prpy.planning import PlanningError, UnsupportedPlanningError
import numpy
import numpy as np

def planEEOffsetCBiRRT(env, robot, armName, start, direction, distance, disable, disablePadding):
    #raw_input('CBIRRT')
    with env:
        arm = robot.GetManipulator(armName)
        arm.SetActive()
        start_config = arm.GetDOFValues()

    with utils.DisableWrapper(env, disable, disablePadding):
        with env:
            finalPose = arm.GetEndEffectorTransform()
            finalPose[:3,3] += numpy.transpose(numpy.array(direction)*distance)
            finalPoseIk = arm.FindIKSolution(finalPose, openravepy.IkFilterOptions.CheckEnvCollisions)
            if finalPoseIk == None:
                raise Exception('no ik')

            manip = robot.GetActiveManipulator()
            H_world_ee = manip.GetEndEffectorTransform()

            # 'object frame w' is at ee, z pointed along direction to move
            H_world_w = prpy.kin.H_from_op_diff(H_world_ee[0:3,3], direction)
            H_w_ee = numpy.dot(prpy.kin.invert_H(H_world_w), H_world_ee)
        
            # Serialize TSR string (goal)
            Hw_end = numpy.eye(4)
            Hw_end[2,3] = distance

            goaltsr = prpy.tsr.tsr.TSR(T0_w = numpy.dot(H_world_w,Hw_end), 
                                     Tw_e = H_w_ee, 
                                     Bw = numpy.zeros((6,2)), 
                                     manip = robot.GetActiveManipulatorIndex())
            goal_tsr_chain = prpy.tsr.tsr.TSRChain(sample_goal = True,
                                                 TSRs = [goaltsr])
            # Serialize TSR string (whole-trajectory constraint)
            Bw = numpy.zeros((6,2))
            epsilon = 0.001
            Bw = numpy.array([[-epsilon,            epsilon],
                              [-epsilon,            epsilon],
                              [min(0.0, distance),  max(0.0, distance)],
                              [-epsilon,            epsilon],
                              [-epsilon,            epsilon],
                              [-epsilon,            epsilon]])

            trajtsr = prpy.tsr.tsr.TSR(T0_w = H_world_w, 
                                   Tw_e = H_w_ee, 
                                   Bw = Bw, 
                                   manip = robot.GetActiveManipulatorIndex())
            traj_tsr_chain = prpy.tsr.tsr.TSRChain(constrain=True, TSRs=[ trajtsr ])
        
        extra_args = ['psample', '0.1']
        extra_args += [ 'TSRChain', goal_tsr_chain.serialize() ]
        extra_args += [ 'TSRChain', traj_tsr_chain.serialize() ]

        problem = openravepy.RaveCreateProblem(env, 'CBiRRT')
        env.LoadProblem(problem, robot.GetName())
        args = [ 'RunCBiRRT' ]
        args += [ 'smoothingitrs', str(1) ]
        args += [ 'timelimit', str(1) ]
        args += [ 'jointstarts', str(len(start_config)), ' '.join([ str(x) for x in start_config ]) ]
        args += [ 'allowlimadj', str(1) ]
        args += extra_args

        # FIXME: Why can't we write to anything other than cmovetraj.txt or
        # /tmp/cmovetraj.txt with CBiRRT?
        traj_path = 'cmovetraj.txt'
        args += [ 'filename', traj_path ]
        args_str = ' '.join(args)

        response = problem.SendCommand(args_str, True)
        if not response.strip().startswith('1'):
            raise PlanningError('Unknown error: ' + response)

        with open(traj_path, 'rb') as traj_file:
            traj_xml = traj_file.read()
            with env:
                traj = openravepy.RaveCreateTrajectory(env, '')
            traj.deserialize(traj_xml)

    with env:
        arm.SetDOFValues(finalPoseIk)
    goal = utils.saveEnv(env, robot)

    return [(start, goal, traj)]

def planArmRRTConnect(env, robot, armName, start, config, startToTree, disable, disablePadding):
    with env:
        arm = robot.GetManipulator(armName)
    with utils.DisableWrapper(env, disable, disablePadding):

        with env:
            activeDOFs = arm.GetArmIndices()
            robot.SetActiveDOFs(activeDOFs)
            starts = [ robot.GetDOFValues()[robot.GetActiveDOFIndices()] ]
        planner = openravepy.RaveCreatePlanner(env, 'ompl')

        params = openravepy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(robot)
        params.SetGoalConfig(config)
        params.SetExtraParameters("<planner_type>RRTConnect</planner_type>" +
                                  "<time_limit>5</time_limit>")
        planner.InitPlan(robot, params)

        traj = openravepy.RaveCreateTrajectory(env, '')
        status = planner.PlanPath(traj)

        if status != status.HasSolution:
            raise Exception('planning failed')
        with env:
            arm.SetDOFValues(config)

    goal = utils.saveEnv(env, robot)
    startGoalPairs = set()
    startGoalPairs.add((start, goal, traj))
    return startGoalPairs

def planArmAnytimePRM(env, robot, armName, start, config, startToTree, disable, disablePadding):
    with env:
        arm = robot.GetManipulator(armName)
    with utils.DisableWrapper(env, disable, disablePadding):
        with env:
            activeDOFs = arm.GetArmIndices()
            robot.SetActiveDOFs(activeDOFs)
        with env:
            starts = [ robot.GetDOFValues()[robot.GetActiveDOFIndices()] ]
        goals = [ config ]

        planner = startToTree.getTree(start, robot, activeDOFs)

        planner = openravepy.RaveCreatePlanner(env, 'ompl')

        params = openravepy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(robot)
        params.SetGoalConfig(config)
        params.SetExtraParameters("<planner_type>PRM</planner_type>" +
                                  "<is_anytime>1</is_anytime>" +
                                  "<time_limit>5</time_limit>")
        planner.InitPlan(robot, params)

        user_data = env.GetUserData()
        if user_data is None:
            user_data = dict()
            env.SetUserData(user_data)
        USERDATA_DESTRUCTOR = '__destructor__'

        def planCB(_):
            return openravepy.PlannerAction.Interrupt
        handle = planner.RegisterPlanCallback(planCB)
        user_data[USERDATA_DESTRUCTOR] = handle

        traj = openravepy.RaveCreateTrajectory(env, '')
        status = planner.PlanPath(traj)

        if status != status.HasSolution and status != status.InterruptedWithSolution:
            raise Exception('planning failed')
        with env:
            arm.SetDOFValues(config)
        #return traj

def planArmReuseMultiPRM(env, robot, armName, startEnvs, configs, startToTree, disable, disablePadding):
    print 'START'
    #NOTE we assume starts are compatible
    with env:
        arm = robot.GetManipulator(armName)
    
    #for start in startEnvs:
    #    with env:
    #        utils.restoreEnv(env, robot, start)
    #    raw_input('S')
    #for config in configs:
    #    with env:
    #        arm.SetDOFValues(config)
    #    raw_input('G')
    #raw_input('D')

    with env:
        activeDOFs = arm.GetArmIndices()
    starts = []
    for start in startEnvs:
        with env:
            utils.restoreEnv(env, robot, start)
            robot.SetActiveDOFs(activeDOFs)
            starts.append(robot.GetDOFValues()[robot.GetActiveDOFIndices()])
        import time
        time.sleep(.1)
    for config in configs:
        import time
        with env:
            arm.SetDOFValues(config)
        time.sleep(.1)
    with env:
        utils.restoreEnv(env, robot, startEnvs[0])
    goals = configs
    #if len(list(startEnvs)[0]['grabbed']) > 0:
    #    t = list(startEnvs)[0]['bodies']['glass_3']['transform']
    #    objInEE = np.dot(np.linalg.inv(arm.GetEndEffectorTransform()), t)
    #    print('objInEE', np.around(objInEE, 4))
    with utils.DisableWrapper(env, disable, disablePadding):
        with env:
            if startToTree == None:
                planner = None
            else:
                planner = startToTree.getTree(list(startEnvs)[0], robot, activeDOFs)
        isNewPlanner = False
        if planner == None:
            planner = openravepy.RaveCreatePlanner(env, 'ompl')

            params = openravepy.Planner.PlannerParameters()
            params.SetRobotActiveJoints(robot)
            params.SetGoalConfig(configs[0])
            params.SetExtraParameters("<planner_type>ReusePRM</planner_type>" +
                                      "<anytime_time_limit>2</anytime_time_limit>" +
                                      "<is_anytime>1</is_anytime>")
            planner.InitPlan(robot, params)

            isNewPlanner = True

            user_data = env.GetUserData()
            if user_data is None:
                user_data = dict()
                env.SetUserData(user_data)
            USERDATA_DESTRUCTOR = '__destructor__'

            MAX_TIMES = 3
            if robot.GetName() == 'TIM':
                MAX_TIMES = 8

            times = { 'n': 0, 'bail': False }
            def planCB(_):
                times['n'] += 1
                print 'times', times['n'], times['bail']
                if times['n'] < MAX_TIMES and not times['bail']:
                    return openravepy.PlannerAction.ReturnWithAnySolution
                else:
                    return openravepy.PlannerAction.Interrupt
            handle = planner.RegisterPlanCallback(planCB)
            #import IPython; IPython.embed()
            planner.SetUserData(times)
            import random
            user_data[USERDATA_DESTRUCTOR + str(random.random())] = handle

            with env:
                if startToTree != None:
                    compatiblePlanner = startToTree.getCompatibleTree(list(startEnvs)[0], robot, activeDOFs)
                else:
                    compatiblePlanner = None
        print 0
        planner.GetUserData()['n'] = 0
        planner.GetUserData()['bail'] = False

        try:
            planner.SendCommand('prm_clearQuery')
        except:
            #import IPython; IPython.embed()
            raise
        print starts, goals
        setStartAndGoalStatesCmd = 'prm_setStartAndGoalStates '
        setStartAndGoalStatesCmd += str(len(starts)) + ' '
        for state in starts:
            for dof in state:
                setStartAndGoalStatesCmd += str(dof) + ' ';
        setStartAndGoalStatesCmd += str(len(goals)) + ' '
        for state in goals:
            for dof in state:
                setStartAndGoalStatesCmd += str(dof) + ' ';
        planner.SendCommand(setStartAndGoalStatesCmd)

        print 1
        planner.SendCommand('prm_setup')
        print 1.5

        if isNewPlanner:
            #raw_input('new planner')
            #import IPython; IPython.embed()
            if compatiblePlanner != None:
                serialized = compatiblePlanner.SendCommand('prm_serialize')
                try:
                    planner.SendCommand('prm_deserialize ' + serialized)
                except Exception as e:
                    print e
                    print '========================'
                    print serialized
                    print '========================'
                    raw_input('excpt')
                    raise
                #raw_input('deserialized')
        #else:
        #    raw_input('reuse')

        with env:
            traj = openravepy.RaveCreateTrajectory(env, '')
        try:
            with env:
                status = planner.PlanPath(traj)
        except Exception as e:
            print e
            #import IPython; IPython.embed()
            raise

        print 2

        traj = {
            'has': False,
        }

        trajs = []

        def checkTrajs():
            trajStr = planner.SendCommand('prm_listTrajectories')
            if len(trajStr) > 0:
                trajSegments = trajStr.split(' ')[0:-1]
                print trajSegments
                for i in xrange(0, len(trajSegments), 2):
                    trajs.append((int(trajSegments[i]), int(trajSegments[i+1])))
                    traj['has'] = True
                    planner.GetUserData()['bail'] = True
        print 3
        if not traj['has']:
            checkTrajs()
        print 4

        utils.restoreEnv(env, robot, list(startEnvs)[0])
        with env:
            if startToTree != None:
                startToTree.putTree(start, planner, robot, activeDOFs)

        if not traj['has']:
            raise Exception('planning failed')

    print 5
    startGoalPairs = set()
    #goals = set()
    for (startIdx, goalIdx) in trajs:
        with env:
            arm.SetDOFValues(configs[goalIdx])
        goal = utils.saveEnv(env, robot)
        with env:
            traj = openravepy.RaveCreateTrajectory(env, '')
        planner.PlanPath(traj)
        success = planner.SendCommand('prm_getTrajectory ' + str(startIdx) + ' ' + str(goalIdx))
        if success == "success":
            startGoalPairs.add((startEnvs[startIdx], goal, traj))
    print 'END'
    return startGoalPairs
