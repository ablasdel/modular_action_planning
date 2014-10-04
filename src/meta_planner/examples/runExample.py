import roslib; package_name='meta_planner'; roslib.load_manifest(package_name)
import argparse
import herbpy
import importlib
from components import coreNodeComponents as components
import components.metaNodeComponents as metaComponents

import numpy as np
import time
import sys
import random

from utils import utils

def test():

    sys.setrecursionlimit(20000)

    parser = argparse.ArgumentParser(description='tool for running example plans')
    parser.add_argument('--example', type=str, help='what example to load')
    parser.add_argument('--trials', type=int, default=-1, help='how many trials to run')
    parser.add_argument('--viewExec', action='store_true', default=False, help='which viewer to show')
    parser.add_argument('--viewSim', action='store_true', default=False, help='which viewer to show')
    parser.add_argument('--seed', type=int, default=-1, help='random number generator seed')
    parser.add_argument('--saveto', type=str, default=None, help='where to save execution results')
    parser.add_argument('--numStarts', type=int)
    parser.add_argument('--startToTree', action='store_true', default=False)
    parser.add_argument('--heuristic', action='store_true', default=False)
    parser.add_argument('--maxtime', type=int, default=300)
    parser.add_argument('--saveEnvs', action='store_true', default=False)
    parser.add_argument('--real', action='store_true', default=False)
    args = parser.parse_args()

    if args.seed != -1:
        random.seed(args.seed)

    isReal = args.real
    env, robot = herbpy.initialize(sim=True, attach_viewer=args.viewSim)
    execEnv, execRobot = herbpy.initialize(sim=not isReal, attach_viewer=args.viewExec, segway_sim=True)
    if isReal:
        execRobot.left_arm.SetVelocityLimits(np.ones(7)*.5, .5)
        execRobot.right_arm.SetVelocityLimits(np.ones(7)*.5, .5)

    example = importlib.import_module(args.example)

    example.init_env(execEnv, execRobot)

    start = utils.saveEnv(execEnv, execRobot)
    utils.restoreEnv(env, robot, utils.saveEnv(execEnv, execRobot))

    common = {}
    common['env'] = env
    common['robot'] = robot
    common['execEnv'] = execEnv
    common['execRobot'] = execRobot
    common['args'] = args

    ts = []

    i = 0
    while i < args.trials or args.trials == -1:
        utils.restoreEnv(execEnv, execRobot, start)

        example.reset_env(execEnv, execRobot)

        start = utils.saveEnv(execEnv, execRobot)
        utils.restoreEnv(env, robot, start)

        t0 = time.time()
        node = example.get_plan(common)
        components.statesContainer['val'] = 0
        node.addConnectedStart(start)
        limit = args.maxtime
        node.runFor(limit)
        #node.waitTillDone()
        ts.append(time.time() - t0)
        print '================================='
        print len(ts)
        print sum(ts)/len(ts)
        print np.std(ts)
        print '---------------------------------'
        print ts
        print '================================='
        #node.execute(list(node.getConnectedStartGoalPairs())[0])
        #import components.metaNodeComponents
        #allEnvs = components.metaNodeComponents.allEnvs
        import pickle

        if args.saveto:
            try:
                with open(args.saveto, 'rb') as f:
                    data = pickle.load(f)
            except:
                data = []

            with open(args.saveto, 'wrb') as f:
                data.append({
                    "time": ts[-1],
                    "limit": limit,
                    "states": components.statesContainer['val'],
                    "seed": args.seed
                })
                pickle.dump(data, f)
            #import IPython; IPython.embed()
        i += 1

        #if args.saveEnvs:
        #    allEnvs = metaComponents.allEnvs
        #    import pickle
        #    pickle.dump(allEnvs, open('envs.p', 'wb'))
        #    import IPython; IPython.embed()
test()

