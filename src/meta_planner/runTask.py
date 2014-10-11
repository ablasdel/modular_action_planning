import roslib; package_name='meta_planner'; roslib.load_manifest(package_name)
import argparse
import herbpy
import timpy
import importlib

import numpy as np
import time
import sys
import random

from utils import utils

def main():
    parser, args = setupArgs()
    common = setupEnv(parser, args)
    task, start = setupTask(parser, args, common)
    run(args, common, task, start)

def setupArgs():
    sys.setrecursionlimit(20000)

    parser = argparse.ArgumentParser(description='tool for running task plans')
    parser.add_argument('--task', type=str, help='what task to load')
    parser.add_argument('--trials', type=int, default=1, help='how many trials to run')
    parser.add_argument('--viewExec', action='store_true', default=False, help='which viewer to show')
    parser.add_argument('--viewSim', action='store_true', default=False, help='which viewer to show')
    parser.add_argument('--maxtime', type=int, default=300)
    parser.add_argument('--robot', type=str, default="herb")
    parser.add_argument('--real', action='store_true', default=False)
    args, unknown = parser.parse_known_args()
    return parser, args

def setupEnv(parser, args):
    if args.robot == 'herb':
        robotpy = herbpy
        realArgs = { 'segway_sim': True }
    else:
        robotpy = timpy
        realArgs = { 'base_sim': True }

    env, robot = robotpy.initialize(sim=True, attach_viewer=args.viewSim)
    execEnv, execRobot = robotpy.initialize(sim=not args.real, attach_viewer=args.viewExec, **realArgs)
    if args.real:
        execRobot.left_arm.SetVelocityLimits(np.ones(7)*.5, .5)
        execRobot.right_arm.SetVelocityLimits(np.ones(7)*.5, .5)

    common = {}
    common['env'] = env
    common['robot'] = robot
    common['execEnv'] = execEnv
    common['execRobot'] = execRobot
    common['args'] = args

    if args.robot == 'herb':
        dl = common['robot'].GetDOFLimits()
        dl[1][11] = 4.8
        common['robot'].SetDOFLimits(dl[0], dl[1])
        common['execRobot'].SetDOFLimits(dl[0], dl[1])
    return common

def setupTask(parser, args, common):
    task = importlib.import_module(args.task)
    task.add_arguments(parser)
    args = parser.parse_args()
    common['args'] = args

    task.init_env(common)

    start = utils.saveEnv(common['execEnv'], common['execRobot'])
    utils.restoreEnv(common['env'], common['robot'], utils.saveEnv(common['execEnv'], common['execRobot']))
    return task, start

def run(args, common, task, start):
    ts = []
    execEnv = common['execEnv']
    execRobot = common['execRobot']
    env = common['env']
    robot = common['robot']

    i = 0
    while i < args.trials or args.trials == -1:
        utils.restoreEnv(execEnv, execRobot, start)

        task.reset_env(execEnv, execRobot)

        start = utils.saveEnv(execEnv, execRobot)
        utils.restoreEnv(env, robot, start)

        t0 = time.time()
        node = task.get_plan(common)
        node.addConnectedStart(start)
        node.runFor(args.maxtime)
        #node.waitTillDone()
        ts.append(time.time() - t0)
        print '================================='
        print 'trials:', len(ts), '/', args.trials
        print 'avg(s):', sum(ts)/len(ts)
        print 'dev(s):',  np.std(ts)
        print '---------------------------------'
        print 'raw:'
        print ts
        print '================================='
        i += 1
main()
