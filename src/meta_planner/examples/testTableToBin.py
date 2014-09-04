import roslib; package_name='map_planner'; roslib.load_manifest(package_name)
import rospkg
import herbpy
import openravepy

import numpy as np
import copy
import time
import random
import math
import sys

from tableDynToBin import tableDynToBin

from utils import utils
from examples import setupTableEnv
from utils.startToTree import StartToTree

def test():
    viewExec = True
    viewExec = False

    sys.setrecursionlimit(20000)

    env, robot, table, bin = setupTableEnv.setup_env(isReal=False, attach_viewer=not viewExec)
    execEnv, execRobot, _, _ = setupTableEnv.setup_env(isReal=False, attach_viewer=viewExec)

    locs = [(.8, .9), (.6, .8), (.5, .8)]
    random.shuffle(locs)

    glass = setupTableEnv.place_glass_on_table(execEnv, table, locs[0][0], locs[0][1])
    bowl = setupTableEnv.place_bowl_on_table(execEnv, table, locs[1][0], locs[1][1])
    plate = setupTableEnv.place_plate_on_table(execEnv, table, locs[2][0], locs[2][1])
    arm = robot.left_arm
    start = utils.saveEnv(execEnv, execRobot)


    startToTree = StartToTree()

    N = 5000

    ts = []

    utils.restoreEnv(env, robot, utils.saveEnv(execEnv, execRobot))

    for i in xrange(N):
        utils.restoreEnv(execEnv, execRobot, start)

        random.shuffle(locs)
        setupTableEnv.place_obj_on_table(execEnv, table, glass, locs[0][0], locs[0][1])
        setupTableEnv.place_obj_on_table(execEnv, table, bowl, locs[1][0], locs[1][1])
        setupTableEnv.place_obj_on_table(execEnv, table, plate, locs[2][0], locs[2][1])

        start = utils.saveEnv(execEnv, execRobot)
        utils.restoreEnv(env, robot, start)

        t0 = time.time()
        #node = tableSeqToBin(bowl, glass, plate, startToTree, arm, env, robot, execEnv, execRobot)
        node = tableDynToBin(startToTree, arm, env, robot, execEnv, execRobot)
        node.addConnectedStart(start)
        node.run()
        node.waitTillDone()
        ts.append(time.time() - t0)
        print '================================='
        print len(ts)
        print sum(ts)/len(ts)
        print np.std(ts)
        print '---------------------------------'
        print ts
        print '================================='
test()
