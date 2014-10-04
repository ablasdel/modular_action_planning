from utils.startToTree import StartToTree
from examples import setupTableEnv

from nodes import metaNodes
from nodes import robotNodes

from tableActions.GrabGlass2 import GrabGlass
from tableActions.PlaceObjInBinNode import PlaceObjInBin

import numpy

def init_env(env, robot):

    table = setupTableEnv.add_table(env, robot)
    setupTableEnv.set_robot_pose(env, robot, table)
    bin = setupTableEnv.add_bin(env)

    x = .5 + 2 / (2.0*3)
    y = .6 + 2 / (2.0*3)
    glass = setupTableEnv.place_glass_on_table(env, table, x, y)

def reset_env(env, robot):
    pass
    #setupTableEnv.place_obj_on_table(execEnv, table, glass, locs[0][0], locs[0][1])

def get_plan(common):
    if 'startToTree' not in common:
        common['startToTree'] = StartToTree()

    for body in common['env'].GetBodies():
        if body.GetName().startswith('glass'):
            glass = body
            break
    arm = common['robot'].left_arm

    home_config = numpy.copy(common['robot'].configurations.get_configuration('home')[1][0:7])

    return metaNodes.ExecNode(
        metaNodes.PrioritizedSeqNode([
            GrabGlass(glass.GetName(), arm, **common),
            PlaceObjInBin(glass.GetName(), arm, **common),
            robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
        ])
    )
