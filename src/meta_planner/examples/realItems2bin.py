from utils.startToTree import StartToTree
from examples import setupTableEnv
from components import coreNodeComponents as components

from nodes import metaNodes
from nodes import robotNodes

from tableActions.DynamicToBin import DynamicToBin
from utils import tableManip

import numpy
import random

import math

import rospkg, roslib
objects_path = rospkg.RosPack().get_path('ng_demo') + '/ordata/objects/'

def init_env(env, robot):

    table = setupTableEnv.add_table(env, robot)
    table.SetName('table')
    setupTableEnv.set_robot_pose(env, robot, table)
    bin = setupTableEnv.add_bin(env)
    #tableManip.reset_env(env, robot, N=2, includePlate=False, includeBowls=False)
    #setupTableEnv.place_bowl_on_table(env, table, .7, .8)
    #setupTableEnv.place_glass_on_table(env, table, .6, .7)

    robot.head.MoveTo([math.pi/16, -math.pi/16])

    import percy.kinbody as kb
    detector = kb.KinBodyDetector(env, '', 'object_poses_array', objects_path, '/herb_base', '/kinect2_rgb')
    detector.update()

    snap_to_table(env, table, [ obj for obj in env.GetBodies() if obj.GetName().startswith('tag_glass') ])


def snap_to_table(env, table, objects):
    with env:
        table_aabb = table.ComputeAABB()
        table_height = table_aabb.pos()[2] + table_aabb.extents()[2]

        for obj in objects:
            T = obj.GetTransform()
            T[0:3,0:3] = numpy.eye(3)
            T[2,3] = table_height + .01
            obj.SetTransform(T)

def reset_env(env, robot):
    pass
    #tableManip.reset_env(env, robot)

def get_plan(common):
    if common['args'].startToTree:
        if 'startToTree' not in common:
            common['startToTree'] = StartToTree()
    else:
        common['startToTree'] = None#StartToTree()
    arm = common['robot'].left_arm

    subnodes = []
    numToMove = 0
    env = common['env']
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('glass')])
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('tag_glass')])
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('bowl')])
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('plate')])
    

    #common['planType'] = 'RRTConnect'
    common['numStarts'] = common['args'].numStarts

    for _ in xrange(numToMove):
        subnodes.append(metaNodes.CheckpointNode(DynamicToBin(arm, **common)))

    #robot = common['robot']
    #home_config = robot.configurations.get_configuration('home')[1][0:7]
    #relaxed_home_config = robot.configurations.get_configuration('relaxed_home')[1][0:7]
    #subnodes = [
    #    robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
    #    robotNodes.PlanArmToConfigNode(config=relaxed_home_config, armName=arm.GetName(), **common),
    #    robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
    #    robotNodes.PlanArmToConfigNode(config=relaxed_home_config, armName=arm.GetName(), **common),
    #    robotNodes.PlanArmToConfigNode(config=home_config, armName=arm.GetName(), **common),
    #]

    node = metaNodes.PrioritizedSeqNode(subnodes)
    return metaNodes.ExecNode(node)

