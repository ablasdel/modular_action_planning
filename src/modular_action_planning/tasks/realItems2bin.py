from utils.startToTree import StartToTree
from taskUtils import setupTableEnv
from taskUtils import setupTableEnv

import nodes.meta
import nodes.robot

from tableNodes.item2bin import Item2Bin
from taskUtils import tableManip

import numpy
import random

import math

import rospkg, roslib
objects_path = rospkg.RosPack().get_path('modular_action_planning') + '/ordata/objects/'

def add_arguments(parser):
    pass

def init_env(common):
    env = common['execEnv']
    robot = common['execRobot']

    table = setupTableEnv.add_table(env, robot)
    table.SetName('table')
    setupTableEnv.set_robot_pose(env, robot, table)
    if robot.GetName() != 'TIM':
        bin = setupTableEnv.add_bin(env)
    else:
        bin = setupTableEnv.add_bin(env, y=1.3)
    #tableManip.reset_env(env, robot, N=2, includePlate=False, includeBowls=False)

    if robot.GetName() != 'TIM':
        robot.head.SetStiffness(1)
        robot.head.MoveTo([math.pi/16, -math.pi/16])

    detect=False

    if robot.GetName() == 'HERB' and not robot.head.simulated and detect:
        import percy.kinbody as kb
        detector = kb.KinBodyDetector(env, '', 'object_poses_array', objects_path, '/herb_base', '/kinect2_rgb')
        detector.update()
    else:
        #setupTableEnv.place_bowl_on_table(env, table, .7, .8)
        setupTableEnv.place_glass_on_table(env, table, .7, .8)
        setupTableEnv.place_glass_on_table(env, table, .6, .7)
        setupTableEnv.place_glass_on_table(env, table, .65, .55)
        setupTableEnv.place_glass_on_table(env, table, .75, .6)

    snap_to_table(env, table, [ obj for obj in env.GetBodies() if obj.GetName().startswith('tag_glass') ])

    import IPython; IPython.embed()
    if robot.GetName() == 'HERB':
        robot.left_arm.SetStiffness(1)


def snap_to_table(env, table, objects):
    with env:
        table_aabb = table.ComputeAABB()
        table_height = table_aabb.pos()[2] + table_aabb.extents()[2]

        for obj in objects:
            T = obj.GetTransform()
            T[0:3,0:3] = numpy.eye(3)
            T[2,3] = table_height + .01
            if 'bowl' in obj.GetName():
                T[2,3] = table_height + .03
            obj.SetTransform(T)

def reset_env(env, robot):
    pass
    #tableManip.reset_env(env, robot)

def get_plan(common):
    arm = common['robot'].left_arm

    subnodes = []
    numToMove = 0
    env = common['env']
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('glass')])
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('tag_glass')])
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('bowl')])
    numToMove += sum([1 for b in env.GetBodies() if b.GetName().startswith('plate')])
    

    for _ in xrange(numToMove):
        subnodes.append(nodes.meta.Exec(Item2Bin(arm, **common)))

    node = nodes.meta.NormalizedSeq(subnodes)
    return node

