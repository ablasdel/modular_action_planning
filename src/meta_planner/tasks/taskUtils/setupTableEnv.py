import rospkg, roslib

import numpy as np
import herbpy

from utils import utils
import math

objects_path = rospkg.RosPack().get_path('ng_demo') + '/ordata/objects/'

def setup_env(isReal, attach_viewer=True):
    env, robot = herbpy.initialize(sim=not isReal, attach_viewer=attach_viewer)
    if isReal:
        robot.left_arm.SetVelocityLimits(np.ones(7)*1.2, 1.2)

    table = add_table(env, robot)

    bin = add_bin(env)

    return env, robot, table, bin

def set_robot_pose(env, robot, table):

    if robot.left_arm.simulated:
        if robot.GetName() == 'TIM':
            robot.right_arm.SetDOFValues(robot.configurations.get_configuration('home')[1][6:13])
            robot.left_arm.SetDOFValues(robot.configurations.get_configuration('home')[1][0:6])
        else:
            robot.right_arm.SetDOFValues(robot.configurations.get_configuration('relaxed_home')[1][7:14])
            robot.left_arm.SetDOFValues(robot.configurations.get_configuration('relaxed_home')[1][0:7])
    
    if (robot.GetName() == 'TIM' and robot.base_sim) or robot.segway_sim:
        robot_in_table = np.array([[0., 1., 0.,  0.], 
                                      [0., 0., 1.,  0.],
                                      [1., 0., 0., -1.025],
                                      [0., 0., 0.,  1.]])
        pose = np.dot(table.GetTransform(), robot_in_table)
        pose[2,3] = 0
        robot.SetTransform(pose)


def add_table(env, robot, x=-.915, y=.38):
    with env:
        table = env.ReadKinBodyXMLFile(objects_path + 'table.kinbody.xml')
        table.SetName('table_' + str(len(env.GetBodies())))
        env.AddKinBody(table)

        table_pose = np.eye(4)

        table_pose = np.array([[1., 0.,  0., x],
                                    [0., 0., -1., y],
                                    [0., 1.,  0., 0.0], 
                                    [0., 0.,  0., 1.]])

        table.SetTransform(table_pose)
    return table

def add_bin(env, x=0, y=1.2):
    with env:
        bin = env.ReadKinBodyXMLFile(objects_path + 'bin.kinbody.xml')
        env.AddKinBody(bin)

        bin_aabb = bin.ComputeAABB()

        bin_pose = np.eye(4)
        bin_pose[:3,:3] = utils.zrot(math.pi/2)

        #corner of table closest to fridge
        #bin_pose[:3,3] = [ -.2, .59, .73 + .04 + bin_aabb.extents()[2] ]
        #ground next to herb
        #bin_pose[:3,3] = [ -.1, 1.5, 0 + bin_aabb.extents()[2] ]
        #cart next to herb
        bin_pose[:3,:3] = utils.zrot(math.pi)
        bin_pose[:3,3] = [ x, y, 0 + bin_aabb.extents()[2] + .5 ]

        bin.SetTransform(bin_pose)
    return bin

def place_obj_on_table(env, table, obj, xperc, yperc):
    with env:
        obj_aabb = obj.ComputeAABB()

        table_aabb = table.ComputeAABB()

        obj_pose = np.eye(4)
        x = table_aabb.pos()[0] + table_aabb.extents()[0]*(2*xperc-1)
        y = table_aabb.pos()[1] + table_aabb.extents()[1]*(2*yperc-1)
        z = table_aabb.pos()[2] + table_aabb.extents()[2] + .01
        obj_pose[:3,3] = np.transpose([x, y, z])

        obj.SetTransform(obj_pose)

def place_glass_on_table(env, table, xperc, yperc):
    with env:
        glass = env.ReadKinBodyXMLFile(objects_path + 'glass.kinbody.xml')
        glass.SetName('glass_' + str(len(env.GetBodies())))
        env.AddKinBody(glass)
        place_obj_on_table(env, table, glass, xperc, yperc)
        return glass
        
def place_bowl_on_table(env, table, xperc, yperc):
    with env:
        bowl = env.ReadKinBodyXMLFile(objects_path + 'bowl.kinbody.xml')
        bowl.SetName('bowl_' + str(len(env.GetBodies())))
        env.AddKinBody(bowl)
        place_obj_on_table(env, table, bowl, xperc, yperc)
        return bowl

def place_plate_on_table(env, table, xperc, yperc):
    with env:
        plate = env.ReadKinBodyXMLFile(objects_path + 'plate.kinbody.xml')
        plate.SetName('plate_' + str(len(env.GetBodies())))
        env.AddKinBody(plate)
        place_obj_on_table(env, table, plate, xperc, yperc)
        return plate
