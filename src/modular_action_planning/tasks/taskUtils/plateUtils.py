import math
import numpy as np
import openravepy
from utils import utils


def get_plate_poses(env, robot, c, arm, plate):
    plate_pose_at_edge = get_plate_pose_at_edge(env, robot, arm, c, plate)
    with env:
        table = env.GetKinBody('table')
        tableAABB = table.ComputeAABB()

        start_to_end = np.array(plate.GetTransform()[:2,3] - plate_pose_at_edge[:2,3])
        moveLen = np.linalg.norm(start_to_end)
        start_to_end = start_to_end / moveLen


        pose_above_plate = np.eye(4)
        pose_above_plate[2,3] = tableAABB.pos()[2] + tableAABB.extents()[2] + c['startDistanceAboveTable']
        pose_above_plate[:2,3] = plate.GetTransform()[:2,3] + start_to_end*c['startPlateOffset']
        pose_above_plate[:3,:3] = utils.zrot(math.atan2(start_to_end[1], start_to_end[0]))
        pose_above_plate[:3,:3] = np.dot(pose_above_plate[:3,:3], utils.yrot(math.pi))
        pose_above_plate[:3,:3] = np.dot(pose_above_plate[:3,:3], utils.zrot(math.pi))

        pose_above_plate[:3,3] += np.dot(pose_above_plate[:3,:3], [0, 0, -.25])

        moveDir = np.zeros(3)
        moveDir[:2] = -start_to_end
        
    #if not isExec:
    #    A = openravepy.misc.DrawAxes(env, pose_above_plate)
    #    A.SetShow(True)
    #    B = openravepy.misc.DrawAxes(env, plate_pose_at_edge)
    #    B.SetShow(True)
    #    raw_input()
    #    A.SetShow(False)
    #    B.SetShow(False)

    return pose_above_plate, plate_pose_at_edge, moveDir, moveLen

def get_plate_pose_at_edge(env, robot, arm, c, plate):
    #TODO for smaller sides (0,2) cap amount plate can go off edge
    with env:
        table = env.GetKinBody('table')
        tableAABB = table.ComputeAABB()
        plateFracOffTable = 1/4.
        xw = tableAABB.extents()[0] - plate.ComputeAABB().extents()[1]*2*plateFracOffTable
        yw = tableAABB.extents()[1] - plate.ComputeAABB().extents()[1]*2*plateFracOffTable
        tableXYs = [
            tableAABB.pos()[:2] + [ xw, yw ],
            tableAABB.pos()[:2] + [ xw, -yw ],
            tableAABB.pos()[:2] + [ -xw, yw ],
            tableAABB.pos()[:2] + [ -xw, -yw ],
        ]
        tableEdges = [
            [ tableXYs[0], tableXYs[1] ],
            [ tableXYs[0], tableXYs[2] ],
            [ tableXYs[3], tableXYs[1] ],
            [ tableXYs[3], tableXYs[2] ],
        ]
        tableEdge = tableEdges[c['tableEdge']]

    #V = np.eye(4)
    #V[:2,3] = [ (tableEdge[0][0] + tableEdge[1][0])/2, (tableEdge[0][1] + tableEdge[1][1])/2 ]
    #A = openravepy.misc.DrawAxes(env, V)
    #A.SetShow(True)
    #raw_input('%!')
    #A.SetShow(False)

    with env:
        #robotXY = robot.GetTransform()[:2,3]
        if robot.GetName() == 'TIM':
            shoulderTransform = robot.GetLink('TEMASim/ARM_l/pba_base').GetTransform()
        else:
            shoulderTransform = robot.GetLink('/' + arm.GetName() + '/wam1').GetTransform()

        shoulderXY = shoulderTransform[0:2,3]
        plateXY = plate.GetTransform()[:2,3]
        robotPlate = [shoulderXY, plateXY]
        M1 = (tableEdge[1][1] - tableEdge[0][1]+.0001)/(tableEdge[1][0] - tableEdge[0][0]+.0001)
        B1 = tableEdge[0][1] - M1*tableEdge[0][0]
        M2 = (robotPlate[1][1] - robotPlate[0][1]+.0001)/(robotPlate[1][0] - robotPlate[0][0]+.0001)
        B2 = robotPlate[0][1] - M2*robotPlate[0][0]
        intersect = [ (B2-B1)/(M1-M2), (B2-B1)/(M1-M2)*M1 + B1 ]

        plate_pose_at_edge = np.eye(4)
        plate_pose_at_edge[:2,3] = intersect
        edgeUnitVec = np.array([tableEdge[0][0] - tableEdge[1][0], tableEdge[0][1] - tableEdge[1][1] ])
        edgeUnitVec = edgeUnitVec / np.linalg.norm(edgeUnitVec)

    with env:
        slideTargetOffset = c['slideTargetOffset']
        plate_pose_at_edge[:2,3] += slideTargetOffset*edgeUnitVec
        plate_pose_at_edge[2,3] = tableAABB.pos()[2] + tableAABB.extents()[2]

    return plate_pose_at_edge


def get_pose_near_plate(env, c, arm, plate):
    with env:
        table = env.GetKinBody('table')
        tableAABB = table.ComputeAABB()
        plate_pose = plate.GetTransform()
        xp = (tableAABB.pos()[0] - plate_pose[0,3])/tableAABB.extents()[0]
        yp = (tableAABB.pos()[1] - plate_pose[1,3])/tableAABB.extents()[1]
        table_edge = np.copy(plate_pose)
        plateToTable = np.zeros(2)
        if abs(xp) < abs(yp):
            if yp > 0:
                plateToTable[1] = -(1-yp)*tableAABB.extents()[1]
            else:
                plateToTable[1] = -(-1-yp)*tableAABB.extents()[1]
        else:
            if xp > 0:
                plateToTable[0] += -(1-xp)*tableAABB.extents()[0]
            else:
                plateToTable[0] += -(-1-xp)*tableAABB.extents()[0]
        pose_near_plate = plate_pose
        plateRadius = plate.ComputeAABB().extents()[1]
        plateToTableLen = np.linalg.norm(plateToTable)
        unit_plate_to_table = plateToTable / plateToTableLen
        pose_near_plate[:2,3] += unit_plate_to_table*(c['startDistanceFromPlate']+plateRadius)
        pose_near_plate[:3,:3] = np.dot(pose_near_plate[:3,:3], utils.xrot(math.pi/2))
        pose_near_plate[:3,:3] = np.dot(pose_near_plate[:3,:3], utils.zrot(-math.pi/2))
        pose_near_plate[:3,3] += np.dot(pose_near_plate[:3,:3], [0, 0, -.25])

        moveDir = np.zeros(3)
        moveDir[:2] = -unit_plate_to_table

    #if not domain.isExec:
    #    A = openravepy.misc.DrawAxes(domain.env, plate_pose)
    #    A.SetShow(True)
    #    B = openravepy.misc.DrawAxes(domain.env, pose_near_plate)
    #    B.SetShow(True)
    #    raw_input()
    #    A.SetShow(False)
    #    B.SetShow(False)

    return pose_near_plate, moveDir, c['startDistanceFromPlate'] + plateRadius - plateToTableLen
