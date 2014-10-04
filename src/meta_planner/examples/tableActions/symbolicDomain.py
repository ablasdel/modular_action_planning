
import numpy
import pddlpy

from nodes import robotNodes

from tableActions.GrabGlass2 import GrabGlass
from tableActions.GraspBowlNode import GrabBowl
from tableActions.SlideAndGraspPlateNode import SlideAndGraspPlate
from tableActions.PlaceObjInBinNode import PlaceObjInBin
from tableActions.PlaceObjOnObj import PlaceObjOnObj
from tableActions.PlaceObjNextToObj import PlaceObjNextToObj

def get(env, robot, common):

    left_home_config = numpy.copy(robot.configurations.get_configuration('home')[1][0:7])
    right_home_config = numpy.copy(robot.configurations.get_configuration('home')[0][0:7])

    objects = [
        robot.left_arm.GetName(),
        robot.right_arm.GetName(),
    ]
    objects += [ b.GetName() for b in env.GetBodies() ]

    def ssd(A, B):
        return sum((A-B)**2)

    @pddlpy.predicate
    def true():
        return True

    @pddlpy.predicate
    def robot_arm_home(armName):
        with env:
            arm = robot.GetManipulator(armName)
            if arm == robot.left_arm:
                return bool(ssd(arm.GetDOFValues(), left_home_config) < .001)
            elif arm == robot.right_arm:
                return bool(ssd(arm.GetDOFValues(), right_home_config) < .001)
            return False

    @pddlpy.predicate
    def obj_on_obj(objname, on_objname):
        with env:
            obj = env.GetKinBody(objname)
            on_obj = env.GetKinBody(on_objname)
            if obj == None or on_obj == None:
                return False

            obj_aabb = obj.ComputeAABB()
            obj_base_z = obj_aabb.pos()[2] - obj_aabb.extents()[2]

            support_aabb = on_obj.ComputeAABB()
            support_z = support_aabb.pos()[2] + support_aabb.extents()[2]

            if abs(obj_base_z - support_z) > 0.15:#lenient
                return False

            if (obj_aabb.pos()[0] > support_aabb.pos()[0] + 9.0/10*support_aabb.extents()[0]) or \
               (obj_aabb.pos()[0] < support_aabb.pos()[0] - 9.0/10*support_aabb.extents()[0]) or \
               (obj_aabb.pos()[1] > support_aabb.pos()[1] + 9.0/10*support_aabb.extents()[1]) or \
               (obj_aabb.pos()[1] < support_aabb.pos()[1] - 9.0/10*support_aabb.extents()[1]):
                return False
            return True

    @pddlpy.predicate
    def obj_negx_of_obj(objname, reference_objname):
        return False
        obj = env.GetKinBody(objname)
        reference_obj = env.GetKinBody(reference_objname)
        if obj == None or reference_obj == None:
            return False
        obj_aabb = obj.ComputeAABB()
        reference_obj_aabb = reference_obj.ComputeAABB()
        return  obj_aabb.pos()[0] < reference_obj_aabb.pos()[0] and \
                abs(obj_aabb.pos()[1] - reference_obj_aabb.pos()[1]) < min(obj.extents()[1], obj.extents()[1])

    @pddlpy.predicate
    def obj_posx_of_obj(objname, reference_objname):
        return False
        obj = env.GetKinBody(objname)
        reference_obj = env.GetKinBody(reference_objname)
        if obj == None or reference_obj == None:
            return False
        obj_aabb = obj.ComputeAABB()
        reference_obj_aabb = reference_obj.ComputeAABB()
        return  obj_aabb.pos()[0] > reference_obj_aabb.pos()[0] and \
                abs(obj_aabb.pos()[1] - reference_obj_aabb.pos()[1]) < min(obj.extents()[1], obj.extents()[1])

    @pddlpy.predicate
    def obj_negy_of_obj(objname, reference_objname):
        return False
        obj = env.GetKinBody(objname)
        reference_obj = env.GetKinBody(reference_objname)
        if obj == None or reference_obj == None:
            return False
        obj_aabb = obj.ComputeAABB()
        reference_obj_aabb = reference_obj.ComputeAABB()
        return  obj_aabb.pos()[1] < reference_obj_aabb.pos()[1] and \
                abs(obj_aabb.pos()[0] - reference_obj_aabb.pos()[0]) < min(obj.extents()[0], obj.extents()[0])

    @pddlpy.predicate
    def obj_posy_of_obj(objname, reference_objname):
        return False
        obj = env.GetKinBody(objname)
        reference_obj = env.GetKinBody(reference_objname)
        if obj == None or reference_obj == None:
            return False
        obj_aabb = obj.ComputeAABB()
        reference_obj_aabb = reference_obj.ComputeAABB()
        return  obj_aabb.pos()[1] > reference_obj_aabb.pos()[1] and \
                abs(obj_aabb.pos()[0] - reference_obj_aabb.pos()[0]) < min(obj.extents()[0], obj.extents()[0])

    @pddlpy.predicate
    def obj_is_glass(objname):
        return objname.startswith('glass')

    @pddlpy.predicate
    def obj_is_bowl(objname):
        return objname.startswith('bowl')

    @pddlpy.predicate
    def obj_is_plate(objname):
        return objname.startswith('plate')

    @pddlpy.predicate
    def obj_is_grasped(objname):
        with env:
            obj = env.GetKinBody(objname)
            if obj is None:
                return False
            if robot.IsGrabbing(obj):
                return True
            return False

    @pddlpy.predicate
    def robot_is_grasping():
        with env:
            bodies = env.GetBodies()
            for b in bodies:
                if obj_is_grasped(b.GetName()): 
                    return True
            return False

    @pddlpy.predicate
    def is_item(objname):
        return bool(obj_is_plate(objname) or obj_is_glass(objname) or obj_is_bowl(objname))

    def no_item_on_item(on_objname):
        no_item_on_objname = true()
        if not is_item(on_objname):
            return no_item_on_objname
        for objname in objects:
            if objname != on_objname:
                no_item_on_objname = no_item_on_objname & ~obj_on_obj(objname, on_objname)
        return no_item_on_objname

    @pddlpy.action
    class grasp_obj:
        @staticmethod
        def __call__(objname):
            if obj_is_glass(objname):
                return GrabGlass(objname, robot.left_arm, **common)
            elif obj_is_plate(objname):
                return SlideAndGraspPlate(objname, robot.left_arm, **common)
            elif obj_is_bowl(objname):
                return GrabBowl(objname, robot.left_arm, **common)
            else:
                raise Exception('bad objname')
        @staticmethod
        def pre(objname):
            #TODO iterate objects, add predicate for (no other obj on on_objname)
            return ~robot_is_grasping() & is_item(objname) & no_item_on_item(objname)
        @staticmethod
        def post(objname):
            return robot_is_grasping() & obj_is_grasped(objname) & ~robot_arm_home('left') & ~robot_arm_home('right')

    @pddlpy.action
    class place_obj_on_obj:
        @staticmethod
        def __call__(objname, on_objname):
            return PlaceObjOnObj(objname, on_objname, robot.left_arm, **common)
        @staticmethod
        def pre(objname, on_objname):
            return robot_is_grasping() & obj_is_grasped(objname) & no_item_on_item(on_objname)
        @staticmethod
        def post(objname, on_objname):
            return ~robot_is_grasping() & ~obj_is_grasped(objname) & obj_on_obj(objname, on_objname) & ~robot_arm_home('left') & ~robot_arm_home('right')

    @pddlpy.action
    class place_obj_negx_of_obj:
        @staticmethod
        def __call__(objname, ref_objname, ref_obj_on_objname):
            obj = env.GetKinBody(objname)
            ref_obj = env.GetKinBody(ref_objname)
            obj_aabb = obj.ComputeAABB()
            ref_obj_aabb = reference_obj.ComputeAABB()
            return PlaceObjNextToObj(objname, ref_objname, ref_obj_on_objname, 
                                    [ -ref_obj_aabb.extents()[0] - obj_aabb.extents()[0], 0,
                                      .1, min(.1, ref_obj_aabb.extents()[1]) ], robot.left_arm, **common)
        @staticmethod
        def pre(objname, ref_objname, ref_obj_on_objname):
            return robot_is_grasping() & obj_is_grasped(objname) & obj_on_obj(ref_objname, ref_obj_on_objname)
        @staticmethod
        def post(objname, ref_objname, ref_obj_on_objname):
            return ~robot_is_grasping() & ~obj_is_grasped(objname) & obj_on_obj(ref_objname, ref_obj_on_objname) & obj_on_obj(objname, ref_obj_on_objname) & obj_negx_of_obj(objname, ref_objname) & ~robot_arm_home('left') & ~robot_arm_home('right')

    @pddlpy.action
    class place_obj_posx_of_obj:
        @staticmethod
        def __call__(objname, ref_objname, ref_obj_on_objname):
            obj = env.GetKinBody(objname)
            ref_obj = env.GetKinBody(ref_objname)
            obj_aabb = obj.ComputeAABB()
            ref_obj_aabb = ref_obj.ComputeAABB()
            return PlaceObjNextToObj(objname, ref_objname, ref_obj_on_objname, 
                                    [ ref_obj_aabb.extents()[0] + obj_aabb.extents()[0], 0,
                                      .1, min(.1, ref_obj_aabb.extents()[1]) ], robot.left_arm, **common)
        @staticmethod
        def pre(objname, ref_objname, ref_obj_on_objname):
            return robot_is_grasping() & obj_is_grasped(objname) & obj_on_obj(ref_objname, ref_obj_on_objname)
        @staticmethod
        def post(objname, ref_objname, ref_obj_on_objname):
            return ~robot_is_grasping() & ~obj_is_grasped(objname) & obj_on_obj(ref_objname, ref_obj_on_objname) & obj_on_obj(objname, ref_obj_on_objname) & obj_posx_of_obj(objname, ref_objname) & ~robot_arm_home('left') & ~robot_arm_home('right')

    @pddlpy.action
    class place_obj_negy_of_obj:
        @staticmethod
        def __call__(objname, ref_objname, ref_obj_on_objname):
            obj = env.GetKinBody(objname)
            ref_obj = env.GetKinBody(ref_objname)
            obj_aabb = obj.ComputeAABB()
            ref_obj_aabb = ref_obj.ComputeAABB()
            return PlaceObjNextToObj(objname, ref_objname, ref_obj_on_objname, 
                                    [ 0, -ref_obj_aabb.extents()[1] - obj_aabb.extents()[1], 
                                      min(.1, ref_obj_aabb.extents()[0]), .1 ], robot.left_arm, **common)
        @staticmethod
        def pre(objname, ref_objname, ref_obj_on_objname):
            return robot_is_grasping() & obj_is_grasped(objname) & obj_on_obj(ref_objname, ref_obj_on_objname)
        @staticmethod
        def post(objname, ref_objname, ref_obj_on_objname):
            return ~robot_is_grasping() & ~obj_is_grasped(objname) & obj_on_obj(ref_objname, ref_obj_on_objname) & obj_on_obj(objname, ref_obj_on_objname) & obj_negy_of_obj(objname, ref_objname) & ~robot_arm_home('left') & ~robot_arm_home('right')

    @pddlpy.action
    class place_obj_posy_of_obj:
        @staticmethod
        def __call__(objname, ref_objname, ref_obj_on_objname):
            obj = env.GetKinBody(objname)
            ref_obj = env.GetKinBody(ref_objname)
            obj_aabb = obj.ComputeAABB()
            ref_obj_aabb = ref_obj.ComputeAABB()
            return PlaceObjNextToObj(objname, ref_objname, ref_obj_on_objname, 
                                    [ 0, ref_obj_aabb.extents()[1] + obj_aabb.extents()[1], 
                                      min(.1, ref_obj_aabb.extents()[0]), .1 ], robot.left_arm, **common)
        @staticmethod
        def pre(objname, ref_objname, ref_obj_on_objname):
            return robot_is_grasping() & obj_is_grasped(objname) & obj_on_obj(ref_objname, ref_obj_on_objname)
        @staticmethod
        def post(objname, ref_objname, ref_obj_on_objname):
            return ~robot_is_grasping() & ~obj_is_grasped(objname) & obj_on_obj(ref_objname, ref_obj_on_objname) & obj_on_obj(objname, ref_obj_on_objname) & obj_posy_of_obj(objname, ref_objname) & ~robot_arm_home('left') & ~robot_arm_home('right')

    @pddlpy.action
    class move_arm_home:
        @staticmethod
        def __call__(armName):
            with env:
                arm = robot.GetManipulator(armName)
            if arm == robot.left_arm:
                return robotNodes.PlanArmToConfigNode(config=left_home_config, armName=armName, **common)
            else:
                return robotNodes.PlanArmToConfigNode(config=right_home_config, armName=armName, **common)
        @staticmethod
        def pre(armName):
            return true()
        @staticmethod
        def post(armName):
            return robot_arm_home(armName)

    actions = [ grasp_obj, place_obj_on_obj, place_obj_negx_of_obj, place_obj_posx_of_obj, place_obj_negy_of_obj, place_obj_posy_of_obj, move_arm_home ]
    predicates = [ true, robot_arm_home, obj_on_obj, obj_negx_of_obj, obj_posx_of_obj, obj_negy_of_obj, obj_posy_of_obj, obj_is_glass, obj_is_bowl, obj_is_plate, obj_is_grasped, robot_is_grasping, is_item ]

    

    actionsByName = {}
    for action in actions:
        actionsByName[action.name] = action

    predicatesByName = {}
    for predicate in predicates:
        predicatesByName[predicate.name] = predicate

    return objects, actionsByName, predicatesByName

