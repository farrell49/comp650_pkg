#!/usr/bin/env python

import sys
import rospy
from comp650_pkg.srv import *
from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import moveit_commander
from moveit_msgs.msg import PlanningSceneComponents, CollisionObject, PlanningScene
from moveit_msgs.srv import GetPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import rosparam
import rospkg
from yaml import load
from rospy_message_converter import message_converter


def request_plan():
    try:
        plan_request = rospy.ServiceProxy('planning_server', PlanningRequest)

        REQUEST_TYPE = "plan_gripper"
        TRAJECTORY = JointTrajectory()

        response = plan_request(REQUEST_TYPE, TRAJECTORY)

        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    return False

def request_show():
    try:
        plan_request = rospy.ServiceProxy('planning_server', PlanningRequest)

        REQUEST_TYPE = "show_gripper"
        #TRAJECTORY = JointTrajectory()
        f = open(rospack.get_path("comp650_pkg")+"/traj_gripper.yaml")
        TRAJECTORY = load(f)
        f.close()
        rospy.loginfo("traj_yaml = %s", TRAJECTORY)
        TRAJECTORY = message_converter.convert_dictionary_to_ros_message('trajectory_msgs/JointTrajectory', TRAJECTORY)
        response = plan_request(REQUEST_TYPE, TRAJECTORY)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    return False

def request_execute():
    try:
        plan_request = rospy.ServiceProxy('planning_server', PlanningRequest)

        REQUEST_TYPE = "execute_gripper"
        f = open(rospack.get_path("comp650_pkg")+"/traj.yaml")
        TRAJECTORY = load(f)
        f.close()
        rospy.loginfo("traj_yaml = %s", TRAJECTORY)
        TRAJECTORY = message_converter.convert_dictionary_to_ros_message('trajectory_msgs/JointTrajectory', TRAJECTORY)

        rospy.loginfo("Requesting Execute")
        response = plan_request(REQUEST_TYPE, TRAJECTORY)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    return True
'''
def request_pose():
    try:
        plan_request = rospy.ServiceProxy('planning_server', PlanningRequest)

        REQUEST_TYPE = "pose_gripper"
        TRAJECTORY = JointTrajectory()
        rospy.loginfo('requesting pose')
        resp = plan_request(REQUEST_TYPE, TRAJECTORY)
        rospy.loginfo('pose = {}'.format(resp))

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    return True
'''
def request_joint_states():
    try:
        plan_request = rospy.ServiceProxy('planning_server', PlanningRequest)

        REQUEST_TYPE = "joint_states_gripper"
        TRAJECTORY = JointTrajectory()
        rospy.loginfo('requesting pose')
        resp = plan_request(REQUEST_TYPE, TRAJECTORY)
        rospy.loginfo('pose = {}'.format(resp))

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    return True

def request_planning_scene():
    try:
        scene_request = rospy.ServiceProxy('get_planning_scene', GetPlanningScene)
        req = PlanningSceneComponents()
        #build using the bit operators found in the PlanningSceneComponents msg
        req.components = req.WORLD_OBJECT_NAMES + req.WORLD_OBJECT_GEOMETRY

        resp = scene_request(req)
        rospy.loginfo('request_planning_scene() -- resp = {}'.format(resp))

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    return True

def add_object():
    moveit_commander.roscpp_initialize(sys.argv)  #initialize movit commander
    robot = moveit_commander.RobotCommander()   #create robotcommander instance
    scene = moveit_commander.PlanningSceneInterface()   #setup for planning scene

    scene_pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

    plan_scene = PlanningScene()

    box1 = CollisionObject()
    box1.header = Header()
    box1.id = 'box1'

    solid_primitive = SolidPrimitive()
    solid_primitive.type = solid_primitive.BOX
    solid_primitive.dimensions = [2, 2, 2]

    pose = PoseStamped()
    pose.header = Header()
    pose.pose.position.x = 1
    pose.pose.position.y = 1
    pose.pose.position.z = 1
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 0

    box1.primitives = solid_primitive
    box1.primitive_poses = pose
    box1.operation = box1.ADD

    plan_scene.world.collision_objects = box1

    scene.add_box('box1', pose, (10,10))

    return True


if __name__ == "__main__":
    rospy.init_node("planning_service_client_test")

    rospy.loginfo("planning_service_client_test()")

    rospy.loginfo("-------------------")
    rospy.loginfo("Testing a plan call")

    rospack = rospkg.RosPack()
    if add_object():
        rospy.loginfo("Plan call success")
    else:
        rospy.loginfo("Plan call FAIL")
