#!/usr/bin/env python

import sys
import rospy
from comp650_pkg.srv import *
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import moveit_commander
import moveit_msgs.msg
import rosparam
import rospkg
from yaml import load
from rospy_message_converter import message_converter

class PlanningServiceClient(object):

    def read_tmkit_plan(self):
        plan_list = []
        traj_temp = None
        gripper_current_states = self.request_joint_states('gripper')
        self.gripper_state_initial = gripper_current_states[3]

        with open(rospack.get_path("comp650_pkg") + '/plans/ur5-sussman-working.tmp') as tmp:
            for line in tmp:
                line_temp = line.split()
                #rospy.loginfo('line_temp = {}'.format(line_temp))
                if line_temp[0] == 'a': #line that defines the action, the start to a motion plan
                    #rospy.loginfo('found a line with a')
                    traj_temp = JointTrajectory()
                    traj_temp.header = Header()
                    traj_temp.header.frame_id = "/world"
                    traj_temp.points = []
                    #rospy.loginfo('traj_temp = {}'.format(traj_temp))

                elif line_temp[0] == 'm' and traj_temp is not None:   #m defines the joint list, make sure we hit an action first
                    joint_names_ = []
                    for joint in range(1, len(line_temp)):
                        joint_names_.append(line_temp[joint])
                    traj_temp.joint_names = joint_names_
                    time = -1
                    #rospy.loginfo('traj_temp.joint_names = {}'.format(joint_names_))

                elif line_temp[0] == 'p' and traj_temp is not None: #p defines points in trajectory, make sure we hit an action first
                    positions_ = []
                    time = time + 1;
                    for pos in range(1, len(line_temp)):
                        positions_.append(float(line_temp[pos]))
                    point_ = JointTrajectoryPoint()
                    point_.positions = positions_
                    point_.time_from_start.secs = time
                    traj_temp.points.append(point_)

                elif line_temp[0] == 'r' and traj_temp is not None: #r defines gripper action and end of trajectory points
                    plan_list.append(traj_temp)
                    #rospy.loginfo('PlanningServiceClient::read_tmkit_plan() -- added trajectory to list {}'.format(traj_temp))
                    traj_temp = None
                    time = -1

                    #build gripper trajectory command
                    f = open(rospack.get_path("comp650_pkg")+"/config/ur5_commands.yaml")
                    commands = load(f)
                    f.close()
                    for cmd in commands:
                        if line_temp[-1] == cmd:
                            gripper_command = commands.get(cmd)
                            #rospy.loginfo('gripper_command = {}'.format(gripper_command))
                        #else:
                            #rospy.loginfo('gripper_command does not match')
                        rospy.loginfo("command = {}".format(cmd))
                    if gripper_command == 'ee_open':
                        ee_traj = self.create_ee_traj('open')
                    elif gripper_command == 'ee_close':
                        ee_traj = self.create_ee_traj('close')
                    else:
                        rospy.logwarn('PlanningServiceClient::read_tmkit_plan() -- unrecognized ee command')

                    plan_list.append(ee_traj)

            rospy.loginfo('PlanningServiceClient::read_tmkit_plan() -- trajectory list {}'.format(plan_list))


                    #traj_temp.joint_names =

                #rospy.loginfo( "\t".join(line.split()) + "\n" )

        return plan_list

    def create_ee_traj(self, state):
        gripper_traj = JointTrajectory()
        gripper_current_states = self.request_joint_states('gripper')
        gripper_state = gripper_current_states[3]   #we only care about the 3rd joint which is the active joint

        gripper_traj.header = Header()
        gripper_traj.header.frame_id = '/world'
        gripper_traj.joint_names = ['robotiq_85_left_knuckle_joint']
        point_ = JointTrajectoryPoint()
        gripper_traj.points.append(point_)
        if self.gripper_state_initial is not None: #if this is the first operation
            gripper_traj.points[0].positions = [self.gripper_state_initial]
            self.gripper_state_initial = None
        elif self.gripper_state_current is not None:
            gripper_traj.points[0].positions = self.gripper_state_current
        else:
            rospy.logwarn('PlanningServiceClient::create_ee_traj() -- Damn dude, not even sure how you did that')

        point_ = JointTrajectoryPoint()
        if 'open' in state:    # add open point to trajectory
            point_.positions = [0.0]
            self.gripper_state_current = [0.0]
        elif 'close' in state:  #add closed point to trajectory
            point_.positions = [0.803]
            self.gripper_state_current = [0.803]
        else:
            rospy.logwarn('PlanningServiceClient::create_ee_traj() -- incorrect state sent')
            return False
        point_.time_from_start.secs = 2 # how long it will take to actuate

        gripper_traj.points.append(point_)

        rospy.loginfo('PlanningServiceClient::create_ee_traj() -- gripper traj = {}'.format(gripper_traj))
        return gripper_traj

    def show_tmkit_plan(self):
        plan_list = self.read_tmkit_plan()
        rospy.loginfo('PlanningServiceClient::execute_tmkit_plan() -- executing tmkit plan provided')

        for trajectory in plan_list:
            rospy.loginfo('type(trajectory) = {}'.format(type(trajectory)))
            rospy.loginfo('PlanningServiceClient::execute_tmkit_plan() -- executing plan {}'.format(trajectory))
            self.request_show(trajectory)
            rospy.sleep(2)

        return True

    def execute_tmkit_plan(self):
        plan_list = self.read_tmkit_plan()
        rospy.loginfo('PlanningServiceClient::execute_tmkit_plan() -- executing tmkit plan provided')

        for trajectory in plan_list:
            rospy.loginfo('type(trajectory) = {}'.format(type(trajectory)))
            rospy.loginfo('PlanningServiceClient::execute_tmkit_plan() -- executing plan {}'.format(trajectory))
            self.request_execute(trajectory)
            rospy.sleep(2)

        return True

    def request_joint_states(self, group):
        try:
            plan_request = rospy.ServiceProxy('planning_server', PlanningRequest)

            if 'gripper' in group:
                REQUEST_TYPE = "joint_states_gripper"
            elif 'manipulator' in group:
                REQUEST_TYPE = "joint_states_manipulator"
            else:
                rospy.logwarn('PlanningServiceClient::request_joint_states() -- incorrect group requested')
                return False

            TRAJECTORY = JointTrajectory()
            rospy.loginfo('requesting pose')
            resp = plan_request(REQUEST_TYPE, TRAJECTORY)
            rospy.loginfo('pose = {}'.format(resp))
            return resp.joint_states

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return True

    def request_plan(self):
        try:
            plan_request = rospy.ServiceProxy('planning_server', PlanningRequest)

            REQUEST_TYPE = "plan_manipulator"
            TRAJECTORY = JointTrajectory()

            response = plan_request(REQUEST_TYPE, TRAJECTORY)

            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return False

    def request_show(self, trajectory):
        try:
            plan_request = rospy.ServiceProxy('planning_server', PlanningRequest)
            if any('robotiq_85' in s for s in trajectory.joint_names):  #means its a gripper trajectory
                REQUEST_TYPE = "show_gripper"
                rospy.loginfo("gripper execute requested")
            else:
                REQUEST_TYPE = "show_manipulator"

            TRAJECTORY = trajectory
            response = plan_request(REQUEST_TYPE, TRAJECTORY)
            return response
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return False

    def request_execute(self, trajectory):
        try:
            plan_request = rospy.ServiceProxy('planning_server', PlanningRequest)
            if any('robotiq_85' in s for s in trajectory.joint_names):  #means its a gripper trajectory
                REQUEST_TYPE = "execute_gripper"
                rospy.loginfo("gripper execute requested")
            else:
                REQUEST_TYPE = "execute_manipulator"

            TRAJECTORY = trajectory

            rospy.loginfo("Requesting Execute")
            response = plan_request(REQUEST_TYPE, TRAJECTORY)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        return True

if __name__ == "__main__":
    rospy.init_node("planning_service_client_test")

    rospy.loginfo("planning_service_client() Initialized")

    ps_client = PlanningServiceClient()

    rospack = rospkg.RosPack()
    if ps_client.execute_tmkit_plan():
        rospy.loginfo("function call success")
    else:
        rospy.loginfo("function call FAIL")
