#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String, Header
from trajectory_msgs.msg import JointTrajectory, MultiDOFJointTrajectory


from comp650_pkg.srv import *

class PlanningService(object):

    def handle_planning_service_request(self, req):

        if "plan" in req.REQUEST_TYPE:
            rospy.loginfo("PlanningServer::handle_planning_service_request -- plan requested")
            plan_resp = self.handle_plan_request(req)
            return plan_resp, None
        elif "show" in req.REQUEST_TYPE:
            rospy.loginfo("PlanningServer::handle_planning_service_request -- show requested")
            self.handle_show_request(req)
            return True, None
        elif "execute" in req.REQUEST_TYPE:
            rospy.loginfo("PlanningServer::handle_planning_service_request -- execute requested")
            self.handle_execute_request(req)
            return True, None
        elif "joint_states" in req.REQUEST_TYPE:
            rospy.loginfo("PlanningServer::handle_planning_service_request -- position requested")
            joint_states = self.handle_joint_state_request(req)
            rospy.loginfo('pose returned = {}'.format(joint_states))
            return (True, joint_states)
        else:
            rospy.logwarn("A service request has been made with an unrecognized request type")
            return False

        return True

    def handle_plan_request(self, req):
        #set group
        if "manipulator" in req.REQUEST_TYPE:
            self.group = moveit_commander.MoveGroupCommander("manipulator")
        elif "gripper" in req.REQUEST_TYPE:
            self.group = moveit_commander.MoveGroupCommander("gripper")
        else:
            rospy.logwarn("PlanningService::handle_plan_request() -- requested group is not recognized")
            return False
        self.group.set_planner_id("RRTConnectkConfigDefault")

        self.group.set_random_target()

        plan_resp = self.group.plan()
        rospy.loginfo("plan_resp.joint_trajectory.joint_names = %s",plan_resp.joint_trajectory.joint_names)
        if plan_resp.joint_trajectory.joint_names:
            rospy.loginfo("PlanningService::handle_plan_request() -- plan successful")
            rospy.loginfo("Plan = %s", plan_resp)
            return True
        else:
            rospy.loginfo("PlanningService::handle_plan_request() -- plan FAIL")
            return False

    def handle_show_request(self, req):
        display_trajectory = DisplayTrajectory()

        display_trajectory.trajectory_start = self.robot.get_current_state()
        #build moveit msg for display
        joint_multi_traj_msg = MultiDOFJointTrajectory()
        robot_traj_msg = RobotTrajectory()
        robot_traj_msg.joint_trajectory = req.JOINT_TRAJECTORY
        robot_traj_msg.multi_dof_joint_trajectory = joint_multi_traj_msg

        display_trajectory.trajectory.append(robot_traj_msg)
        rospy.loginfo("PlanningService::handle_show_request() -- showing trajectory %s", req.JOINT_TRAJECTORY)

        self.display_trajectory_publisher.publish(display_trajectory);
        return True

    def handle_execute_request(self, req):
        if "manipulator" in req.REQUEST_TYPE:
            self.group = moveit_commander.MoveGroupCommander("manipulator")
        elif "gripper" in req.REQUEST_TYPE:
            self.group = moveit_commander.MoveGroupCommander("gripper")
        else:
            rospy.logwarn("PlanningService::handle_plan_request() -- requested group is not recognized")
            return False

        #build moveit msg for display
        joint_multi_traj_msg = MultiDOFJointTrajectory()
        robot_traj_msg = RobotTrajectory()
        robot_traj_msg.joint_trajectory = req.JOINT_TRAJECTORY
        robot_traj_msg.multi_dof_joint_trajectory = joint_multi_traj_msg

        self.group.execute(robot_traj_msg, wait=True)

        if "gripper" in req.REQUEST_TYPE:
            req.REQUEST_TYPE = 'gripper'
            req.JOINT_TRAJECTORY = JointTrajectory()
            joint_states = self.handle_joint_state_request(req)
            rospy.loginfo('gripper pos = {}'.format(joint_states[3]))
            if joint_states[3] > 0.5:
                rospy.logwarn('attempting to attach box to gripper')
                self.attach_box1()
            else:
                rospy.logwarn('releasing box')
                self.release_box1()

        return True

    def handle_joint_state_request(self, req):
        if 'manipulator' in req.REQUEST_TYPE:
            self.group = moveit_commander.MoveGroupCommander('manipulator')
            joint_states = self.group.get_current_joint_values()
            rospy.loginfo('joint_states manipulator= {}'.format(joint_states))
            rospy.loginfo('type(joint_states) = {}'.format(type(joint_states)))
            return joint_states
        elif 'gripper' in req.REQUEST_TYPE:
            self.group = moveit_commander.MoveGroupCommander('gripper')
            joint_states = self.group.get_current_joint_values()
            rospy.loginfo('joint_states gripper = {}'.format(joint_states))
            return joint_states
        else:
            rospy.logwarn('group type not recognized')
            return False

    def add_box(self):
        rospy.sleep(1)

        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = self.robot.get_planning_frame()
        pose.pose.position.x = 0.349
        pose.pose.position.y = -0.233
        pose.pose.position.z = .970
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1

        self.scene.add_box('box1', pose, size=(0.07,0.07,0.07))
        return True

    def attach_box1(self):
        self.scene.attach_box('robotiq_85_base_link', 'box1')

        return True
    def release_box1(self):
        self.scene.remove_attached_object('robotiq_85_base_link', 'box1')

        return True

    '''Don't need right now actually, but I'll just leave this right here...
    def handle_pose_request(self, req):
        pose = PoseStamped()
        if 'manipulator' in req.REQUEST_TYPE:
            self.group = moveit_commander.MoveGroupCommander('manipulator')
            pose = self.group.get_current_pose()
            rospy.loginfo('pose = {}'.format(pose))
            rospy.loginfo('type(pose) = {}'.format(type(pose)))
            return pose
        elif 'gripper' in req.REQUEST_TYPE:
            self.group = moveit_commander.MoveGroupCommander('gripper')
            pose = self.group.get_current_pose()
            rospy.loginfo('pose = {}'.format(pose))
            return pose
        else:
            return False
    '''

    def planning_service(self):
        self.s = rospy.Service('planning_server', PlanningRequest, self.handle_planning_service_request)

        moveit_commander.roscpp_initialize(sys.argv)  #initialize movit commander
        self.robot = moveit_commander.RobotCommander()   #create robotcommander instance
        self.scene = moveit_commander.PlanningSceneInterface()   #setup for planning scene

        self.add_box()

        rospy.loginfo('planning_scene = {}'.format(self.scene))
        #create publisher to visualize later
        self.display_trajectory_publisher = rospy.Publisher(
                                          '/move_group/display_planned_path',
                                          DisplayTrajectory,queue_size=10)

        rospy.loginfo("planning_service -- Ready to recieve planning related requests")
        rospy.spin()
        self.release_box1()


if __name__=='__main__':
  rospy.init_node('planning_server')
  planning_service = PlanningService()
  planning_service.planning_service()
