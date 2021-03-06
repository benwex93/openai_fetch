#! /usr/bin/env python

import actionlib
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint



import sys
import copy
import rospy
# import moveit_commander
# import moveit_msgs.msg
# ####
# from moveit_python import MoveGroupInterface
import geometry_msgs.msg
import trajectory_msgs.msg
from std_msgs.msg import Float64
from fetch_train.srv import JointTraj, JointFK, RandPose

from std_msgs.msg import String, Header
import moveit_commander
from moveit_msgs.srv import GetPositionFK
from moveit_msgs.msg import RobotState


from moveit_python import MoveGroupInterface

class ExecTrajService(object):
    
    def __init__(self):


        rospy.loginfo("Waiting for head_controller...")
        self.head_client = actionlib.SimpleActionClient("head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.head_client.wait_for_server()
        rospy.loginfo("...connected.")

        rospy.loginfo("Waiting for arm_controller...")
        self.arm_client = actionlib.SimpleActionClient("arm_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
        self.arm_client.wait_for_server()
        rospy.loginfo("...connected.")

        rospy.loginfo("Waiting for gripper_controller...")
        self.gripper_client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        self.gripper_client.wait_for_server()
        rospy.loginfo("...connected.")




        self.arm_joint_names = ["elbow_flex_joint", "forearm_roll_joint", "shoulder_lift_joint", \
                        "shoulder_pan_joint", "upperarm_roll_joint",\
                        "wrist_flex_joint", "wrist_roll_joint"]





        # head_tilt_topic ='/fetch/head_tilt_joint_position_controller/command'
        # self.head_tilt_pub = rospy.Publisher(head_tilt_topic, Float64, queue_size=10)


        self.head_move_srv = rospy.Service('/head_move_srv', JointTraj, self.head_move_callback)      

        self.joint_traj_srv = rospy.Service('/joint_traj_srv', JointTraj, self.joint_traj_callback)      

        self.fk_srv = rospy.Service('/fk_srv', JointFK, self.fk_callback)      

        self.rand_xyz = rospy.Service('/rand_xyz_srv', RandPose, self.rand_xyz_callback)     

        # #bellows_topic = '/fetch/bellows_joint_position_controller/command'
        # elbow_flex_topic = '/fetch/elbow_flex_joint_position_controller/command'
        # forearm_roll_topic = '/fetch/forearm_roll_joint_position_controller/command'
        # l_gripper_topic = '/fetch/l_gripper_finger_joint_position_controller/command'
        # r_gripper_topic = '/fetch/r_gripper_finger_joint_position_controller/command'
        # shoulder_lift_topic = '/fetch/shoulder_lift_joint_position_controller/command'
        # shoulder_pan_topic = '/fetch/shoulder_pan_joint_position_controller/command'
        # upperarm_roll_topic = '/fetch/upperarm_roll_joint_position_controller/command'
        # wrist_flex_topic = '/fetch/wrist_flex_joint_position_controller/command'
        # wrist_roll_topic = '/fetch/wrist_roll_joint_position_controller/command'
        
        # #MAKE QUEUE SIZE 1 SO THAT ACTIONS DON"T PILE UP AND ARE EXECUTED IMMEDIATELY FROM OBS
        # #self.bellows_pub = rospy.Publisher(bellows_topic, Float64, queue_size=10)
        # self.elbow_flex_pub = rospy.Publisher(elbow_flex_topic, Float64, queue_size=1)
        # self.forearm_roll_pub = rospy.Publisher(forearm_roll_topic, Float64, queue_size=1)
        # self.l_gripper_pub = rospy.Publisher(l_gripper_topic, Float64, queue_size=1)
        # self.r_gripper_pub = rospy.Publisher(r_gripper_topic, Float64, queue_size=1)
        # self.shoulder_lift_pub = rospy.Publisher(shoulder_lift_topic, Float64, queue_size=1)
        # self.shoulder_pan_pub = rospy.Publisher(shoulder_pan_topic, Float64, queue_size=1)
        # self.upperarm_roll_pub = rospy.Publisher(upperarm_roll_topic, Float64, queue_size=1)
        # self.wrist_flex_pub = rospy.Publisher(wrist_flex_topic, Float64, queue_size=1)
        # self.wrist_roll_pub = rospy.Publisher(wrist_roll_topic, Float64, queue_size=1)

        self.arm_group = moveit_commander.MoveGroupCommander('arm')
        # self.move_group = MoveGroupInterface("arm", "base_link")
        print('Subscribed')
    def fk_callback(self, request):


        try:
            moveit_fk = rospy.ServiceProxy('compute_fk', GetPositionFK)
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s"%e)
        fkln = ['gripper_link']
        joint_names = ["elbow_flex_joint", "forearm_roll_joint", "shoulder_lift_joint", \
                        "shoulder_pan_joint", "upperarm_roll_joint",\
                        "wrist_flex_joint", "wrist_roll_joint"]
        #joint_positions = arm_group.get_random_joint_values()
        # joint_positions = [0, 0, 0, 0, 0, 0, 0]
        joint_positions = [request.point.positions[0], request.point.positions[1], \
                    request.point.positions[4], request.point.positions[5], \
                    request.point.positions[6], request.point.positions[7], \
                    request.point.positions[8]]

        header = Header(0,rospy.Time.now(),'')
        rs = RobotState()
        rs.joint_state.name = joint_names
        rs.joint_state.position = joint_positions

        # gripper_pose = moveit_fk(header, fkln, rs) # Lookup the pose
        # print(gripper_pose)

        # response = JointTrajResponse()
        # response.success = True
        # response.message = str(gripper_pose)
        try:
            result = moveit_fk(header, fkln, rs)
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s"%e)
        #error_code = int(result.error_code)
        return [result.pose_stamped, result.fk_link_names, 0]

    def rand_xyz_callback(self, request):
        try:
            response = self.arm_group.get_random_pose()
        except rospy.ServiceException, e:
            rospy.logerror("Service call failed: %s"%e)
        return response

    def joint_traj_callback(self, request):


        # arm_intermediate_positions  = [0, 0, 0, 0, 0.0, 0, 0.0]
        # arm_intermediate_positions  = [1.32, 0, -1.4, 1.72, 0.0, 1.66, 0.0]
        # arm_joint_positions  = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

        # trajectory.points.append(JointTrajectoryPoint())
        # trajectory.points[0].positions = [0.0] * len(arm_joint_positions)
        # trajectory.points[0].velocities =  [0.0] * len(arm_joint_positions)
        # trajectory.points[0].accelerations = [0.0] * len(arm_joint_positions)
        # trajectory.points[0].time_from_start = rospy.Duration(1.0)
        # trajectory.points.append(JointTrajectoryPoint())
        # trajectory.points[1].positions = arm_intermediate_positions
        # trajectory.points[1].velocities =  [0.0] * len(arm_joint_positions)
        # trajectory.points[1].accelerations = [0.0] * len(arm_joint_positions)
        # trajectory.points[1].time_from_start = rospy.Duration(4.0)




        ###########################
        arm_joint_positions  = [request.point.positions[0], request.point.positions[1], \
                    request.point.positions[4], request.point.positions[5], \
                    request.point.positions[6], request.point.positions[7], \
                    request.point.positions[8]]

        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = arm_joint_positions
        trajectory.points[0].velocities =  [0.0] * len(arm_joint_positions)
        trajectory.points[0].accelerations = [0.0] * len(arm_joint_positions)
        trajectory.points[0].time_from_start = rospy.Duration(0.0)


        arm_goal = FollowJointTrajectoryGoal()
        arm_goal.trajectory = trajectory
        #arm_goal.goal_time_tolerance = rospy.Duration(1.0)

        # gripper_goal = GripperCommandGoal()
        # gripper_goal.command.max_effort = 10.0
        # gripper_goal.command.position = 0.1

        #rospy.loginfo("Setting positions...")
        self.arm_client.send_goal(arm_goal)
        self.arm_client.wait_for_result(rospy.Duration(10.0))
        # gripper_client.send_goal(gripper_goal)
        # gripper_client.wait_for_result(rospy.Duration(10.0))
        #rospy.loginfo("...done")


        ##########################







        # #self.bellows_pub.publish(request.point.positions[0])
        # self.elbow_flex_pub.publish(request.point.positions[0])
        # self.forearm_roll_pub.publish(request.point.positions[1])
        # self.l_gripper_pub.publish(request.point.positions[2])
        # self.r_gripper_pub.publish(request.point.positions[3])
        # self.shoulder_lift_pub.publish(request.point.positions[4])
        # self.shoulder_pan_pub.publish(request.point.positions[5])
        # self.upperarm_roll_pub.publish(request.point.positions[6])
        # self.wrist_flex_pub.publish(request.point.positions[7])
        # self.wrist_roll_pub.publish(request.point.positions[8])
        
        #rospy.sleep(1)
        # response = JointTrajResponse()
        # response.success = True
        # response.message = "Everything went OK"

        # arm_joint_positions  = [request.point.positions[0], request.point.positions[1], \
        #     request.point.positions[4], request.point.positions[5], \
        #     request.point.positions[6], request.point.positions[7], \
        #     request.point.positions[8]]

        # joint_names = ["elbow_flex_joint", "forearm_roll_joint", "shoulder_lift_joint", \
        #                 "shoulder_pan_joint", "upperarm_roll_joint",\
        #                 "wrist_flex_joint", "wrist_roll_joint"]

        # result = self.move_group.moveToJointPosition(joint_names, arm_joint_positions, 0.02)
        # # if result.error_code.val != MoveItErrorCodes.SUCCESS:
        #     # return [False, "Everythings gone to hell"]








        
        return [True, "Everything went OK"]
    def head_move_callback(self, request):
        #self.head_tilt_pub.publish(request.point.positions[0])
        
        #rospy.sleep(1)
        # response = JointTrajResponse()
        # response.success = True
        # response.message = "Everything went OK"
        
        # return response
        return [True, "Everything went OK"]


        # trajectory = JointTrajectory()
        # trajectory.joint_names = head_joint_names
        # trajectory.points.append(JointTrajectoryPoint())
        # trajectory.points[0].positions = head_joint_positions
        # trajectory.points[0].velocities = [0.0] * len(head_joint_positions)
        # trajectory.points[0].accelerations = [0.0] * len(head_joint_positions)
        # trajectory.points[0].time_from_start = rospy.Duration(5.0)

        # head_joint_names = ["head_pan_joint", "head_tilt_joint"]
        # head_joint_positions = [0.0, 0.0]
        # head_goal = FollowJointTrajectoryGoal()
        # head_goal.trajectory = trajectory
        # head_goal.goal_time_tolerance = rospy.Duration(0.0)
        # head_client.send_goal(head_goal)
        # head_client.wait_for_result(rospy.Duration(6.0))
        
if __name__ == "__main__":
    
    rospy.init_node('execute_trajectories')
    traj_serv_object = ExecTrajService()
    rospy.spin() # mantain the service open.