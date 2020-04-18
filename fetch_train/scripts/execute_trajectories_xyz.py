#!/usr/bin/python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from fetch_train.srv import JointTraj, JointTrajResponse
from moveit_python import MoveGroupInterface
import actionlib
from control_msgs.msg import (FollowJointTrajectoryAction,
                              FollowJointTrajectoryGoal,
                              GripperCommandAction,
                              GripperCommandGoal)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes


class ExecTrajService(object):
    
    def __init__(self):

        self.move_group = MoveGroupInterface("arm", "base_link")

        # pose_target = group.get_current_pose().pose
        # print("Current Pose=="+str(pose_target))

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

        self.head_move_srv = rospy.Service('/arm_move_srv', JointTraj, self.arm_move_xyz)      
        self.joint_traj_srv = rospy.Service('/joint_traj_srv', JointTraj, self.joint_traj_callback)      


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

        print('Subscribed')

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
        # self.arm_client.send_goal(arm_goal)
        # self.arm_client.wait_for_result(rospy.Duration(1.0))
        result = self.move_group.moveToJointPosition(self.arm_joint_names, arm_joint_positions, 0.02)
        # if result.error_code.val == MoveItErrorCodes.SUCCESS:
        #     pass
                # gripper_client.send_goal(gripper_goal)
        # gripper_client.wait_for_result(rospy.Duration(5.0))
        #rospy.loginfo("...done")

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
        response = JointTrajResponse()
        response.success = True
        response.message = "Everything went OK"
        
        return response
    def arm_move_xyz(self, request):
        pose_target = geometry_msgs.msg.Pose()

        pose_target.orientation.x = 0
        pose_target.orientation.y = 0
        pose_target.orientation.z = 0
        pose_target.orientation.w = 0.99
        pose_target.position.x = request.point.positions[0]
        pose_target.position.y = request.point.positions[1]
        pose_target.position.z = request.point.positions[2]

        self.group.set_pose_target(pose_target)
        print("Executing..START>"+str(pose_target.position))
        plan1 = self.group.plan()
        self.group.go(wait=True)
        print("Executing..DONE>"+str(pose_target.position))

        response = JointTrajResponse()
        response.success = True
        response.message = "Everything went OK"
        
        return response
    def head_move_callback(self, request):
        #self.head_tilt_pub.publish(request.point.positions[0])
        
        #rospy.sleep(1)
        response = JointTrajResponse()
        response.success = True
        response.message = "Everything went OK"
        
        return response


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