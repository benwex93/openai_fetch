#! /usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import trajectory_msgs.msg
from std_msgs.msg import Float64
from fetch_train.srv import JointTraj, JointTrajResponse

class ExecTrajService(object):
    
    def __init__(self):

        # moveit_commander.roscpp_initialize(sys.argv)
        
        # self.robot = moveit_commander.RobotCommander()
        # self.scene = moveit_commander.PlanningSceneInterface()    
        # self.group = moveit_commander.MoveGroupCommander("arm")
        # display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory)

        # self.ee_traj_srv = rospy.Service('/ee_traj_srv', EeTraj , self.ee_traj_callback)
        # self.joint_traj_srv = rospy.Service('/joint_traj_srv', JointTraj , self.joint_traj_callback)
        # self.ee_pose_srv = rospy.Service('/ee_pose_srv', EePose , self.ee_pose_callback)
        # self.ee_rpy_srv = rospy.Service('/ee_rpy_srv', EeRpy , self.ee_rpy_callback)

        self.joint_traj_srv = rospy.Service('/joint_traj_srv', JointTraj, self.joint_traj_callback)      




        bellows_topic = '/fetch/bellows_joint_position_controller/command'
        elbow_flex_topic = '/fetch/elbow_flex_joint_position_controller/command'
        forearm_roll_topic = '/fetch/forearm_roll_joint_position_controller/command'
        l_gripper_topic = '/fetch/l_gripper_finger_joint_position_controller/command'
        r_gripper_topic = '/fetch/r_gripper_finger_joint_position_controller/command'
        upperarm_roll_topic = '/fetch/upperarm_roll_joint_position_controller/command'
        wrist_roll_topic = '/fetch/wrist_roll_joint_position_controller/command'
        

        self.bellows_sub = rospy.Publisher(bellows_topic, Float64, queue_size=10)
        self.elbow_flex_sub = rospy.Publisher(elbow_flex_topic, Float64, queue_size=10)
        self.forearm_roll_sub = rospy.Publisher(forearm_roll_topic, Float64, queue_size=10)
        self.l_gripper_sub = rospy.Publisher(l_gripper_topic, Float64, queue_size=10)
        self.r_gripper_sub = rospy.Publisher(r_gripper_topic, Float64, queue_size=10)
        self.upperarm_roll_sub = rospy.Publisher(upperarm_roll_topic, Float64, queue_size=10)
        self.wrist_roll_sub = rospy.Publisher(wrist_roll_topic, Float64, queue_size=10)

        print('Subscribed')
        # self.pose_target = geometry_msgs.msg.Pose()
        
    # def ee_traj_callback(self, request):
        
    #     self.pose_target.orientation.w = request.pose.orientation.w
    #     self.pose_target.position.x = request.pose.position.x
    #     self.pose_target.position.y = request.pose.position.y
    #     self.pose_target.position.z = request.pose.position.z
    #     self.group.set_pose_target(self.pose_target)
    #     self.execute_trajectory()
        
    #     response = EeTrajResponse()
    #     response.success = True
    #     response.message = "Everything went OK"
        
    #     return response
        
    # def joint_traj_callback(self, request):
        
    #     self.group_variable_values = self.group.get_current_joint_values()
    #     print ("Group Vars:")
    #     print (self.group_variable_values)
    #     print ("Point:")
    #     print (request.point.positions)
    #     self.group_variable_values[0] = request.point.positions[0]
    #     self.group_variable_values[1] = request.point.positions[1]
    #     self.group_variable_values[2] = request.point.positions[2]
    #     self.group_variable_values[3] = request.point.positions[3]
    #     self.group_variable_values[4] = request.point.positions[4]
    #     self.group_variable_values[5] = request.point.positions[5]
    #     self.group_variable_values[6] = request.point.positions[6]
    #     self.group.set_joint_value_target(self.group_variable_values)
    #     self.execute_trajectory()
        
    #     response = JointTrajResponse()
    #     response.success = True
    #     response.message = "Everything went OK"
        
    #     return response
        
    def joint_traj_callback(self, request):
        
        # self.group_variable_values = self.group.get_current_joint_values()
        # print ("Group Vars:")
        # print (self.group_variable_values)
        # print ("Point:")
        # print (request.point.positions)
        # self.group_variable_values[0] = request.point.positions[0]
        # self.group_variable_values[1] = request.point.positions[1]
        # self.group_variable_values[2] = request.point.positions[2]
        # self.group_variable_values[3] = request.point.positions[3]
        # self.group_variable_values[4] = request.point.positions[4]
        # self.group_variable_values[5] = request.point.positions[5]
        # self.group_variable_values[6] = request.point.positions[6]
        # self.group.set_joint_value_target(self.group_variable_values)
        # self.execute_trajectory()
        self.bellows_sub.publish(request.point.positions[0])
        self.elbow_flex_sub.publish(request.point.positions[1])
        self.forearm_roll_sub.publish(request.point.positions[2])
        self.l_gripper_sub.publish(request.point.positions[3])
        self.r_gripper_sub.publish(request.point.positions[4])
        self.upperarm_roll_sub.publish(request.point.positions[5])
        self.wrist_roll_sub.publish(request.point.positions[6])


        
        response = JointTrajResponse()
        response.success = True
        response.message = "Everything went OK"
        
        return response

    def execute_trajectory(self):
        
        self.plan = self.group.plan()
        self.group.go(wait=True)

    # def ee_pose_callback(self, request):
        
    #     gripper_pose = self.group.get_current_pose()
        
    #     gripper_pose_res = EePoseResponse()
    #     gripper_pose_res = gripper_pose.pose
        
    #     return gripper_pose_res
        
    # def ee_rpy_callback(self, request):
        
    #     gripper_rpy = self.group.get_current_rpy()
    #     gripper_rpy_res = EeRpyResponse()
    #     gripper_rpy_res.r = gripper_rpy[0]
    #     gripper_rpy_res.y = gripper_rpy[1]
    #     gripper_rpy_res.p = gripper_rpy[2]
        
    #     return gripper_rpy_res
        
if __name__ == "__main__":
    
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)
    traj_serv_object = ExecTrajService()
    rospy.spin() # mantain the service open.
