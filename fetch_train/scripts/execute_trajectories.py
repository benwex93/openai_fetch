#! /usr/bin/env python

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
from fetch_train.srv import JointTraj, JointTrajResponse

class ExecTrajService(object):
    
    def __init__(self):

        # moveit_commander.roscpp_initialize(sys.argv)
        # self.move_group = MoveGroupInterface("arm", "base_link")

        head_tilt_topic ='/fetch/head_tilt_joint_position_controller/command'
        self.head_tilt_pub = rospy.Publisher(head_tilt_topic, Float64, queue_size=10)


        self.head_move_srv = rospy.Service('/head_move_srv', JointTraj, self.head_move_callback)      

        self.joint_traj_srv = rospy.Service('/joint_traj_srv', JointTraj, self.joint_traj_callback)      


        #bellows_topic = '/fetch/bellows_joint_position_controller/command'
        elbow_flex_topic = '/fetch/elbow_flex_joint_position_controller/command'
        forearm_roll_topic = '/fetch/forearm_roll_joint_position_controller/command'
        l_gripper_topic = '/fetch/l_gripper_finger_joint_position_controller/command'
        r_gripper_topic = '/fetch/r_gripper_finger_joint_position_controller/command'
        shoulder_lift_topic = '/fetch/shoulder_lift_joint_position_controller/command'
        shoulder_pan_topic = '/fetch/shoulder_pan_joint_position_controller/command'
        upperarm_roll_topic = '/fetch/upperarm_roll_joint_position_controller/command'
        wrist_flex_topic = '/fetch/wrist_flex_joint_position_controller/command'
        wrist_roll_topic = '/fetch/wrist_roll_joint_position_controller/command'
        
        #MAKE QUEUE SIZE 1 SO THAT ACTIONS DON"T PILE UP AND ARE EXECUTED IMMEDIATELY FROM OBS
        #self.bellows_pub = rospy.Publisher(bellows_topic, Float64, queue_size=10)
        self.elbow_flex_pub = rospy.Publisher(elbow_flex_topic, Float64, queue_size=1)
        self.forearm_roll_pub = rospy.Publisher(forearm_roll_topic, Float64, queue_size=1)
        self.l_gripper_pub = rospy.Publisher(l_gripper_topic, Float64, queue_size=1)
        self.r_gripper_pub = rospy.Publisher(r_gripper_topic, Float64, queue_size=1)
        self.shoulder_lift_pub = rospy.Publisher(shoulder_lift_topic, Float64, queue_size=1)
        self.shoulder_pan_pub = rospy.Publisher(shoulder_pan_topic, Float64, queue_size=1)
        self.upperarm_roll_pub = rospy.Publisher(upperarm_roll_topic, Float64, queue_size=1)
        self.wrist_flex_pub = rospy.Publisher(wrist_flex_topic, Float64, queue_size=1)
        self.wrist_roll_pub = rospy.Publisher(wrist_roll_topic, Float64, queue_size=1)

        print('Subscribed')

        #self.tuck()
        #rospy.sleep(10)

    # def tuck_arm_callback(self, request):

    #     joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
    #               "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

    #     pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]

    #     self.shoulder_pan_pub.publish(pose[0])
    #     self.shoulder_lift_pub.publish(pose[1])
    #     self.upperarm_roll_pub.publish(pose[2])
    #     self.elbow_flex_pub.publish(pose[3])
    #     self.forearm_roll_pub.publish(pose[4])
    #     self.wrist_flex_pub.publish(pose[5])
    #     self.wrist_roll_pub.publish(pose[6])
        
    #     rospy.sleep(5)
    #     response = JointTrajResponse()
    #     response.success = True
    #     response.message = "Everything went OK"
        
    #     return response
    def joint_traj_callback(self, request):
        

        # joints = ["elbow_flex_joint", "forearm_roll_joint", "shoulder_lift_joint",
        #           "shoulder_pan_joint", "upperarm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        # print(request.point.positions)
        # pose = [
        #         request.point.positions[1], \
        #         request.point.positions[2], \
        #         request.point.positions[3], \
        #         request.point.positions[4], \
        #         request.point.positions[5], \
        #         request.point.positions[6],  \
        #         request.point.positions[7]  \
        #         ]
        # #while not rospy.is_shutdown():
        # result = self.move_group.moveToJointPosition(joints, pose, 0.02)



        #self.bellows_pub.publish(request.point.positions[0])
        self.elbow_flex_pub.publish(request.point.positions[0])
        self.forearm_roll_pub.publish(request.point.positions[1])
        self.l_gripper_pub.publish(request.point.positions[2])
        self.r_gripper_pub.publish(request.point.positions[3])
        self.shoulder_lift_pub.publish(request.point.positions[4])
        self.shoulder_pan_pub.publish(request.point.positions[5])
        self.upperarm_roll_pub.publish(request.point.positions[6])
        self.wrist_flex_pub.publish(request.point.positions[7])
        self.wrist_roll_pub.publish(request.point.positions[8])
        
        #rospy.sleep(1)
        response = JointTrajResponse()
        response.success = True
        response.message = "Everything went OK"
        
        return response
    def head_move_callback(self, request):
        self.head_tilt_pub.publish(request.point.positions[0])
        
        #rospy.sleep(1)
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
