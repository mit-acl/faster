#!/usr/bin/env python
import roslib
import rospy
import math
from acl_msgs.msg import QuadGoal, State
from gazebo_msgs.msg import ModelState
import numpy as np
from numpy import linalg as LA

from tf.transformations import quaternion_from_euler, euler_from_quaternion

from pyquaternion import Quaternion
import tf



class FakeSim:

    def __init__(self):
        self.state=State()

        self.state.quat.x = 0
        self.state.quat.y = 0
        self.state.quat.z = 0
        self.state.quat.w = 1

        self.pubGazeboState = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
        self.pubState = rospy.Publisher('state', State, queue_size=1, latch=True)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.pubTF)
        name = rospy.get_namespace()
        self.name = name[1:-1]
        
        rospy.sleep(1.0)

        self.state.header.frame_id="world"
        self.pubState.publish(self.state)  



    def goalCB(self, data):
        state = State()
        gazebo_state = ModelState()
        gazebo_state.model_name = self.name
        axis_z=[0,0,1]

        accel=[data.accel.x, data.accel.y, data.accel.z + 9.81];

        gazebo_state.pose.position.x = data.pos.x
        gazebo_state.pose.position.y = data.pos.y
        gazebo_state.pose.position.z = data.pos.z


        drone_quaternion_with_yaw=[];

        if(LA.norm(accel)>0.001 and LA.norm(np.cross(accel, axis_z))>0.0001):
          norm_accel=LA.norm(accel)
          accel=accel/norm_accel
          axis=np.cross(accel, axis_z);

          dot=np.dot(accel,axis_z)
          angle=math.acos(dot)        
          drone_quaternion = Quaternion(axis=axis, angle=-angle)

          #Use the yaw from goal
          euler =euler_from_quaternion((drone_quaternion[1], drone_quaternion[2], drone_quaternion[3], drone_quaternion[0]), 'szyx')
          yaw=euler[0]
          pitch=euler[1]
          roll=euler[2]
          drone_quaternion_with_yaw = quaternion_from_euler(data.yaw, pitch, roll, 'szyx')

        else: #Take only the yaw angle
          drone_quaternion_with_yaw = quaternion_from_euler(data.yaw, 0, 0, 'szyx')

        gazebo_state.pose.orientation.x = drone_quaternion_with_yaw[0]
        gazebo_state.pose.orientation.y = drone_quaternion_with_yaw[1]
        gazebo_state.pose.orientation.z = drone_quaternion_with_yaw[2]
        gazebo_state.pose.orientation.w = drone_quaternion_with_yaw[3]



        #self.gazebo_state.twist = data.twist
        gazebo_state.reference_frame = "world" 
        self.pubGazeboState.publish(gazebo_state)  


        self.state.header.frame_id="world"
        self.state.pos=data.pos
        self.state.vel=data.vel
        self.state.quat=gazebo_state.pose.orientation
        self.pubState.publish(self.state)  

    def pubTF(self, timer):
        br = tf.TransformBroadcaster()
        br.sendTransform((self.state.pos.x, self.state.pos.y, self.state.pos.z),
                         (self.state.quat.x, self.state.quat.y, self.state.quat.z, self.state.quat.w),
                         rospy.Time.now(),
                         self.name,
                         "vicon")


             

def startNode():
    c = FakeSim()
    rospy.Subscriber("goal", QuadGoal, c.goalCB)

    rospy.spin()

if __name__ == '__main__':

    ns = rospy.get_namespace()
    try:
        rospy.init_node('relay')
        if str(ns) == '/':
            rospy.logfatal("Need to specify namespace as vehicle name.")
            rospy.logfatal("This is tyipcally accomplished in a launch file.")
            rospy.logfatal("Command line: ROS_NAMESPACE=mQ01 $ rosrun quad_control joy.py")
        else:
            print "Starting joystick teleop node for: " + ns
            startNode()
    except rospy.ROSInterruptException:
        pass
