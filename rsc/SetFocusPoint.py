#!/usr/bin/env python

'''
Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Heller Florian, Meissner Pascal, Schleicher Ralf, Stoeckle Patrick, Stroh Daniel, Trautmann Jeremias, Walter Milena, Wittenbeck Valerij
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

4. The use is explicitly not permitted to any application which deliberately try to kill or do harm to any living creature.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import rospy
from geometry_msgs.msg import Point, PointStamped, Quaternion
from actionlib import *
from actionlib.msg import *
import numpy
from time import time
from ptu_controller.msg import PTUMovementGoal, PTUMovementAction
from next_best_view.srv import GetPose, CalculateCameraPoseCorrection, CalculateCameraPose, TriggerFrustumVisualization
from next_best_view.msg import RobotStateMessage
from sensor_msgs.msg import JointState
import tf
import tf2_ros

ptu = None

def get_robot_pose_cpp():
    rospy.wait_for_service('/nbv_robot_model/GetRobotPose', timeout=5)
    pose = rospy.ServiceProxy('/nbv_robot_model/GetRobotPose',GetPose)
    return pose().pose

def get_camera_pose_cpp():
    rospy.wait_for_service('/nbv_robot_model/GetCameraPose', timeout=5)
    pose = rospy.ServiceProxy('/nbv_robot_model/GetCameraPose',GetPose)
    return pose().pose

def get_camera_frustum(camera_pose):
    try:
        rospy.wait_for_service('/nbv/trigger_frustum_visualization', timeout=5)
        get_frustum = rospy.ServiceProxy(
            '/nbv/trigger_frustum_visualization', TriggerFrustumVisualization)
        get_frustum(camera_pose)
    except (rospy.ServiceException, rospy.exceptions.ROSException), e:
        rospy.logwarn(str(e))

def ptu_callback(data):
    global ptu
    ptu = data.position

def get_robot_state():
    global ptu

    sub_ptu = rospy.Subscriber('/asr_flir_ptu_driver/state', JointState, ptu_callback)

    future = time() + 2
    while not ptu and time() < future:
        pass

    rospy.sleep(2)
    if not ptu:
        ptu = [0,0]

    pose = get_robot_pose_cpp()

    robot_state = RobotStateMessage()
    robot_state.pan = ptu[0]
    robot_state.tilt = ptu[1]
    robot_state.x = pose.position.x
    robot_state.y = pose.position.y
    robot_state.rotation = get_rotation_from_pose(pose)
    return robot_state

def get_rotation_from_pose(pose):
    """
    Returns the rotation in degrees from pose
    """
    pose.position.x = pose.position.x
    pose.position.y = pose.position.y
    pose.position.z = 0
    q = numpy.array([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w])
    (_, _, yaw) = tf.transformations.euler_from_quaternion(q)
    return yaw

def point_published(data):
    rospy.loginfo(rospy.get_caller_id() + "Set focus point to (" + str(data.point.x) + ", " + str(data.point.y) + ", "+ str(data.point.z) + ")")

    fcp = rospy.get_param("/nbv_robot_model/fcp")
    ncp = rospy.get_param("/nbv_robot_model/ncp")

    targetPosition = Point()
    targetOrientation = Quaternion()
    targetPosition.x = data.point.x - (fcp + ncp) / 2.0
    targetPosition.y = data.point.y
    targetPosition.z = data.point.z
    targetOrientation.w = 1.0
    targetOrientation.x = 0.0
    targetOrientation.y = 0.0
    targetOrientation.z = 0.0

    try:
        rospy.wait_for_service('/nbv_robot_model/CalculateCameraPoseCorrection', timeout=5)
        calculateCameraPoseCorrection = rospy.ServiceProxy(
            '/nbv_robot_model/CalculateCameraPoseCorrection',
            CalculateCameraPoseCorrection)
        actual_state = get_robot_state()

        rospy.loginfo('Current robot state is ' + str(actual_state))

        actual_pan = actual_state.pan * numpy.pi/180
        actual_tilt = actual_state.tilt * numpy.pi/180
        rospy.loginfo('Pan and tilt in rad: ' + str(actual_pan) + ", " + str(actual_tilt))

        ptuConfig = calculateCameraPoseCorrection(actual_state, targetOrientation, targetPosition)
    except (rospy.ServiceException, rospy.exceptions.ROSException), e:
        rospy.logwarn(str(e))
        return 'aborted'

    rospy.loginfo('Moving PTU to corrected goal (' + str(ptuConfig.pan) + ', ' + str(ptuConfig.tilt) + ')')

    client = actionlib.SimpleActionClient('ptu_controller_actionlib',
                                          PTUMovementAction)
    if not client.wait_for_server(rospy.Duration.from_sec(10)):
        rospy.logwarn("Could not connect to ptu action server")
        return 'aborted'

    ptu_goal = PTUMovementGoal()
    ptu_goal.target_joint.header.seq = 0
    ptu_goal.target_joint.name = ['pan', 'tilt']
    ptu_goal.target_joint.velocity = [0, 0]
    ptu_goal.target_joint.position = [ptuConfig.pan*180/numpy.pi, ptuConfig.tilt*180/numpy.pi]

    rospy.loginfo("Moving PTU to goal (" + str(ptuConfig.pan) + ", " + str(ptuConfig.tilt) + ')')

    ptu_result = client.send_goal_and_wait(ptu_goal)
    rospy.loginfo("PTU action returned (3 being 'succeeded'): " + str(ptu_result))

    future = time() + 1
    while time() < future:
        pass

    #Visualize new frustum
    camera_pose = get_camera_pose_cpp()
    get_camera_frustum(camera_pose)


    
if __name__ == '__main__':
    rospy.init_node('FPListener', anonymous=True)

    rospy.Subscriber("clicked_point", PointStamped, point_published)

    rospy.spin()
