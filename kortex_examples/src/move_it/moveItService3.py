#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman

# Inspired from http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/move_group_python_interface/move_group_python_interface_tutorial.html
# Modified by Alexandre Vannobel to test the FollowJointTrajectory Action Server for the Kinova Gen3 robot

# To run this node in a given namespace with rosrun (for example 'my_gen3'), start a Kortex driver and then run :
# rosrun kortex_examples example_moveit_trajectories.py __ns:=my_gen3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
import tf
from std_srvs.srv import Empty

# Joe Lisk code
#from chatGPTService import getTargetFromGPT
#from jointGPTPrompt import system_string, user_string
# import error: moduleNotFound

# from chatGPTService
import os
import openai
import rospy

openai.api_key = os.environ["OPENAI_API_KEY"]

def getTargetFromGPT(system_string, user_string):
    messages = [{"role": "system", "content": system_string}]
    messages.append({"role": "user", "content": user_string})
    completion = openai.ChatCompletion.create(
    model="gpt-4",
    messages=messages)
    response = completion['choices'][0]['message']['content']
    rospy.loginfo('Successfully got response from GPT-4!')
    rospy.loginfo('Response from GPT-4:')
    rospy.loginfo(response)
    target = eval(response)
    return target

# From jointGPTPrompt
system_string = """You are a Kinova gen3 arm that is tasked with determining which objects in the environment are trash, picking up the
            objects that are trash, and moving them to a specified point. You have 7 degrees of freedom and a gripper. It is possible
            that none of the objects are trash. You have a known starting cartesian pose called "home".

            Your output should be a string that represents the object that is trash (if any).

            Also, I'm going to execute your code in Python, so please format any steps you outline as a comment or the code will
            fail to run.
            """

# user_string = """There are two objects in the evnironment. beer_A: an empty can of beer at the position (x=0.7, y=0.0, z=0.0). And beer_B: an full, unopened can of beer at (x=0, y=-0.7, z=0).
#                Provide the variable name of the object that is trash. Also, I'm going to execute your code in Python, so please do not explain, just provide the variable name"""

#Joe Lisk code
#Helper function for joint angles
# def deg2rad(x):
#   return x * pi / 180

#Adapted from https://github.com/turtlebot/turtlebot/blob/kinetic/turtlebot_teleop/scripts/turtlebot_teleop_key
msg = """
Give your environment for your Kinova arm!
---------------------------
Describe your workspace and the objects present.

ChatGPT will then help control the robot.

Currently, you need to supply the poses

Of the objects in the form:

position.x = num, position.y = num, position.z = num, orientation.x = num, orientation.y = num, orientation.z = num, orientation.w = num

Grasp pose detection coming soon!

commands
---------------------------
enter : sends your input to ChatGPT

CTRL-C to quit
"""

def getUserString():
    str = input()
    return str

class ExampleMoveItTrajectories(object):
  """ExampleMoveItTrajectories"""
  def __init__(self):

    # Initialize the node
    super(ExampleMoveItTrajectories, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    try:
      self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
      if self.is_gripper_present:
        gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        self.gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns=rospy.get_namespace())
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=rospy.get_namespace())
      self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True


  def reach_named_position(self, target):
    arm_group = self.arm_group

    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, trajectory_message, planning_time, error_code) = arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(trajectory_message, wait=True)

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, target_obj, target_pose, tolerance, constraints):
    arm_group = self.arm_group

    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Joe Lisk code
    # Just spotting ChatGPT the trash can pose
    if target_obj == 'trash_can':
      target_pose.position.x = -0.007253318946339758
      target_pose.position.y = 0.4578600037286752
      target_pose.position.z = 0.1964750840769468
      target_pose.orientation.x = 0.08187678131496287
      target_pose.orientation.y = 0.6676512236268767
      target_pose.orientation.z = 0.7399520009621545
      target_pose.orientation.w = 0.003012066257614919

    # Get the current Cartesian Position
    arm_group.set_pose_target(target_pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
 
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()

    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False

def main():
  example = ExampleMoveItTrajectories()

  # For testing purposes
  success = example.is_init_success
  try:
      rospy.delete_param("/kortex_examples_test_results/moveit_general_python")
  except:
      pass


  # comment out when using Gazebo sim. for real arm need to send home first
  if success:
    rospy.loginfo("Reaching Named Target Home...")
    success &= example.reach_named_position("home")
    print(success)
  
  # Joe Lisk code
  # get user input
  print(msg)
  gettingInput = True
  while(gettingInput):
    str = getUserString()
    if str != '':
      print("THIS IS YOUR INPUT:")
      print(str)
      user_string = str
      gettingInput = False

  # Joe Lisk code
  # chatgpt service
  gpt_list = getTargetFromGPT(system_string, user_string)

  targetObj = gpt_list[0] # string
  targetPreGraspPose = gpt_list[1] # dictionary
  targetGraspPose = gpt_list[2] # dictionary

  # Pre-Grasp
  if success:
    rospy.loginfo("Reaching Cartesian Pose...")

    actual_pose = example.get_cartesian_pose()

    actual_pose.position.x = targetPreGraspPose['position']['x']
    actual_pose.position.y = targetPreGraspPose['position']['y']
    actual_pose.position.z = targetPreGraspPose['position']['z']
    actual_pose.orientation.x = targetPreGraspPose['orientation']['x']
    actual_pose.orientation.y = targetPreGraspPose['orientation']['y']
    actual_pose.orientation.z = targetPreGraspPose['orientation']['z']
    actual_pose.orientation.w = targetPreGraspPose['orientation']['w']

    success &= example.reach_cartesian_pose(targetObj, target_pose=actual_pose, tolerance=0.01, constraints=None)
    print (success)

  if success:
    actual_pose = example.get_cartesian_pose()
    rospy.loginfo("Cartesian Pose: PRE-GRASP")
    rospy.loginfo(actual_pose)

  # Grasp
  if success:
    rospy.loginfo("Reaching Cartesian Pose...")

    actual_pose = example.get_cartesian_pose()

    actual_pose.position.x = targetGraspPose['position']['x']
    actual_pose.position.y = targetGraspPose['position']['y']
    actual_pose.position.z = targetGraspPose['position']['z']
    actual_pose.orientation.x = targetGraspPose['orientation']['x']
    actual_pose.orientation.y = targetGraspPose['orientation']['y']
    actual_pose.orientation.z = targetGraspPose['orientation']['z']
    actual_pose.orientation.w = targetGraspPose['orientation']['w']
  
    success &= example.reach_cartesian_pose(targetObj, target_pose=actual_pose, tolerance=0.01, constraints=None)
    print (success)

  if success:
    actual_pose = example.get_cartesian_pose()
    rospy.loginfo("Cartesian Pose: GRASP")
    rospy.loginfo(actual_pose)

    rospy.loginfo("Closing the gripper 100%...")
    success &= example.reach_gripper_position(0)
    print (success)

  if success:
    rospy.loginfo("Reaching Cartesian Pose...")

    success &= example.reach_cartesian_pose('trash_can', target_pose=actual_pose, tolerance=0.01, constraints=None)
    print (success)

  if success:
    actual_pose = example.get_cartesian_pose()
    rospy.loginfo("Cartesian Pose: TRASH CAN")
    rospy.loginfo(actual_pose)

  if example.is_gripper_present and success:
    rospy.loginfo("Opening the gripper...")
    success &= example.reach_gripper_position(1)
    print (success)

  if success:
    rospy.loginfo("Reaching Named Target Home...")
    success &= example.reach_named_position("home")
    print (success)

  # comment out when using Gazebo sim and running real arm tests. this is just to guide the robot to a safe off position when done testing.
  # if success:
  #   rospy.loginfo("Reaching Target Off Position...")

  #   off_list = ["off_position"
  #     {"position": {"x": 0.002531766299089388, "y": -0.6028243179191973, "z": 0.02571023473658379},
  #       "orientation": {"x": 0.046498299219275584, "y": -0.7693984296865566, "z": 0.6370337922518455, "w": 0.0072050048444401256}}]

  #   targetGraspPose = off_list[1] # dictionary

  #   actual_pose = example.get_cartesian_pose()

  #   actual_pose.position.x = targetGraspPose['position']['x']
  #   actual_pose.position.y = targetGraspPose['position']['y']
  #   actual_pose.position.z = targetGraspPose['position']['z']
  #   actual_pose.orientation.x = targetGraspPose['orientation']['x']
  #   actual_pose.orientation.y = targetGraspPose['orientation']['y']
  #   actual_pose.orientation.z = targetGraspPose['orientation']['z']
  #   actual_pose.orientation.w = targetGraspPose['orientation']['w']
  
  #   success &= example.reach_cartesian_pose(targetObj, target_pose=actual_pose, tolerance=0.01, constraints=None)
  #   print(success)

  # For testing purposes
  rospy.set_param("/kortex_examples_test_results/moveit_general_python", success)

  if not success:
      rospy.logerr("The example encountered an error.")

if __name__ == '__main__':
  main()

# trash: beer_B
# user input: There are two objects in the evnironment. beer_A: A full, unopened can of beer with a pre-grasp pose of target_pose.position.x = 0.5338950844247246, target_pose.position.y = 0.013882795317536337, target_pose.position.z = 0.03560667199606036, target_pose.orientation.x = 0.5233042320042509, target_pose.orientation.y = 0.4906332994586916, target_pose.orientation.z = 0.47538816211295915, target_pose.orientation.w = 0.509350313194742 and a grasp pose of, target_pose.position.x = 0.7069753526986098, target_pose.position.y = 0.012251526063553923, target_pose.position.z = 0.030152609462021807, target_pose.orientation.x = 0.517385163838614, target_pose.orientation.y = 0.49491109344252465, target_pose.orientation.z = 0.4797642455445715, target_pose.orientation.w = 0.507150737477788. And beer_B: an empty can of beer with a pre-grasp pose of, target_pose.position.x = -0.008528227056494208, target_pose.position.y = -0.4545102051969075, target_pose.position.z = 0.031031940091151258, target_pose.orientation.x = 0.05375719323178181, target_pose.orientation.y = -0.7757844834263641, target_pose.orientation.z = 0.6283301669563721, target_pose.orientation.w = 0.021674887388595025 and a grasp pose of, target_pose.position.x = 0.002531766299089388, target_pose.position.y = -0.6028243179191973, target_pose.position.z = 0.02571023473658379, target_pose.orientation.x = 0.046498299219275584, target_pose.orientation.y = -0.7693984296865566, target_pose.orientation.z = 0.6370337922518455, target_pose.orientation.w = 0.0072050048444401256. Provide a Python list containing 3 elements: the variable name of the object that is trash, the pre-grasp pose of the object that is trash, the grasp pose of the object that is trash. Also, I'm going to execute your code in Python, so please do not explain or provide any additional text, just provide the list. You don't need to assign the list to a variable.

# trash: beer_A
# user input: There are two objects in the evnironment. beer_A: An empty can of beer with a pre-grasp pose of target_pose.position.x = 0.5338950844247246, target_pose.position.y = 0.013882795317536337, target_pose.position.z = 0.03560667199606036, target_pose.orientation.x = 0.5233042320042509, target_pose.orientation.y = 0.4906332994586916, target_pose.orientation.z = 0.47538816211295915, target_pose.orientation.w = 0.509350313194742 and a grasp pose of, target_pose.position.x = 0.7069753526986098, target_pose.position.y = 0.012251526063553923, target_pose.position.z = 0.030152609462021807, target_pose.orientation.x = 0.517385163838614, target_pose.orientation.y = 0.49491109344252465, target_pose.orientation.z = 0.4797642455445715, target_pose.orientation.w = 0.507150737477788. And beer_B: a full, unopened can of beer with a pre-grasp pose of, target_pose.position.x = -0.008528227056494208, target_pose.position.y = -0.4545102051969075, target_pose.position.z = 0.031031940091151258, target_pose.orientation.x = 0.05375719323178181, target_pose.orientation.y = -0.7757844834263641, target_pose.orientation.z = 0.6283301669563721, target_pose.orientation.w = 0.021674887388595025 and a grasp pose of, target_pose.position.x = 0.002531766299089388, target_pose.position.y = -0.6028243179191973, target_pose.position.z = 0.02571023473658379, target_pose.orientation.x = 0.046498299219275584, target_pose.orientation.y = -0.7693984296865566, target_pose.orientation.z = 0.6370337922518455, target_pose.orientation.w = 0.0072050048444401256. Provide a Python list containing 3 elements: the variable name of the object that is trash, the pre-grasp pose of the object that is trash, the grasp pose of the object that is trash. Also, I'm going to execute your code in Python, so please do not explain or provide any additional text, just provide the list. You don't need to assign the list to a variable.
