#!/usr/bin/env python3

# JOE LISK CODE
import os
import openai
import rospy

openai.api_key = os.environ["OPENAI_API_KEY"]

system_string = """You are a Kinova gen3 arm that is tasked with determining which objects in the environment are trash, picking up the
            objects that are trash, and moving them to a specified point. You have 7 degrees of freedom and a gripper. It is possible
            that none of the objects are trash. You have a known starting cartesian pose called "home".

            Your output should be a string that represents the object that is trash (if any).

            Also, I'm going to execute your code in Python, so please format any steps you outline as a comment or the code will
            fail to run.
            """
# user_string = """There are two objects in the evnironment. beer_A: A full, unopened can of beer with a pre-grasp pose of target_pose.position.x = 0.5338950844247246, target_pose.position.y = 0.013882795317536337, target_pose.position.z = 0.03560667199606036, target_pose.orientation.x = 0.5233042320042509, target_pose.orientation.y = 0.4906332994586916, target_pose.orientation.z = 0.47538816211295915, target_pose.orientation.w = 0.509350313194742 and a grasp pose of, target_pose.position.x = 0.7069753526986098, target_pose.position.y = 0.012251526063553923, target_pose.position.z = 0.030152609462021807, target_pose.orientation.x = 0.517385163838614, target_pose.orientation.y = 0.49491109344252465, target_pose.orientation.z = 0.4797642455445715, target_pose.orientation.w = 0.507150737477788. And beer_B: an empty can of beer with a pre-grasp pose of, target_pose.position.x = -0.008528227056494208, target_pose.position.y = -0.4545102051969075, target_pose.position.z = 0.031031940091151258, target_pose.orientation.x = 0.05375719323178181, target_pose.orientation.y = -0.7757844834263641, target_pose.orientation.z = 0.6283301669563721, target_pose.orientation.w = 0.021674887388595025 and a grasp pose of, target_pose.position.x = 0.002531766299089388, target_pose.position.y = -0.6028243179191973, target_pose.position.z = 0.02571023473658379, target_pose.orientation.x = 0.046498299219275584, target_pose.orientation.y = -0.7693984296865566, target_pose.orientation.z = 0.6370337922518455, target_pose.orientation.w = 0.0072050048444401256. Provide a Python list containing 3 elements: the variable name of the object that is trash, the pre-grasp pose of the object that is trash, the grasp pose of the object that is trash. Also, I'm going to execute your code in Python, so please do not explain, just provide the list."""

def getTargetFromGPT(system_string, user_string):
    messages = [{"role": "system", "content": system_string}]
    messages.append({"role": "user", "content": user_string})
    completion = openai.ChatCompletion.create(
    model="gpt-4",
    messages=messages)
    response = completion['choices'][0]['message']['content']
    # rospy.loginfo('Successfully got response from GPT-4!')
    # rospy.loginfo('Response from GPT-4:')
    # rospy.loginfo(response)
    print('Successfully got response from GPT-4!')
    print('Response from GPT-4:')
    print(response)
    #target = exec(response)
    #return target

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

def getInputFromUser():
    user_string = ''
    print(msg)
    gettingInput = True
    while(gettingInput):
        str = getUserString()
        if str != '':
            print("THIS IS YOUR INPUT:")
            print(str)
            user_string = str
            gettingInput = False
    getTargetFromGPT(system_string, user_string)

if __name__ == '__main__':
  getInputFromUser()


# if target_obj == 'beer_A':
#       if pose_str == 'pre-grasp':
#         target_pose.position.x = 0.5338950844247246
#         target_pose.position.y = 0.013882795317536337
#         target_pose.position.z = 0.03560667199606036
#         target_pose.orientation.x = 0.5233042320042509
#         target_pose.orientation.y = 0.4906332994586916
#         target_pose.orientation.z = 0.47538816211295915
#         target_pose.orientation.w = 0.509350313194742
#       else:
#         target_pose.position.x = 0.7069753526986098
#         target_pose.position.y = 0.012251526063553923
#         target_pose.position.z = 0.030152609462021807
#         target_pose.orientation.x = 0.517385163838614
#         target_pose.orientation.y = 0.49491109344252465
#         target_pose.orientation.z = 0.4797642455445715
#         target_pose.orientation.w = 0.507150737477788

#     if target_obj == 'beer_B':
#       if pose_str == 'pre-grasp':
#         target_pose.position.x = -0.008528227056494208
#         target_pose.position.y = -0.4545102051969075
#         target_pose.position.z = 0.031031940091151258
#         target_pose.orientation.x = 0.05375719323178181
#         target_pose.orientation.y = -0.7757844834263641
#         target_pose.orientation.z = 0.6283301669563721
#         target_pose.orientation.w = 0.021674887388595025
#       else:
#         target_pose.position.x = 0.002531766299089388
#         target_pose.position.y = -0.6028243179191973
#         target_pose.position.z = 0.02571023473658379
#         target_pose.orientation.x = 0.046498299219275584
#         target_pose.orientation.y = -0.7693984296865566
#         target_pose.orientation.z = 0.6370337922518455
#         target_pose.orientation.w = 0.0072050048444401256

#     if target_obj == 'trash_can':
#       target_pose.position.x = -0.007253318946339758
#       target_pose.position.y = 0.4578600037286752
#       target_pose.position.z = 0.1964750840769468
#       target_pose.orientation.x = 0.08187678131496287
#       target_pose.orientation.y = 0.6676512236268767
#       target_pose.orientation.z = 0.7399520009621545
#       target_pose.orientation.w = 0.003012066257614919


#Successfully got response from GPT-4!
#Response from GPT-4:
#["beer_B", {"position": {"x": -0.008528227056494208, "y": -0.4545102051969075, "z": 0.031031940091151258}, "orientation": {"x": 0.05375719323178181, "y": -0.7757844834263641, "z": 0.6283301669563721, "w": 0.021674887388595025}}, {"position": {"x": 0.002531766299089388, "y": -0.6028243179191973, "z": 0.02571023473658379}, "orientation": {"x": 0.046498299219275584, "y": -0.7693984296865566, "z": 0.6370337922518455, "w": 0.0072050048444401256}}]
