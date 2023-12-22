#!/usr/bin/env python3

# JOE LISK CODE
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