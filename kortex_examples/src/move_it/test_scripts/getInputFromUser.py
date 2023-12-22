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
    print(msg)
    gettingInput = True
    while(gettingInput):
        key = getUserString()
        if key != '':
            print("THIS IS YOUR INPUT:")
            print(key)
            gettingInput = False

if __name__ == '__main__':
  getInputFromUser()

