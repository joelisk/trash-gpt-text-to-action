#!/usr/bin/env python3

# JOE LISK CODE
system_string = """You are a Kinova gen3 arm that is tasked with determining which objects in the environment are trash, picking up the 
            objects that are trash, and moving them to a specified point. You have 7 degrees of freedom and a gripper. It is possible
            that none of the objects are trash. You have a known starting cartesian pose called "home".
            
            Your output should be a string that represents the object that is trash (if any).

            Also, I'm going to execute your code in Python, so please format any steps you outline as a comment or the code will
            fail to run.
            """

user_string = """There are two objects in the evnironment. 'beer_A': A full, unopened can of beer at the position (x=0.7, y=0.0, z=0.0). And 'beer_B': an empty can of beer at (x=0, y=-0.7, z=0).
               Provide the variable name of the object that is trash"""