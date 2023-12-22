# trash-gpt-text-to-action
ROB 5994 Project
<!-- ![trash-gpt](https://github.com/joelisk/trash-gpt/assets/72030405/f55cf213-2243-4c3d-9d52-cfeb9f47ea32) -->
Block diagram of the system architecture:
![Start](https://github.com/joelisk/trash-gpt-text-to-action/assets/72030405/b0e3b684-3362-4cbd-9582-3d9cd36b5185)

Check out the video of the final project [here](https://www.youtube.com/watch?v=LHBu9tNYiP0&t=4s)

*Formatted for readability.*

```
System prompt for GPT-4:

You are a Kinova gen3 arm that is tasked with 
determining which objects in the environment 
are trash, picking up the objects that are 
trash, and moving them to a specified point. 
You have 7 degrees of freedom and a gripper.
It is possible that none of the objects are 
trash. You have a known starting cartesian 
pose called "home". Your output should be a 
string that represents the object that is 
trash (if any). Also, I'm going to execute 
your code in Python, so please format any 
steps you outline as a comment or the code 
will fail to run.
```

```
Beer A user prompt for GPT-4:

There are two objects in the environment.

beer_A: An empty can of beer with a pre-grasp
pose of
target_pose.position.x = 0.5338950844247246,
target_pose.position.y = 0.013882795317536337,
target_pose.position.z = 0.03560667199606036,
target_pose.orientation.x = 0.5233042320042509,
target_pose.orientation.y = 0.4906332994586916,
target_pose.orientation.z = 0.47538816211295915,
target_pose.orientation.w = 0.509350313194742
and a grasp pose of,
target_pose.position.x = 0.7069753526986098,
target_pose.position.y = 0.012251526063553923,
target_pose.position.z = 0.030152609462021807,
target_pose.orientation.x = 0.517385163838614,
target_pose.orientation.y = 0.49491109344252465,
target_pose.orientation.z = 0.4797642455445715,
target_pose.orientation.w = 0.507150737477788.

And beer_B: a full, unopened can of beer with
a pre-grasp pose of,
target_pose.position.x = -0.008528227056494208,
target_pose.position.y = -0.4545102051969075,
target_pose.position.z = 0.031031940091151258,
target_pose.orientation.x = 0.05375719323178181,
target_pose.orientation.y = -0.7757844834263641,
target_pose.orientation.z = 0.6283301669563721,
target_pose.orientation.w = 0.021674887388595025
and a grasp pose of,
target_pose.position.x = 0.002531766299089388,
target_pose.position.y = -0.6028243179191973,
target_pose.position.z = 0.02571023473658379,
target_pose.orientation.x = 0.046498299219275584,
target_pose.orientation.y = -0.7693984296865566,
target_pose.orientation.z = 0.6370337922518455,
target_pose.orientation.w = 0.0072050048444401256.

Provide a Python list containing 3 elements:
the variable name of the object that is trash,
the pre-grasp pose of the object that is trash,
the grasp pose of the object that is trash.
Also, I'm going to execute your code in Python,
so please do not explain or provide any additional
text, just provide the list. You don't need to
assign the list to a variable.
```

```
Beer B user prompt for GPT-4:

There are two objects in the environment.

beer_A: a full, unopened can of beer with a pre-grasp
pose of
target_pose.position.x = 0.5338950844247246,
target_pose.position.y = 0.013882795317536337,
target_pose.position.z = 0.03560667199606036,
target_pose.orientation.x = 0.5233042320042509,
target_pose.orientation.y = 0.4906332994586916,
target_pose.orientation.z = 0.47538816211295915,
target_pose.orientation.w = 0.509350313194742
and a grasp pose of,
target_pose.position.x = 0.7069753526986098,
target_pose.position.y = 0.012251526063553923,
target_pose.position.z = 0.030152609462021807,
target_pose.orientation.x = 0.517385163838614,
target_pose.orientation.y = 0.49491109344252465,
target_pose.orientation.z = 0.4797642455445715,
target_pose.orientation.w = 0.507150737477788.

And beer_B: An empty can of beer with
a pre-grasp pose of,
target_pose.position.x = -0.008528227056494208,
target_pose.position.y = -0.4545102051969075,
target_pose.position.z = 0.031031940091151258,
target_pose.orientation.x = 0.05375719323178181,
target_pose.orientation.y = -0.7757844834263641,
target_pose.orientation.z = 0.6283301669563721,
target_pose.orientation.w = 0.021674887388595025
and a grasp pose of,
target_pose.position.x = 0.002531766299089388,
target_pose.position.y = -0.6028243179191973,
target_pose.position.z = 0.02571023473658379,
target_pose.orientation.x = 0.046498299219275584,
target_pose.orientation.y = -0.7693984296865566,
target_pose.orientation.z = 0.6370337922518455,
target_pose.orientation.w = 0.0072050048444401256.

Provide a Python list containing 3 elements:
the variable name of the object that is trash,
the pre-grasp pose of the object that is trash,
the grasp pose of the object that is trash.
Also, I'm going to execute your code in Python,
so please do not explain or provide any additional
text, just provide the list. You don't need to
assign the list to a variable.
```

```
Code to process output from GPT-4:

gpt_list = getTargetFromGPT(system_string, user_string)

targetObj = gpt_list[0] # string
targetPreGraspPose = gpt_list[1] # dictionary
targetGraspPose = gpt_list[2] # dictionary
```
