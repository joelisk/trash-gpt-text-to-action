gpt_list = ["beer_B", {"position": {"x": -0.008528227056494208, "y": -0.4545102051969075, "z": 0.031031940091151258}, "orientation": {"x": 0.05375719323178181, "y": -0.7757844834263641, "z": 0.6283301669563721, "w": 0.021674887388595025}}, {"position": {"x": 0.002531766299089388, "y": -0.6028243179191973, "z": 0.02571023473658379}, "orientation": {"x": 0.046498299219275584, "y": -0.7693984296865566, "z": 0.6370337922518455, "w": 0.0072050048444401256}}]
targetObj = gpt_list[0] #string
targetPreGraspPose = gpt_list[1] #dictionary
targetGraspPose = gpt_list[2] #dictionary
print(type(targetObj))
print(type(targetPreGraspPose))
actual_pose_pre_grasp = targetPreGraspPose['position']['x']
print(actual_pose_pre_grasp)