roslaunch sawyer_full_stack sawyer_camera_track.launch 
rosrun intera_interface joint_trajectory_action_server.py
roslaunch sawyer_moveit_config sawyer_moveit.launch electric_gripper:=true