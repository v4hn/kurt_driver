spawn:
	rosservice call gazebo/delete_model '{model_name: kurt}'
	roslaunch kurt_spawn.launch
