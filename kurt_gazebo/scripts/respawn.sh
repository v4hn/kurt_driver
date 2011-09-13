#!/bin/sh

rosservice call gazebo/delete_model '{model_name: kurt}';
roslaunch kurt_gazebo spawn_kurt.launch;
