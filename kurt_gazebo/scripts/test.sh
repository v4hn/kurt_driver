#!/bin/sh
rosservice call gazebo/clear_body_wrenches '{body_name: "kurt::kurt_body"}';
rosservice call gazebo/apply_body_wrench '{body_name: "kurt::left_front_wheel_link" , wrench: { force: { x: 0.0, y: 0.0, z: 0.0 } , torque: { x: 0.0, y: 1.0 , z: 0.0 } }, duration: -1 }'&
rosservice call gazebo/apply_body_wrench '{body_name: "kurt::right_front_wheel_link" , wrench: { force: { x: 0.0, y: 0.0, z: 0.0 } , torque: { x: 0.0, y: -1.0 , z: 0.0 } }, duration: -1 }';
