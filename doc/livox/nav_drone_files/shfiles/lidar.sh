sudo chmod 777 /dev/ttyACM0;
cd livox;
source ./devel/setup.bash;
roslaunch mavros px4.launch & sleep 10;
rosrun mavros mavcmd long 511 105 5000 0 0 0 0 0 & sleep 1;
rosrun mavros mavcmd long 511 31 5000 0 0 0 0 0 & sleep 1;

roslaunch livox_ros_driver2 msg_MID360.launch & sleep 5;
roslaunch fast_lio mapping_mid360.launch rviz:=false;




