# rospibot5
Robot ROS - STM32F, Pi, MPU6050, VL53L0X, SG-90


STM32duino HAL library
https://github.com/rogerclarkmelbourne/Arduino_STM32

Raspberry Pi Libraries
https://github.com/jrowberg/i2cdevlib
http://www.airspayce.com/mikem/bcm2835/index.html

ROS packages
https://github.com/chrisspen/ros_mpu6050_node

sudo apt-get install ros-kinetic-gmapping
#rosrun gmapping slam_gmapping scan:=scan

sudo bash -c "source /opt/ros/kinetic/setup.bash; source /home/ubuntu/catkin_ws/devel/setup.bash; roslaunch rospibot5 robot.launch"
rostopic pub --once mcu_cmd std_msgs/Int16 "data: 3"
roslaunch rospibot6 teleop.launch
