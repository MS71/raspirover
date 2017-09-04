# raspirover
small ROS/Raspberry controlled roboter

Features:
* RPI based robot
* DC motor control by using wiringpi PWM
* Motor ODM encoder by using wiringpi interrupts
* RTIMULib (git clone https://github.com/RPi-Distro/RTIMULib.git)
* Raspberry Camera Node
* Controlled by Wireless Joystick connected on RPI USB Port
* HC-SR04 ultrasonic range sensor

TODO/Plan:
* Autonomous navigation and map building

RPI Installation:
* install Ubuntu Mate
* Install ROS (kinetic) using apt-get

>git clone https://github.com/MS71/raspirover.git

>source /opt/ros/kinetic/setup.sh
>cd raspirover/ros_catkin_ws
>source devel/setup.sh 
>roslaunch raspirover raspirover.launch

On Host:
>ROS_MASTER_URI=http://raspirover:11311



