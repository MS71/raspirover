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



RPI Connection:
* PWM Motor Driver
#define GPIO_PWML  19
#define GPIO_IN0L  26
#define GPIO_IN1L  13

#define GPIO_PWMR  21
#define GPIO_IN0R  20
#define GPIO_IN1R  16

* Sonar (HC-SR04 PWM signal)
#define GPIO_SONAR 4

* Line Laser On/Off
#define GPIO_LASER 23

* DC Motor wheel encoder input
#define GPIO_ODOL  5
#define GPIO_ODOR  6

* RPI Camera

* IMU/MPU-9250
RPI I2C Pins

* Real Time Clock
RPI I2C Pins



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



