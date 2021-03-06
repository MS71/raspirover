# RaspiRover
Small ROS/Raspberry controlled roboter

## Features:
* RPI based robot
* DC motor control by using wiringpi PWM
* Motor ODM encoder by using wiringpi interrupts
* RTIMULib (git clone https://github.com/RPi-Distro/RTIMULib.git)
* Raspberry Camera Node
* Controlled by Wireless Joystick connected on RPI USB Port
* HC-SR04 ultrasonic range sensor

## TODO/Plan:
* Autonomous navigation and map building

## RPI Connection:
* PWM Motor Driver<br>
GPIO_PWML  19<br>
GPIO_IN0L  26<br>
GPIO_IN1L  13<br>
GPIO_PWMR  21<br>
GPIO_IN0R  20<br>
GPIO_IN1R  16<br>

* Sonar (HC-SR04 PWM signal)<br>
GPIO_SONAR 4<br>

* Line Laser On/Off<br>
GPIO_LASER 23<br>

* DC Motor wheel encoder input<br>
GPIO_ODOL 5<br>
GPIO_ODOR 6<br>

* RPI Camera<br>

* IMU/MPU-9250
RPI I2C Pins<br>

* Real Time Clock<br>
RPI I2C Pins<br>

## RPI Installation:
install Ubuntu Mate<br>
Install ROS (kinetic) using apt-get<br>

>git clone https://github.com/MS71/raspirover.git<br>

>source /opt/ros/kinetic/setup.sh<br>
>cd raspirover/ros_catkin_ws<br>
>source devel/setup.sh<br>
>roslaunch raspirover raspirover.launch<br>

install ROS web interface<br>
>sudo apt-get install lighttpd<br>
>cd /var/www<br>
>mv html html.org<br>
>ln -s /home/pi/raspirover/ros-control-center html<br>

## On Host:
>ROS_MASTER_URI=http://raspirover:11311

## ROS configuration:
![TF](https://github.com/MS71/raspirover/blob/master/misc/tf.png)

![ROS-Graph](https://github.com/MS71/raspirover/blob/master/misc/rosgraph.png)
