/*
 * This file is part of the raspirover distribution (https://github.com/MS71/raspirover).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/console.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Byte.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include <geometry_msgs/Twist.h>
#include "raspirovernode.h"
#include <math.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/LaserScan.h>
#include <signal.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <sys/time.h>

#define HAVE_WIRINGPI
#ifdef HAVE_WIRINGPI
#include <wiringPi.h>
#include <softPwm.h>
#include <wiringPiI2C.h>
#endif

#include "VL53L0X.h"

#define ENABLE_SONAR_PUB
#define ENABLE_ODOM_PUB

/*
 * some defines
 */
#define GPIO_SONAR 4

#define GPIO_LASER 23

#define GPIO_ODOL  5
#define GPIO_ODOR  6

#define GPIO_PWML  19
#define GPIO_IN0L  26
#define GPIO_IN1L  13

#define GPIO_PWMR  21
#define GPIO_IN0R  20
#define GPIO_IN1R  16

#define GPIO_TEST1 17
#define GPIO_TEST2 27
#define GPIO_TEST3 22

#define GPIO_LIDAR_PWM		12
#define GPIO_LIDAR_GPIO1	24
#define GPIO_LIDAR_XSHUT	25

/*
 * global data
 */
ros::Time odo_last_time;
ros::Time sonar_last_time;
static struct
{
	double x;
	double y;
	double th;

	double vx;
	double vy;
	double vth;
} odo = {
		0.0,
		0.0,
		0.0,
		0.0,
		-0.1,
		0.1 };

ros::Publisher odom_pub;

volatile int odo_0_step = 0;
volatile int odo_0_cnt = 0;
volatile int odo_0_cntx = 0;
volatile int odo_0_pin = 0;

volatile int odo_1_step = 0;
volatile int odo_1_cnt = 0;
volatile int odo_1_cntx = 0;
volatile int odo_1_pin = 0;

/*
 *
 */
void motor_ctrl( int gpio_en, int gpio_in0, int gpio_in1 , int vel )
{
	//printf("motor_ctrl(%d,%d,%d,%d)}\n",gpio_en,gpio_in0,gpio_in1,vel);
	if( vel == 0 )
	{
#ifdef HAVE_WIRINGPI
		pinMode(gpio_in0,INPUT);
		pinMode(gpio_in1,INPUT);

		softPwmWrite (gpio_en, vel) ;
		digitalWrite (gpio_in0, HIGH);
		digitalWrite (gpio_in1, HIGH);
#endif
	}
	else if( vel > 0 )
	{
#ifdef HAVE_WIRINGPI
		pinMode(gpio_in0,OUTPUT);
		pinMode(gpio_in1,OUTPUT);

		softPwmWrite (gpio_en, vel) ;
		digitalWrite (gpio_in0, HIGH);
		digitalWrite (gpio_in1, LOW);
#endif
	}
	else if ( vel < 0 )
	{
#ifdef HAVE_WIRINGPI
		pinMode(gpio_in0,OUTPUT);
		pinMode(gpio_in1,OUTPUT);

		softPwmWrite (gpio_en, -vel) ;
		digitalWrite (gpio_in0, LOW);
		digitalWrite (gpio_in1, HIGH);
#endif
	}
}

/*
 *
 */
void motor( int m, int vel )
{
	switch(m)
	{
	case 0:
		motor_ctrl(GPIO_PWML,GPIO_IN0L,GPIO_IN1L,vel);
		if( vel > 0 )
			odo_0_step = 1;
		else if( vel < 0 )
			odo_0_step = -1;
		else
			odo_0_step = 0;
		break;
	case 1:
		motor_ctrl(GPIO_PWMR,GPIO_IN0R,GPIO_IN1R,vel);
		if( vel > 0 )
			odo_1_step = 1;
		else if( vel < 0 )
			odo_1_step = -1;
		else
			odo_1_step = 0;
		break;
	}

}

#ifdef HAVE_WIRINGPI
/*
 *
 */
void motor_odoint_0(void)
{
	//digitalWrite (GPIO_TEST2, HIGH);
	odo_0_cnt += odo_0_step;
	odo_0_cntx += 1;
	//printf("motor_odoint_0 %d %d %d %d %d %d\n”",odo_0_step,digitalRead(GPIO_ODOR),odo_0_cntx,odo_0_cnt,odo_1_cntx,odo_1_cnt);
	//digitalWrite (GPIO_TEST2, LOW);
}

/*
 *
 */
void motor_odoint_1(void)
{
	//digitalWrite (GPIO_TEST3, HIGH);
	odo_1_cnt += odo_1_step;
	odo_1_cntx += 1;
	//printf("motor_odoint_1 %d %d %d %d %d %d\n”",odo_1_step,digitalRead(GPIO_ODOR),odo_0_cntx,odo_0_cnt,odo_1_cntx,odo_1_cnt);
	//digitalWrite (GPIO_TEST3, LOW);
}

/*
 *
 */
unsigned long t_sonar = 0;
void sonarint(void)
{
	static int first = 1;
	static int _pin = 0;
	static ros::Time _t;
	ros::Time t = ros::Time::now();
	if( first == 0 )
	{
		int pin = digitalRead(GPIO_SONAR);
		if( pin != _pin )
		{
			if( pin == 0 )
			{
				unsigned long dt = (t-_t).toSec()*1000000.0;
				t_sonar = ((2*t_sonar)+dt)/3;
			}
			_pin = pin;
		}
	}
	first = 0;
	_t = t;
	return;
}
#endif

/*
 * control the line laser
 */
void laser_callback(const std_msgs::String& laser_cmd)
{
	char s[256];
	if( laser_cmd.data == "on" )
	{
		sprintf(s,"gpio -g write %d 1",GPIO_LASER); system(s);
	}
	else
	{
		sprintf(s,"gpio -g write %d 0",GPIO_LASER); system(s);
	}
}

/*
 *
 */
void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{
	double x = (double)1.0*vel_cmd.linear.x;
	double y = (double)1.0*vel_cmd.linear.y;
	x = (x>1.0)?1.0:((x<-1.0)?-1.0:x);
	y = (y>1.0)?1.0:((y<-1.0)?-1.0:y);
	if((fabs(x)+fabs(y))>1.0)
	{
		double k = (fabs(x)+fabs(y));
		x = x/k;
		y = y/k;
	}

	int motor_l = (int)(100.0*x)+(100.0*(y));
	int motor_r = (int)(100.0*x)-(100.0*(y));

	ROS_DEBUG("cmd_vel_callback(%f,%f) x=%1.3f y=%1.3f motor-l=%d motor-r=%d",
			vel_cmd.linear.x,vel_cmd.angular.y,
			x,y,
			motor_l,motor_r);

	motor(0,motor_l);
	motor(1,motor_r);
}

/*
 *
 */
#ifdef ENABLE_ODOM_PUB
void handleODO(tf::TransformBroadcaster& tf_broadcaster)
{
	ros::Time current_time = ros::Time::now();
	double dt = (current_time - odo_last_time).toSec();
	if(dt>=0.1)
	{
		/*
		 * https://answers.ros.org/question/207392/generating-odom-message-from-encoder-ticks-for-robot_pose_ekf/
		 * http://www.seattlerobotics.org/encoder/200610/Article3/IMU%20Odometry,%20by%20David%20Anderson.htm
		 */
		static double x = 0;
		static double y = 0;
		static double th = 0;

		/*
		 * 230 pulse auf 0.95m
		 */
		//double DistancePerCount = 50.0/12.0 * 0.6/17.0;
		double DistancePerCount = 0.95/230.0;
		double lengthBetweenTwoWheels = 0.12*1.64/1.03169;

		//extract the wheel velocities from the tick signals count
		double deltaLeft = odo_1_cnt;
		double deltaRight = odo_0_cnt;

		double v_left = (deltaLeft * DistancePerCount) / dt;
		double v_right = (deltaRight * DistancePerCount) / dt;

		double vx = ((v_right + v_left) / 2);
		double vy = 0;
		double vth = ((v_right - v_left)/lengthBetweenTwoWheels);

		//double dt = (current_time - last_time).toSec();
		double delta_x = (vx * cos(th)) * dt;
		double delta_y = (vx * sin(th)) * dt;
		double delta_th = vth * dt;

		x += delta_x;
		y += delta_y;
		th += delta_th;

		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_footprint";

		odom_trans.transform.translation.x = x;
		odom_trans.transform.translation.y = y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		tf_broadcaster.sendTransform(odom_trans);

		//Odometry message
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		odom.pose.covariance = {
				0.01,  0.0,  0.0,  0.0,  0.0,  0.0,
				0.0,  0.01,  0.0,  0.0,  0.0,  0.0,
				0.0,   0.0, 0.01,  0.0,  0.0,  0.0,
				0.0,   0.0,  0.0,  0.1,  0.0,  0.0,
				0.0,   0.0,  0.0,  0.0,  0.1,  0.0,
				0.0,   0.0,  0.0,  0.0,  0.0,  0.1 };

		//set the velocity
		odom.child_frame_id = "base_footprint";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;
		odom.twist.covariance = {
				0.01,  0.0,  0.0,  0.0,  0.0,  0.0,
				0.0,  0.01,  0.0,  0.0,  0.0,  0.0,
				0.0,   0.0, 0.01,  0.0,  0.0,  0.0,
				0.0,   0.0,  0.0,  0.1,  0.0,  0.0,
				0.0,   0.0,  0.0,  0.0,  0.1,  0.0,
				0.0,   0.0,  0.0,  0.0,  0.0,  0.1 };

#if 0
		printf("motor_odoint L(%d,%d,%f) R=(%d,%d,%f) vx=%f vy=%f vth=%f x=%f y=%f v=(%f,%f,%f) th=%f\n”",
				odo_1_cntx,odo_1_cnt,v_left,
				odo_0_cntx,odo_0_cnt,v_right,
				vx,vy,
				vth,
				x,y,
				v_left,v_right,lengthBetweenTwoWheels,
				th);
#endif

		ROS_DEBUG("handleODO() dt=%f encoder=%d,%d,%d,%d position=%f,%f twist=%f,%f,%f ",
				dt,
				odo_0_cnt,odo_0_step,
				odo_1_cnt,odo_1_step,
				x,
				y,
				vx,
				vy,
				vth);

		//publish the message
		odom_pub.publish(odom);

		odo_last_time = current_time;

		odo_0_cnt = 0;
		odo_1_cnt = 0;
	}
}
#endif

/*
 *
 */
#ifdef ENABLE_SONAR_PUB
ros::Publisher sonar_pub;
ros::Publisher scan_pub;

void handleSonar()
{
	ros::Time current_time = ros::Time::now();
	double dt = (current_time - sonar_last_time).toSec();
	if(dt>=0.05)
	{
#ifdef HAVE_WIRINGPI
		const static float DIST_SCALE = 58.0;
		{

			sensor_msgs::Range range;
			range.header.frame_id = "sonar_link";
			range.radiation_type = sensor_msgs::Range::ULTRASOUND;
			range.min_range = 0.0;
			range.max_range = 6;
			range.field_of_view = 15.0/90.0 * 3.14/2.0;

			range.header.stamp = ros::Time::now();
			range.range = 0.01 * t_sonar / DIST_SCALE;

			sonar_pub.publish(range);
		}
#if 0
		{
#define num_readings 3
#define laser_frequency 10

			ros::Time scan_time = ros::Time::now();

			//populate the LaserScan message
			sensor_msgs::LaserScan scan;
			scan.header.stamp = scan_time;
			scan.header.frame_id = "laser_link";
			scan.angle_min = -(5.0/360.0 * 2.0*3.14);
			scan.angle_max = +(5.0/360.0 * 2.0*3.14);
		    scan.angle_increment = (10.0/360.0 * 2.0*3.14) / num_readings;
			scan.time_increment = (1 / laser_frequency) / (num_readings);
			scan.range_min = 0.0;
			scan.range_max = 100.0;

			scan.ranges.resize(num_readings);
			scan.intensities.resize(num_readings);
			for(unsigned int i = 0; i < num_readings; i++)
			{
				scan.ranges[i] = 0.01 * t_sonar / DIST_SCALE;
				scan.intensities[i] = 100;
			}

			scan_pub.publish(scan);
		}
#endif
#endif
	}
}
#endif

#if 0
void handle_TF(tf::TransformBroadcaster& tf_broadcaster)
{
	static ros::Time _time;
	ros::Time time = ros::Time::now();
	double dt = (time - _time).toSec();
	{
		if( dt >= 0.5 )
		{
#if 0
			<!--
			<node pkg="tf" type="static_transform_publisher" name="tf_00" args="0 0 0 0 0 0 /map /odom 1"/>
			<node pkg="tf" type="static_transform_publisher" name="tf_02" args="0 0 0 0 0 0 /base_footprint /base_link 1"/>
			<node pkg="tf" type="static_transform_publisher" name="tf_03" args="0.04 0.02 0.04 0 0 0 /base_link /imu_link 1"/>
			<node pkg="tf" type="static_transform_publisher" name="tf_04" args="0.08 0.01 0.04 0 0 0 /base_link /sonar_link 1"/>
			<node pkg="tf" type="static_transform_publisher" name="tf_05" args="0.08 0.01 0.04 0 0 0 /base_link /laser_link 1"/>
			<node pkg="tf" type="static_transform_publisher" name="tf_06" args="0.02 0.0 0.08 0 0 0 /base_link /camera_link 1"/>
			-->
#endif
			{
				geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

				geometry_msgs::TransformStamped odom_trans;
				odom_trans.header.stamp = current_time;
				odom_trans.header.frame_id = "odom";
				odom_trans.child_frame_id = "base_footprint";

				odom_trans.transform.translation.x = x;
				odom_trans.transform.translation.y = y;
				odom_trans.transform.translation.z = 0.0;
				odom_trans.transform.rotation = odom_quat;

				//send the transform
				tf_broadcaster.sendTransform(odom_trans);
			}


			_time = time;
		}
	}
}
#endif

static enum {
	LIDAR_DIR_UP=0,
	LIDAR_DIR_DOWN
} lidar_pwm_dir = LIDAR_DIR_UP;
#define GPIO_LIDAR_PWM_MAX 100
//#define lidar_pwm_value_min (int)((0.7/10.0)*GPIO_LIDAR_PWM_MAX)
//#define lidar_pwm_value_max (int)((2.9/10.0)*GPIO_LIDAR_PWM_MAX)
#define lidar_pwm_value_min (int)((0.50/10.0)*GPIO_LIDAR_PWM_MAX)
#define lidar_pwm_value_max (int)((2.0/10.0)*GPIO_LIDAR_PWM_MAX)
static int lidar_pwm_value = lidar_pwm_value_min + (lidar_pwm_value_max-lidar_pwm_value_min)/2;
static int lidar_pwm_step = 1;
#define lidar_num_readings (1 + lidar_pwm_value_max - lidar_pwm_value_min)
static float lidar_data[lidar_num_readings] = {0.0};
static int lidar_send_topic = 0;

VL53L0X* sensor = NULL;

void handle_lidar_servo()
{
#if 1
	static ros::Time _time;
	ros::Time time = ros::Time::now();
	double dt = (time - _time).toSec();
	if( dt >= 0.05 )
	{
		_time = time;

		if( scan_pub.getNumSubscribers() < 2 )
		{
			lidar_pwm_value = lidar_pwm_value_min + (lidar_pwm_value_max-lidar_pwm_value_min)/2;
			lidar_send_topic = 0;
		}
		else
		{
			{
				int i = lidar_pwm_value - lidar_pwm_value_min;
				lidar_data[i] = 0.001 * sensor->readRangeSingleMillimeters();
			}

			if( lidar_send_topic == 1 )
			{
				lidar_send_topic = 0;
				sensor_msgs::LaserScan lidar_scan;
				lidar_scan.ranges.resize(lidar_num_readings);
				lidar_scan.intensities.resize(lidar_num_readings);
				lidar_scan.header.stamp = time;
				lidar_scan.header.frame_id = "laser_link";
				lidar_scan.angle_min = -(2.0*3.14 * 80.0/360.0);
				lidar_scan.angle_max = +(2.0*3.14 * 80.0/360.0);
				lidar_scan.angle_increment = (lidar_scan.angle_max-lidar_scan.angle_min) / lidar_num_readings;
				lidar_scan.time_increment = 0.02;
				lidar_scan.range_min = 0.0;
				lidar_scan.range_max = 2.0;

				for( int i=0;i<lidar_num_readings; i++ )
				{
					lidar_scan.ranges[i]= lidar_data[i];
					//lidar_scan.ranges[i]= 0.5;
					if( lidar_scan.ranges[i] > 2.0 )
					{
						lidar_scan.intensities[i] = 0.0;
					}
					else
					{
						lidar_scan.intensities[i] = 1.0;
					}
				}
				scan_pub.publish(lidar_scan);
			}

			if( lidar_pwm_dir == LIDAR_DIR_UP )
			{
				lidar_pwm_value += lidar_pwm_step;
				if( lidar_pwm_value >= lidar_pwm_value_max )
				{
					lidar_pwm_value = lidar_pwm_value_max;
					lidar_pwm_dir = LIDAR_DIR_DOWN;
					lidar_send_topic = 1;
				}
			}
			else
			{
				lidar_pwm_value -= lidar_pwm_step;
				if( lidar_pwm_value <= lidar_pwm_value_min )
				{
					lidar_pwm_value = lidar_pwm_value_min;
					lidar_pwm_dir = LIDAR_DIR_UP;
					lidar_send_topic = 1;
				}
			}
		}

		/*
		 * next servo angle ...
		 */
		softPwmWrite (GPIO_LIDAR_PWM, lidar_pwm_value) ;
	}
#endif
}

/*
 * mySigintHandler
 */
void mySigintHandler(int sig)
{
	ROS_DEBUG("mySigintHandler ...");

	pinMode(GPIO_LIDAR_PWM,PWM_OUTPUT);
	softPwmCreate(GPIO_LIDAR_PWM, lidar_pwm_value, GPIO_LIDAR_PWM_MAX);

#ifdef HAVE_WIRINGPI
	{
		char s[256];
		ROS_DEBUG("initializing GPIOs ...");
		sprintf(s,"gpio export %d out",GPIO_LASER); system(s);
		sprintf(s,"gpio -g write %d 1",GPIO_LASER); system(s);

		sprintf(s,"gpio -g write %d 0",GPIO_PWML); system(s);
		sprintf(s,"gpio -g write %d 0",GPIO_IN0L); system(s);
		sprintf(s,"gpio -g write %d 0",GPIO_IN1L); system(s);

		sprintf(s,"gpio -g write %d 0",GPIO_PWMR); system(s);
		sprintf(s,"gpio -g write %d 0",GPIO_IN0R); system(s);
		sprintf(s,"gpio -g write %d 0",GPIO_IN1R); system(s);

		sprintf(s,"gpio -g write %d 0",GPIO_LASER); system(s);
	}
#endif
	ROS_DEBUG("mySigintHandler ... done");
	ros::shutdown();
}

/*
 *
 */
int main(int argc, char **argv)
{
	uint64_t rateTimer;
	uint64_t displayTimer;
	uint64_t now;

	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "raspirover" , ros::init_options::NoSigintHandler);
	signal(SIGINT, mySigintHandler);

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle node;

#ifdef HAVE_WIRINGPI
	{
		char s[256];
		ROS_DEBUG("initializing GPIOs ...");
		sprintf(s,"gpio export %d out",GPIO_LASER); system(s);
		sprintf(s,"gpio -g write %d 1",GPIO_LASER); system(s);
		sprintf(s,"gpio export %d in",GPIO_SONAR); system(s);
		sprintf(s,"gpio export %d in",GPIO_ODOL); system(s);
		sprintf(s,"gpio -g mode %d up",GPIO_ODOL); system(s);
		sprintf(s,"gpio export %d in",GPIO_ODOR); system(s);
		sprintf(s,"gpio -g mode %d up",GPIO_ODOR); system(s);
		sprintf(s,"gpio export %d out",GPIO_PWML); system(s);
		sprintf(s,"gpio export %d out",GPIO_IN0L); system(s);
		sprintf(s,"gpio export %d out",GPIO_IN1L); system(s);
		sprintf(s,"gpio export %d out",GPIO_PWMR); system(s);
		sprintf(s,"gpio export %d out",GPIO_IN0R); system(s);
		sprintf(s,"gpio export %d out",GPIO_IN1R); system(s);
		sprintf(s,"gpio export %d out",GPIO_TEST1); system(s);
		sprintf(s,"gpio export %d out",GPIO_TEST2); system(s);
		sprintf(s,"gpio export %d out",GPIO_TEST3); system(s);

		sprintf(s,"gpio export %d out",GPIO_LIDAR_PWM); system(s);
		sprintf(s,"gpio export %d in",GPIO_LIDAR_GPIO1); system(s);
		sprintf(s,"gpio export %d out",GPIO_LIDAR_XSHUT); system(s);

		sprintf(s,"gpio -g write %d 0",GPIO_LASER); system(s);
	}

	wiringPiSetupSys();

	piHiPri(99);

	pwmSetMode(PWM_MODE_MS);

	pwmSetRange(1024);
	pwmSetClock(100);

	sensor = new VL53L0X(GPIO_LIDAR_XSHUT);
	sensor->init();
	sensor->setTimeout(200);

	// Lower the return signal rate limit (default is 0.25 MCPS)
	sensor->setSignalRateLimit(0.1);
	// Increase laser pulse periods (defaults are 14 and 10 PCLKs)
	sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
	sensor->setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

	// Reduce timing budget to 20 ms (default is about 33 ms)
	sensor->setMeasurementTimingBudget(20000);

	/*
	 * enable VL53L0X
	 */
	pinMode(GPIO_LIDAR_XSHUT,OUTPUT);
	digitalWrite (GPIO_LIDAR_XSHUT, HIGH);

	pinMode(GPIO_LIDAR_PWM,PWM_OUTPUT);
	softPwmCreate(GPIO_LIDAR_PWM, lidar_pwm_value, GPIO_LIDAR_PWM_MAX);

	pinMode(GPIO_PWML,PWM_OUTPUT);
	softPwmCreate(GPIO_PWML, 0, 100);

	pinMode(GPIO_PWMR,PWM_OUTPUT);
	softPwmCreate (GPIO_PWMR, 0, 100);

	pinMode(GPIO_ODOL ,INPUT);
	pullUpDnControl(GPIO_ODOL ,PUD_UP);
	wiringPiISR(GPIO_ODOL, INT_EDGE_BOTH, &motor_odoint_0);

	pinMode(GPIO_ODOR ,INPUT);
	pullUpDnControl(GPIO_ODOR ,PUD_UP);
	wiringPiISR(GPIO_ODOR, INT_EDGE_BOTH, &motor_odoint_1);

	pinMode(GPIO_SONAR ,INPUT);
	wiringPiISR(GPIO_SONAR, INT_EDGE_BOTH, &sonarint);
#endif    

	ROS_DEBUG("initializing cmd_vel ...");
	ros::Subscriber sub = node.subscribe("cmd_vel", 10, cmd_vel_callback);

	sonar_last_time = ros::Time::now();
#ifdef ENABLE_SONAR_PUB
	sonar_pub = node.advertise<sensor_msgs::Range>("sonar", 10);
	scan_pub = node.advertise<sensor_msgs::LaserScan>("scan", 10);
#endif

	odo_last_time = ros::Time::now();
#ifdef ENABLE_ODOM_PUB
	odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);
#endif

	ros::Rate loop_rate(100);
	tf::TransformBroadcaster tf_broadcaster;
	while( node.ok() )
	{
		//handle_odoint();
		//handle_TF(tf_broadcaster);
		handle_lidar_servo();

#ifdef ENABLE_SONAR_PUB
		handleSonar();
#endif
#ifdef ENABLE_ODOM_PUB
		handleODO(tf_broadcaster);
#endif
		ros::spinOnce();

		digitalWrite (GPIO_TEST1, HIGH);
		loop_rate.sleep();
		digitalWrite (GPIO_TEST1, LOW);
	}

	return 0;
}

/*
 * EOF
 */
