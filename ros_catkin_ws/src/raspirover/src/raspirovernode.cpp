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

/*
 * global data
 */
ros::Time odo_last_time;

static struct
{
	int update_flag;
	double x;
	double y;
	double th;

	double vx;
	double vy;
	double vth;
} odo = {
		0,
		0.0,
		0.0,
		0.0,
		0.0,
		-0.1,
		0.1 };

ros::Publisher odom_pub;

int odo_0_step = 0;
int odo_0_cnt = 0;

int odo_1_step = 0;
int odo_1_cnt = 0;

/*
 *
 */
unsigned long uNow()
{
	struct timeval tv;
	gettimeofday(&tv,NULL);
	return tv.tv_usec + 1000000UL * tv.tv_sec;
}

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
	odo_0_cnt += odo_0_step;
	odo.update_flag = 1;
	return;
}

/*
 *
 */
void motor_odoint_1(void)
{
	odo_1_cnt += odo_1_step;
	odo.update_flag = 1;
	return;
}

/*
 *
 */
unsigned long t_sonar = 0;
void sonarint(void)
{
	static unsigned long _t = 0;
	unsigned long t = uNow();
	if( _t != 0 )
	{
		if( digitalRead(GPIO_SONAR) == 0 )
		{
			t_sonar = ((9*t_sonar)+(t-_t))/10;
		}
	}
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
	if(( odo.update_flag == 1 )&&(dt>=0.1))
	{
		/*
		 * https://answers.ros.org/question/207392/generating-odom-message-from-encoder-ticks-for-robot_pose_ekf/
		 * http://www.seattlerobotics.org/encoder/200610/Article3/IMU%20Odometry,%20by%20David%20Anderson.htm
		 */
		static double x = 0;
		static double y = 0;
		static double th = 0;

		double DistancePerCount = 0.6/17.0;
		double lengthBetweenTwoWheels = 0.1;

		//extract the wheel velocities from the tick signals count
		double deltaLeft = odo_0_cnt;
		double deltaRight = odo_1_cnt;

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
		odom_trans.child_frame_id = "base_link";

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

		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;

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
		odo.update_flag = 0;
	}
}
#endif

/*
 *
 */
#ifdef ENABLE_SONAR_PUB
ros::Publisher sonar_pub;
void handleSonar()
{
#ifdef HAVE_WIRINGPI
	const static float MAX_DISTANCE = 30;
	const static float DIST_SCALE = 58.0;
	const static float TRAVEL_TIME_MAX = MAX_DISTANCE * DIST_SCALE;

	sensor_msgs::Range range;
	range.header.frame_id = "sonar_link";
	range.radiation_type = sensor_msgs::Range::ULTRASOUND;
	range.min_range = 0.0;
	range.max_range = MAX_DISTANCE;

	range.header.stamp = ros::Time::now();
	range.range = t_sonar / DIST_SCALE;

	sonar_pub.publish(range);
#endif
}
#endif

/*
 * mySigintHandler
 */
void mySigintHandler(int sig)
{
	ROS_DEBUG("mySigintHandler ...");
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
		sprintf(s,"gpio -g write %d 0",GPIO_LASER); system(s);
	}

	wiringPiSetupSys();

	pwmSetMode(PWM_MODE_MS);

	pinMode(GPIO_PWML,PWM_OUTPUT);
	softPwmCreate(GPIO_PWML, 0, 100);

	pinMode(GPIO_ODOL ,INPUT);
	pullUpDnControl(GPIO_ODOL ,PUD_UP);
	wiringPiISR(GPIO_ODOL, INT_EDGE_FALLING, &motor_odoint_0);

	pinMode(GPIO_PWMR,PWM_OUTPUT);
	softPwmCreate (GPIO_PWMR, 0, 100);

	pinMode(GPIO_ODOR ,INPUT);
	pullUpDnControl(GPIO_ODOR ,PUD_UP);
	wiringPiISR(GPIO_ODOR, INT_EDGE_FALLING, &motor_odoint_1);

	pinMode(GPIO_SONAR ,INPUT);
	wiringPiISR(GPIO_SONAR, INT_EDGE_BOTH, &sonarint);
#endif    

	ROS_DEBUG("initializing cmd_vel ...");
	ros::Subscriber sub = node.subscribe("cmd_vel", 10, cmd_vel_callback);

	//ros::Subscriber sub_laser = node.subscribe("laser", 10, laser_callback);

#ifdef ENABLE_SONAR_PUB
	sonar_pub = node.advertise<sensor_msgs::Range>("sonar", 10);
#endif

	odo_last_time = ros::Time::now();
#ifdef ENABLE_ODOM_PUB
	odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);
#endif

	ros::Rate loop_rate(10);
	tf::TransformBroadcaster tf_broadcaster;
	while( node.ok() )
	{
		tf_broadcaster.sendTransform(
				tf::StampedTransform(
						tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
						ros::Time::now(),"map", "odom"));
		tf_broadcaster.sendTransform(
				tf::StampedTransform(
						tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
						ros::Time::now(),"odom", "base_footprint"));
		tf_broadcaster.sendTransform(
				tf::StampedTransform(
						tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
						ros::Time::now(),"base_footprint", "base_link"));
		tf_broadcaster.sendTransform(
				tf::StampedTransform(
						tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.1, 0.1)),
						ros::Time::now(),"base_link", "imu_link"));
		tf_broadcaster.sendTransform(
				tf::StampedTransform(
						tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.2, 0.1)),
						ros::Time::now(),"base_link", "sonar_link"));
		tf_broadcaster.sendTransform(
				tf::StampedTransform(
						tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.2, 0.2)),
						ros::Time::now(),"base_link", "camera_link"));

#ifdef ENABLE_SONAR_PUB
		handleSonar();
#endif
#ifdef ENABLE_ODOM_PUB
		handleODO(tf_broadcaster);
#endif
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

/*
 * EOF
 */
