/*
 * raspirover/generic RPI motor/odom node
 */

#include <math.h>
#include <signal.h>

/*
 * wiringPI
 */
#include <wiringPi.h>
#include <softPwm.h>

/*
 * ROS
 */
#include <ros/console.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "std_msgs/Float32.h"

/*
 * local includes
 */
#include "pid.h"

/*
 * some defines
 */
#define MOTOR_L 0
#define MOTOR_R 1

#define ODOM_PERIOD 0.05

/*
 * publisher
 */
ros::Publisher motor_pub_setpoint[2];
ros::Publisher motor_pub_value[2];
ros::Publisher motor_pub_rate[2];
ros::Publisher motor_pub_odomcnt[2];
ros::Publisher odom_pub;

/*
 * last odom timestamp
 */
ros::Time odo_last_time;

/*
 * odom state
 */
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

/*
 * motor data
 */
struct _motor_md_
{
	MiniPID 	pid;
	double 		setpoint;
	double  	value;
	double  	odom_rate;

	int         odom_step;
	int         odom_cnt;
	int         odom_cnt_forever;
} motor_md[2] = {
		{ MiniPID(0,0,0),0.0, 0.0, 0.0, 0, 0, 0 },
		{ MiniPID(0,0,0),0.0, 0.0, 0.0, 0, 0, 0 } };

/*
 * ROS parameter
 */
bool param_updateparam = false;
double pid_kp = 0.0;
double pid_kd = 0.0;
double pid_ki = 0.0;
double pid_of = 0.0;

double param_distancepercount = 0.0;
double param_widthbetweenwheels = 0.0;

double param_pid_test_period = 0.0;
double param_pid_test_speedA = 0.0;
double param_pid_test_speedB = 0.0;

/*
 * GPIO pins
 */
int gpio_odol = 6;		// ODOM input left
int gpio_odor = 5;		// ODOM input right

int gpio_pwmr = 19;		// motor PWM output right
int gpio_in0r = 26;		// motor IN0 output right
int gpio_in1r = 13;		// motor IN1 output right

int gpio_pwml = 21;		// motor PWM output left
int gpio_in0l = 20;		// motor IN0 output left
int gpio_in1l = 16;		// motor IN0 output left

/*
 * motor control function
 */
void motor_ctrl( int gpio_en, int gpio_in0, int gpio_in1 , int vel )
{
	if( vel == 0 )
	{
		pinMode(gpio_in0,INPUT);
		pinMode(gpio_in1,INPUT);

		softPwmWrite (gpio_en, vel) ;
		digitalWrite (gpio_in0, HIGH);
		digitalWrite (gpio_in1, HIGH);
	}
	else if( vel > 0 )
	{
		pinMode(gpio_in0,OUTPUT);
		pinMode(gpio_in1,OUTPUT);

		softPwmWrite (gpio_en, vel) ;
		digitalWrite (gpio_in0, HIGH);
		digitalWrite (gpio_in1, LOW);
	}
	else if ( vel < 0 )
	{
		pinMode(gpio_in0,OUTPUT);
		pinMode(gpio_in1,OUTPUT);

		softPwmWrite (gpio_en, -vel) ;
		digitalWrite (gpio_in0, LOW);
		digitalWrite (gpio_in1, HIGH);
	}
}

/*
 * motor control function
 */
void motor( int m, double vel )
{
	switch(m)
	{
	case MOTOR_L:
		motor_ctrl(gpio_pwml,gpio_in0l,gpio_in1l,vel);
		if( vel > 0.0 )
			motor_md[MOTOR_L].odom_step = 1;
		else if( vel < 0.0 )
			motor_md[MOTOR_L].odom_step = -1;
		else
			motor_md[MOTOR_L].odom_step = 0;
		break;
	case MOTOR_R:
		motor_ctrl(gpio_pwmr,gpio_in0r,gpio_in1r,vel);
		if( vel > 0.0 )
			motor_md[MOTOR_R].odom_step = 1;
		else if( vel < 0.0 )
			motor_md[MOTOR_R].odom_step = -1;
		else
			motor_md[MOTOR_R].odom_step = 0;
		break;
	}

}

/*
 * ODOM interrupt for left motor
 */
//int motor_odoint_L_cnt = 0;
void motor_odoint_L(void)
{
	//printf("motor_odoint_L %d\n",motor_odoint_L_cnt++);
	motor_md[MOTOR_L].odom_cnt += motor_md[MOTOR_L].odom_step;
	motor_md[MOTOR_L].odom_cnt_forever += motor_md[MOTOR_L].odom_step;
}

/*
 * ODOM interrupt for right motor
 */
//int motor_odoint_R_cnt = 0;
void motor_odoint_R(void)
{
	//printf("motor_odoint_R %d\n",motor_odoint_R_cnt++);
	motor_md[MOTOR_R].odom_cnt += motor_md[MOTOR_R].odom_step;
	motor_md[MOTOR_R].odom_cnt_forever += motor_md[MOTOR_R].odom_step;
}

/*
 * called by other ROS nodes
 */
void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{
	double x = (double)1.0*vel_cmd.linear.x;
	//double y = (double)1.0*vel_cmd.linear.y;
	double y = (double)1.0*vel_cmd.angular.z;
	x = (x>1.0)?1.0:((x<-1.0)?-1.0:x);
	y = (y>1.0)?1.0:((y<-1.0)?-1.0:y);
	if((fabs(x)+fabs(y))>1.0)
	{
		double k = (fabs(x)+fabs(y));
		x = x/k;
		y = y/k;
	}

	int motor_r = (int)(100.0*x)+(100.0*(y));
	int motor_l = (int)(100.0*x)-(100.0*(y));

	ROS_DEBUG("cmd_vel_callback(%f,%f) x=%1.3f y=%1.3f motor-l=%d motor-r=%d",
			vel_cmd.linear.x,vel_cmd.angular.y,
			x,y,
			motor_l,motor_r);

	if( motor_l > 0 )
	{
		motor_md[MOTOR_L].setpoint = 20.0 + motor_l;
	}
	else if( motor_l < 0 )
	{
		motor_md[MOTOR_L].setpoint = motor_l - 20.0;
	}
	else
	{
		motor_md[MOTOR_L].setpoint = 0.0;
	}

	if( motor_r > 0 )
	{
		motor_md[MOTOR_R].setpoint = 20.0 + motor_r;
	}
	else if( motor_r < 0 )
	{
		motor_md[MOTOR_R].setpoint = motor_r - 20.0;
	}
	else
	{
		motor_md[MOTOR_R].setpoint = 0.0;
	}
}

/*
 * ODOM handler function
 */
void handleODOM(ros::NodeHandle& node)
{
	double XXX = (0.1 / 50.0);

	ros::Time current_time = ros::Time::now();
	double dt = (current_time - odo_last_time).toSec();
	if(dt>=ODOM_PERIOD)
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
		double DistancePerCount = param_distancepercount;
		double lengthBetweenTwoWheels = param_widthbetweenwheels;

		//extract the wheel velocities from the tick signals count
		double deltaLeft = motor_md[MOTOR_L].odom_cnt;
		double deltaRight = motor_md[MOTOR_R].odom_cnt;

		double v_left = (deltaLeft * DistancePerCount) / dt;
		double v_right = (deltaRight * DistancePerCount) / dt;

		//motor_md[MOTOR_L].odom_rate = v_left;
		//motor_md[MOTOR_R].odom_rate = v_right;

		motor_md[MOTOR_L].odom_rate = (4.0 * motor_md[MOTOR_L].odom_rate + v_left) / 5.0;
		motor_md[MOTOR_R].odom_rate = (4.0 * motor_md[MOTOR_R].odom_rate + v_right) / 5.0;

		//motor_md[MOTOR_L].odom_rate = (19.0 * motor_md[MOTOR_L].odom_rate + v_left) / 20.0;
		//motor_md[MOTOR_R].odom_rate = (19.0 * motor_md[MOTOR_R].odom_rate + v_right) / 20.0;

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

#if 1
		{
			static tf::TransformBroadcaster tf_broadcaster;

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
#endif

		//Odometry message
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		//odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		double max = 1000000000000.0;
		double min = 0.001;
		odom.pose.covariance = {
				min,    0.0, 0.0,  0.0,  0.0,  0.0,
				0.0,    min, 0.0,  0.0,  0.0,  0.0,
				0.0,    0.0, max,  0.0,  0.0,  0.0,
				0.0,    0.0, 0.0,  max,  0.0,  0.0,
				0.0,    0.0, 0.0,  0.0,  max,  0.0,
				0.0,    0.0, 0.0,  0.0,  0.0,  max };

		//set the velocity
		//odom.child_frame_id = "base_footprint";
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = vy;
		odom.twist.twist.angular.z = vth;
		odom.twist.covariance = {
				min,     0.0,  0.0,  0.0,  0.0,  0.0,
				0.0,     min,  0.0,  0.0,  0.0,  0.0,
				0.0,     0.0,  max,  0.0,  0.0,  0.0,
				0.0,     0.0,  0.0,  max,  0.0,  0.0,
				0.0,     0.0,  0.0,  0.0,  max,  0.0,
				0.0,     0.0,  0.0,  0.0,  0.0,  max };

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
				motor_md[MOTOR_R].odom_cnt,motor_md[MOTOR_R].odom_step,
				motor_md[MOTOR_L].odom_cnt,motor_md[MOTOR_L].odom_step,
				x,
				y,
				vx,
				vy,
				vth);

		//publish the message
		odom_pub.publish(odom);

		odo_last_time = current_time;

		if( pid_kp==0.0 && pid_ki==0.0 && pid_kd==0.0 )
		{
			/*
			 * no PID mode
			 */
			if( param_pid_test_period != 0.0 )
			{
				if( fmod(current_time.toSec(),param_pid_test_period) < (param_pid_test_period/2.0) )
				{
					motor_md[MOTOR_L].setpoint = param_pid_test_speedA;
					motor_md[MOTOR_R].setpoint = param_pid_test_speedA;
				}
				else
				{
					motor_md[MOTOR_L].setpoint = param_pid_test_speedB;
					motor_md[MOTOR_R].setpoint = param_pid_test_speedB;
				}
			}

			motor(MOTOR_L,motor_md[MOTOR_L].setpoint);
			motor(MOTOR_R,motor_md[MOTOR_R].setpoint);
		}
		else
		{
			/*
			 * PID mode
			 */
			if( param_pid_test_period != 0.0 )
			{
				if( fmod(current_time.toSec(),param_pid_test_period) < (param_pid_test_period/2.0) )
				{
					motor_md[MOTOR_L].setpoint = param_pid_test_speedA;
					motor_md[MOTOR_R].setpoint = param_pid_test_speedA;
				}
				else
				{
					motor_md[MOTOR_L].setpoint = param_pid_test_speedB;
					motor_md[MOTOR_R].setpoint = param_pid_test_speedB;
				}
			}

			motor_md[MOTOR_L].value = motor_md[MOTOR_L].pid.getOutput(
					motor_md[MOTOR_L].setpoint,
					motor_md[MOTOR_L].odom_rate/XXX);
			motor(MOTOR_L,motor_md[MOTOR_L].value);

			motor_md[MOTOR_R].value = motor_md[MOTOR_R].pid.getOutput(
					motor_md[MOTOR_R].setpoint,
					motor_md[MOTOR_R].odom_rate/XXX);
			motor(MOTOR_R,motor_md[MOTOR_R].value);

			printf("motor_handle() L(%d,%d,set=%f,ist=%f,m=%f) R(%d,%d,set=%f,ist=%f,m=%f) pid(%f,%f,%f,%f)\n",
					motor_md[MOTOR_L].odom_step,
					motor_md[MOTOR_L].odom_cnt,
					motor_md[MOTOR_L].setpoint,
					motor_md[MOTOR_L].odom_rate/XXX,
					motor_md[MOTOR_L].value,
					motor_md[MOTOR_R].odom_step,
					motor_md[MOTOR_R].odom_cnt,
					motor_md[MOTOR_R].setpoint,
					motor_md[MOTOR_R].odom_rate/XXX,
					motor_md[MOTOR_R].value,
					pid_kp,pid_ki,pid_kd,pid_of);
		}

		/*
		 * publish diagnostic data
		 */
		{
			std_msgs::Float32 msg;

			msg.data = motor_md[MOTOR_L].setpoint;
			motor_pub_setpoint[MOTOR_L].publish(msg);
			msg.data = motor_md[MOTOR_R].setpoint;
			motor_pub_setpoint[MOTOR_R].publish(msg);

			msg.data = motor_md[MOTOR_L].value;
			motor_pub_value[MOTOR_L].publish(msg);
			msg.data = motor_md[MOTOR_R].value;
			motor_pub_value[MOTOR_R].publish(msg);

			msg.data = motor_md[MOTOR_L].odom_rate/XXX;
			motor_pub_rate[MOTOR_L].publish(msg);
			msg.data = motor_md[MOTOR_R].odom_rate/XXX;
			motor_pub_rate[MOTOR_R].publish(msg);

			msg.data = motor_md[MOTOR_L].odom_cnt_forever;
			motor_pub_odomcnt[MOTOR_L].publish(msg);
			msg.data = motor_md[MOTOR_R].odom_cnt_forever;
			motor_pub_odomcnt[MOTOR_R].publish(msg);

		}

		motor_md[MOTOR_L].odom_cnt = 0;
		motor_md[MOTOR_R].odom_cnt = 0;
	}
}

/*
 * my SIGINT Handler
 */
void sigintHandler(int sig)
{
	ROS_DEBUG("mySigintHandler ...");

	/*
	 * halt motors
	 */
	motor(MOTOR_L,0);
	motor(MOTOR_R,0);

	{
		char s[256];
		ROS_DEBUG("shutdown GPIOs ...");

		if((gpio_pwml != 0)&&(gpio_in0l != 0)&&(gpio_in1l != 0))
		{
			sprintf(s,"gpio -g write %d 0",gpio_pwml); system(s);
			sprintf(s,"gpio -g write %d 0",gpio_in0l); system(s);
			sprintf(s,"gpio -g write %d 0",gpio_in1l); system(s);
		}

		if((gpio_pwmr != 0)&&(gpio_in0r != 0)&&(gpio_in1r != 0))
		{
			sprintf(s,"gpio -g write %d 0",gpio_pwmr); system(s);
			sprintf(s,"gpio -g write %d 0",gpio_in0r); system(s);
			sprintf(s,"gpio -g write %d 0",gpio_in1r); system(s);
		}
	}
	ROS_DEBUG("mySigintHandler ... done");
	ros::shutdown();
}

/*
 * main
 */
int main(int argc, char **argv)
{
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
	ros::init(argc, argv, "rpi_motor_odom_driver_node" , ros::init_options::NoSigintHandler);

	/*
	 * install SIGINT handler
	 */
	signal(SIGINT, sigintHandler);

	/**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle node("rpi_motor_odom_driver_node");

	{
		char s[256];
		ROS_DEBUG("initializing GPIOs ...");
		if((gpio_odol != 0)&&(gpio_odor != 0))
		{
			sprintf(s,"gpio export %d in",gpio_odol); system(s);
			sprintf(s,"gpio -g mode %d up",gpio_odol); system(s);
			sprintf(s,"gpio export %d in",gpio_odor); system(s);
			sprintf(s,"gpio -g mode %d up",gpio_odor); system(s);
		}
		if((gpio_pwml != 0)&&(gpio_in0l != 0)&&(gpio_in1l != 0))
		{
			sprintf(s,"gpio export %d out",gpio_pwml); system(s);
			sprintf(s,"gpio export %d out",gpio_in0l); system(s);
			sprintf(s,"gpio export %d out",gpio_in1l); system(s);
		}
		if((gpio_pwmr != 0)&&(gpio_in0r != 0)&&(gpio_in1r != 0))
		{
			sprintf(s,"gpio export %d out",gpio_pwmr); system(s);
			sprintf(s,"gpio export %d out",gpio_in0r); system(s);
			sprintf(s,"gpio export %d out",gpio_in1r); system(s);
		}
	}

	wiringPiSetupSys();

	piHiPri(99);

	pwmSetMode(PWM_MODE_MS);

	pwmSetRange(1024);
	pwmSetClock(100);

	node.getParam("odom/distancepercount", param_distancepercount);
	node.getParam("odom/widthbetweenwheels", param_widthbetweenwheels);

	/*
	 * motor GPIO params
	 */
	node.getParam("gpio/gpio_in_odol", gpio_odol);
	node.getParam("gpio/gpio_in_odor", gpio_odor);
	node.getParam("gpio/gpio_out_pwml", gpio_pwml);
	node.getParam("gpio/gpio_out_in0l", gpio_in0l);
	node.getParam("gpio/gpio_out_in1l", gpio_in1l);
	node.getParam("gpio/gpio_out_pwmr", gpio_pwmr);
	node.getParam("gpio/gpio_out_in0r", gpio_in0r);
	node.getParam("gpio/gpio_out_in1r", gpio_in1r);

	/*
	 * motor PWM wiringPI configuration
	 */
	if((gpio_pwml != 0)&&(gpio_in0l != 0)&&(gpio_in1l != 0))
	{
		pinMode(gpio_pwml,PWM_OUTPUT);
		softPwmCreate(gpio_pwml, 0, 100);
	}

	if((gpio_pwmr != 0)&&(gpio_in0r != 0)&&(gpio_in1r != 0))
	{
		pinMode(gpio_pwmr,PWM_OUTPUT);
		softPwmCreate (gpio_pwmr, 0, 100);
	}

	/*
	 * motor ODOM wiringPI configuration
	 */
	if((gpio_odol != 0)&&(gpio_odor != 0))
	{
		pinMode(gpio_odol ,INPUT);
		pullUpDnControl(gpio_odol ,PUD_UP);
		wiringPiISR(gpio_odol, INT_EDGE_BOTH, &motor_odoint_L);

		pinMode(gpio_odor ,INPUT);
		pullUpDnControl(gpio_odor ,PUD_UP);
		wiringPiISR(gpio_odor, INT_EDGE_BOTH, &motor_odoint_R);
	}


	/*
	 * global update param flag
	 */
	node.getParam("updateparam", param_updateparam);

	/*
	 * motor PID parameter
	 */
	node.getParam("pid/Kp", pid_kp);
	node.getParam("pid/Ki", pid_ki);
	node.getParam("pid/Kd", pid_kd);
	node.getParam("pid/of", pid_of);

	motor_md[MOTOR_L].pid.setOutputLimits(-100.0,100.0);
	//motor_md[MOTOR_L].pid.setOutputRampRate(10.0);
	motor_md[MOTOR_L].pid.setPID(pid_kp,pid_ki,pid_kd);
	if( pid_of != 0.0 )
	{
		motor_md[MOTOR_L].pid.setOutputFilter(pid_of);
	}
	motor_md[MOTOR_L].pid.setDirection(true);


	motor_md[MOTOR_R].pid.setOutputLimits(-100.0,100.0);
	//motor_md[MOTOR_R].pid.setOutputRampRate(10.0);
	motor_md[MOTOR_R].pid.setPID(pid_kp,pid_ki,pid_kd);
	if( pid_of != 0.0 )
	{
		motor_md[MOTOR_R].pid.setOutputFilter(pid_of);
	}
	motor_md[MOTOR_R].pid.setDirection(true);

	/*
	 * initialize motor subscriber ...
	 */
	ros::Subscriber sub = node.subscribe("cmd_vel", 10, cmd_vel_callback);

	/*
	 * initialize motor publisher ...
	 */
	motor_pub_setpoint[MOTOR_L] = node.advertise<std_msgs::Float32>("motor/L/setpoint", 2);
	motor_pub_setpoint[MOTOR_R] = node.advertise<std_msgs::Float32>("motor/R/setpoint", 2);
	motor_pub_value[MOTOR_L] = node.advertise<std_msgs::Float32>("motor/L/value", 2);
	motor_pub_value[MOTOR_R] = node.advertise<std_msgs::Float32>("motor/R/value", 2);
	motor_pub_rate[MOTOR_L] = node.advertise<std_msgs::Float32>("motor/L/rate", 2);
	motor_pub_rate[MOTOR_R] = node.advertise<std_msgs::Float32>("motor/R/rate", 2);
	motor_pub_odomcnt[MOTOR_L] = node.advertise<std_msgs::Float32>("motor/L/odomcnt", 2);
	motor_pub_odomcnt[MOTOR_R] = node.advertise<std_msgs::Float32>("motor/R/odomcnt", 2);

	/*
	 * initialize odom publisher ...
	 */
	odo_last_time = ros::Time::now();
	odom_pub = node.advertise<nav_msgs::Odometry>("odom", 10);

	ros::Rate loop_rate(1/ODOM_PERIOD);
	while( node.ok() )
	{
		handleODOM(node);

		if( param_updateparam==true )
		{
			node.getParam("updateparam", param_updateparam);

			node.getParam("odom/distancepercount", param_distancepercount);
			node.getParam("odom/widthbetweenwheels", param_widthbetweenwheels);

			node.getParam("pid/Kp", pid_kp);
			node.getParam("pid/Ki", pid_ki);
			node.getParam("pid/Kd", pid_kd);
			node.getParam("pid/of", pid_of);
			motor_md[MOTOR_L].pid.setPID(pid_kp,pid_ki,pid_kd);
			if( pid_of != 0.0 )
			{
				motor_md[MOTOR_L].pid.setOutputFilter(pid_of);
			}
			motor_md[MOTOR_R].pid.setPID(pid_kp,pid_ki,pid_kd);
			if( pid_of != 0.0 )
			{
				motor_md[MOTOR_R].pid.setOutputFilter(pid_of);
			}

			node.getParam("pid_test/period", param_pid_test_period);
			node.getParam("pid_test/speedA", param_pid_test_speedA);
			node.getParam("pid_test/speedB", param_pid_test_speedB);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

/*
 * EOF
 */
