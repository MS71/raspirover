#include <stdlib.h>
#include <stdio.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <sys/time.h>
#include <math.h>

#include <wiringPiI2C.h>

#undef  ENABLE_hmc5883l
//https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/blob/master/examples/MPU9250BasicAHRS_I2C/MPU9250BasicAHRS_I2C.ino
#define ENABLE_mpu9250

#ifdef ENABLE_mpu9250
#include "MPU9250.h"
#include "quaternionFilters.h"
#endif

#ifdef ENABLE_hmc5883l
#include "hmc5883l.h"
#endif

#define PI 3.14159265

#define GPIO_SONAR 4

#define GPIO_ODOL  5
#define GPIO_ODOR  6

#define GPIO_PWML  19
#define GPIO_IN0L  26
#define GPIO_IN1L  13

#define GPIO_PWMR  21
#define GPIO_IN0R  20
#define GPIO_IN1R  16

#ifdef ENABLE_mpu9250
MPU9250 mpu9250;
#endif

unsigned long uNow()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_usec + 1000000UL * tv.tv_sec;
}

void motor_ctrl( int gpio_en, int gpio_in0, int gpio_in1 , int vel )
{
    printf("motor_ctrl(%d,%d,%d,%d)\n",gpio_en,gpio_in0,gpio_in1);
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

void motor( int m, int vel )
{
    switch(m)	
    {
        case 0: motor_ctrl(GPIO_PWML,GPIO_IN0L,GPIO_IN1L,vel); break;
        case 1: motor_ctrl(GPIO_PWMR,GPIO_IN0R,GPIO_IN1R,vel); break;
    }
    
}

double odo_0_step = 0;
double odo_0_cnt = 0;
void motor_odoint_0(void)
{
    static unsigned long _t = 0;
    unsigned long t = uNow();
    if( _t != 0 )
    {
  //      printf("motor_odoint_0 t=%ld us\n",t-_t);
        odo_0_cnt += odo_0_step;
    }
    
    _t = t;
    return;
}

#define ODO_SPEP (100.0/28.0)

double odo_1_step = 0;
double odo_1_cnt = 0;
void motor_odoint_1(void)
{
    static unsigned long _t = 0;
    unsigned long t = uNow();
    if( _t != 0 )
    {
//        printf("motor_odoint_1 t=%ld us\n",t-_t);
        odo_1_cnt += odo_1_step;
    }
    
    _t = t;
    return;
}

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

void motor_init()
{
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
}

int main (void)
{
    {
        char s[256];
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
    }
                                                
    wiringPiSetupSys();
    //piHiPri(99);

    // Open an I2C connection
#ifdef ENABLE_hmc5883l
    int fd = wiringPiI2CSetup(HMC5883L_ADDRESS);
    printf("HMC5883L id=%c%c%c\n",
    wiringPiI2CReadReg8(fd, HMC5883L_REG_ID_A),
    wiringPiI2CReadReg8(fd, HMC5883L_REG_ID_B),
    wiringPiI2CReadReg8(fd, HMC5883L_REG_ID_C));
#endif

#ifdef ENABLE_mpu9250
    printf("WHO_AM_I_MPU9250=0x%02x (0x71)",
	mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250));

    // Start by performing self test and reporting values
    mpu9250.MPU9250SelfTest(mpu9250.SelfTest);
    printf("acceleration trim within : %f,%f,%f\n",
	mpu9250.SelfTest[0],mpu9250.SelfTest[1],mpu9250.SelfTest[2]);
    printf("gyration trim within : %f,%f,%f\n",
	mpu9250.SelfTest[3],mpu9250.SelfTest[4],mpu9250.SelfTest[5]);    

    // Calibrate gyro and accelerometers, load biases in bias registers
    mpu9250.calibrateMPU9250(mpu9250.gyroBias, mpu9250.accelBias);	

     mpu9250.initMPU9250();

    printf("WHO_AM_I_MPU9250=0x%02x (0x48)",
	mpu9250.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250));

    // Get magnetometer calibration from AK8963 ROM
    mpu9250.initAK8963(mpu9250.magCalibration);

#endif

#if 0
    // Perform I2C self test
    wiringPiI2CWriteReg8(fd, HCM5883L_REG_CONFIG_A, 0x00);
    wiringPiI2CWriteReg8(fd, HCM5883L_REG_CONFIG_B, 0x01);
    wiringPiI2CWriteReg8(fd, HMC5883L_REG_MODE, 0x02);
    delay(6);
    while(1)
    {
        unsigned char msb = wiringPiI2CReadReg8(fd, HMC5883L_REG_MSB_X);
        unsigned char lsb = wiringPiI2CReadReg8(fd, HMC5883L_REG_LSB_X);
        short x = msb << 8 | lsb;

        msb = wiringPiI2CReadReg8(fd, HMC5883L_REG_MSB_Y);
        lsb = wiringPiI2CReadReg8(fd, HMC5883L_REG_LSB_Y);
        short y = msb << 8 | lsb;

        msb = wiringPiI2CReadReg8(fd, HMC5883L_REG_MSB_Z);
        lsb = wiringPiI2CReadReg8(fd, HMC5883L_REG_LSB_Z);
        short z = msb << 8 | lsb;
        delay(67);
        printf("%d %d %d\n",x,y,z);
    }
#endif
                
#ifdef ENABLE_hmc5883l
    // Perform I2C work
    wiringPiI2CWriteReg8(fd, HCM5883L_REG_CONFIG_A, 0x70);
    wiringPiI2CWriteReg8(fd, HCM5883L_REG_CONFIG_B, (3<<5) | 1);
    wiringPiI2CWriteReg8(fd, HMC5883L_REG_MODE, HMC5883L_MODE_CONTINUOUS);
#endif
        
    pinMode(GPIO_SONAR ,INPUT); 
    wiringPiISR(GPIO_SONAR, INT_EDGE_BOTH, &sonarint);

    motor_init();

#define SPEED 50    

#if 0
    odo_0_step = 1;
    odo_1_step = 1;
    while(1)
    {
    motor(0,50);
    motor(1,50);
    delay(1000);
    printf("odo_cnt =%3.1f %3.1f\n",odo_0_cnt,odo_1_cnt);
    }
#endif

#if 1
    odo_0_cnt = 0;
    odo_1_cnt = 0;

    odo_0_step = 1;
    odo_1_step = 1;

    motor(0,SPEED);
    motor(1,SPEED);
    delay(2000);
    motor(0,0);
    motor(1,0);
    printf("odo_cnt =%3.1f %3.1f\n",odo_0_cnt,odo_1_cnt);

    odo_0_step = -1;
    odo_1_step = -1;

    motor(0,-SPEED);
    motor(1,-SPEED);
    delay(2000);
    motor(0,0);
    motor(1,0);
    printf("odo_cnt =%3.1f %3.1f\n",odo_0_cnt,odo_1_cnt);

    odo_0_step = 1;
    odo_1_step = -1;

    motor(0,SPEED);
    motor(1,-SPEED);
    delay(2000);
    motor(0,0);
    motor(1,0);
    printf("odo_cnt =%3.1f %3.1f\n",odo_0_cnt,odo_1_cnt);

    odo_0_step = -1;
    odo_1_step = 1;

    motor(0,-SPEED);
    motor(1,SPEED);
    delay(2000);
    motor(0,0);
    motor(1,0);
    printf("odo_cnt =%3.1f %3.1f\n",odo_0_cnt,odo_1_cnt);
#endif
    
    while(1)
    {
#ifdef ENABLE_mpu9250
	  if (mpu9250.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
	  { 
	    mpu9250.readAccelData(mpu9250.accelCount);  // Read the x/y/z adc values
	    mpu9250.getAres();

	    // Now we'll calculate the accleration value into actual g's
	    // This depends on scale being set
	    mpu9250.ax = (float)mpu9250.accelCount[0]*mpu9250.aRes; // - accelBias[0];
	    mpu9250.ay = (float)mpu9250.accelCount[1]*mpu9250.aRes; // - accelBias[1];
	    mpu9250.az = (float)mpu9250.accelCount[2]*mpu9250.aRes; // - accelBias[2];

	    mpu9250.readGyroData(mpu9250.gyroCount);  // Read the x/y/z adc values
	    mpu9250.getGres();

	    // Calculate the gyro value into actual degrees per second
	    // This depends on scale being set
	    mpu9250.gx = (float)mpu9250.gyroCount[0]*mpu9250.gRes;
	    mpu9250.gy = (float)mpu9250.gyroCount[1]*mpu9250.gRes;
	    mpu9250.gz = (float)mpu9250.gyroCount[2]*mpu9250.gRes;

	    mpu9250.readMagData(mpu9250.magCount);  // Read the x/y/z adc values
	    mpu9250.getMres();
	    // User environmental x-axis correction in milliGauss, should be
	    // automatically calculated
	    mpu9250.magbias[0] = +470.;
	    // User environmental x-axis correction in milliGauss TODO axis??
	    mpu9250.magbias[1] = +120.;
	    // User environmental x-axis correction in milliGauss
	    mpu9250.magbias[2] = +125.;

	    // Calculate the magnetometer values in milliGauss
	    // Include factory calibration per data sheet and user environmental
	    // corrections
	    // Get actual magnetometer value, this depends on scale being set
	    mpu9250.mx = (float)mpu9250.magCount[0]*mpu9250.mRes*mpu9250.magCalibration[0] -
		       mpu9250.magbias[0];
	    mpu9250.my = (float)mpu9250.magCount[1]*mpu9250.mRes*mpu9250.magCalibration[1] -
		       mpu9250.magbias[1];
	    mpu9250.mz = (float)mpu9250.magCount[2]*mpu9250.mRes*mpu9250.magCalibration[2] -
	    mpu9250.magbias[2];
	}

	// Must be called before updating quaternions!
	mpu9250.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  //  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
        MahonyQuaternionUpdate(mpu9250.ax, mpu9250.ay, mpu9250.az, mpu9250.gx*DEG_TO_RAD,
                         mpu9250.gy*DEG_TO_RAD, mpu9250.gz*DEG_TO_RAD, mpu9250.my,
	mpu9250.mx, mpu9250.mz, mpu9250.deltat);
#endif


        printf("t_sonar=%3.3fm ",344.0*t_sonar/(1000000*2));
#ifdef ENABLE_hmc5883l
        unsigned char msb = wiringPiI2CReadReg8(fd, HMC5883L_REG_MSB_X);
        unsigned char lsb = wiringPiI2CReadReg8(fd, HMC5883L_REG_LSB_X);
        short x = msb << 8 | lsb;

        msb = wiringPiI2CReadReg8(fd, HMC5883L_REG_MSB_Y);
        lsb = wiringPiI2CReadReg8(fd, HMC5883L_REG_LSB_Y);
        short y = msb << 8 | lsb;

        msb = wiringPiI2CReadReg8(fd, HMC5883L_REG_MSB_Z);
        lsb = wiringPiI2CReadReg8(fd, HMC5883L_REG_LSB_Z);
        short z = msb << 8 | lsb;

        double angle = atan2((double) y, (double)x) * (180 / PI) + 180;
        printf(" angle(%d,%d,%d)=%f",x,y,z,angle);
#endif
#ifdef ENABLE_mpu9250
	mpu9250.delt_t = millis() - mpu9250.count;
        if (mpu9250.delt_t > 500)
        {
		printf("(ax=%d ay=%d az=%d gx=%1.0f gy=%1.0f gz=%1.0f mx=%1.0f my=%1.0f mz=%1.0f q0=%1.0f) ",
		(int)1000*mpu9250.ax,(int)1000*mpu9250.ay,(int)1000*mpu9250.az,
		mpu9250.gx,mpu9250.gy,mpu9250.gz,
		mpu9250.mx,mpu9250.my,mpu9250.mz);
        
	      mpu9250.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ() *
		            *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1) * *(getQ()+1)
		            - *(getQ()+2) * *(getQ()+2) - *(getQ()+3) * *(getQ()+3));
	      mpu9250.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ() *
		            *(getQ()+2)));
	      mpu9250.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2) *
		            *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1) * *(getQ()+1)
		            - *(getQ()+2) * *(getQ()+2) + *(getQ()+3) * *(getQ()+3));
	      mpu9250.pitch *= RAD_TO_DEG;
	      mpu9250.yaw   *= RAD_TO_DEG;
	      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
	      // 	8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
	      // - http://www.ngdc.noaa.gov/geomag-web/#declination
	      mpu9250.yaw   -= 2.3;
	      mpu9250.roll *= RAD_TO_DEG;

		printf("%3.1f° %3.1f° %3.1f° %2.0fHz ",
			mpu9250.yaw,mpu9250.pitch,mpu9250.roll,(float)mpu9250.sumCount/mpu9250.sum);

		mpu9250.count = millis();
		mpu9250.sumCount = 0;
		mpu9250.sum = 0;
        }
#endif

	printf("\n");
        delay(1000);
    }
        
    return 0 ;
}

