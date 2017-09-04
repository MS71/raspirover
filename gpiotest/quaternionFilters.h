#ifndef _QUATERNIONFILTERS_H_
#define _QUATERNIONFILTERS_H_

#define DEG_TO_RAD 0.01745329238
#define RAD_TO_DEG 57.29578

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz,
                            float deltat);
const float * getQ();

#endif // _QUATERNIONFILTERS_H_
