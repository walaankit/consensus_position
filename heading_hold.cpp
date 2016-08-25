
#ifndef _HEADHOLD_CPP_
#define _HEADHOLD_CPP_

#include <cmath>

/* run this program using the console pauser or add your own getch, system("pause") or input loop */
using namespace std;




#define Kp 1.5//2
#define Kd 0.5
#define Ki 0


void head_hold(int num) {
	
	int temp=0;
	num=num-1;

//yaw_setpt = atan2(2.*quat_x*quat_y + 2.*quat_w*quat_z, quat_x*quat_x + quat_w*quat_w - quat_z*quat_z - quat_y*quat_y);
//yaw_setpt=M_PI/4;
yaw_setpt=0;
// Loop Start

yaw[num]= atan2(2.*quat_x[num]*quat_y[num] + 2.*quat_w[num]*quat_z[num], quat_x[num]*quat_x[num] + quat_w[num]*quat_w[num] - quat_z[num]*quat_z[num] - quat_y[num]*quat_y[num]);  
	
	
//*********************************************PID Algorithm implementation***********************************************************
prev_err[num] = err[num];
err[num]=yaw[num] - yaw_setpt;
integral[num] += err[num];
control[num] = Kp * (err[num]) + Kd * (err[num] - prev_err[num]) + Ki * (integral[num]);

//if(control > M_PI) control = 10;//M_PI;

//if(control < -M_PI) control = -10;//M_PI;


}


#endif
