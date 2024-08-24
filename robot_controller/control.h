#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define PI 3.14159265 
#define DIFFERENCE 100

typedef struct {
	int pid_value;
	int last_bias; 
	int encoder;
	int target;
	int pwm;
	int position;
}Motor_PI;

extern int Balance_Pwm,Velocity_Pwm,Turn_Pwm;

extern u8 l2, l1, cl, cr, r1, r2;   // IR sensor realted variables
int EXTI15_10_IRQHandler(void);
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d);
void Kinematic_Analysis(float Vx,float Vy,float Vz);
void Limiter_Pwm(int amplitude);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
void Incremental_PI (Motor_PI * motor);
void Motors_reset(void);
void Update_PI(void);
void Motor_reset(Motor_PI * motor);
void enable_rotation(int pwm);
#if 0
int Incremental_PI_A (int Encoder,long *Target);
int Incremental_PI_B (int Encoder,long *Target);
int Incremental_PI_C (int Encoder,long *Target);
int Incremental_PI_D (int Encoder,long *Target);
#endif

void Get_RC(u8 mode);
#endif
