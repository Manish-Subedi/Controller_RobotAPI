#include "control.h"	
#include "filter.h"
#include "config.h"
#include "ir_config.h"

//#define printf(...)  
#define DEFAULT_SPEED 10

u8 l2, l1, cl, cr, r1, r2;   // IR sensor realted variables
u8 Flag_Left, Flag_Right, Flag_Direction = 'Z';     // Line following control related variables
u8 Target_Z;
u8 Flag_Target,Flag_Change;				//Related flags
u8 temp1;														//Temporary variable
float Voltage_Count,Voltage_All;		//Variables related to battery sampling
float Gyro_K=-0.6;									//Gyroscope proportional coefficient
int Gyro_Bias = 0;
int j;
unsigned int TimClk = 200;
#define a_PARAMETER          (0.311f)               
#define b_PARAMETER          (0.3075f)    
long int myCounter = 0;
u8 Start_cmd = 0;   // Main Switch

float * coefficient = &Velocity_KP;

Motor_PI motor_a, motor_b, motor_c, motor_d;  // structs for all motors

/**************************************************************************
Function Description: Mathematical model of the vehicle's motion
Input Parameters: X Y Z three-axis speed or position
Return Value: None
**************************************************************************/
void Kinematic_Analysis(float Vx,float Vy,float Vz)
{

#if	AXLE_Z_RESTRAIN
	int temp;
	if(!KEY1)	Gyro_Bias = Yaw;
	temp = Yaw - Gyro_Bias;
	if (temp > 180)
		temp = 360-temp;
	if (temp < -180)
		temp = 360 + temp;
	if(temp > 1 || temp < -1)
		Vz += Gyro_K * temp;
#endif
	motor_a.target   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
	motor_b.target   = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
	motor_c.target   = +Vx+Vy-Vz*(a_PARAMETER+b_PARAMETER);
	motor_d.target   = -Vx+Vy+Vz*(a_PARAMETER+b_PARAMETER);
	
	//if (myCounter % 300 == 0) printf("Kinematic_Analysis sequence completed\n\r");
	//if (myCounter % 300 == 0) printf("targets A : %li, B : %li, C : %li, D : %li\n\r", Target_A, Target_B, Target_C, Target_D);
}

#if DEBUG_CHAR
int do_debug_modify(char c){
	static char buffer[32];
	static int current = 0;
	
	if(c == ' '){
		buffer[current] = 0;	
		*coefficient = (float) atof(buffer);
		//printf("new coef: %f\n\r", *coefficient);
		current = 0;
		return 1;
	}else{
		buffer[current++] = c;
		if(current > 31) current = 0;
		return 0;
	}
}
#endif

/**************************************************************************
Function description: All control codes are here
         The 5ms scheduled interrupt is triggered by the INT pin of MPU6050
         Ensures time synchronization of sampling and data processing			 
**************************************************************************/
int EXTI15_10_IRQHandler(void) 
{    
	float ret = 0.0;
	int Yuzhi=20;
	if(INT==0)		
	{     
		EXTI->PR=1<<15;                                                     //Clear the interrupt flag bit on LINE5
		if(TimClk)
		{
			TimClk--;
			if(TimClk == 0)
			{
				TimClk = 200;
				LED = ~LED;
			}
		}
		Flag_Target=!Flag_Target;
		if(delay_flag==1)
		{
			if(++delay_50==10) delay_50=0, delay_flag=0;                     //Provide a precise 50ms delay for the main function
		}
		if(myCounter >= 999999998) myCounter = 0; //resets in 11.6 days
																			//Control once every 10ms. In order to ensure the time base of M method speed measurement, first read the encoder data

		motor_a.encoder	=	+Read_Encoder(2);                                          //Read the value of the encoder
		motor_a.position	+=	motor_a.encoder;                                                 //Integrate to get position
		motor_b.encoder	=	-Read_Encoder(3);                                          //Read the value of the encoder
		motor_b.position	+=	motor_b.encoder;                                                 //Integrate to get position 
		motor_c.encoder	=	-Read_Encoder(4);                                         //Read the value of the encoder
		motor_c.position	+=	motor_c.encoder;                                                 //Integrate to get position 
		motor_d.encoder	=	+Read_Encoder(5);                                       //Read the value of the encoder
		motor_d.position	+=	motor_d.encoder;                                                 //Integrate to get position   
		//if (myCounter % 300 == 0){
			//printf("encoderA reading %d, positionA %li\r\n", Encoder_A, Position_A);
			//printf("encoderB reading %d, positionB %li\n\r", Encoder_B, Position_B);
			//printf("encoderC readingC %d, positionC %li\n\r", Encoder_C, Position_C);
			//printf("ea %d %d, eb %d %d, ec %d %d, ed %d %d\n\r", motor_a.encoder, motor_a.position, motor_b.encoder, motor_b.position, motor_c.encoder, motor_c.position, motor_d.encoder, motor_d.position);
		//}
		//myCounter++;, 

		Read_DMP();                                                               //Update status	
		Voltage_All+=Get_battery_volt();                                          //Accumulate multiple samplings
		if(++Voltage_Count==100) {
			Voltage=Voltage_All/100;
			Voltage_All=0;
			Voltage_Count=0;      //Calculate average value to get battery voltage
		}
#if		0		
		motor_a.pwm = Incremental_PI_A(Encoder_A,&Target_A);                         //Speed closed-loop control calculates the final PWM for motor 
		Motor_B=Incremental_PI_B(Encoder_B,&Target_B);                         //Speed closed-loop control calculates the final PWM for motor B
		Motor_C=Incremental_PI_C(Encoder_C,&Target_C);                         //Speed closed-loop control calculates the final PWM for motor C
		Motor_D=Incremental_PI_D(Encoder_D,&Target_D);                         //Speed closed-loop control calculates the final PWM for motor D
		
#if 0		
		Motor_A = Target_A;                         //Speed closed-loop control calculates the final PWM for motor A
		Motor_B = Target_B;                         //Speed closed-loop control calculates the final PWM for motor B
		Motor_C = Target_C;                         //Speed closed-loop control calculates the final PWM for motor C
		Motor_D = Target_D;                         //Speed closed-loop control calculates the final PWM for motor D
#endif
#endif
		if(InspectQueue())
		{

#if DEBUG_CHAR
			static int modify = 0;
			char c = OutQueue();
			
			if(!modify) switch(c){
				case 'P':
					coefficient = &Velocity_KP;
					modify = 1;
					break;
				case 'I':
					coefficient = &Velocity_KI;
					modify = 1;
					break;
				case 'D':
					coefficient = &Velocity_KD;
					modify = 1;
					break;
				case 'i':
					coefficient = &initial_correction;
					modify = 1;
					break;
				case 'd':
					coefficient = &dynamic_correction;
					modify = 1;
					break;
				default:
					Flag_Direction = c;
					Motors_reset();
					break;
			}
			if(modify){
				while(InspectQueue()){
					if(do_debug_modify(OutQueue())) {
							modify = 0;
					}
				}
			}
#else 
			Flag_Direction = OutQueue();
			Motors_reset();
#endif
		}

		ret = line_state();
		if(Flag_Direction != 'Z'){
				if( ret != 254 && ret != 255 && Flag_Direction != 'C'){ //
					#if 0
					if (ret == 100) {
							Flag_Direction = 'C';
							enable_rotation(5000);
					}
					else  { 
					#endif
							static int state = 0;						
							int nstate = ret < 2.4 ? -1 : (ret > 2.6 ? 1 : 0);							
							if(state != nstate){ // if the mean value changes
								state = nstate;
								if(state < 0) Move_X -= initial_correction;
								else if(state > 0) Move_X += initial_correction;								
							}				
							if (ret > 2.6) Move_X += dynamic_correction*(ret-2.5);
							else if (ret < 2.4) Move_X -= dynamic_correction*(2.5-ret);
							Get_RC(0);
							Update_PI();
							//printf("dc %f  ic %f KP %f  KI %f  KD %f \n\r", dynamic_correction, initial_correction, Velocity_KP, Velocity_KI, Velocity_KD);
						//} //
				}
				else if( Flag_Direction == 'C'){
						if (ret > 2.4 && ret < 2.6) {
							Motors_reset();
							Flag_Direction = 'A';
							Get_RC(0);
							Update_PI();
						}
				}			
				
				Limiter_Pwm(7200);
				Set_Pwm(motor_a.pwm, motor_b.pwm, motor_c.pwm, motor_d.pwm);
			} //if(Flag_Direction != 'Z')
		else {
			//reset everything 
				Motors_reset();
				Set_Pwm(motor_a.pwm, motor_b.pwm, motor_c.pwm, motor_d.pwm);
		}
		//if (myCounter % 100 == 0) {
			//printf("e:%d t:%d pid: %d pos:%d\r\n", motor_b.encoder, motor_b.target, motor_b.pid_value, motor_b.position);
			//printf("\t%d %d %d %d %d %d ret:%f", l2, l1, cl, cr, r1, r2, ret);
			//printf("MA:%ld, MB:%ld, MC:%li, MD:%li\n\r\n", motor_a.pwm, motor_b.pwm, motor_c.pwm, motor_d.pwm);	
			//printf("ret %f\n\r", ret);
		//}myCounter++;
	}
	return 0;	 
}

void Motors_reset(){
	Motor_reset(&motor_a);
	Motor_reset(&motor_b);
	Motor_reset(&motor_c);
	Motor_reset(&motor_d);
	Move_X = Move_Y = Move_Z = 0;
}
	

void Motor_reset(Motor_PI * motor){
		motor->encoder = 0;
		motor->last_bias = 0;
		motor->pwm = 0;
		motor->target = 0;
	  motor->pid_value = 0;
		motor->position = 0;
}

void Update_PI(){
		Incremental_PI(&motor_a);   //Speed closed-loop control calculates the final PWM for motor A
		Incremental_PI(&motor_b);   //Speed closed-loop control calculates the final PWM for motor B
		Incremental_PI(&motor_c);   //Speed closed-loop control calculates the final PWM for motor C
		Incremental_PI(&motor_d);   //Speed closed-loop control calculates the final PWM for motor D
}

/**************************************************************************
Function Description: Assign values to PWM registers
Input Parameters: PWM
Return Value: None
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d)
{
	if(motor_a<0)							INA2=1,			INA1=0;
	else if(motor_a>0)				INA2=0,			INA1=1;
	else INA2=0,			INA1=0;
	PWMA=myabs(motor_a);

	if(motor_b<0)							INB2=1,			INB1=0;
	else if(motor_b>0)				INB2=0,			INB1=1;
	else INB2=0,			INB1=0;
	PWMB=myabs(motor_b);

	if(motor_c<0)							INC2=1,			INC1=0;
	else if(motor_c>0)				INC2=0,			INC1=1;
	else INC2=0,			INC1=0;
	PWMC=myabs(motor_c);

	if(motor_d>0)							IND2=1,			IND1=0;
	else if(motor_d<0)				IND2=0,			IND1=1;
	else IND2=0,			IND1=0;
	PWMD=myabs(motor_d);
}

/**************************************************************************
Function Description: Limit PWM values
Input Parameters: Amplitude
Return Value: None
**************************************************************************/
void Limiter_Pwm(int amplitude)
{	
	if(motor_a.pwm < -amplitude) motor_a.pwm = -amplitude;	
	if(motor_a.pwm > amplitude)  motor_a.pwm = amplitude;	
	if(motor_b.pwm < -amplitude) motor_b.pwm=-amplitude;	
	if(motor_b.pwm > amplitude)  motor_b.pwm = amplitude;		
	if(motor_c.pwm < -amplitude) motor_c.pwm = -amplitude;	
	if(motor_c.pwm > amplitude)  motor_c.pwm = amplitude;		
	if(motor_d.pwm < -amplitude) motor_d.pwm = -amplitude;	
	if(motor_d.pwm > amplitude)  motor_d.pwm = amplitude;		
}

/**************************************************************************
Function Description: Emergency shutdown
Input Parameters: Voltage
Return Value: 1 (emergency), 0 (normal)
**************************************************************************/
u8 Turn_Off( int voltage)
{
	u8 temp;
	if(voltage<2000||EN==0)		//Battery voltage below 22.2V, shut down the motor
	{	                                                
		temp=1;      
		PWMA=0;
		PWMB=0;
		PWMC=0;
		PWMD=0;							
	}
	else
		temp=0;
	return temp;			
}

/**************************************************************************
Function Description: Absolute value function
Input Parameters: long int
Return Value: unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	u32 temp;
		if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}



/**************************************************************************
Function Description: Incremental PI controller
Input Parameters: Encoder measured value, Target speed
Return Value: Motor PWM
Formula: Incremental PID formula
pwm += Kp[e(k) - e(k-1)] + Ki*e(k) + Kd[e(k) - 2e(k-1) + e(k-2)]
e(k) represents current deviation
e(k-1) represents previous deviation
pwm represents incremental output
In our speed control closed-loop system, only PI control is used
pwm += Kp[e(k) - e(k-1)] + Ki*e(k)
**************************************************************************/
#if 0
int Incremental_PI_A (int Encoder, long *Target)
{ 	
	 static int Bias = 0, Pwm = 0, Last_bias = 0;
	 Bias = Encoder - *Target;                // Calculate deviation
	 if (Bias > BIAS_LIMIT) *Target = Encoder-BIAS_LIMIT; if(Bias < -BIAS_LIMIT) *Target = Encoder+BIAS_LIMIT;
	 if (myCounter % 300 == 0) printf("Incremental_PI_A: Bias %d\t", Bias);
	 Pwm += Velocity_KP * ( Bias - Last_bias ) + Velocity_KI * Bias;   // Incremental PI controller
	 if (myCounter % 300 == 0) printf("Pwm %d\t", Pwm);
	 if ( Pwm > 7200 )	Pwm = 7200;
	 if( Pwm < -7200)	Pwm = -7200;
	 if (myCounter % 300 == 0)printf("Last_bias before %d\t", Last_bias);
	 Last_bias = Bias;	                   // Save the previous deviation 
	 if (myCounter % 300 == 0)printf("Last_bias after %d\n\r", Last_bias);
	 return Pwm;                         // Incremental output
}
int Incremental_PI_B (int Encoder,long *Target)
{ 	
	 static int Bias = 0, Pwm = 0, Last_bias = 0;
	 Bias = Encoder - *Target;                // Calculate deviation
	 if (Bias > BIAS_LIMIT) *Target = Encoder-BIAS_LIMIT; if(Bias < -BIAS_LIMIT) *Target = Encoder+BIAS_LIMIT;
	 if (myCounter % 300 == 0) printf("Incremental_PI_B: Bias %d\t", Bias);
	 Pwm += Velocity_KP * ( Bias - Last_bias ) + Velocity_KI * Bias;   // Incremental PI controller
	 if (myCounter % 300 == 0) printf("Pwm %d\t", Pwm);
	 if ( Pwm > 7200 )	Pwm = 7200;
	 if( Pwm < -7200)	Pwm = -7200;
	 if (myCounter % 300 == 0)printf("Last_bias before %d\t", Last_bias);
	 Last_bias = Bias;	                   // Save the previous deviation 
	 if (myCounter % 300 == 0)printf("Last_bias after %d\n\r", Last_bias);
	 return Pwm;                         // Incremental output
}
int Incremental_PI_C (int Encoder,long *Target)
{ 	
	 static int Bias = 0, Pwm = 0, Last_bias = 0;
	 Bias = Encoder - *Target;                // Calculate deviation
	 if (Bias > BIAS_LIMIT) *Target = Encoder-BIAS_LIMIT; if(Bias < -BIAS_LIMIT) *Target = Encoder+BIAS_LIMIT;
	 if (myCounter % 300 == 0) printf("Incremental_PI_C: Bias %d\t", Bias);
	 Pwm += Velocity_KP * ( Bias - Last_bias ) + Velocity_KI * Bias;   // Incremental PI controller
	 if (myCounter % 300 == 0) printf("Pwm %d\t", Pwm);
	 if ( Pwm > 7200 )	Pwm = 7200;
	 if( Pwm < -7200)	Pwm = -7200;
	 if (myCounter % 300 == 0)printf("Last_bias before %d\t", Last_bias);
	 Last_bias = Bias;	                   // Save the previous deviation 
	 if (myCounter % 300 == 0)printf("Last_bias after %d\n\r", Last_bias);
	 return Pwm;                         // Incremental output
}
int Incremental_PI_D (int Encoder,long *Target)
{ 	
	 static int Bias = 0, Pwm = 0, Last_bias = 0;
	 Bias = Encoder - *Target;                // Calculate deviation
	 if (Bias > BIAS_LIMIT) *Target = Encoder-BIAS_LIMIT; if(Bias < -BIAS_LIMIT) *Target = Encoder+BIAS_LIMIT;
	 if (myCounter % 300 == 0) printf("Incremental_PI_D: Bias %d\t", Bias);
	 Pwm += Velocity_KP * ( Bias - Last_bias ) + Velocity_KI * Bias;   // Incremental PI controller
	 if (myCounter % 300 == 0) printf("Pwm %d\t", Pwm);
	 if ( Pwm > 7200 )	Pwm = 7200;
	 if( Pwm < -7200)	Pwm = -7200;
	 if (myCounter % 300 == 0)printf("Last_bias before %d\t", Last_bias);
	 Last_bias = Bias;	                   // Save the previous deviation 
	 if (myCounter % 300 == 0)printf("Last_bias after %d\n\r", Last_bias);
	 return Pwm;                         // Incremental output
}
#endif


#define BIAS_LIMIT 1000

void Incremental_PI(Motor_PI * motor)
{ 	
	 int Bias = 0; 
	 Bias = motor->position - motor->target;                // Calculate deviation
	 //if (Bias > BIAS_LIMIT) motor->target = motor->position - BIAS_LIMIT; 
	 //if(Bias < -BIAS_LIMIT) motor->target = motor->position + BIAS_LIMIT;
	 //if (myCounter % 300 == 0) printf("Incremental_PI_D: Bias %d\t", Bias);
	 motor->pid_value += Velocity_KP * ( Bias - motor->last_bias ) + Velocity_KI * Bias ;  // Incremental PI controller
	 //if (myCounter % 300 == 0) printf("Pwm %d\t", Pwm);
	 if ( motor->pid_value > 7200 )	motor->pid_value = 7200;
	 if( motor->pid_value < -7200)	motor->pid_value = -7200;
	 //if (myCounter % 300 == 0) printf("Last_bias before %d\t", Last_bias);
	 motor->last_bias = Bias;	                   // Save the previous deviation 
	 //if (myCounter % 300 == 0)printf("Last_bias after %d\n\r", Last_bias);
   motor->pwm = motor->pid_value - Velocity_KD * ( Bias - motor->last_bias );                    // Incremental output
}
/**************************************************************************
Function Description: Keep the robot aligned with the line on the floor
Input Parameters: Data from the IR sensor
Return Value: None
**************************************************************************/

	
/**************************************************************************
Function Description: Control the vehicle through serial commands
Input Parameters: Serial command
Return Value: None
**************************************************************************/
void Get_RC(u8 mode)
{
	static uint8_t step=DEFAULT_SPEED;  // Set the step size of speed control.
	u8 Flag_Move=1;
	if(mode == 0){ 
			switch(Flag_Direction)   // Direction control
			{		
				case 'A':										  
						Move_Y+=step;
						Flag_Move=1; 				
						break; 
				case 'C':	
						Move_X+=step;
						Flag_Move=1;	
						break;
				case 'Z':	
				default:
						Move_X=0;				
						Move_Y=0;		
						Move_Z=0;	
						Motors_reset();
						//Set_Pwm(0, 0, 0 ,0);
						step = DEFAULT_SPEED; 
						break;
				case 'L':	step+=5;	Flag_Direction = 'A'; break;
				case 'M':	step-=5;	Flag_Direction = 'A'; break;
			}
		if (step > 20) step = 20;
		if (step < 3) step = 3;
		//if (myCounter % 300 == 0) printf("X %lf, Y %lf, Z %lf\n\r", Move_X, Move_Y, Move_Z);
	}
	Kinematic_Analysis(Move_X,Move_Y,Move_Z); // Obtain control target value, perform motion analysis
}
