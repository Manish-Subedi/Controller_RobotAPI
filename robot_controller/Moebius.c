#include "sys.h"
#include "Systick.h"
/**************************************************************************
main loop
**************************************************************************/

u8 Flag_Stop = 1, Flag_Show = 0;                  // Stop flag and display flag, default to stop, display is on
//int Encoder_A, Encoder_B, Encoder_C, Encoder_D;   // Encoder pulse counts
long int Position_A, Position_B, Position_C, Position_D, Rate_A, Rate_B, Rate_C, Rate_D; // PID control related variables
int Encoder_A_EXTI;                               // Encoder data read through external interrupt
//long int Motor_A = 0, Motor_B = 0, Motor_C = 0, Motor_D = 0;   // Motor PWM variables
//long int Target_A = 6, Target_B = 6, Target_C = 6, Target_D = 6;  // Motor target values
int Voltage;                                      // Battery voltage sampling related variable
float Show_Data_Mb;                               // Global display variable, used for displaying required data
u8 delay_50, delay_flag;                          // Delay related variables
u8 Run_Flag = 0;                                  // Line following control related variables and operation state flag
u8 rxbuf[8], Urxbuf[8], CAN_ON_Flag = 0, Usart_ON_Flag = 0, PS2_ON_Flag = 0, Usart_Flag, PID_Send, Flash_Send;  // CAN and serial control related variables
u8 txbuf[8], txbuf2[8], Turn_Flag;                // CAN transmission related variables
float Pitch, Roll, Yaw, Move_X, Move_Y, Move_Z;   // Three-axis angles and XYZ axis target speed
u16 PID_Parameter[10], Flash_Parameter[10];       // Flash related arrays
float Position_KP = 6, Position_KI = 0, Position_KD = 3;    // Position control PID parameters

float Velocity_KP = 6, Velocity_KI = 0.001, Velocity_KD = 0.01;         // Speed control PID parameters
float initial_correction = 25, dynamic_correction = 4.5;

int RC_Velocity = 30, RC_Position = 1000;         // Remote control speed and position values

int Gryo_Z;

/*************************************************
Function: Peripheral_Init()
Description: Init system peripheral
Input:  void
Output: void
Return: void
*************************************************/
void Peripheral_Init()
{
    LED_Init();                     // Initialize the hardware interface connected to LED
    KEY_Init();                     // Initialize the keys

    USART1_Init(115200);              // Initialize USART1 (stm32 -> PC*)
    USART3_Init(9600);              // Initialize USART3 for line following (stm32 -> Raspi)

    MiniBalance_PWM_Init(7199, 0);  // Initialize motor drive

    Adc_Init();

#if MPU6xxx_ENABLE
    IIC_Init();
    MPU6050_initialize();           // Initialize MPU6050
    DMP_Init();                     // Initialize DMP   
#endif
    
#if OLED_DISPLAY_ENABLE
    OLED_Init();                    // Initialize OLED
#endif
    
#if ENCODER_ENABLE                    // Initialize encoders
    Encoder_Init_TIM2();
    Encoder_Init_TIM3();
    Encoder_Init_TIM4();
    Encoder_Init_TIM5();
#endif
    EXTI15_Init();
}

/*************************************************
Function: main()
Description: Main function entry
Input:  void
Output: void
Return: void
*************************************************/
int main(void)
{
    // uint8_t PS2_Hold;
    Peripheral_Init();
    while (1)
    {	
				//run the IR check and control commands
    }
}
