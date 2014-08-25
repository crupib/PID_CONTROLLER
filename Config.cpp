#include "stdafx.h"
#include "Main.h"
void Config_Watchdog(int ms) // Times vary according to chip used.
{
	printf("Config Watchdog To be implented in hardware\n");
}
void Watchdog(WatchDogstate state)
{
	printf("Set Watchdog To be implented in hardware\n");
}
void Config_intx(int_state stat, int interupt)
{
	printf("Set int%i To be implented in hardware\n",interupt);
}
//Config Timer1 = Pwm, Pwm = 8, Compare A Pwm = Clear Down, Compare B Pwm = Clear Down, Prescale = 1       '39.0622Khz pwm

void Config_timer1(timer_type type, int_state edge,
					int noise_cancel, int clear_timer,
					Compare Compare_A, Compare Compare_B,
					int PWM, Compare Compare_A_PWM,
					Compare Compare_B_PWM,
					char * Configuration, int Prescale)
{
	printf("Set timer1 to be implemeted in hardware\n");
}
void Config_timer0(timer_type type, int_state edge, int clear_timer,
	char * Configuration, int Prescale)
{
	printf("Set timer0 to be implemeted in hardware\n");
}
void On_interrupt(char * Name, void * routine)
{
	printf("Turn on interrupt %s %x\n", Name, routine);
}
void Timer_01(void)
{
	printf("Timer_01\n");
}
void Int_0(void)
{
	printf("Int_0\n");
}
void Int_1(void)
{
	printf("Int_1\n");
}
void Init_parameter(BYTE mtr)
{
	Pid_time = 4;
	Sum1 = 1000;

	Vmax_pos[mtr] = 126;
	Vmax_neg[mtr] = -Vmax_pos[mtr];

	Vel_pos[mtr] = 126; //      'Velocity Initialize
	Vel_last[mtr] = Vel_pos[mtr]; //      'last velocity value
	Vel_neg[mtr] = -Vel_pos[mtr]; //      'Initialize negative velocity

	mybits.Flag_velocity = False;
	mybits.Start_move = True; //       'start move

	Max_pwm[mtr] = 250;//       'Max pwm value
	Deg[mtr] = 45; //'acceleration slope in grad(1)i
	Factor_acc[mtr] = 100;
	Point_p1[mtr] = 20000; //      'Point 1
	Motor_setpoint[mtr] = 0; //      'Initialize position = 0

	//   if (Mtr == 1) 
	//		M1_pwm = 0; //       'pwm x = 0
	//   if (Mtr == 2) 
	//		M2_pwm = 0; //       'pwm y = 0

	Mode_ctrl = Mode_trp;  //      'trapezoidal control

}


