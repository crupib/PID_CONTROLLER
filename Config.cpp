#include "stdafx.h"

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


