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
void Config_timer1()
{

}
