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
