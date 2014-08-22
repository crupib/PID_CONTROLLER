enum WatchDogstate
{
	STOP,
	START,
	RESET
};
enum int_state
{
	LOWLEVEL,
	FALLING,
	RISING,
	CHANGE
};
void Config_Watchdog(int ms); // Times vary according to chip used.
void Watchdog(WatchDogstate state);
void Config_intx(int_state stat,int interupt);
void Config_timer1();