enum WatchDogstate
{
	STOP,
	START,
	RESET
};
enum int_state
{
	NONE,
	LOWLEVEL,
	FALLING,
	RISING,
	CHANGE
};
enum timer_type
{
	COUNTER, TIMER,PWM
};
enum Compare
{
	NOTUSED,
	CLEAR,
	SET,
	TOGGLE,
	DISCONNECT,
	CLEAR_UP,
	CLEAR_DOWN
};

void Config_Watchdog(int ms); // Times vary according to chip used.
void Watchdog(WatchDogstate state);
void Config_intx(int_state stat,int interupt);
void Config_timer1(timer_type type, int_state edge,
	int noise_cancel, int clear_timer,
	Compare Compare_A, Compare Compare_B,
	int PWM, Compare Compare_A_PWM,
	Compare Compare_B_PWM,
	char * Configuration, int Prescale);
void Config_timer0(timer_type type, int_state edge, int clear_timer,
	char * Configuration, int Prescale);
void On_interrupt(char * Name, void * routine);
void Timer_01(void);
void Int_0(void);
void Int_1(void);
