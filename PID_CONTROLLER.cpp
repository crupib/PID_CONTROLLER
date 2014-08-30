
#include "stdafx.h"
#include "Main.h"
//-------------------------------------------------------------------------------------
//Registers to be filled in per board
#define PORTA  0x00  
#define DDRA   0x01
#define PINA   0x02 
#define PORTB  0x03
#define PORTF  0x10
#define DDRB   0x04
#define PINB   0x05
#define PORTC  0x06
#define DDRC   0x07
#define PINC   0x08
#define PORTD  0x09
#define DDRD   0x0a
#define PIND   0x0b
#define UDR    0x0c
#define USR    0x0d
byte ports[40];

static const char regfile[] = "m128def.dat"; //      ' specify the used micro

int  crystal = 20000000; //' used crystal frequency - Obsolete
int  baud = 19200;       //' use baud rate
int  hwstack = 64;       //' default use 32 for the hardware stack
int  swstack = 64;       //'10       ' default use 10 for the SW stack
int  framesize = 64;     //'40     ' default use 40 for the frame space

#define Si = 1
#define No = 0
#define Vb6 = "No"

void Hctl_2032(byte Mtrnum);
void Exe_pid(byte Mtrnum, long Pid_setpoint, long Pid_actual);
void Configure_pid(byte M, long Mtrnum_kp, long Mtrnum_ki, long Mtrnum_kd);
void Init_parameter(byte M);
void Calc_trapez(byte M);
void Print_com(byte Mt, char * Stringa, long Value);
void Set_mode(byte Mode_control);
long Max_minl(long I);

struct
{
	byte Test1_led : 1;
}Test1_led;
struct
{
	byte Motor_led : 1;
}Motor_led;

struct
{
	byte Tx_enable : 1;
}Tx_enable;
void main(void)
{
	
//	byte * F = (byte *)PORTF;
	byte  F = ports[PORTF];
//	byte * C = (byte *)PORTC;
	byte C = ports[PORTC];
	Test1_led.Test1_led = F & 11111110;
	Motor_led.Motor_led = C & 11111110;

	M1_pwm = 0;
	M2_pwm = 0;
	
	void(*foo)(void);
	Config_Watchdog(16);
	Watchdog(STOP);
	Config_intx(FALLING,0);
	Config_intx(FALLING,1);
	Config_timer1(PWM, NONE,
		0, 0, CLEAR_DOWN,
		NOTUSED, 
		8, CLEAR_DOWN,
		CLEAR_DOWN,
		"Timer1", 1);
	Config_timer0(TIMER, NONE,  0, "Timer0", 64);
	On_interrupt("Timer01", &Timer_01);
	On_interrupt("Int0", &Int_0);
	On_interrupt("Int0", &Int_1);
	foo = &Int_1;
	(*foo)();
	Init_parameter(1);
	Init_parameter(2);

	Test1_led.Test1_led = True;

	Motor_led.Motor_led = True;
	Tx_enable.Tx_enable = 1;


	while (true)
	{

		Test1_led.Test1_led = True;
		if (Tx_enable.Tx_enable == 1)
		{
			Tx_enable.Tx_enable = 0;
			printf("%s\n", Str_tx_1);
		}
		Test1_led.Test1_led = False;

	}
}
