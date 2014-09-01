
#include "stdafx.h"
#include "Main.h"
#include <string.h>
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
byte ports[40] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,3 };
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
void Configure_pid(byte mtr, long Mtrnum_kp, long Mtrnum_ki, long Mtrnum_kd);
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
	Test1_led.Test1_led = F & 00000001;
	Motor_led.Motor_led = C & 00000001;

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
	On_interrupt("Urxc", &Rs232);
	Enable_interrupt("Urxc");
	Enable_interrupt("Timer0");
	Enable_interrupt("Int0");
	Enable_interrupt("Int1");
	Enable_all_interrupts();
	foo = &Int_0;
	(*foo)();
	foo = &Int_1;
	(*foo)();
	Init_parameter(1);
	Init_parameter(2);
	Configure_pid(1, 500, 100, 500);
	Configure_pid(2, 500, 100, 500);
	printf("0 Motor Control, 0\n");
	Rs232();
	Test1_led.Test1_led = True;
	Motor_led.Motor_led = True;

	//Tx_enable.Tx_enable = 1;//Test code


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
void Configure_pid(byte mtr, long Mtrnum_kp, long Mtrnum_ki, long Mtrnum_kd)
{
	Pid_kp[mtr] = Mtrnum_kp;
	Pid_ki[mtr] = Mtrnum_ki;
	Pid_kd[mtr] = Mtrnum_kd;
	Pid_scale[mtr] = 100;
}

void Rs232(void)
{	
	int i, count;
	char *p;
	char *array[5]; 
	
	count = i = 0;
	Serial_input();
	printf("ucommand = %s\n", ucommand);
	strupr(ucommand);
	printf("ucommand = %s\n", ucommand);	
	p = strtok(ucommand, " ");
	while (p != NULL)
	{
		array[i++] = p;
		p = strtok(NULL, " ");
		count++;
	}

	for (i = 0; i<count; ++i)
		printf("%s\n", array[i]);
}