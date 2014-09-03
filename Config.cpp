#include "stdafx.h"
#include <string.h>

extern char ucommand[30];
extern long Vel_pos[2];
extern BYTE Pid_time;
extern long Sum1;
extern long Vmax_pos[2];
extern long Vmax_neg[2];

extern long Vel_neg[2];
extern long Vel_last[2];
extern BYTE Max_pwm[2];
extern long Factor_acc[2];
extern float Deg[2];
extern long Point_p1[2];
extern BYTE Mtr_num;
extern BYTE Mtr;
extern short M1_pwm;
extern short M2_pwm;
extern BYTE Mode_ctrl;
extern long Motor_setpoint[2];
extern void Configure_pid(byte mtr, long Mtrnum_kp, long Mtrnum_ki, long Mtrnum_kd);
char *cmd_array[] = {
	"RSTF",
	"MADE",
	"VER",
	"SKP",
	"SKI",
	"SKD",
	"GO",
	"VMAX",
	"GMAX",
	"SVM",
	"GVM",
	"GVE",
	"ENC",
	"SPID",
	"GPID",
	"STIME",
	"GTIME",
	"SZP",
	"IDLE",
	"GSM",
	"SMODE",
	"MODE",
	"GMODE",
	"SVEL",
	"VELO",
	"SPWM",
	"GPWM",
	"RPWM",
	"SACC",
	"GACC",
	"SANG",
	"GANG",
	"POS",
	"MOVE",
	"VELP",
	"SUM",
	"STARTC",
	"STOPC",
	"RST",
	"NIL"
};

#define Mode_trp 2

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
void Enable_interrupt(char * Name)
{
	printf("interrupt enabled for  %s\n", Name);
}
void Timer_01(void)
{
	printf("Timer_01\n");
}
void Int_0(void)
{
	printf("Int_0\n");
	Configure_pid(1, 0, 0, 0);
	printf("1 FCM, 1\n");
	
	
}
void Int_1(void)
{
	printf("Int_1\n");
	Configure_pid(1, 0, 0, 0);
	printf("2 FCM, 1\n");
	
}
void Enable_all_interrupts(void)
{
	printf("All interrupts enable\n");
};
struct misc_bits
{
	unsigned  Start_move : 1;
	unsigned Tx_enable : 1;
	unsigned Flag_velocity : 1;
	unsigned Temp_bit : 1;
};
void Init_parameter(BYTE mtr)
{
	Pid_time = 4;
	Sum1 = 1000;
	misc_bits mybits;
	Vmax_pos[mtr] = 126;
	Vmax_neg[mtr] = -Vmax_pos[mtr];

	Vel_pos[mtr] = 126; //      'Velocity Initialize
	Vel_last[mtr] = Vel_pos[mtr]; //      'last velocity value
	Vel_neg[mtr] = -Vel_pos[mtr]; //      'Initialize negative velocity

	mybits.Flag_velocity = false;
	mybits.Start_move = true; //       'start move

	Max_pwm[mtr] = 250;//       'Max pwm value
	Deg[mtr] = 45; //'acceleration slope in grad(1)i
	Factor_acc[mtr] = 100;
	Point_p1[mtr] = 20000; //      'Point 1
	Motor_setpoint[mtr] = 0; //      'Initialize position = 0


	if (Mtr == 1)
		M1_pwm = 0; //       'pwm x = 0
	if (Mtr == 2)
		M2_pwm = 0; //       'pwm y = 0

	Mode_ctrl = Mode_trp;  //      'trapezoidal control
}

void Serial_input(void)
{	
	strcpy(ucommand,"FUCK 0 1 5");	
}
int searchforcommand(char * cmd)
{
	int index;
	index = 0;
	while (strcmp(cmd_array[index], "NIL")!=0)
	{
		if (strcmp(cmd_array[index], cmd) == 0)
			break;
		index++;
	}
	return index;
}


