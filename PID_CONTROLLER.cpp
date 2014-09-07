
#include "stdafx.h"
#include "Main.h"
#include <string.h>
#define _USE_MATH_DEFINES
#include <math.h>
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
#define Deg2rad(angle) ((angle) / 180.0 * M_PI)
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
const byte Mode_pos = 0;
const byte Mode_vel = 1;
const byte Mode_trp = 2;
const byte Mode_velp = 3;
const byte Mode_idle = 4;

void Hctl_2032(byte Mtrnum);
void Exe_pid(byte Mtrnum, long Pid_setpoint, long Pid_actual);
void Configure_pid(byte mtr, long Mtrnum_kp, long Mtrnum_ki, long Mtrnum_kd);
void Init_parameter(byte M);
void Calc_trapez(byte M);
void Print_com(byte Mt, char * Stringa, long Value);
void Set_mode(byte Mode_control);
long Max_minl(long I);

void main(void)
{
	
	byte  F = ports[PORTF];
	byte  D4 = ports[DDRC];
	byte C = ports[PORTC];
	Test1_led.Test1_led = F & 00000001;
	Motor_led.Motor_led = C & 00000001;
	Hctl_rst_1.Hctl_rst_1 = C & 00001000;
	Hctl_rst_2.Hctl_rst_2 = C & 00010000;

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
	On_interrupt("Timer0", &Timer_0);
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
	int i, count,n_cmd;
	char *p;
	char *cmd_array[5]; 
    command command_entered;
	int cmdnum = 0;
	count = i = 0;
	Serial_input();
	printf("ucommand = %s\n", ucommand);
	strupr(ucommand);
	printf("ucommand = %s\n", ucommand);	
	p = strtok(ucommand, " ");
	while (p != NULL)
	{
		cmd_array[i++] = p;
		p = strtok(NULL, " ");
		count++;
	}

	for (i = 0; i<count; ++i)
		printf("%s\n", cmd_array[i]);
	if (strcmp(ucommand, "")==0)
		strcpy(ucommand, ucommand_old);
	n_cmd = count;
	Mtr = (byte)atoi(cmd_array[1]); 
	cmdnum = searchforcommand(cmd_array[0]);
	command_entered = (command)cmdnum;
	switch (command_entered)
	{
	case RSTF:
			printf("0 RST Program, 0\n");
			Watchdog(START);
			break;
	case MADE:
			printf("09-02-2014\n");
			break;
	case VER:
			printf("Version 3.0, 0\n");
			break;
	case SKP:
			Pid_kp[Mtr] = atoi(cmd_array[2]);
			printf("; Mtr; SKP %i\n", Pid_kp[Mtr]);
			break;
	case SKI:
			Pid_ki[Mtr] = atoi(cmd_array[2]);
			printf("; Mtr; SKI %i\n", Pid_ki[Mtr]);
			break;
	case SKD:
			Pid_kd[Mtr] = atoi(cmd_array[2]);
			printf("; Mtr; SKD %i\n", Pid_kd[Mtr]);
			break;
	case GO:
			if (Mode_ctrl != Mode_pos)
				Set_mode(Mode_pos);
			Mtr = atoi(cmd_array[1]);
			Pos_final[Mtr] = atoi(cmd_array[2]);
			printf("; Mtr; GO %i\n", Pos_final[Mtr]);
			break;
	case VMAX:
			Vmax_pos[Mtr] = atoi(cmd_array[2]);
			Vmax_neg[Mtr] = -Vmax_pos[Mtr];
			printf("; Mtr; VMAX %i\n", Vmax_pos[Mtr]);
			break;
	case GMAX:
			printf("; Mtr; GMAX %i\n", Vmax_pos[Mtr]);
			break;
	case SVM:
			Vel_pos[Mtr] = atoi(cmd_array[2]);
			Vel_last[Mtr] = Vel_pos[Mtr];
			Vel_neg[Mtr] = -Vel_pos[Mtr];
			printf("; Mtr; SVM %i\n", Vel_pos[Mtr]);
			break;
	case GVM:
			printf("; Mtr; GVM %i\n", Vel_pos[Mtr]);
			break;
	case GVE:
			Tx_enable.Tx_enable = 1;
			Print_com(Mtr, cmd_array[0], Act_speed[Mtr]);
			break;
	case ENC:
			Tx_enable.Tx_enable = 1;
			Print_com(Mtr, cmd_array[0], Pos_encoder[Mtr]);
			break;
	case SPID:
			Pid_kp[Mtr] = atoi(cmd_array[2]);
			Pid_ki[Mtr] = atoi(cmd_array[3]);
			Pid_kd[Mtr] = atoi(cmd_array[4]);
			printf("Mtr, SPID, kp %i ki %i kd %i", Pid_kp[Mtr], Pid_ki[Mtr], Pid_kd[Mtr]);
			break;
	case GPID:
			printf("Mtr, GPID, kp %i ki %i kd %i", Pid_kp[Mtr], Pid_ki[Mtr], Pid_kd[Mtr]);
			break;
	case STIME:
			Pid_time = atoi(cmd_array[1]);
			printf("; Mtr; STIME %i\n", Pid_time);
			break;
	case GTIME:
			printf("; Mtr; GTIME %i\n", Pid_time);
			break;
	case SZP:
			switch (Mtr)
			{
			case 1:
				Hctl_rst_1.Hctl_rst_1 = 0;
				Sleep(1);
				Hctl_rst_1.Hctl_rst_1 = 1;
				break;
			case 2:
				Hctl_rst_2.Hctl_rst_2 = 0;
				Sleep(1);
				Hctl_rst_2.Hctl_rst_2 = 1;
				break;
			}
			Pos_encoder[Mtr] = 0;
			Old_encoder[Mtr] = 0;
			Motor_setpoint[Mtr] = 0;
			Pos_final[Mtr] = 0;
			New_speed[Mtr] = 0;
			Hctl_2032(Mtr);
			printf("; Mtr; SZP %i\n", Pos_encoder[Mtr]);
			break;
	case IDLE:
			Configure_pid(Mtr, 0, 0, 0);
			printf("Mtr IDLE\n");
			break;
	case GSM:
			if ((Act_speed[Mtr] > 3) || (Act_speed[Mtr] < -3))
			{
				Tx_enable.Tx_enable = 1;
				Print_com(Mtr, cmd_array[Mtr], 0);
			}
			else
			{
				Tx_enable.Tx_enable = 1;
				Print_com(Mtr, cmd_array[Mtr], 0);
			}
			break;
	case SMODE:
			Mode_ctrl = atoi(cmd_array[2]);
			switch (Mode_ctrl)
			{
			case Mode_pos:
				Configure_pid(Mtr, 200, 0, 200);
				Motor_setpoint[Mtr] = Pos_encoder[Mtr];
				break;
			case Mode_vel:
				Configure_pid(Mtr, 700, 100, 700);
				break;
			case Mode_trp:
				Configure_pid(Mtr, 500, 100, 500);
				Pos_final[Mtr] = Pos_encoder[Mtr];
				break;
			case Mode_velp:
				Configure_pid(Mtr, 100, 10, 100);
				break;
			case Mode_idle:
				Configure_pid(Mtr, 0, 0, 0);
				break;
			}
			printf("0 MODE %i\n", Mode_ctrl);
			break;
	case MODE:
			Mode_ctrl = atoi(cmd_array[1]);
			Set_mode(Mode_ctrl);
			printf("0 MODE %i\n", Mode_ctrl);
			break;
	case GMODE:
			printf("0 GMODE %i\n", Mode_ctrl);
			break;
	case SVEL:
			if (Mode_ctrl != Mode_vel)
				Set_mode(Mode_vel);
			Motor_setpoint[Mtr] = atoi(cmd_array[2]);
			printf("Mtr SVEL %i\n", Motor_setpoint[Mtr]);
			break;
	case VELO:
			if (Mode_ctrl != Mode_trp)
				Set_mode(Mode_vel);
			Motor_setpoint[1] = atoi(cmd_array[1]);
			Motor_setpoint[2] = atoi(cmd_array[2]);
			break;
	case SPWM:
			Max_pwm[Mtr] = atoi(cmd_array[2]);
			printf("Mtr %i SPWM %i\n", Mtr, Max_pwm[Mtr]);
			break;
	case GPWM:
			printf("Mtr GPWM %i\n", Max_pwm[Mtr]);
			break;
	case RPWM:
			printf("Mtr RPWM %i\n", Rpwm[Mtr]);
			break;
	case SACC:
			Deg[Mtr] = (float)atof(cmd_array[2]);
			Rad[Mtr] = (float)Deg2rad(Deg[Mtr]);
			Rad[Mtr] = tan(Rad[Mtr]);
			Vel_max[Mtr] = (float)Vel_pos[Mtr]*10;
			Point_p1[Mtr] = (long) (Vel_max[Mtr] / Rad[Mtr]);
			Point_p1[Mtr] = Point_p1[Mtr] * 10;
			printf("Mtr %i SACC %i\n",Mtr, Deg[Mtr]);
			break;
	case GACC:
			printf("Mtr %i GACC %f\n",Mtr, Deg[Mtr]);
			break;
	case SANG:
			Factor_acc[Mtr] = atoi(cmd_array[2]);
			printf("Mtr %i SANG %i\n", Mtr, Factor_acc[Mtr]);
			break;
	case GANG:
			printf("Mtr %i GANG %f\n",Mtr ,Factor_acc[Mtr]);
			break;
	case POS:
			if (Mode_ctrl != Mode_trp)
				Set_mode(Mode_trp);
			Pos_final[Mtr] = atoi(cmd_array[2]);
			Vel_pos[Mtr] = atoi(cmd_array[3]);
			Calc_trapez(Mtr);
			Start_move.Start_move = True;
			break;
	case MOVE:
			if (Mode_ctrl != Mode_trp)
				Set_mode(Mode_trp);
			Pos_final[1] = atoi(cmd_array[1]);
			Pos_final[2] = atoi(cmd_array[2]);
			Vel_pos[1] = atoi(cmd_array[3]);
			Vel_pos[2] = atoi(cmd_array[3]);
			Calc_trapez(1);
			Calc_trapez(2);
			printf("0 Move %i %i\n", Pos_final[1], Pos_final[2]);
			break;
	case VELP:
			if (Mode_ctrl != Mode_velp)
				Set_mode(Mode_velp);
			Acc_speed_p[1] = Vmax_pos[1];
			Acc_speed_p[2] = Vmax_pos[2];
			Acc_speed_n[1] = Vmax_neg[1];
			Acc_speed_n[2] = Vmax_neg[2];
			Factor_ang[1] = Factor_acc[1];
			Factor_ang[2] = Factor_acc[2];
			Pos_final[1] = atoi(cmd_array[1]);
			Pos_final[2] = atoi(cmd_array[2]);
			break;
	case SUM:
			Sum1 = atoi(cmd_array[1]);
			break;
	case STARTC:
			Start_move.Start_move = True;
			printf("0 START, 0\n");
			break;
	case STOPC:
			Mode_ctrl = Mode_idle;
			printf("0 STOP, 0\n");
			break;
	case RST:
			if (Mtr == 1)
			{
				Hctl_rst_1.Hctl_rst_1= 0;
				Sleep(1);
				Hctl_rst_1.Hctl_rst_1= 1;
			}
			if (Mtr == 2)
			{
				Hctl_rst_2.Hctl_rst_2 = 0;
				Sleep(1);
				Hctl_rst_2.Hctl_rst_2 = 1;
			}
			Hctl_2032(Mtr);
			printf("Mtr %i RST %i\n\r", Mtr, Pos_encoder[Mtr]);
			break;
	default:
			printf("What the hell!\n");
	}
	strcpy(Ucommand_old, Ucommand);
	strcpy(Ucommand, "");
}
void Hctl_2032(byte Mtrnum)
{
	//needs to be implemented
}
void Calc_trapez(byte M)
{
	//needs to be implemented
}
void Timer_0(void)
{
	printf("Timer_01\n");
	if (Timer_pid == Pid_time)
	{
		Timer_pid = 0;
		for (Mtr_num = 1; Mtr_num < 3; Mtr_num++)
		{
			Motor_led.Motor_led = True;
			Hctl_2032(Mtr_num);
			Act_speed[Mtr_num] = Pos_encoder[Mtr_num] - Old_encoder[Mtr_num];
			Old_encoder[Mtr_num] = Pos_encoder[Mtr_num];
			New_speed[Mtr_num] = Pos_final[Mtr_num] - Pos_encoder[Mtr_num]; 
			Difference[Mtr_num] = New_speed[Mtr_num];
			if (Flag_velocity.Flag_velocity == True)
			{
				if (Pos_final[Mtr_num] >= Pos_encoder[Mtr_num])
					Dir_diff[Mtr_num] = True;
				if (Difference[Mtr_num] < 10000)
				{
					Acc_speed_p[Mtr_num] = New_speed[Mtr_num] / Factor_acc[Mtr_num];
					if (Acc_speed_p[Mtr_num] < 1)
						Acc_speed_p[Mtr_num] = 1;
					if (New_speed[Mtr_num] > Vmax_pos[Mtr_num])
						New_speed[Mtr_num] = Vmax_pos[Mtr_num];
					if (New_speed[Mtr_num] > Acc_speed_p[Mtr_num])
						New_speed[Mtr_num] = Acc_speed_p[Mtr_num];
				}
				else
				{
					Dir_diff[Mtr_num] = False;
					if (Difference[Mtr_num] >= -10000)
						Acc_speed_n[Mtr_num] = New_speed[Mtr_num] / Factor_acc[Mtr_num];
					if (Acc_speed_n[Mtr_num] > -1)
						Acc_speed_n[Mtr_num] = -1;
					if (New_speed[Mtr_num] < Vmax_neg[Mtr_num])
						New_speed[Mtr_num] = Vmax_neg[Mtr_num];
					if (New_speed[Mtr_num] < Acc_speed_n[Mtr_num])
						New_speed[Mtr_num] = Acc_speed_n[Mtr_num];
				}
			}
			else
			{
				if (Dir_diff[Mtr] = True)
				{
					New_encoder[Mtr_num] = Trapez_1[Mtr_num] - Pos_encoder[Mtr_num];
					New_encoder[Mtr_num] = Point_p1[Mtr_num] - New_encoder[Mtr_num];
					if (Pos_encoder[Mtr_num]< Trapez_1[Mtr_num])  Acc_speed[Mtr_num] = New_encoder[Mtr_num] / Factor_acc[Mtr_num];      
					if (Pos_encoder[Mtr_num] >= Trapez_2[Mtr_num]) Acc_speed[Mtr_num] = New_speed[Mtr_num] / Factor_acc[Mtr_num];
					if (Acc_speed[Mtr_num] < 1) Acc_speed[Mtr_num] = 1;
					if (New_speed[Mtr_num] > Vel_pos[Mtr_num])  New_speed[Mtr_num] = Vel_pos[Mtr_num];
					if (New_speed[Mtr_num] > Acc_speed[Mtr_num]) New_speed[Mtr_num] = Acc_speed[Mtr_num];
				}
				else
				{ }
			}


		}
	}


}