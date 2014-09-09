#include "targetver.h"
#include <windows.h>
#include <stdio.h>
#include <tchar.h>
#include <string>
#include <math.h>

char ucommand[30];

char ucommand_old[30];
#define True 1
//Const True = 1
#define False 0
//#define Mode_trp 2
long Pid_kp[3];
long Pid_ki[3];
long Pid_kd[3];
long Pid_scale[3];
long Pid_prev_error[3];
long Pid_integral_error[3];
long Pid_error[3];
long Ptemp[3];
long Motor_setpoint[3];
long Pid_out[3];
BYTE Timer_pid;
long Pos_encoder[3];
long Old_encoder[3];
long New_speed[3];
long Act_speed[3];
long Acc_speed[3];
long Pos_final[3];
WORD Temp_enc;
char Ucommand[30];
char Ucommand_old[30];
char Char[20];
BYTE Idt = 1;
BYTE N_cmd;
char Cmd_ar[50];
BYTE Mode_ctrl;
long Mtrnum_kp;
long Mtrnum_ki;
long Mtrnum_kd;
long Vel_pos[3];
long Vel_neg[3];
long Vel_last[3];
BYTE Rpwm[3];
long Factor_acc[3];
long Point_p1[3];
BYTE Dir_diff[3];
long Diff_position[3];
long Trapez_1[3];
long Trapez_2[3];
long New_encoder[3];
float Deg[3];
float Rad[3];
float Vel_max[3];
long Diff_2[3];
BYTE Max_pwm[3];
BYTE Pwm_temp;
BYTE Dir_temp;
BYTE Mtr_num;
BYTE Mtr;

char Str_tx_1[50];   //       'print in Main do-loop if Tx-enable = 1
long Sum1;
long Vmax_pos[3];
long Vmax_neg[3];
long Factor_ang[3];
long Difference[3];
long Acc_speed_p[3];
long Acc_speed_n[3];
BYTE Pid_time;
BYTE Mstop_ctr[3];
short COMPAREA;
short COMPAREB;
short Pwm1a;
short Pwm1b;
short M1_pwm;
short M2_pwm;
void Rs232(void);

struct
{
	byte Temp_bit : 1;
}Temp_bit;

struct
{
	byte Stop1_bit : 1;
}Stop1_bit;
struct
{
	byte Stop2_bit : 1;
}Stop2_bit;

struct 
{
	byte Test1_led : 1;
}Test1_led;

struct 
{
	byte Tx_enable : 1;
}Tx_enable;
struct 
{
	byte Hctl_rst_1 : 1;
}Hctl_rst_1;
struct 
{
	byte Hctl_rst_2 : 1;
}Hctl_rst_2;
struct 
{
	byte Start_move : 1;
}Start_move;
struct 
{
	byte Motor_led : 1;
}Motor_led;
struct
{
	byte Flag_velocity : 1;
}Flag_velocity;

enum command {
	RSTF,
	MADE,
	VER,
	SKP,
	SKI,
	SKD,
	GO,
	VMAX,
	GMAX,
	SVM,
	GVM,
	GVE,
	ENC,
	SPID,
	GPID,
	STIME,
	GTIME,
	SZP,
	IDLE,
	GSM,
	SMODE,
	MODE,
	GMODE,
	SVEL,
	VELO,
	SPWM,
	GPWM,
	RPWM,
	SACC,
	GACC,
	SANG,
	GANG,
	POS,
	MOVE,
	VELP,
	SUM,
	STARTC,
	STOPC,
	RST
};
