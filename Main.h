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
#define Mode_trp 2
long Pid_kp[2];
long Pid_ki[2];
long Pid_kd[2];
long Pid_scale[2];
long Pid_prev_error[2];
long Pid_integral_error[2];
long Pid_error[2];
long Ptemp[2];
long Motor_setpoint[2];
long Pid_out[2];
BYTE Timer_pid;
long Pos_encoder[2];
long Old_encoder[2];
long New_speed[2];
long Act_speed[2];
long Acc_speed[2];
long Pos_final[2];
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
long Vel_pos[2];
long Vel_neg[2];
long Vel_last[2];
BYTE Rpwm[2];
long Factor_acc[2];
long Point_p1[2];
BYTE Dir_diff[2];
long Diff_position[2];
long Trapez_1[2];
long Trapez_2[2];
long New_encoder[2];
float Deg[2];
float Rad[2];
float Vel_max[2];
long Diff_2[2];
BYTE Max_pwm[2];
BYTE Pwm_temp;
BYTE Dir_temp;
BYTE Mtr_num;
BYTE Mtr;


char Str_tx_1[20];   //       'print in Main do-loop if Tx-enable = 1
long Sum1;
long Vmax_pos[2];
long Vmax_neg[2];
long Factor_ang[2];
long Difference[2];
long Acc_speed_p[2];
long Acc_speed_n[2];
BYTE Pid_time;
BYTE Mstop_ctr[2];
short COMPAREA;
short COMPAREB;
short Pwm1a;
short Pwm1b;
short M1_pwm;
short M2_pwm;
void Rs232(void);

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
