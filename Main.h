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
/*
'---------------------- VARIABLES ----------------------------------------------
Dim Pid_kp(2) As Long, Pid_ki(2) As Long, Pid_kd(2) As Long
Dim Pid_scale(2) As Long
Dim Pid_prev_error(2) As Long
Dim Pid_integral_error(2) As Long
Dim Pid_error(2) As Long
Dim Ptemp(2) As Long
Dim Motor_setpoint(2) As Long
Dim Pid_out(2) As Long       'value to assign pwm
Dim Timer_pid As Byte       'sample timer pid
Dim Pos_encoder(2) As Long       'encoder position
Dim Old_encoder(2) As Long       'previous encoder position
Dim New_speed(2) As Long       'velocity calculated in  pid
Dim Act_speed(2) As Long       'velocity actual
Dim Acc_speed(2) As Long       'velocity acceleration
Dim Pos_final(2) As Long       'position trapezoidale final
Dim Temp_enc As Word       'Long KJL      'temp encoder position
Dim Ucommand As String * 30
Dim Ucommand_old As String * 30
Dim Char(20) As Byte
Dim Idt As Byte : Idt = 1
Dim N_cmd As Byte
Dim Cmd_ar(5) As String * 10
Dim Mode_ctrl As Byte       'mode control
Dim Mtrnum_kp As Long
Dim Mtrnum_ki As Long
Dim Mtrnum_kd As Long
Dim Vel_pos(2) As Long
Dim Vel_neg(2) As Long
Dim Vel_last(2) As Long
Dim Rpwm(2) As Byte
Dim Factor_acc(2) As Long
Dim Point_p1(2) As Long
Dim Dir_diff(2) As Byte
Dim Diff_position(2) As Long
Dim Trapez_1(2) As Long
Dim Trapez_2(2) As Long
Dim New_encoder(2) As Long
Dim Deg(2) As Single
Dim Rad(2) As Single
Dim Vel_max(2) As Single
Dim Diff_2(2) As Long
Dim Max_pwm(2) As Byte
Dim Pwm_temp As Byte
Dim Dir_temp As Byte
Dim Mtr_num As Byte
Dim Mtr As Byte
Dim Start_move As Bit
Dim Tx_enable As Bit       'if Flag=1 alert Main do-loop to print Str-tx_1
Dim Str_tx_1 As String * 20       'print in Main do-loop if Tx-enable = 1
Dim Flag_velocity As Bit
Dim Sum1 As Long
Dim Vmax_pos(2) As Long
Dim Vmax_neg(2) As Long
Dim Factor_ang(2) As Long       ' angular factor 0-99
Dim Difference(2) As Long       ' difference
Dim Acc_speed_p(2) As Long       'acceleration positive x control velocity
Dim Acc_speed_n(2) As Long       'acceleration negative x control velocity
Dim Pid_time As Byte       '     'time max pid overflow
Dim Mstop_ctr(2) As Byte       '   'number of Motionless Motor reading time
Dim Temp_bit As Bit
*/
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
long Pos_encoder[4];
long Old_encoder[3];
long New_speed[3];
long Act_speed[3];
long Acc_speed[3];
long Pos_final[3];
long * Temp_enc;
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
/*
'------------------- ALIAS -----------------------------------------------------
M1_pwm Alias Pwm1a : M1_pwm = 0       'pwm out Motor x
M2_pwm Alias Pwm1b : M2_pwm = 0       'pwm out Motor y
Ddrd.2 = 0 : Portd.2 = 1       'pullup x int 0
Ddrd.3 = 0 : Portd.3 = 1       'pullup x int 1
Limit_1 Alias Pinb.0 : Ddrb.0 = 0       'limit 1 - Interrupt 1
Limit_2 Alias Pinb.1 : Ddrb.1 = 0       'limit 2 - Interupt 2
Aux_0 Alias Pinb.2 : Ddrb.2 = 0 : Portb.2 = 1       '*pullup aux 0 *CHANGE-to (X)&(Y)*
Aux_1 Alias Pinb.3 : Ddrb.3 = 0 : Portb.3 = 1       '*pullup aux 1 *encoder overflow *
Stop1_bit Alias Portb.4 : Ddrb.4 = 1       '*status Motor 1 *CHANGE-to encoder(X)&(Y)*
Stop2_bit Alias Portc.6 : Ddrc.6 = 1       '*status Motor 2 *direction.*
M1dir_bit Alias Portd.6 : Ddrd.6 = 1       'direction Motor x
M2dir_bit Alias Portd.7 : Ddrd.7 = 1       'direction Motor y
Motor_led Alias Portc.7 : Ddrc.7 = 1       'led test
Test1_led Alias Portf.7 : Ddrf.7 = 1       'KJL Added
Hctl_xy Alias Portc.0 : Ddrc.0 = 1       'selection xy hctl2032
Hctl_sel1 Alias Portc.1 : Ddrc.1 = 1       'selection byte bit 0
Hctl_sel2 Alias Portc.2 : Ddrc.2 = 1       'selection byte bit 1
Hctl_oe Alias Portc.3 : Ddrc.3 = 1       'selecion oe hctl data
Hctl_rst_1 Alias Portc.4 : Ddrc.4 = 1       'reset encoder x
Hctl_rst_2 Alias Portc.5 : Ddrc.5 = 1       'reset encoder y
Hctl_data Alias Pina : Ddra = &H00       'porta input data encoder
'-------------------------------------------------------------------------------
Hctl_xy = 0         'selection encoder x
Hctl_sel1 = 0       'selection first byte msb
Hctl_sel2 = 1       'selection first byte msb
Hctl_oe = 1         'no output enable
Hctl_rst_1 = 1      'no reset
Hctl_rst_2 = 1      'no reset
*/
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

struct
{
	byte M1_dir_bit : 1;
}M1_dir_bit;

struct
{
	byte M2_dir_bit : 1;
}M2_dir_bit;

struct
{
	byte Hctl_xy : 1;
}Hctl_xy;
struct
{
	byte Hctl_oe : 1;
}Hctl_oe;

struct
{
	byte Hctl_sel1 : 1;
}Hctl_sel1;
struct
{
	byte Hctl_sel2 : 1;
}Hctl_sel2;

struct
{
	byte Hctl_data : 1;
}Hctl_data;
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
