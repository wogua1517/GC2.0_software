#include "System_Config.h"
#include "arm_math.h"
#include "math.h"
#include "Points.h"


#define pos_x   -OPS.ActVal[4]
#define pos_y   -OPS.ActVal[5]
#define zangle  -OPS.ActVal[1]
#define xangle  -OPS.ActVal[2]
#define yangle  -OPS.ActVal[3]
#define w_z     -OPS.ActVal[6]
#define zangle_plot zangle/180*PI
#define	Trans_x  (pos_x + Len * arm_sin_f32(zangle_plot))
#define	Trans_y  (pos_y + Len * arm_cos_f32(zangle_plot))
#define Trans_angle 180 * atan2(Trans_x - RX,Trans_y - RY) / PI
#define Bias_zangle (Trans_angle + R_Angle - zangle)
#define Bias_zplot  ((Bias_zangle+90+R_Angle)/180*PI)

extern volatile __Position Reference_Position;
extern float center_x, center_y;

float Ops_abs(float a);
float turn_angle_caculate(float to_angle, float Now_car_angle);

void Location_TIM3_init(u16 arr,u16 psc);
void Location_TIM7_init(u16 arr,u16 psc);
void TIM3_IRQHandler(void);
void TIM7_IRQHandler(void);

void Goto_Point(__POINT Target_Point,int en,float EX,float EY);
void Alin_Point(__POINT Target_Point,int en,float EX,float EY);
void Pass_Point(__POINT Target_Point,int en,float EX,float EY);
void Rotate_Point(float r, float rx, float ry, float end_angle, float v_rotate, float r_angle, float EX, float EY);

void Absolute_Location_Plot(float x, float y, float z);
void Absolute_Pass_Plot(float x, float y, float z);
void Absolute_Rotate_Plot(float r, float rx, float ry, float end_angle, float v_rotate , float r_angle);

void Move_Relative(__POINT Target_Point,float a,float b, float c);
void Location_Relative(float a,float b, float c);
void Pass_Relative(float a,float b, float c);

void Pass_Retreat(float a);
void Location_Front(float a,float b);

void Location_Plot_plus(float x,float y,float z);
void Visual_Location_Plot(__POINT Target_Point);
void Pass_Plot_plus(float x, float y, float z);
void Rotate_Plot_plus(float r, float rx, float ry, float end_angle, float v_rotate , float r_angle);
void Rotate_Pass(float end_angle);

void vis_pos_y_pid(float target);
void vis_pos_x_pid(float target);

void pos_y_pid(float target);
void pos_x_pid(float target);
void pos_z_pid(int target);

void rotate_pos_x_pid(void);
void rotate_pos_z_pid(void);
