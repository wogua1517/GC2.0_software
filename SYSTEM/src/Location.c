#include "Location.h"

float Location_X = 0, Location_Y = 0, Locaton_Anger = 0 ;
float RX = 0,RY = 0,R = 0,R_Angle = 0,R_pidx_max = 700,vel_ratio = 1;
float TIM_delay = 0; //是否已经进入定时器中断的标志位
float pid_out_x = 0, pid_out_y = 0, pid_out_z = 0;
float vis_pid_out_x = 0, vis_pid_out_y = 0;
float rotate_pid_out_x = 0, rotate_pid_out_y = 0, rotate_pid_out_z = 0;

volatile __Position Reference_Position;				//参考坐标原点
 

// 计算要转动的角度
float turn_angle_caculate(float to_angle, float Now_car_angle)   // 计算所要转动的角度
{
    s16 E_k_angle, abs_angle;             // 相对角度和绝对角度
    E_k_angle = to_angle - Now_car_angle; //角度差=目标位置角度-当前位置角度
    abs_angle = Ops_abs(E_k_angle);          //角度差取绝对值
    if (abs_angle > (360 - abs_angle))    //判断角度差绝对值是否大于180度
    {
        if (E_k_angle > 0)               //角度差为正，且大于180度，说明需要逆时针旋转（360-角度差）度
            E_k_angle = abs_angle - 360; //规定逆时针旋转为负角度，故取负值转角，逆时针转动的角度
        else                             //角度差为负，且大于180度，说明需要顺时针旋转（360-角度差的绝对值）度
            E_k_angle = 360 - abs_angle; // 顺时针转动的角度
    }

    return E_k_angle;
}


float Ops_abs(float a)
{
    if(a >= 0)
        return a;
		else
        return -a;
}

void Location_TIM3_init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);            //使能TIM3的时钟
    TIM_TimeBaseInitStructure.TIM_Period = arr;                     //自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;                  //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); // 允许定时器3的更新中断
    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); // 初始化NVIC
    TIM_Cmd(TIM3, DISABLE);         // 不使能定时器3
}

void Location_TIM7_init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);            //使能TIM7的时钟
    TIM_TimeBaseInitStructure.TIM_Period = arr;                     //自动重装载值
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;                  //定时器分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStructure);

    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE); // 允许定时器7的更新中断
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure); // 初始化NVIC
    TIM_Cmd(TIM7, DISABLE);         // 不使能定时器7
}

// 定时器7中断服务函数
// 用于控制校准
void TIM7_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM7, TIM_IT_Update) == SET) // 更新中断
    {
			vis_pos_x_pid(final_x);
			vis_pos_y_pid(final_y);
			pos_z_pid(Locaton_Anger);
    }
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update); //清除中断标志位
}

// 定时器3中断服务函数
// 用于控制路径与定位
void TIM3_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET) // 更新中断
    {
        TIM_delay = 1; // 已经进入了中断
        pos_x_pid(Location_X);
        pos_y_pid(Location_Y);
        pos_z_pid(Locaton_Anger); // Locaton_Anger
		rotate_pos_x_pid();
		rotate_pos_z_pid();
    }
    TIM_ClearITPendingBit(TIM3, TIM_IT_Update); //清除中断标志位
}

/**
* @brief  去某个点	实现了是否使用绝对坐标
* @param  Point指针
* @return None.
*/

//车身定位到某个点
void Goto_Point(__POINT Target_Point,int en,float EX,float EY)
{
	Absolute_Location_Plot(Target_Point.pos.x + EX,Target_Point.pos.y + EY,Target_Point.pos.z);
}

//圆盘定位到某个点
void Alin_Point(__POINT Target_Point,int en,float EX,float EY)
{
	Absolute_Location_Plot(Target_Point.pos.x + EX - (D_disc * arm_sin_f32(Target_Point.pos.z/180*PI)),
						   Target_Point.pos.y + EY - (D_disc * arm_cos_f32(Target_Point.pos.z/180*PI)),
						   Target_Point.pos.z);
}

//车身经过某个点（不停止）
void Pass_Point(__POINT Target_Point,int en,float EX,float EY)
{
	Absolute_Pass_Plot(Target_Point.pos.x + EX,Target_Point.pos.y + EY,Target_Point.pos.z);
}

//以（rx,ry）为圆心，r为半径进行圆周运动（坐标为圆盘）
void Rotate_Point(float r, float rx, float ry, float end_angle, float v_rotate, float r_angle, float EX, float EY)
{
	Absolute_Rotate_Plot(r, rx+EX, ry+EY,end_angle,v_rotate,r_angle);
}

/**
* @brief  相对当前位置小范围变化
* @param  Point指针
* @return None.
*/

//相对目标位置定位移动
void Move_Relative(__POINT Target_Point,float a,float b, float c)
{
	Absolute_Location_Plot(Target_Point.pos.x+a,Target_Point.pos.y+b,Target_Point.pos.z+c);
}

//相对当前位置定位移动
void Location_Relative(float a,float b, float c)
{
	Absolute_Location_Plot(pos_x+a,pos_y+b,zangle+c);
}

//相对当前位置移动（但不停止，作为途经点）
void Pass_Relative(float a,float b, float c)
{
	Absolute_Pass_Plot(pos_x+a,pos_y+b,zangle+c);
}

//相对圆盘位置后退a（但不停止，作为途经点）
void Pass_Retreat(float a)
{
	Absolute_Pass_Plot(pos_x - a * arm_sin_f32(zangle_plot),pos_y - a * arm_cos_f32(zangle_plot),zangle);
}

//相对圆盘位置x方向前进a，y方向移动b（定位移动，会停止）
void Location_Front(float a,float b)
{
	Absolute_Location_Plot(pos_x + a * arm_sin_f32(zangle_plot) + b * arm_cos_f32(zangle_plot),
						   pos_y + a * arm_cos_f32(zangle_plot) + b * arm_sin_f32(zangle_plot),
						   zangle);
}

/**
* @brief  使用绝对位置移动
* @param  x坐标、y坐标、z角度
* @return None
*/
void Absolute_Location_Plot(float x, float y, float z)
{
	Location_Plot_plus(	x-Reference_Position.x,
						y-Reference_Position.y,
						z
	);			//使用与参考点的向量差进行移动
}
void Absolute_Pass_Plot(float x, float y, float z)
{
	Pass_Plot_plus(	x-Reference_Position.x,
						y-Reference_Position.y,
						z
	);			//使用与参考点的向量差进行移动
}
void Absolute_Rotate_Plot(float r, float rx, float ry, float end_angle, float v_rotate , float r_angle)
{
	Rotate_Plot_plus(r,rx-Reference_Position.x,ry-Reference_Position.y,end_angle,v_rotate,r_angle);			//使用与参考点的向量差进行移动
}
/////////////////////////////////////////////
//     定位函数
/////////////////////////////////////////////
void Location_Plot_plus(float x, float y, float z)
{
	Pass_Plot_plus(x,y,z);
    TIM_Cmd(TIM3, ENABLE);
    //  定位误差，误差1mm，1度，采用pid自己运算的速度
    while (Ops_abs(pos_x - x) > 2.0f || Ops_abs(pos_y - y) > 2.0f || Ops_abs(zangle - z) > 1.0f)
    {
        // 随着旋转坐标轴发生一系列的改变
        // 下面发送的速度是经过坐标变换的
        send((pid_out_x * arm_cos_f32(zangle_plot) - pid_out_y * (arm_sin_f32(zangle_plot))), (pid_out_x * arm_sin_f32(zangle_plot) + pid_out_y * arm_cos_f32(zangle_plot)),
             pid_out_z, 0, 0, 0, 0);
        if (Locaton_Anger == 180 || Locaton_Anger == -180)
        {
            if (Ops_abs(zangle + 180) < 1 && (Ops_abs(zangle - 180) < 1)&&Ops_abs(pos_x - x) < 1.0f&&Ops_abs(pos_y - y) < 1.0f)
            {
                break; 
            }
        }
    }
    ///////////////////////////////////////
    send(0, 0, 0, 0, 0, 0, 0); // 发送让小车停止的指令
    TIM_Cmd(TIM3, DISABLE);
    //data_commit_flag = 1;
    delay_ms(10);
}
/////////////////////////////////////////////
//		途经点函数
/////////////////////////////////////////////
void Pass_Plot_plus(float x, float y, float z)
{
    float Length = 0, cosx = 0, sinx = 0, V_slow = 0, Vz_slow = 0; // 这里的cosx和sinx后面要经过修正才能正常使用
    float V_Length_max = 0, V_X_max = 0, V_Y_max = 0;
    float cosx_PID = 0, sinx_PID = 0;

    Location_X = x;    // PID X 目标参数赋值
    Location_Y = y;    // PID Y 目标参数赋值
    Locaton_Anger = z; // PID Z 目标参数赋值
    //data_commit_flag = 0;
    Length = sqrt((x - pos_x) * (x - pos_x) + (y - pos_y) * (y - pos_y)); // 算出总的距离
    cosx = (x - pos_x) / Length;                                          //计算目标航向角的余弦值
    sinx = (y - pos_y) / Length;                                          //计算目标航向角的正弦值
    // 短距离的情况下
    if (Length > 500.0f) // 长距离大于0.5m的时候，采用从加速，匀速到减速的pid衔接pid
    {
        TIM_Cmd(TIM3, ENABLE); // 新的定位函数采用不同的pid
        // 加速
		V_slow = V_straight;
		if(turn_angle_caculate(z, zangle) > 20)
							Vz_slow = V_straight*0.375f; // z轴旋转速度的加速
        else if(turn_angle_caculate(z, zangle) < -20)
							Vz_slow = -V_straight*0.375f; // z轴旋转速度的加速
        
        // 匀速
        while (sqrt((x - pos_x) * (x - pos_x) + (y - pos_y) * (y - pos_y)) >350.0f) // 采用匀速的形式
        {
            if (TIM_delay == 1)
            {
                if (Ops_abs(turn_angle_caculate(z, zangle)) > 10) // 如果目标z轴和当前的角度相差大于10度
                    send((V_slow * cosx * arm_cos_f32(zangle_plot) - V_slow * sinx * arm_sin_f32(zangle_plot)),
                         (V_slow * cosx * arm_sin_f32(zangle_plot) + V_slow * sinx * arm_cos_f32(zangle_plot)), Vz_slow, 0, 0, 0, 0); // 发送经过修正后的自定义速度
                else
                    send((V_slow * cosx * arm_cos_f32(zangle_plot) - V_slow * sinx * arm_sin_f32(zangle_plot)),
                         (V_slow * cosx * arm_sin_f32(zangle_plot) + V_slow * sinx * arm_cos_f32(zangle_plot)), pid_out_z, 0, 0, 0, 0); // 角度较小，有pid进行解算
                TIM_delay = 0;
            }
        }
        // 减速衔接pid
        while (V_slow > V_Length_max)
        {
            if (TIM_delay)
            {
                V_slow -= 40;//40
                V_X_max = pid_out_x;                                        //估计PID解算X方向速度
                V_Y_max = pid_out_y;                                        //估计PID解算Y方向速度
                V_Length_max = sqrt(V_X_max * V_X_max + V_Y_max * V_Y_max); //估计PID解算最大合速度
                cosx_PID = V_X_max / V_Length_max;                          //用于衔接PID制动，计算当前PID航向角
                sinx_PID = V_Y_max / V_Length_max;                          //用于衔接PID制动，计算当前PID航向角
                                                                            //航向角修正，逐渐趋近PID解算的航向角     衔接PID制动 精度cosx、sinx （ +— 0.025）
                if (cosx > (cosx_PID + 0.01f))
                    cosx -= 0.01f;
                else if (cosx < (cosx_PID - 0.01f))
                    cosx += 0.01f;
                if (sinx > (sinx_PID + 0.01f))
                    sinx -= 0.01f;
                else if (sinx < (sinx_PID - 0.01f))
                    sinx += 0.01f;
                // 后面的z轴都是系统进行自我调节的
                send((V_slow * cosx * arm_cos_f32(zangle_plot) - V_slow * sinx * arm_sin_f32(zangle_plot)),
                     (V_slow * cosx * arm_sin_f32(zangle_plot) + V_slow * sinx * arm_cos_f32(zangle_plot)), pid_out_z, 0, 0, 0, 0);
                TIM_delay = 0; //清空进入标志位 等待10ms后再次进入
            }
        }
    }
    else
    {
        V_Length_max = V_pid; // 直接设置最大的速度进行加速
        TIM_Cmd(TIM3, ENABLE);                      // 开启定时器中断
        while (V_slow < V_Length_max)               // 设置一个最大的速度
        {
            if (TIM_delay == 1) // 已经进入过了定时器中断
            {
                V_slow += 10;
                if (turn_angle_caculate(Locaton_Anger, zangle) > 0)
                    Vz_slow += 10;
                else
                    Vz_slow -= 10;
                if (Ops_abs(turn_angle_caculate(Locaton_Anger, zangle)) > 10) // 如果目标z轴和当前的角度相差大于10度                                            //(500/20)*10ms=250ms加速至峰值
                    send((V_slow * cosx * arm_cos_f32(zangle_plot) - V_slow * sinx * arm_sin_f32(zangle_plot)),
                         (V_slow * cosx * arm_sin_f32(zangle_plot) + V_slow * sinx * arm_cos_f32(zangle_plot)), Vz_slow, 0, 0, 0, 0); // 发送经过修正后的自定义速度
                else
                    send((V_slow * cosx * arm_cos_f32(zangle_plot) - V_slow * sinx * arm_sin_f32(zangle_plot)),
                         (V_slow * cosx * arm_sin_f32(zangle_plot) + V_slow * sinx * arm_cos_f32(zangle_plot)), pid_out_z, 0, 0, 0, 0); // 发送经过修正后的自定义速度
                V_X_max = pid_out_x;                                                                                                    // 估计PID结算出来的x方向的速度
                V_Y_max = pid_out_y;                                                                                                    // 估计PID结算出来的y方向的速度
                V_Length_max = sqrt(V_X_max * V_X_max + V_Y_max * V_Y_max);                                                             //??PID???????
                TIM_delay = 0;                                                                                                          //清空已经进入的标志位，等待10ms再次进入定时器中断
            }
        }
    }
    TIM_Cmd(TIM3, DISABLE);
}
/////////////////////////////////////////////
//		路径（圆弧）函数
/////////////////////////////////////////////
void Rotate_Plot_plus(float r, float rx, float ry, float end_angle , float v_rotate , float r_angle)
{                                                                                           
	float end_a,end_b;	//退出函数的上下限
	RX=rx;
	RY=ry;
	R=r;
	R_Angle=r_angle;
	vel_ratio=Ops_abs(v_rotate)*3/1400;
	//R_pidx_max=Ops_abs(v_rotate)/1.1f;
	TIM_Cmd(TIM3, ENABLE); // 新的定位函数采用不同的pid
	//给定上下限
	if(v_rotate > 0) {
		end_a=end_angle + 2;
		end_b=end_angle - 20;
	}
	else {
		end_a=end_angle + 20;
		end_b=end_angle - 2;
	}
	while(end_a>180)  end_a -= 360;
	while(end_a<-180) end_a += 360;
	while(end_b>180)  end_b -= 360;
	while(end_b<-180) end_b += 360;
	//运动直到角度到达终点
	while (Trans_angle>end_a || Trans_angle < end_b)
	{
		send(rotate_pid_out_x*arm_cos_f32(Bias_zplot) - v_rotate*arm_sin_f32(Bias_zplot)	,
			 rotate_pid_out_x*arm_sin_f32(Bias_zplot) + v_rotate*arm_cos_f32(Bias_zplot)	, 
			 rotate_pid_out_z	, 0, 0, 0, 0); // 发送经过修正后的自定义速度
	}
	TIM_Cmd(TIM3, DISABLE);
}
void Rotate_Pass(float end_angle)
{                                                                                           
	float end_a,end_b;	//退出函数的上下限
	TIM_Cmd(TIM3, ENABLE); // 新的定位函数采用不同的pid
	//给定上下限
	end_a=end_angle + 30;
	end_b=end_angle - 30;
	while(end_a>180)  end_a -= 360;
	while(end_a<-180) end_a += 360;
	while(end_b>180)  end_b -= 360;
	while(end_b<-180) end_b += 360;
	//运动直到角度到达终点
	while (zangle>end_a || zangle < end_b)
	{	
		send(0,0,1000, 0, 0, 0, 0); // 发送经过修正后的自定义速度
	}
	TIM_Cmd(TIM3, DISABLE);
}
/////////////////////////////////////////////
//		视觉定位函数
/////////////////////////////////////////////
float center_x, center_y;
void Visual_Location_Plot(__POINT Target_Point)
{
	u8 cnt = 3;
	Locaton_Anger = Target_Point.pos.z;
    TIM_Cmd(TIM7, ENABLE);
	while(cnt != 0)
	{
		
		while(Ops_abs(center_x - final_x) > 0.5f || Ops_abs(center_y - final_y) > 0.5f || Ops_abs(zangle - Locaton_Anger) > 1.0f)
		{
			u8 xbuf[4];
			xbuf[0] = rxd_buf[0];
			xbuf[1] = rxd_buf[1];
			xbuf[2] = rxd_buf[2];
			xbuf[3] = rxd_buf[3];
					
			center_x = *((float*)&xbuf[0]);
		
			xbuf[0] = rxd_buf[4];
			xbuf[1] = rxd_buf[5];
			xbuf[2] = rxd_buf[6];
			xbuf[3] = rxd_buf[7];
					
			center_y = *((float*)&xbuf[0]);
			
			if(!center_x || !center_y)
				continue;
			
			
			// 随着旋转坐标轴发生一系列的改变
			// 下面发送的速度是经过坐标变换的
			send(vis_pid_out_x,vis_pid_out_y,
				pid_out_z, 0, 0, 0, 0);
		}
		
		send(0, 0, 0, 0, 0, 0, 0); // 发送让小车停止的指令
		BUZZER_ON;
		delay_ms(100);
		BUZZER_OFF;
		if(Ops_abs(center_x - final_x) > 0.5f || Ops_abs(center_y - final_y) > 0.5f || Ops_abs(zangle - Locaton_Anger) > 1.0f)
			cnt = 3;
		else
			cnt--;
	}
	
	
		
	///////////////////////////////////////
	TIM_Cmd(TIM7, DISABLE);
	//data_commit_flag = 1;
	delay_ms(10);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 用于视觉定位的X，Y的pid

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float vis_Bias_x, vis_Last_Bias_x, vis_Integral_bias_x;
void vis_pos_x_pid(float target) // 直
{
    vis_Bias_x = target - center_x;
    if (vis_Bias_x < 5 && vis_Bias_x > -5)
    {
        vis_Integral_bias_x += vis_Bias_x;
        if (vis_Integral_bias_x > 20)
            vis_Integral_bias_x = 20;
        else if (vis_Integral_bias_x < -20)
            vis_Integral_bias_x = -20;
    }
//    vis_pid_out_x = 6.5f * vis_Bias_x + 0.33f * vis_Integral_bias_x + 5.0f * (vis_Bias_x - vis_Last_Bias_x);
    vis_pid_out_x = 9.0f * vis_Bias_x + 0.66f * vis_Integral_bias_x + 7.0f * (vis_Bias_x - vis_Last_Bias_x);
	vis_Last_Bias_x = vis_Bias_x; //保存上一次的误差

    if (vis_pid_out_x < 10 && vis_pid_out_x > 0)
        vis_pid_out_x = 0;
    else if (vis_pid_out_x > -10 && vis_pid_out_x < 0)
        vis_pid_out_x = 0;
    if (vis_pid_out_x > 550)
        vis_pid_out_x = 550;
    else if (vis_pid_out_x < -550)
        vis_pid_out_x = -550;
}
float vis_Bias_y, vis_Last_Bias_y, vis_Integral_bias_y;
void vis_pos_y_pid(float target) // 直
{
    vis_Bias_y = target - center_y;
    if (vis_Bias_y < 5 && vis_Bias_y > -5)
    {
        vis_Integral_bias_y += vis_Bias_y;
        if (vis_Integral_bias_y > 20)
            vis_Integral_bias_y = 20;
        else if (vis_Integral_bias_y < -20)
            vis_Integral_bias_y = -20;
    }
    vis_pid_out_y = -6.5f * vis_Bias_y - 0.33f * vis_Integral_bias_y - 5.0f * (vis_Bias_y - vis_Last_Bias_y);
//    vis_pid_out_y = -13.0f * vis_Bias_y - 0.66f * vis_Integral_bias_y - 10.0f * (vis_Bias_y - vis_Last_Bias_y);
	vis_Last_Bias_y = vis_Bias_y; //保存上一次的误差

    if (vis_pid_out_y < 10 && vis_pid_out_y > 0)
        vis_pid_out_y = 0;
    else if (vis_pid_out_y > -10 && vis_pid_out_y < 0)
        vis_pid_out_y = 0;
    if (vis_pid_out_y > 550)
        vis_pid_out_y = 550;
    else if (vis_pid_out_y < -550)
        vis_pid_out_y = -550;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 用于路径（圆弧）运动Y，Z的pid

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float rotate_Bias_x, rotate_Last_Bias_x, rotate_Integral_bias_x;
void rotate_pos_x_pid(void) // 直
{
    rotate_Bias_x = R - sqrt((RX - Trans_x) * (RX - Trans_x) + (RY - Trans_y) * (RY - Trans_y));
    if (rotate_Bias_x < 5 && rotate_Bias_x > -5)
    {
        rotate_Integral_bias_x += rotate_Bias_x;
        if (rotate_Integral_bias_x > 20)
            rotate_Integral_bias_x = 20;
        else if (rotate_Integral_bias_x < -20)
            rotate_Integral_bias_x = -20;
    }
//    rotate_pid_out_x = 5.0f * rotate_Bias_x + 0.0f * rotate_Integral_bias_x + 0.00f * (rotate_Bias_x - rotate_Last_Bias_x);
    rotate_pid_out_x = (20.0f * rotate_Bias_x + 1.0f * rotate_Integral_bias_x + 10.0f * (rotate_Bias_x - rotate_Last_Bias_x));
	rotate_pid_out_x = vel_ratio*rotate_pid_out_x;
	rotate_Last_Bias_x = rotate_Bias_x; //保存上一次的误差

    if (rotate_pid_out_x < 10 && rotate_pid_out_x > 0)
        rotate_pid_out_x = 0;
    else if (rotate_pid_out_x > -10 && rotate_pid_out_x < 0)
        rotate_pid_out_x = 0;
    if (rotate_pid_out_x > vel_ratio*R_pidx_max)
        rotate_pid_out_x = vel_ratio*R_pidx_max;
    else if (rotate_pid_out_x < 0-vel_ratio*R_pidx_max)
        rotate_pid_out_x = 0-vel_ratio*R_pidx_max;
}
float rotate_Bias_z, rotate_Last_Bias_z, rotate_Integral_bias_z;
void rotate_pos_z_pid(void) // 直
{
	rotate_Bias_z = Bias_zangle;
	while(rotate_Bias_z<-180) rotate_Bias_z += 360;
	while(rotate_Bias_z>180)  rotate_Bias_z -= 360;
    if (rotate_Bias_z < 5 && rotate_Bias_z > -5)
    {
        rotate_Integral_bias_z += rotate_Bias_z;
        if (rotate_Integral_bias_z > 20)
            rotate_Integral_bias_z = 20;
        else if (rotate_Integral_bias_z < -20)
            rotate_Integral_bias_z = -20;
    }
    rotate_pid_out_z = 6.5f * rotate_Bias_z + 0.0f * rotate_Integral_bias_z + 0.0f * (rotate_Bias_z - rotate_Last_Bias_z);
    rotate_pid_out_z = vel_ratio*rotate_pid_out_z;
	//rotate_pid_out_z = 0.05f * rotate_Bias_z + 0.0f * rotate_Integral_bias_z + 0.0f * (rotate_Bias_z - rotate_Last_Bias_z);
	rotate_Last_Bias_z = rotate_Bias_z; //保存上一次的误差

//    if (rotate_pid_out_z < 10 && rotate_pid_out_z > 0)
//        rotate_pid_out_z = 0;
//    else if (rotate_pid_out_z > -10 && rotate_pid_out_z < 0)
//        rotate_pid_out_z = 0;
    if (rotate_pid_out_z > vel_ratio*700)
        rotate_pid_out_z = vel_ratio*700;
    else if (rotate_pid_out_z < 0-vel_ratio*700)
        rotate_pid_out_z = 0-vel_ratio*700;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// 用于定点运动的X，Y，Z的pid

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float Bias_x, Last_Bias_x, Integral_bias_x;
void pos_x_pid(float target) // 横
{
    Bias_x = target - pos_x;
    if (Bias_x < 5 && Bias_x > -5)
    {
        Integral_bias_x += Bias_x;
        if (Integral_bias_x > 20)
            Integral_bias_x = 20;
        else if (Integral_bias_x < -20)
            Integral_bias_x = -20;
    }
    pid_out_x = 6.5f * Bias_x + 0.33f * Integral_bias_x + 5.0f * (Bias_x - Last_Bias_x);
    Last_Bias_x = Bias_x; //保存上一次的误差

    if (pid_out_x < 10 && pid_out_x > 0)
        pid_out_x = 0;
    else if (pid_out_x > -10 && pid_out_x < 0)
        pid_out_x = 0;
    if (pid_out_x > 550)
        pid_out_x = 550;
    else if (pid_out_x < -550)
        pid_out_x = -550;
}
float Bias_y, Last_Bias_y, Integral_bias_y;
void pos_y_pid(float target) // 直
{
    Bias_y = target - pos_y;
    if (Bias_y < 5 && Bias_y > -5)
    {
        Integral_bias_y += Bias_y;
        if (Integral_bias_y > 20)
            Integral_bias_y = 20;
        else if (Integral_bias_y < -20)
            Integral_bias_y = -20;
    }
    pid_out_y = 6.5f * Bias_y + 0.33f * Integral_bias_y + 5.0f * (Bias_y - Last_Bias_y);
    Last_Bias_y = Bias_y; //保存上一次的误差

    if (pid_out_y < 10 && pid_out_y > 0)
        pid_out_y = 0;
    else if (pid_out_y > -10 && pid_out_y < 0)
        pid_out_y = 0;
    if (pid_out_y > 550)
        pid_out_y = 550;
    else if (pid_out_y < -550)
        pid_out_y = -550;
}
float Bias_z, Last_Bias_z, Bias_Intergral = 0;
void pos_z_pid(int target)
{

    Bias_z = turn_angle_caculate(target, zangle);

    if (Bias_z < 5 && Bias_z > -5)
    {
        Bias_Intergral = Bias_Intergral + Bias_z;
        if (Bias_Intergral > 2000)
            Bias_Intergral = 2000;
        else if (Bias_Intergral < -2000)
            Bias_Intergral = -2000;
    }

    pid_out_z = 28 * Bias_z + 0 * Bias_Intergral + 40 * (Bias_z - Last_Bias_z);
    Last_Bias_z = Bias_z; //保存上一次的误差

    if (pid_out_z < 10 && pid_out_z > 0)
        pid_out_z = 0;
    if (pid_out_z < 0 && pid_out_z > -10)
        pid_out_z = -0;

    if (pid_out_z > 500)
        pid_out_z = 500;
    else if (pid_out_z < -500)
        pid_out_z = -500;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
