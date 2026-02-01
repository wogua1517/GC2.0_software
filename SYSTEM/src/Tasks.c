#include "System_Config.h"
#include "Tasks.h"

int position_color[5],position_letter[3],identify_color = 0,DT_rotate;
float	EX,EY;	//记录误差
void(* tasks[30])();

//校准函数
void alin(__POINT Target_Point)
{


	center_x = center_y = 0;
	//delay_ms(100);
	
	Visual_Location_Plot(Target_Point);
	
	EX=pos_x-(Target_Point.pos.x + EX - (D_disc * arm_sin_f32(Target_Point.pos.z/180*PI)));
	EY=pos_y-(Target_Point.pos.y + EY - (D_disc * arm_cos_f32(Target_Point.pos.z/180*PI)));
	
}
void Alin(__POINT Target_Point,int position)
{
	Alin_Point(Target_Point,0,EX,EY);           //到点
	moveServo(1,position+100,R_Speed);			//转盘
	alin(Target_Point);							//矫正
	
	moveServo(1,position,R_Speed);				//转盘
	
	//延时
	getPosition(1);
	receiveHandle();
	while(abs(now_position-position)>=30) {
		getPosition(1);	
		receiveHandle();	
	}
	
	Location_Front(Move_distance,0);			//前进
	if(Target_Point.pos.x == points[ 8].pos.x
	 ||Target_Point.pos.x == points[10].pos.x
	 ||Target_Point.pos.x == points[11].pos.x)
		Pass_Retreat(400);							//后退
}
void Alin_high(__POINT Target_Point,int position,int height)
{
	if(position==POSITION_4)
		moveServo(1,position-100,R_Speed);			//转盘
	else
		moveServo(1,position+100,R_Speed);			//转盘
	moveServo(2,POSITION_MID,500);	
	Emm_V5_Pos_Control(1, 0, 1000, 0, height+3000, true, false);
	if(Target_Point.pos.x == points[3].pos.x)	
		Alin_Point(Target_Point,0,EX+30,EY+5);           //到点
	else
		Alin_Point(Target_Point,0,EX,EY);           //到点
	moveServo(2,POSITION_DOWN,50);	
	//延时
	getPosition(2);
	receiveHandle();
	while(abs(now_position-POSITION_DOWN)>=10) {
		getPosition(2);	
		receiveHandle();	
	}		
	Emm_V5_Pos_Control(1, 0, 1000, 0, height, true, false);	
		
	alin(Target_Point);							//矫正
	moveServo(1,position,R_Speed);				//转盘
	
	//延时
	getPosition(1);
	receiveHandle();
	while(abs(now_position-position)>=30) {
		getPosition(1);	
		receiveHandle();	
	}
	
	Location_Front(Move_distance,0);			//前进
	Pass_Retreat(400);							//后退
}
void Alin_low(__POINT Target_Point,int position,int height)
{
	moveServo(1,position+100,R_Speed);			//转盘
	moveServo(2,POSITION_MID,500);		
	Alin_Point(Target_Point,0,EX,EY);           //到点
	moveServo(2,POSITION_DOWN,50);			
	
	//延时
	getPosition(2);
	receiveHandle();
	while(abs(now_position-POSITION_DOWN)>=10) {
		getPosition(2);	
		receiveHandle();	
	}		
	
	Emm_V5_Pos_Control(1, 0, 1000, 0, height, true, false);	
	
	alin(Target_Point);							//矫正
	moveServo(1,position,R_Speed);				//转盘
	
	//延时
	getPosition(1);
	receiveHandle();
	while(abs(now_position-position)>=30) {
		getPosition(1);	
		receiveHandle();	
	}
	
	Location_Front(Move_distance,0);			//前进
	Pass_Retreat(400);							//后退
}
//颜色识别函数
int get_color(void)
{       							
        u8 color = 0;
//        u8 cnt = 0;
//		int temp = 0;
		
        //delay_ms(500);
    
		BUZZER_ON;            
//        while(cnt < 10)
//        {
                if(rxd_flag)
                {
                        u8 xbuf[4];
                        xbuf[0] = rxd_buf[0];
                        xbuf[1] = rxd_buf[1];
                        xbuf[2] = rxd_buf[2];
                        xbuf[3] = rxd_buf[3];
                                                            
                        color = *((int*)&xbuf[0]);
                        
//                        if(temp == color&&color!=0)
//							cnt++;
//						else
//							cnt = 0;  
//						temp = color;
                }
//        }
//		color = temp ;
		delay_ms(100);
		BUZZER_OFF;            
        return color;
}
//二维码识别函数
void qrcode_scan(__GOOD __goods[],u8 code)
{
	
//	delay_ms(1000);//等待二维码识别完成
	switch(code)
		{
		case 1:	//黑白红绿蓝		ABC
			__goods[1].color = COLOR_BLACK;
			__goods[2].color = COLOR_WHITE;
			__goods[3].color = COLOR_RED;
			__goods[4].color = COLOR_GREEN;
			__goods[5].color = COLOR_BLUE;
			__goods[1].letter = A;
			__goods[2].letter = B;
			__goods[3].letter = C;
			break;
		case 2:	//白黑红绿蓝		ACB
			__goods[1].color = COLOR_WHITE;
			__goods[2].color = COLOR_BLACK;
			__goods[3].color = COLOR_RED;
			__goods[4].color = COLOR_GREEN;
			__goods[5].color = COLOR_BLUE;
			__goods[1].letter = A;
			__goods[2].letter = C;
			__goods[3].letter = B;
			break;
		case 3:	//白黑绿红蓝		BAC
			__goods[1].color = COLOR_WHITE;
			__goods[2].color = COLOR_BLACK;
			__goods[3].color = COLOR_GREEN;
			__goods[4].color = COLOR_RED;
			__goods[5].color = COLOR_BLUE;
			__goods[1].letter = B;
			__goods[2].letter = A;
			__goods[3].letter = C;
			break;
		case 4:	//蓝白黑红绿		BCA
			__goods[1].color = COLOR_BLUE;
			__goods[2].color = COLOR_WHITE;
			__goods[3].color = COLOR_BLACK;
			__goods[4].color = COLOR_RED;
			__goods[5].color = COLOR_GREEN;
			__goods[1].letter = B;
			__goods[2].letter = C;
			__goods[3].letter = A;
			break;
		case 5:	//白红蓝黑绿		CAB
			__goods[1].color = COLOR_WHITE;
			__goods[2].color = COLOR_RED;
			__goods[3].color = COLOR_BLUE;
			__goods[4].color = COLOR_BLACK;
			__goods[5].color = COLOR_GREEN;
			__goods[1].letter = C;
			__goods[2].letter = A;
			__goods[3].letter = B;
			break;
		case 6:	//黑红蓝白绿		CBA
			__goods[1].color = COLOR_BLACK;
			__goods[2].color = COLOR_RED;
			__goods[3].color = COLOR_BLUE;
			__goods[4].color = COLOR_WHITE;
			__goods[5].color = COLOR_GREEN;
			__goods[1].letter = C;
			__goods[2].letter = B;
			__goods[3].letter = A;
			break;
		case 7:	//蓝绿黑白红
			__goods[1].color = COLOR_BLUE;
			__goods[2].color = COLOR_GREEN;
			__goods[3].color = COLOR_BLACK;
			__goods[4].color = COLOR_WHITE;
			__goods[5].color = COLOR_RED;
			break;		
		case 8: //绿白蓝黑红
			__goods[1].color = COLOR_GREEN;
			__goods[2].color = COLOR_WHITE;
			__goods[3].color = COLOR_BLUE;
			__goods[4].color = COLOR_BLACK;
			__goods[5].color = COLOR_RED;
			break;
		case 9:	//白绿黑蓝红
			__goods[1].color = COLOR_WHITE;
			__goods[2].color = COLOR_GREEN;
			__goods[3].color = COLOR_BLACK;
			__goods[4].color = COLOR_BLUE;
			__goods[5].color = COLOR_RED;
			break;
		case 10://黑红蓝绿白
			__goods[1].color = COLOR_BLACK;
			__goods[2].color = COLOR_RED;
			__goods[3].color = COLOR_BLUE;
			__goods[4].color = COLOR_GREEN;
			__goods[5].color = COLOR_WHITE;
			break;
		case 11://红蓝绿黑白
			__goods[1].color = COLOR_RED;
			__goods[2].color = COLOR_BLUE;
			__goods[3].color = COLOR_GREEN;
			__goods[4].color = COLOR_BLACK;
			__goods[5].color = COLOR_WHITE;
			break;
		case 12://绿红黑蓝白
			__goods[1].color = COLOR_GREEN;
			__goods[2].color = COLOR_RED;
			__goods[3].color = COLOR_BLACK;
			__goods[4].color = COLOR_BLUE;
			__goods[5].color = COLOR_WHITE;
			break;
		case 13://白红蓝绿黑
			__goods[1].color = COLOR_WHITE;
			__goods[2].color = COLOR_RED;
			__goods[3].color = COLOR_BLUE;
			__goods[4].color = COLOR_GREEN;
			__goods[5].color = COLOR_BLACK;
			break;
		case 14://红绿白蓝黑
			__goods[1].color = COLOR_RED;
			__goods[2].color = COLOR_GREEN;
			__goods[3].color = COLOR_WHITE;
			__goods[4].color = COLOR_BLUE;
			__goods[5].color = COLOR_BLACK;
			break;
		case 15://蓝白绿红黑
			__goods[1].color = COLOR_BLUE;
			__goods[2].color = COLOR_WHITE;
			__goods[3].color = COLOR_GREEN;
			__goods[4].color = COLOR_RED;
			__goods[5].color = COLOR_BLACK;
			break;
		case 16://绿蓝红白黑
			__goods[1].color = COLOR_GREEN;
			__goods[2].color = COLOR_BLUE;
			__goods[3].color = COLOR_RED;
			__goods[4].color = COLOR_WHITE;
			__goods[5].color = COLOR_BLACK;	
			break;
		default:
			break;
		}
		
}
//出发
void Task_1(void)
{
	EX=0;EY=0;							//误差清零
	moveServo(2,POSITION_DOWN,1000);	//放下圆盘
	//Pass_Point(points[1],0,EX,EY);		
}
//扫码二
void Task_2(void)
{
    Goto_Point(points[2],0,EX,EY);      	
	while(code == 0);	//等待识别二维码
	qrcode_scan(goods,code);
	Rotate_Pass(90);
}
//3-5为抓取CBA
void Task_3(void)
{
	moveServo(1,POSITION_2,R_Speed_load);				//准备抓取物块3
	Rotate_Point(RD-50,615,1075,150,V_rotate,-90,EX,EY);//绕弧线抓取3
	Rotate_Point(RD,615,1075,105,V_rotate,-90,EX,EY);	//绕弧线抓取3
    position_letter[goods[3].letter]=POSITION_2;		//记录3位置
	code = 0;											//清空二维码
}

void Task_4(void)
{
	moveServo(1,POSITION_3,R_Speed_load);				//准备抓取物块2
	Rotate_Point(RD,615,1075,75,V_rotate,-90,EX,EY);	//绕弧线抓取2
	position_letter[goods[2].letter]=POSITION_3;        //记录2位置
}                                               
void Task_5(void)
{
	moveServo(1,POSITION_4,R_Speed_load);       		//准备抓取物块1
	Rotate_Point(RD,615,1075,50,V_rotate,-90,EX,EY);	//绕弧线抓取1
    position_letter[goods[1].letter]=POSITION_4;        //记录1位置
	moveServo(1,POSITION_5,R_Speed_load);       		//准备放置
	Rotate_Point(RD,615,1075,45,V_rotate,-90,EX,EY);	//绕弧线抓取1
}
//放置亚冠季						
void Task_6(void)
{
	Flash_ON();	
	USART_SendData(USART2, 0);
	while ((USART2->SR & 0x40) == 0);			// 退出现有程序
	USART_SendData(USART2, 2);
	while ((USART2->SR & 0x40) == 0);
    Alin_high(points[3],position_letter[B],Height_1);	//亚
    Alin_high(points[4],position_letter[A],Height_2);	//冠
    Alin_low (points[5],position_letter[C],1000	 );		//季
	Flash_OFF();
	USART_SendData(USART2, 0);
	while ((USART2->SR & 0x40) == 0);
	Emm_V5_Pos_Control(1, 0, 1000, 0, 3000, true, false);	
}
//扫码一
void Task_7(void)
{
	moveServo(1,POSITION_1,R_Speed);	//准备抓取物块
	Goto_Point(points[6],0,EX,EY); 	//扫任务一码
	while(code == 0);		
	qrcode_scan(goods,code);
}
//8-12为抓取五个颜色物块
void Task_8(void)
{
	Flash_ON();							//开灯
	USART_SendData(USART2, 1);			//开启颜色识别
	while ((USART2->SR & 0x40) == 0);	
	//半径，圆心x,圆心y，结束的角度，速度，车身方向
	Rotate_Point(817,-1058,1075,-173,-400,90,EX,EY);	//绕弧线抓取1
	identify_color=get_color();							//识别颜色
	position_color[identify_color]=POSITION_1;			//记录位置
	moveServo(1,POSITION_2,R_Speed_load);				//转盘
	code = 0;											//二维码清零
}
void Task_9(void)
{
	Rotate_Point(RD,-575,1075,-105,-500,90,EX,EY);	//绕弧线抓取2
	identify_color=get_color();							//识别颜色
	position_color[identify_color]=POSITION_2;      	//记录位置
	moveServo(1,POSITION_3,R_Speed_load);           	//转盘
}
void Task_10(void)
{
	Rotate_Point(RD,-575,1075,-75,V_rotate_2,90,EX,EY);	//绕弧线抓取3
	identify_color=get_color();							//识别颜色
	position_color[identify_color]=POSITION_3;	    	//记录位置
	moveServo(1,POSITION_4,R_Speed_load);				//转盘
}	
void Task_11(void)	
{	
	Rotate_Point(RD,-575,1075,-45,V_rotate_2,90,EX,EY);	//绕弧线抓取4
	identify_color=get_color();							//识别颜色
	position_color[identify_color]=POSITION_4;	    	//记录位置
	moveServo(1,POSITION_5,R_Speed_load);	        	//转盘
}	
void Task_12(void)	
{	
	Rotate_Point(RD,-575,1075,-15,V_rotate_2,90,EX,EY);	//绕弧线抓取5
	identify_color=get_color();							//识别颜色
	position_color[identify_color]=POSITION_5;      	//记录位置
	moveServo(1,POSITION_5-100,R_Speed_load);           //转盘
	Rotate_Point(RD,-575,1075,-5,V_rotate_2,90,EX,EY);	//绕弧线抓取5
	//退出颜色识别且关灯
	USART_SendData(USART2, 0);						
	while ((USART2->SR & 0x40) == 0);
	Flash_OFF();
}
//放置物块到abcde
void Task_13(void)
{
	Flash_ON();	
	USART_SendData(USART2, 0);
	while ((USART2->SR & 0x40) == 0);			// 退出现有程序
	USART_SendData(USART2, 2);
	while ((USART2->SR & 0x40) == 0);
    Alin(points[7] ,position_color[goods[1].color]);//a
    Alin(points[8] ,position_color[goods[2].color]);//b
    Alin(points[9] ,position_color[goods[3].color]);//c
    Alin(points[10],position_color[goods[4].color]);//d
    Alin(points[11],position_color[goods[5].color]);//e
	Flash_OFF();
	USART_SendData(USART2, 0);
	while ((USART2->SR & 0x40) == 0);
}
//回家
void Task_14(void)
{
	moveServo(1,POSITION_0,R_Speed);
    //Pass_Point(points[12],0,EX,EY);     
	Emm_V5_Pos_Control(1, 0, 1000, 0, 0, true, false);	                  
    moveServo(2,POSITION_UP,1000);
    Goto_Point(points[13],0,EX,EY);                       
}
void task_init()
{
		tasks[0] = Task_1;
		//扫码二
		tasks[1] = Task_2;
		//抓ABC
		tasks[2] = Task_3;
		tasks[3] = Task_4;
		tasks[4] = Task_5;
		//放冠亚季
		tasks[5] = Task_6;
		//扫码一
		tasks[6] = Task_7;
		//抓颜色物块
		tasks[7] = Task_8;
		tasks[8] = Task_9;
		tasks[9] = Task_10;
		tasks[10] = Task_11;
		tasks[11] = Task_12;
		//放颜色物块
		tasks[12] = Task_13;
		//回家
		tasks[13] = Task_14;
}

void Run_Task(void)
{
	int i = 0;
	while(tasks[i] != NULL)		//当前有任务
	{
		tasks[i](); 
		i++;    
	}

	
}

