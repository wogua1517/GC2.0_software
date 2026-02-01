/**
  ******************************************************************************
  * @file   Points.h
  * @brief  任务函数
  ******************************************************************************
  * @note
  *  - 
  *  -
 */


#ifndef  __POINTS_H
#define  __POINTS_H

/* Includes ------------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macros ------------------------------------------------------------*/

/* Private type --------------------------------------------------------------*/
enum __COLOR{
	NOCOLOR,	//0
	COLOR_RED,	//1
	COLOR_GREEN,//2
	COLOR_BLUE, //3
	COLOR_BLACK,//4	
	COLOR_WHITE,//5
};
enum __LETTER{
	A,
	B,
	C,
};
typedef struct{
	float x;
	float y;
	float z;
}__Position;

typedef struct{
	enum __COLOR color;
	enum __LETTER letter;
}__GOOD;
/**
* @brief 各点位结构体 包括该点的位置 和在该点的物料的指针
*/

typedef struct{
	__Position pos;
	__GOOD* good;
}__POINT;

/* Exported macros -----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/


/* Exported function declarations --------------------------------------------*/
void Goods_Init(__GOOD __goods[],u8 code);
void Point_Init(__POINT *__point, float x, float y, float z, __GOOD *__goods);
void AllPoint_Init(__POINT *__point, __GOOD __goods[]);
uint8_t Have_GOODS(__GOOD *__goods, enum __COLOR __color);
uint8_t Take_GOODS(__GOOD *__goods, enum __COLOR __color);

#endif  
