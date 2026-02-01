#include "System_Config.h"
#include "Points.h"

/**
* @brief  初始化点位
* @param  __POINT数组
* @return None.
*/
void Point_Init(__POINT *__point, float x, float y, float z, __GOOD *__goods)
{
	__point->pos.x = 	x;
	__point->pos.y = 	y;
	__point->pos.z = 	z;
	__point->good  = 	__goods;
}

/**
* @brief  初始化所有点位
* @param  __POINT数组
* @return None.
*/
void AllPoint_Init(__POINT *__point, __GOOD __goods[])
{
	Point_Init(&__point[1],		0.0000,400,0,			&__goods[1] );	//出发
																    
	Point_Init(&__point[2],		650,400,-90,		&__goods[2] );	//扫码2
																    
	Point_Init(&__point[3],		250,2125,0,				&__goods[3] );	//亚
	Point_Init(&__point[4],		0,2125,0,				&__goods[4] );	//冠
	Point_Init(&__point[5],		-270,2125,0,			&__goods[5] );	//季
																    
	Point_Init(&__point[6],		-661,350,-90,		&__goods[6] );	//扫码1
																    
	Point_Init(&__point[7],		-631.5985,1235.1437,-36	,	&__goods[7] );	//a
	Point_Init(&__point[8],		-425.9145,953.9004,-36	,	&__goods[8] );	//b
	Point_Init(&__point[9],		220.8006,1236.0254,-22	,	&__goods[9] );	//c
	Point_Init(&__point[10],	334.6277,954.9785,-22	,	&__goods[10]);	//d
	Point_Init(&__point[11],	837.9629,1023.5943,30	,	&__goods[11]);	//e
	
	Point_Init(&__point[12],	0,300,0,				&__goods[12]);	//家门口
	Point_Init(&__point[13],	0,15,0,					&__goods[13]);	//家
}
