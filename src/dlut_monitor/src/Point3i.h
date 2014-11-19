/*
* Copyright (c) 2011
*
* 文件名称： Point3i.h  Point3i.cpp
* 摘    要： 空间整型坐标点类
*
*			 （本程序中主要服务于全局地图的构建，用于全局点云数据的栅格化精简）
* 
*			 与CPoint3d点类似，只不过坐标值为整型，在处理空间栅格等方面有重要意义
*            可以与CPoint3d类一同写成点模板类，但由于此类为后添加，将CPoint3d类写
*            成模板类会牵扯程序中的众多地方，故暂时单独放置。
*
* 作    者： 何国建
* 完成日期： 2011-7-14
*
* 当前版本： 1.0
* 修改历史： 
*/

#pragma once
class CPoint3i
{
public:
	int x;
	int y;
	int z;
public:
	CPoint3i(void);
	CPoint3i(int x,int y,int z);
	~CPoint3i(void);
	CPoint3i(const CPoint3i& tmpP);
		// 重载运算符为成员函数
	CPoint3i & operator += ( const CPoint3i & p);
    CPoint3i & operator -= ( const CPoint3i & p);
    CPoint3i & operator *= ( float s );
		// 重载运算符为友元函数
	friend bool operator >  (const CPoint3i &p1,const CPoint3i &p2);
	friend bool operator <  (const CPoint3i &p1,const CPoint3i &p2);
	friend bool operator == (const CPoint3i &p1,const CPoint3i &p2);
	friend bool operator != (const CPoint3i &p1,const CPoint3i &p2);	
	friend CPoint3i operator + (const CPoint3i &p1 ,const CPoint3i &p2);
	friend CPoint3i operator - (const CPoint3i &p1 ,const CPoint3i &p2);
};