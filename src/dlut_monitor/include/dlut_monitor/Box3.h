/*
* Copyright (c) 2011
*
* 文件名称： Box3.h
* 摘    要： 空间边框类 - Bounding Box
*
*			 （本程序中主要服务于全局地图的构建，用于全局点云数据的栅格化精简）
* 
*            定义成模板类，适用于不同类型数据的空间边框（在本程序中主要是浮点型数据的空间边框 
*            和整型数据的空间边框）。
*
* 作    者： 何国建
* 完成日期： 2011-7-14
*
* 当前版本： 1.0
* 修改历史： 
*/

#pragma once

template <class ScalarType>
class Box3
{
	//边框以所包围空间内最小、最大的两个点表示
public:
	ScalarType min;
	ScalarType max;
public:
	//构造与析构函数
	Box3(void);
	Box3(const Box3<ScalarType> & b);
	Box3(const ScalarType & mi,const ScalarType & ma );
	Box3( const ScalarType & center, const float & radius);
	~Box3(void);
	//重载操作符
	inline bool operator == ( Box3<ScalarType> & p );
	inline bool operator != ( Box3<ScalarType> & p );
public:
	//各功能函数，具体实现见详细定义

	//设置一个点的初始边框
	void Set( const ScalarType & p );
	//设置为不可用
	void SetNull(void);
	//将空间边框b与现有边框融合
	void Add( const Box3<ScalarType> & b );
    //向边框中添加一个点
	void Add( const ScalarType & p );
    //找出当前边框与b相交的部分；若与b不想交，则结果为不可用边框
	void Intersect(const Box3<ScalarType> & b );
    //判断点p是否在当前边框中或上
	bool IsIn( ScalarType & p )const;
    //判断点p是否在当前边框中
	bool IsInEx( ScalarType & p )const;
    //判断边框是否可用
	bool IsNull(void)const;
	//判断边框是否为空
	bool IsEmpty(void)const;
    //边框的大小（max、min两点的距离）
	float Diag(void)const;
    void Offset( const float s );
	void Offset( const ScalarType & delta );
};

//默认构造函数
template<class ScalarType>
Box3<ScalarType>::Box3(void) 
{ 
	min.x= 1;max.x= -1;min.y= 1;max.y= -1;min.z= 1;max.z= -1;
}

//拷贝构造函数
template<class ScalarType>
Box3<ScalarType>::Box3( const Box3<ScalarType> & b ) 
{
	min = b.min; max = b.max; 
}

//由最小、最大的两个点构造边框
template<class ScalarType>
Box3<ScalarType>::Box3( const ScalarType & mi, const ScalarType & ma ) 
{ 
	min = mi; max = ma; 
}

// 构建以给定点center为中心，radius为半径的空间边框
template<class ScalarType>
Box3<ScalarType>::Box3( const ScalarType & center, const float & radius) 
{
	const ScalarType po(radius,radius,radius);
    min = center - po;
    max = center + po;
}

//析构函数
template<class ScalarType>
Box3<ScalarType>::~Box3(void) 
{}

// 重载操作符比较两个空间边框的大小
template<class ScalarType>
inline bool Box3<ScalarType>::operator == ( Box3<ScalarType> & p )
{
	return min==p.min && max==p.max;
}
template<class ScalarType>
inline bool Box3<ScalarType>::operator != ( Box3<ScalarType> & p )
{
	return min!=p.min || max!=p.max;
}

//初始化包围一个点的空间边框
template<class ScalarType>
void Box3<ScalarType>::Set(const ScalarType & p )
{
	min = max = p;
}

//将空间边框设置为不可用
template<class ScalarType>
void Box3<ScalarType>::SetNull(void)
{
	min.x= 9999999; max.x= -9999999;
	min.y= 9999999; max.y= -9999999;
	min.z= 9999999; max.z= -9999999;
}

//将空间边框b与现有边框融合
template<class ScalarType>
void Box3<ScalarType>::Add( const Box3<ScalarType> & b )
{
	if(b.IsNull()) return; //b为不可用边框时直接返回
	if(IsNull()) *this=b;
	else
	{
		if(min.x > b.min.x) min.x = b.min.x;
		if(min.y > b.min.y) min.y = b.min.y;
    	if(min.z > b.min.z) min.z = b.min.z;
		if(max.x < b.max.x) max.x = b.max.x;
		if(max.y < b.max.y) max.y = b.max.y;
		if(max.z < b.max.z) max.z = b.max.z;
	}
}

//向边框中添加一个点
template<class ScalarType>
void Box3<ScalarType>::Add(const ScalarType & p )
{
	if(IsNull()) Set(p);
	else 
	{
		if(min.x > p.x) min.x = p.x;
		if(min.y > p.y) min.y = p.y;
		if(min.z > p.z) min.z = p.z;
		if(max.x < p.x) max.x = p.x;
		if(max.y < p.y) max.y = p.y;
		if(max.z < p.z) max.z = p.z;
	}
}

//找出当前边框与b相交的部分；若与b不想交，则结果为不可用边框
template<class ScalarType>
void Box3<ScalarType>::Intersect( const Box3<ScalarType> & b )
{
	if(min.x < b.min.x) min.x = b.min.x;
	if(min.y < b.min.y) min.y = b.min.y;
	if(min.z < b.min.z) min.z = b.min.z;
	if(max.x > b.max.x) max.x = b.max.x;
	if(max.y > b.max.y) max.y = b.max.y;
	if(max.z > b.max.z) max.z = b.max.z;
	if(min.x>max.x || min.y>max.y || min.z>max.z) SetNull();
}

//判断点p是否在当前边框中或上
template<class ScalarType>
bool Box3<ScalarType>::IsIn( ScalarType & p ) const
{
	return (
		min.x <= p.x && p.x <= max.x &&
		min.y <= p.y && p.y <= max.y &&
		min.z <= p.z && p.z <= max.z
	);
}

//判断点p是否在当前边框中
template<class ScalarType>
bool Box3<ScalarType>::IsInEx( ScalarType & p ) const
{
	return (
		min.x <= p.x && p.x < max.x &&
		min.y <= p.y && p.y < max.y &&
		min.z <= p.z && p.z < max.z
	);
}

//判断边框是否可用
template<class ScalarType>
bool Box3<ScalarType>::IsNull(void) const
{ 
	return min.x>max.x || min.y>max.y || min.z>max.z; 
}

//判断边框是否为空
template<class ScalarType>
bool Box3<ScalarType>::IsEmpty(void) const
{ 
	return min==max;
}

//边框的大小（max、min两点的距离）
template<class ScalarType>
float Box3<ScalarType>::Diag(void) const
{
	ScalarType temp;
	temp = max - min;
	return temp.Dist();
}
template<class ScalarType>
void Box3<ScalarType>::Offset( const float s )
{
	ScalarType po(s,s,s);
	Offset( po );
}
template<class ScalarType>
void Box3<ScalarType>::Offset(const ScalarType & delta )
{
	min = min - delta;
	max = max + delta;
}