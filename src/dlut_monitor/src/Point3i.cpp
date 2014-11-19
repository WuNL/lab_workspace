#include "Point3i.h"

CPoint3i::CPoint3i(void)
{
}
CPoint3i::CPoint3i(int x,int y,int z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}
CPoint3i::~CPoint3i(void)
{
}
CPoint3i::CPoint3i(const CPoint3i& tmpP)
{
	this->x = tmpP.x;
	this->y = tmpP.y;
	this->z = tmpP.z;
}
/////////// 重载运算符为成员函数 ///////////
CPoint3i & CPoint3i::operator += ( const CPoint3i & p)
{
	this->x += p.x;
	this->y += p.y;
	this->z += p.z;
	return *this;
}
CPoint3i & CPoint3i::operator -= (const CPoint3i & p)
{
	this->x -= p.x;
	this->y -= p.y;
	this->z -= p.z;
	return *this;
}
CPoint3i & CPoint3i::operator *= ( float s )
{
	this->x *= s;
	this->y *= s;
	this->z *= s;
	return *this;
}
// 重载运算符为友元函数
bool operator <  ( const CPoint3i &p1 ,const CPoint3i &p2 )
{
	return	(p1.z != p2.z)?(p1.z < p2.z):
			(p1.y != p2.y)?(p1.y < p2.y):
					       (p1.x < p2.x);
}
bool operator >  ( const CPoint3i & p1, const CPoint3i &p2)
{
	return	(p1.z != p2.z)?(p1.z > p2.z):
			(p1.y != p2.y)?(p1.y > p2.y):
					       (p1.x > p2.x);
}
bool operator == (const CPoint3i &p1,const CPoint3i &p2)	// 相等
{
	if(p1.x==p2.x && p1.y==p2.y && p1.z==p2.z)
		return true;
	else 
		return false;
}
bool operator != (const CPoint3i &p1,const CPoint3i &p2)	// 不等
{
	if(p1.x==p2.x && p1.y==p2.y && p1.z==p2.z)
		return false;
	else 
		return true;
}
CPoint3i operator + (const CPoint3i &p1 ,const CPoint3i &p2)	// 相加
{	
	CPoint3i po;
	po.x = p1.x + p2.x;
	po.y = p1.y + p2.y;
	po.z = p1.z + p2.z;
	return po;
}
CPoint3i operator - (const CPoint3i &p1 ,const CPoint3i &p2)	// 
{	
	CPoint3i po;
	po.x = p1.x - p2.x;
	po.y = p1.y - p2.y;
	po.z = p1.z - p2.z;
	return po;
}
