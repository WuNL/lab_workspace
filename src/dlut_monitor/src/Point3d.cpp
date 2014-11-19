#include "Point3d.h"
#include <math.h>

//////////////////////////////////////////////////////////////////////////
/////////// Ĭ�Ϲ��캯�� ///////////
CPoint3d::CPoint3d(void)
:x(0),y(0),z(0)
{
}

CPoint3d::CPoint3d(float x,float y,float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

// ��������
CPoint3d::~CPoint3d(void)
{
}

// ���ƹ��캯��
CPoint3d::CPoint3d(const CPoint3d& tmpP)
{
	this->x = tmpP.x;
	this->y = tmpP.y;
	this->z = tmpP.z;
}
//////////////////////////////////////////////////////////////////////////
/////////// ���������Ϊ��Ա���� ///////////
CPoint3d & CPoint3d::operator += ( const CPoint3d & p)
{
	this->x += p.x;
	this->y += p.y;
	this->z += p.z;
	return *this;
}
CPoint3d & CPoint3d::operator -= (const CPoint3d & p)
{
	this->x -= p.x;
	this->y -= p.y;
	this->z -= p.z;
	return *this;
}
CPoint3d & CPoint3d::operator *= ( float s )
{
	this->x *= s;
	this->y *= s;
	this->z *= s;
	return *this;
}
CPoint3d & CPoint3d::operator /= ( float s )
{
	this->x /= s;
	this->y /= s;
	this->z /= s;
	return *this;
}

// ���������Ϊ��Ԫ����
CPoint3d operator + (const CPoint3d &p1 ,const CPoint3d &p2)	// ���
{	
	CPoint3d po;
	po.x = p1.x + p2.x;
	po.y = p1.y + p2.y;
	po.z = p1.z + p2.z;
	return po;
}
CPoint3d operator - (const CPoint3d &p1 ,const CPoint3d &p2)	// 
{	
	CPoint3d po;
	po.x = p1.x - p2.x;
	po.y = p1.y - p2.y;
	po.z = p1.z - p2.z;
	return po;
}
CPoint3d operator * ( const CPoint3d &p , float s )
{
	CPoint3d po;
	po.x = p.x * s;
	po.y = p.y * s;
	po.z = p.z * s;
	return po;
}

CPoint3d operator / (const CPoint3d &p ,float num)			// ����һ������
{		
	if(num != 0)
	{   
		CPoint3d po;
		po.x = p.x / num;
		po.y = p.y / num;
		po.z = p.z / num;
		return po;
	}
	else 
	{
		//AfxMessageBox(_T("����0������������"));
		return CPoint3d(0,0,0);
	}
}
bool operator <  ( const CPoint3d &p1 ,const CPoint3d &p2 )
{
	return	(p1.z != p2.z)?(p1.z < p2.z):
			(p1.y != p2.y)?(p1.y < p2.y):
					       (p1.x < p2.x);
}
bool operator >  ( const CPoint3d & p1, const CPoint3d &p2)
{
	return	(p1.z != p2.z)?(p1.z > p2.z):
			(p1.y != p2.y)?(p1.y > p2.y):
					       (p1.x > p2.x);
}
bool operator == (const CPoint3d &p1,const CPoint3d &p2)	// ���
{
	if(p1.x==p2.x && p1.y==p2.y && p1.z==p2.z)
		return true;
	else 
		return false;
}
bool operator != (const CPoint3d &p1,const CPoint3d &p2)	// ����
{
	if(p1.x==p2.x && p1.y==p2.y && p1.z==p2.z)
		return false;
	else 
		return true;
}
float operator *(const CPoint3d &p1, const CPoint3d &p2)
{
	return (p1.x*p2.x + p1.y*p2.y + p1.z*p2.z);
}
//////////////////////////////////////////////////////////////////////////
// �㵽ԭ��ľ���
float CPoint3d::Dist(void)
{
	return sqrt(x*x+y*y+z*z);
}

// �㵽ԭ��ľ���
float CPoint3d::Dist(const CPoint3d &p1)
{
	return sqrt((x-p1.x)*(x-p1.x)+(y-p1.y)*(y-p1.y)+(z-p1.z)*(z-p1.z));
}

CPoint3d & CPoint3d::Normalize()
{
	float n = float(sqrt(x*x+y*y+z*z));
	if(n > float(0))
	{ x /= n; y /= n; z /= n;}
	return *this;
}

float CPoint3d::dot(const CPoint3d &p)const
{
	return (*this) * p;
}
