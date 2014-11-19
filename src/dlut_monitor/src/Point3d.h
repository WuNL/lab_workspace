//////////////////////////////////////////////////////////////////////////
// CPoint3d 3D���࣬����������ά�ռ��е����Ϣ,һ��C++��
// CPoint3d����֧��3d��� +, -, /�� ==, <, >�Ȳ�����������
//
// ����ߣ�֣��ǿ	2008.12.03 
//
// �޸ģ��޸������ز���������  �ι��� 2011.7.14
//////////////////////////////////////////////////////////////////////////

#pragma once

class CPoint3d
{
public:
	float x;	// 3d���x����
	float y;	// 3d���y����
	float z;	// 3d���z����

public:
	// ���캯��
	CPoint3d(void);
	CPoint3d(float x,float y,float z);
	~CPoint3d(void);
	// ���ƹ��캯��
	CPoint3d(const CPoint3d& tmpP);

	// ���������Ϊ��Ա����
	CPoint3d & operator += ( const CPoint3d & p);
    CPoint3d & operator -= ( const CPoint3d & p);
    CPoint3d & operator *= ( float s );
    CPoint3d & operator /= ( float s );
	// ���������Ϊ��Ԫ����
	friend bool operator >  (const CPoint3d &p1,const CPoint3d &p2);
	friend bool operator <  (const CPoint3d &p1,const CPoint3d &p2);
	friend bool operator == (const CPoint3d &p1,const CPoint3d &p2);
	friend bool operator != (const CPoint3d &p1,const CPoint3d &p2);	
	friend CPoint3d operator + (const CPoint3d &p1 ,const CPoint3d &p2);
	friend CPoint3d operator - (const CPoint3d &p1 ,const CPoint3d &p2);
	friend CPoint3d operator * ( const CPoint3d &p , float s );
	friend float operator *(const CPoint3d &p1, const CPoint3d &p2);
	friend CPoint3d operator / (const CPoint3d &p ,float num);

	float dot(const CPoint3d &p)const;
	// �㵽ԭ��ľ���
	float Dist(void);
	// �㵽ԭ��ľ���
	float Dist(const CPoint3d &p1);
	CPoint3d & Normalize();
};