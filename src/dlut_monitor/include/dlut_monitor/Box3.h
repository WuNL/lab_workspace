/*
* Copyright (c) 2011
*
* �ļ����ƣ� Box3.h
* ժ    Ҫ�� �ռ�߿��� - Bounding Box
*
*			 ������������Ҫ������ȫ�ֵ�ͼ�Ĺ���������ȫ�ֵ������ݵ�դ�񻯾���
* 
*            �����ģ���࣬�����ڲ�ͬ�������ݵĿռ�߿��ڱ���������Ҫ�Ǹ��������ݵĿռ�߿� 
*            ���������ݵĿռ�߿򣩡�
*
* ��    �ߣ� �ι���
* ������ڣ� 2011-7-14
*
* ��ǰ�汾�� 1.0
* �޸���ʷ�� 
*/

#pragma once

template <class ScalarType>
class Box3
{
	//�߿�������Χ�ռ�����С�������������ʾ
public:
	ScalarType min;
	ScalarType max;
public:
	//��������������
	Box3(void);
	Box3(const Box3<ScalarType> & b);
	Box3(const ScalarType & mi,const ScalarType & ma );
	Box3( const ScalarType & center, const float & radius);
	~Box3(void);
	//���ز�����
	inline bool operator == ( Box3<ScalarType> & p );
	inline bool operator != ( Box3<ScalarType> & p );
public:
	//�����ܺ���������ʵ�ּ���ϸ����

	//����һ����ĳ�ʼ�߿�
	void Set( const ScalarType & p );
	//����Ϊ������
	void SetNull(void);
	//���ռ�߿�b�����б߿��ں�
	void Add( const Box3<ScalarType> & b );
    //��߿������һ����
	void Add( const ScalarType & p );
    //�ҳ���ǰ�߿���b�ཻ�Ĳ��֣�����b���뽻������Ϊ�����ñ߿�
	void Intersect(const Box3<ScalarType> & b );
    //�жϵ�p�Ƿ��ڵ�ǰ�߿��л���
	bool IsIn( ScalarType & p )const;
    //�жϵ�p�Ƿ��ڵ�ǰ�߿���
	bool IsInEx( ScalarType & p )const;
    //�жϱ߿��Ƿ����
	bool IsNull(void)const;
	//�жϱ߿��Ƿ�Ϊ��
	bool IsEmpty(void)const;
    //�߿�Ĵ�С��max��min����ľ��룩
	float Diag(void)const;
    void Offset( const float s );
	void Offset( const ScalarType & delta );
};

//Ĭ�Ϲ��캯��
template<class ScalarType>
Box3<ScalarType>::Box3(void) 
{ 
	min.x= 1;max.x= -1;min.y= 1;max.y= -1;min.z= 1;max.z= -1;
}

//�������캯��
template<class ScalarType>
Box3<ScalarType>::Box3( const Box3<ScalarType> & b ) 
{
	min = b.min; max = b.max; 
}

//����С�����������㹹��߿�
template<class ScalarType>
Box3<ScalarType>::Box3( const ScalarType & mi, const ScalarType & ma ) 
{ 
	min = mi; max = ma; 
}

// �����Ը�����centerΪ���ģ�radiusΪ�뾶�Ŀռ�߿�
template<class ScalarType>
Box3<ScalarType>::Box3( const ScalarType & center, const float & radius) 
{
	const ScalarType po(radius,radius,radius);
    min = center - po;
    max = center + po;
}

//��������
template<class ScalarType>
Box3<ScalarType>::~Box3(void) 
{}

// ���ز������Ƚ������ռ�߿�Ĵ�С
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

//��ʼ����Χһ����Ŀռ�߿�
template<class ScalarType>
void Box3<ScalarType>::Set(const ScalarType & p )
{
	min = max = p;
}

//���ռ�߿�����Ϊ������
template<class ScalarType>
void Box3<ScalarType>::SetNull(void)
{
	min.x= 9999999; max.x= -9999999;
	min.y= 9999999; max.y= -9999999;
	min.z= 9999999; max.z= -9999999;
}

//���ռ�߿�b�����б߿��ں�
template<class ScalarType>
void Box3<ScalarType>::Add( const Box3<ScalarType> & b )
{
	if(b.IsNull()) return; //bΪ�����ñ߿�ʱֱ�ӷ���
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

//��߿������һ����
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

//�ҳ���ǰ�߿���b�ཻ�Ĳ��֣�����b���뽻������Ϊ�����ñ߿�
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

//�жϵ�p�Ƿ��ڵ�ǰ�߿��л���
template<class ScalarType>
bool Box3<ScalarType>::IsIn( ScalarType & p ) const
{
	return (
		min.x <= p.x && p.x <= max.x &&
		min.y <= p.y && p.y <= max.y &&
		min.z <= p.z && p.z <= max.z
	);
}

//�жϵ�p�Ƿ��ڵ�ǰ�߿���
template<class ScalarType>
bool Box3<ScalarType>::IsInEx( ScalarType & p ) const
{
	return (
		min.x <= p.x && p.x < max.x &&
		min.y <= p.y && p.y < max.y &&
		min.z <= p.z && p.z < max.z
	);
}

//�жϱ߿��Ƿ����
template<class ScalarType>
bool Box3<ScalarType>::IsNull(void) const
{ 
	return min.x>max.x || min.y>max.y || min.z>max.z; 
}

//�жϱ߿��Ƿ�Ϊ��
template<class ScalarType>
bool Box3<ScalarType>::IsEmpty(void) const
{ 
	return min==max;
}

//�߿�Ĵ�С��max��min����ľ��룩
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