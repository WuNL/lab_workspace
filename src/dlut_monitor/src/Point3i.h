/*
* Copyright (c) 2011
*
* �ļ����ƣ� Point3i.h  Point3i.cpp
* ժ    Ҫ�� �ռ������������
*
*			 ������������Ҫ������ȫ�ֵ�ͼ�Ĺ���������ȫ�ֵ������ݵ�դ�񻯾���
* 
*			 ��CPoint3d�����ƣ�ֻ��������ֵΪ���ͣ��ڴ���ռ�դ��ȷ�������Ҫ����
*            ������CPoint3d��һͬд�ɵ�ģ���࣬�����ڴ���Ϊ����ӣ���CPoint3d��д
*            ��ģ�����ǣ�������е��ڶ�ط�������ʱ�������á�
*
* ��    �ߣ� �ι���
* ������ڣ� 2011-7-14
*
* ��ǰ�汾�� 1.0
* �޸���ʷ�� 
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
		// ���������Ϊ��Ա����
	CPoint3i & operator += ( const CPoint3i & p);
    CPoint3i & operator -= ( const CPoint3i & p);
    CPoint3i & operator *= ( float s );
		// ���������Ϊ��Ԫ����
	friend bool operator >  (const CPoint3i &p1,const CPoint3i &p2);
	friend bool operator <  (const CPoint3i &p1,const CPoint3i &p2);
	friend bool operator == (const CPoint3i &p1,const CPoint3i &p2);
	friend bool operator != (const CPoint3i &p1,const CPoint3i &p2);	
	friend CPoint3i operator + (const CPoint3i &p1 ,const CPoint3i &p2);
	friend CPoint3i operator - (const CPoint3i &p1 ,const CPoint3i &p2);
};