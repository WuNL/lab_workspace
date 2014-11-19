/*
* Copyright (c) 2011
*
* �ļ����ƣ� MGrid.h  MGrid.cpp
* ժ    Ҫ�� ��άդ����
*            ������άդ���㷨�࣬��Ҫ���ڵ������ݾ��򣻸����������--դ��߳�����������ά��������
*			 ���ӳ�䵽դ��������դ��������ϵ���㷨��Ӧ�õķ����Բ�ͬ�ڰ˲�����Ļ��ַ�������
*			 ��������Ч�ʽϸߡ�
*            ���ֹ����У������õ�CPoint3d���࣬���õ���������ר����CPoint3i�ࡢBox3�࣬�ֱ�Ϊ����
*            ����Ϳռ�߿��ࡣ���͵���CPoint3i����CPoint3d�ֻ࣬���������е�float�͸�Ϊint�ͣ�
*			 Box3��Ϊ��ά�ռ�߿��࣬����ռ�߿����ز�����
*
* ��    �ߣ� �ι���
* ������ڣ� 2011-7-20
*
* ��ǰ�汾�� 1.0
* �޸���ʷ�� <2014-03-04><�ι���><�޼���һЩû��Ҫ�Ĵ洢�ṹ��ʹ��Ӧ���ڳ����ָ��У�Ч�ʸ���>
*/

#pragma once
#include <vector>
#include <map>
#include "Point3d.h"		// ��ά����
#include "Point3i.h"      // ��ά���͵���
#include "Box3.h"         // �ռ�߿���

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////
namespace seg
{
	//Link2�࣬ÿһ��Link2ʵ�����һ�����ڿռ�դ���е�λ�����꣨���ε����������͵������ֵ��
	//����ÿһ���㣬�γ���Link2ʵ�幹�ɵ�����links��Ϊ��ӳ�䵽դ����׼��
	class Link2
	{
	public:
		Link2(void) {}
		Link2(int i, int _pIndexi, int _pIndexj){index = i; mi = _pIndexi; mj = _pIndexj;}
		bool operator < (const  Link2 & l ) const
		{return this->index < l.index;} 
		bool operator == (const Link2 & l ) const { return this->index ==  l.index; }
		bool operator != (const Link2 & l ) const { return this->index !=  l.index; }
	public:
		int index;	//������դ�񣬵�һά����
		int mi;  //�������
		int mj;//(�����Ƕ�ά�������飬mjʼ��Ϊ0)
	};


	///////////////////////////////////////////////////////////////////////////////////////

	//դ���࣬����ÿһ��Сդ��
	class Voxel2
	{
	public:
		Voxel2(void) {}
	public:
		vector<int> indexs;			//դ���������ĵ��ƣ��±꣩
		vector<int> indexs_j;		//դ���������ĵ��ƣ��±꣩
		CPoint3d gravity_center;	//դ�������
		CPoint3d center;			//դ������

		vector<int> nbs;	//�������ڵ�դ���������е��±�
		vector<int> nbs_all;	//����������--26����
    int index_of_voxel;
	};

	///////////////////////////////////////////////////////////////////////////////////////

	//����դ����㷨��
	class Grid3D
	{
	public:
		Grid3D(void);
		~Grid3D(void);

	public:
		vector<Voxel2> grid;  //�洢դ�������	
    vector<int> vec_voxel;
	private:
		typedef Box3<class CPoint3d> Box3f;
		typedef Box3<class CPoint3i> Box3i;
		Box3f bbox;            //�ռ�߿�
		CPoint3d dim;          //�ռ�߿���XYZ���������ϵĳ���
		CPoint3i siz;          //�ռ�߿�Ļ���ά��
		CPoint3d voxel;        //ÿ��դ��ı߳�

		map<int,int> full2hash;	//ȫ��->��ʡ�洢 ��ӳ��
		/////////////////////////////////////////////////

		///////////-��Ա����-/////////////////////////////
	public:
		//դ�񻯲����Ľӿں�������ڲ���Ϊ�洢�������ݵ������դ��ı߳�
		void Voxel2ize( vector<CPoint3d> &LaserData, float _size = 0.5);
		void Voxel2ize( vector<vector<CPoint3d> > &LaserData, float _size = 0.5);
		//�ҵ�
		void findPoints(CPoint3d tm, float juli, vector<int> &_targets);
		void findPoints(CPoint3d tm, float juli, vector<int> &_targets, vector<int> &_targets_j);
	private:
		void PointsToGrid(vector<Link2> &links);
		//����һ�麯�������һ�����դ����������ֵ�������ꡢ��ͬ���ͱ߿��֮���ת��
		inline CPoint3i GridP(const CPoint3d & p )const;             //���ص�����դ��
		inline void PToIP(const CPoint3d & p,CPoint3i &pl)const;     //����ת��Ϊ���͵㣬��GridP������������
		inline void IPiToPf(const CPoint3i & pl, CPoint3d &p )const; //�����ε�ת��Ϊ��ά��
		inline void IPiToBox(const CPoint3i & pl, Box3f & b )const;     //�õ�ĳ���͵��������դ������Ŀռ�߿�
		inline void IPiToBoxCenter( const CPoint3i & pl, CPoint3d & c )const; //�������͵�������դ������ĵ�
		void BoxToIBox( const Box3f & b, Box3i & ib )const;          //����ά�ռ�߿�ת��Ϊ������ռ�߿�

	};
}


