/*
* Copyright (c) 2011
*
* 文件名称： MGrid.h  MGrid.cpp
* 摘    要： 三维栅格类
*            生成三维栅格算法类，主要用于点云数据精简；根据输入参数--栅格边长，决定划分维数，将点
*			 逐个映射到栅格，再生成栅格的邻域关系；算法中应用的方法略不同于八叉树类的划分方法，此
*			 方法划分效率较高。
*            划分过程中，除了用到CPoint3d点类，还用到其他两个专用类CPoint3i类、Box3类，分别为整型
*            点类和空间边框类。整型点类CPoint3i类似CPoint3d类，只不过将其中的float型改为int型；
*			 Box3类为三维空间边框类，处理空间边框的相关操作。
*
* 作    者： 何国建
* 完成日期： 2011-7-20
*
* 当前版本： 1.0
* 修改历史： <2014-03-04><何国建><修剪了一些没必要的存储结构，使其应用在场景分割中，效率更高>
*/

#pragma once
#include <vector>
#include <map>
#include "Point3d.h"		// 三维点类
#include "Point3i.h"      // 三维整型点类
#include "Box3.h"         // 空间边框类

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////
namespace seg
{
	//Link2类，每一个Link2实体包含一个点在空间栅格中的位置坐标（整形点描述），和点的索引值；
	//遍历每一个点，形成由Link2实体构成的数组links，为点映射到栅格做准备
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
		int index;	//点所在栅格，的一维索引
		int mi;  //点的索引
		int mj;//(若不是二维点云数组，mj始终为0)
	};


	///////////////////////////////////////////////////////////////////////////////////////

	//栅格类，描述每一个小栅格
	class Voxel2
	{
	public:
		Voxel2(void) {}
	public:
		vector<int> indexs;			//栅格所包含的点云（下标）
		vector<int> indexs_j;		//栅格所包含的点云（下标）
		CPoint3d gravity_center;	//栅格的重心
		CPoint3d center;			//栅格中心

		vector<int> nbs;	//与其相邻的栅格，在数组中的下标
		vector<int> nbs_all;	//完整的邻域--26邻域
    int index_of_voxel;
	};

	///////////////////////////////////////////////////////////////////////////////////////

	//生成栅格的算法类
	class Grid3D
	{
	public:
		Grid3D(void);
		~Grid3D(void);

	public:
		vector<Voxel2> grid;  //存储栅格的数组	
    vector<int> vec_voxel;
	private:
		typedef Box3<class CPoint3d> Box3f;
		typedef Box3<class CPoint3i> Box3i;
		Box3f bbox;            //空间边框
		CPoint3d dim;          //空间边框在XYZ三个方向上的长度
		CPoint3i siz;          //空间边框的划分维数
		CPoint3d voxel;        //每个栅格的边长

		map<int,int> full2hash;	//全部->节省存储 的映射
		/////////////////////////////////////////////////

		///////////-成员函数-/////////////////////////////
	public:
		//栅格化操作的接口函数，入口参数为存储点云数据的数组和栅格的边长
		void Voxel2ize( vector<CPoint3d> &LaserData, float _size = 0.5);
		void Voxel2ize( vector<vector<CPoint3d> > &LaserData, float _size = 0.5);
		//找点
		void findPoints(CPoint3d tm, float juli, vector<int> &_targets);
		void findPoints(CPoint3d tm, float juli, vector<int> &_targets, vector<int> &_targets_j);
	private:
		void PointsToGrid(vector<Link2> &links);
		//以下一组函数，完成一个点的栅格整型索引值与点的坐标、不同类型边框等之间的转换
		inline CPoint3i GridP(const CPoint3d & p )const;             //返回点所在栅格
		inline void PToIP(const CPoint3d & p,CPoint3i &pl)const;     //将点转换为整型点，与GridP函数功能类似
		inline void IPiToPf(const CPoint3i & pl, CPoint3d &p )const; //将整形点转换为三维点
		inline void IPiToBox(const CPoint3i & pl, Box3f & b )const;     //得到某整型点所代表的栅格区域的空间边框
		inline void IPiToBoxCenter( const CPoint3i & pl, CPoint3d & c )const; //返回整型点所代表栅格的中心点
		void BoxToIBox( const Box3f & b, Box3i & ib )const;          //将三维空间边框转换为整型类空间边框

	};
}


