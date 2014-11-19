//#include "stdafx.h"
#include "Grid3D.h"
#include<algorithm>
#include <cmath>
#include <iostream>
using namespace std;

namespace seg
{
Grid3D::Grid3D(void)
{
}
Grid3D::~Grid3D(void)
{
}
//栅格化操作的接口函数，入口参数为存储点云数据的一维数组
void Grid3D::Voxel2ize( vector< CPoint3d > &LaserData, float _size)
{
	this->bbox.SetNull();
  /*
	for (int i=0; i<(int)LaserData.size(); i++)			// 遍历激光数据
	{  
		if(LaserData[i].x==0 && LaserData[i].y==0) //去除无效点
			continue;
		
		this->bbox.Add( LaserData[i] ); //将点逐个添加进空间边框
	}
  */
  bbox.max.x=bbox.max.y=10.0;
  bbox.min.x=bbox.min.y=-10.0;
  bbox.max.z=3.0;
  bbox.min.z=0.0;
  this->dim = this->bbox.max - this->bbox.min; //计算空间边框的长度

	/**************** ****************/
	voxel.x = _size;
	voxel.y = _size;
	voxel.z = _size;
	siz.x = dim.x / voxel.x;
	siz.y = dim.y / voxel.y;
	siz.z = dim.z / voxel.z;
	cout<<"->"<<dim.x<<" "<<dim.y<<" "<<dim.z<<endl;
	cout<<"->"<<siz.x<<" "<<siz.y<<" "<<siz.z<<endl;
	/****************************************/

	//以下循环计算每一点所在栅格的位置，并存入links
	vector<Link2> links;  //对点进行预处理时用到的结构
	for (int i=0; i<(int)LaserData.size(); i++)			// 遍历激光数据
	{   
		//if语句过滤坏点
		if(LaserData[i].x==0 && LaserData[i].y==0) //去除无效点
			continue;

		CPoint3i voxel_id = GridP(LaserData[i]);
		links.push_back(Link2((voxel_id.z*siz.y + voxel_id.y)*siz.x + voxel_id.x, i, -1));//装入links 先往x方向，在y在z
	}
	sort(links.begin(), links.end());  //排序结果：在同一栅格的点排列在一起
	PointsToGrid(links);      //将点映射到栅格

	//处理重心与中心、邻域
	for (int i=0; i<grid.size(); i++)
	{
		CPoint3d alll(0,0,0);
		for (int j=0; j<grid[i].indexs.size(); j++)
		{
			alll += LaserData[grid[i].indexs[j]];
		}
		grid[i].gravity_center = alll / ((int)(grid[i].indexs.size()));  //重心
		CPoint3i voxelid = GridP(grid[i].gravity_center);
		grid[i].center = CPoint3d(bbox.min.x+voxelid.x*voxel.x+voxel.x/2,
									bbox.min.y+voxelid.y*voxel.y+voxel.y/2,
									bbox.min.z+voxelid.z*voxel.z+voxel.z/2);//中心

		//邻域 ：6-邻域系统 (前右后左上下)
		int dir[6][3] = {0,1,0,  1,0,0,  0,-1,0,  -1,0,0,  0,0,1,  0,0,-1};
		for (int k=0; k<6; k++)
		{
			int xx = voxelid.x + dir[k][0];
			int yy = voxelid.y + dir[k][1];
			int zz = voxelid.z + dir[k][2];
			if(xx<0 || xx>=siz.x) continue;
			if(yy<0 || yy>=siz.y) continue;
			if(zz<0 || zz>=siz.z) continue;
			int index_1d = (zz*siz.y + yy)*siz.x + xx;
			if(full2hash.count(index_1d) <= 0)	//该栅格不存在（即里面没有点）
				continue;
			grid[i].nbs.push_back(full2hash[index_1d]); //存起来
		}

		//邻域 ：26+1邻域系统(包括自身)
		//(层的顺序：上中下；每层顺序：前、右前、右、右后、后、左后、左、左前、中)
		int dir1[27][3] = {0,1,1,  1,1,1,  1,0,1,  1,-1,1,  0,-1,1,  -1,-1,1, -1,0,1, -1,1,1, 0,0,1,
			0,1,0,  1,1,0,  1,0,0,  1,-1,0,  0,-1,0,  -1,-1,0, -1,0,0, -1,1,0, 0,0,0,
			0,1,-1,  1,1,-1,  1,0,-1,  1,-1,-1,  0,-1,-1,  -1,-1,-1, -1,0,-1, -1,1,-1, 0,0,-1};
		grid[i].nbs_all.resize(27);
		for (int k=0; k<27; k++)
		{
			grid[i].nbs_all[k] = -1;
			int xx = voxelid.x + dir1[k][0];
			int yy = voxelid.y + dir1[k][1];
			int zz = voxelid.z + dir1[k][2];
			if(xx<0 || xx>=siz.x) continue;
			if(yy<0 || yy>=siz.y) continue;
			if(zz<0 || zz>=siz.z) continue;
			int index_1d = (zz*siz.y + yy)*siz.x + xx;
			if(full2hash.count(index_1d) <= 0)	//该栅格不存在（即里面没有点）
				grid[i].nbs_all[k] = -1;
			else
				grid[i].nbs_all[k] = full2hash[index_1d]; //存起来
		}
	}
}
//栅格化操作的接口函数，入口参数为存储点云数据的二维数组
void Grid3D::Voxel2ize( vector< vector<CPoint3d> > &LaserData, float _size)
{
	this->bbox.SetNull();
	for (int i=0; i<(int)LaserData.size(); i++)			// 遍历激光数据
	{  
		for (int j=0; j<LaserData[i].size(); j++)
		{
			if(LaserData[i][j].x==0 && LaserData[i][j].y==0) //去除无效点
				continue;
			this->bbox.Add( LaserData[i][j] ); //将点逐个添加进空间边框
		}
	}
	this->dim = this->bbox.max - this->bbox.min; //计算空间边框的长度

	/**************** ****************/
	voxel.x = _size;
	voxel.y = _size;
	voxel.z = _size;
	siz.x = dim.x / voxel.x;
	siz.y = dim.y / voxel.y;
	siz.z = dim.z / voxel.z;
	//cout<<"->"<<dim.x<<" "<<dim.y<<" "<<dim.z<<endl;
	//cout<<"->"<<siz.x<<" "<<siz.y<<" "<<siz.z<<endl;
	/****************************************/

	//以下循环计算每一点所在栅格的位置，并存入links
	vector<Link2> links;  //对点进行预处理时用到的结构
	for (int i=0; i<(int)LaserData.size(); i++)			// 遍历激光数据
	{   
		for (int j=0; j<LaserData[i].size(); j++)
		{
			//if语句过滤坏点
			if(LaserData[i][j].x==0 && LaserData[i][j].y==0) //去除无效点
				continue;
			CPoint3i voxel_id = GridP(LaserData[i][j]);
			links.push_back(Link2((voxel_id.z*siz.y + voxel_id.y)*siz.x + voxel_id.x, i, j));//装入links 先往x方向，在y在z
		}
	}
	sort(links.begin(), links.end());  //排序结果：在同一栅格的点排列在一起
	PointsToGrid(links);      //将点映射到栅格

	//处理重心与中心、邻域
	for (int i=0; i<grid.size(); i++)
	{
		CPoint3d alll(0,0,0);
		for (int j=0; j<grid[i].indexs.size(); j++)
		{
			alll += LaserData[grid[i].indexs[j]][grid[i].indexs_j[j]];
		}
		grid[i].gravity_center = alll / ((int)(grid[i].indexs.size()));  //重心
		CPoint3i voxelid = GridP(grid[i].gravity_center);
		grid[i].center = CPoint3d(bbox.min.x+voxelid.x*voxel.x+voxel.x/2,
									bbox.min.y+voxelid.y*voxel.y+voxel.y/2,
									bbox.min.z+voxelid.z*voxel.z+voxel.z/2);//中心

		//邻域 ：6-邻域系统 (前右后左上下)
		int dir[6][3] = {0,1,0,  1,0,0,  0,-1,0,  -1,0,0,  0,0,1,  0,0,-1};
		for (int k=0; k<6; k++)
		{
			int xx = voxelid.x + dir[k][0];
			int yy = voxelid.y + dir[k][1];
			int zz = voxelid.z + dir[k][2];
			if(xx<0 || xx>=siz.x) continue;
			if(yy<0 || yy>=siz.y) continue;
			if(zz<0 || zz>=siz.z) continue;
			int index_1d = (zz*siz.y + yy)*siz.x + xx;
			if(full2hash.count(index_1d) <= 0)	//该栅格不存在（即里面没有点）
				continue;
			grid[i].nbs.push_back(full2hash[index_1d]); //存起来
		}

		//邻域 ：26+1邻域系统(包括自身)
		//(层的顺序：上中下；每层顺序：前、右前、右、右后、后、左后、左、左前、中)
		int dir1[27][3] = {0,1,1,  1,1,1,  1,0,1,  1,-1,1,  0,-1,1,  -1,-1,1, -1,0,1, -1,1,1, 0,0,1,
			0,1,0,  1,1,0,  1,0,0,  1,-1,0,  0,-1,0,  -1,-1,0, -1,0,0, -1,1,0, 0,0,0,
			0,1,-1,  1,1,-1,  1,0,-1,  1,-1,-1,  0,-1,-1,  -1,-1,-1, -1,0,-1, -1,1,-1, 0,0,-1};
		grid[i].nbs_all.resize(27);
		for (int k=0; k<27; k++)
		{
			grid[i].nbs_all[k] = -1;
			int xx = voxelid.x + dir1[k][0];
			int yy = voxelid.y + dir1[k][1];
			int zz = voxelid.z + dir1[k][2];
			if(xx<0 || xx>=siz.x) continue;
			if(yy<0 || yy>=siz.y) continue;
			if(zz<0 || zz>=siz.z) continue;
			int index_1d = (zz*siz.y + yy)*siz.x + xx;
			if(full2hash.count(index_1d) <= 0)	//该栅格不存在（即里面没有点）
				grid[i].nbs_all[k] = -1;
			else
				grid[i].nbs_all[k] = full2hash[index_1d]; //存起来
		}
	}
}
//将点云数据映射到栅格
void Grid3D::PointsToGrid(vector<Link2> &links)
{
	if(links.size() <= 0)
		return;
	int lastindex = links[0].index;
	Voxel2 tmpVoxel;
	for (int i=0; i<links.size(); i++)
	{
		if(links[i].index != lastindex || i==(links.size()-1)) //添加一个栅格
		{
			grid.push_back(tmpVoxel);
      vec_voxel.push_back(lastindex);
			tmpVoxel.indexs.clear();
			tmpVoxel.indexs_j.clear();
			full2hash[lastindex] = grid.size()-1;//本应存为lastindex，实际为节省，存为grid.size()-1
		}

		tmpVoxel.indexs.push_back(links[i].mi);
		if(links[i].mj >= 0)
			tmpVoxel.indexs_j.push_back(links[i].mj);
		lastindex = links[i].index;
    tmpVoxel.index_of_voxel=lastindex;
	}
}

void Grid3D::findPoints(CPoint3d tm, float juli, vector<int> &_targets)
{
	_targets.clear();
	Box3f btemp(tm, juli);
	btemp.Intersect(bbox);
	if(!btemp.IsNull())
	{
		Box3i ib;
		BoxToIBox(btemp, ib);

		int x,y,z;
		for(z=ib.min.z;z<=ib.max.z;++z)
		{
			int bz = z*this->siz.y;
			for(y=ib.min.y;y<=ib.max.y;++y)
			{
				int by = (y+bz)*this->siz.x;
				for(x=ib.min.x;x<=ib.max.x;++x)
				{
					int index1D = by + x;
					if(full2hash.count(index1D) <= 0) //此栅格没有
						continue;
					int the_index = full2hash[index1D];
					for(size_t k=0; k<grid[the_index].indexs.size(); k++)
					{		
						_targets.push_back(grid[the_index].indexs[k]);
					}
				}
			}
		}
	}
}


void Grid3D::findPoints(CPoint3d tm, float juli, vector<int> &_targets, vector<int> &_targets_j)
{
	_targets.clear();
	Box3f btemp(tm, juli);
	btemp.Intersect(bbox);
	if(!btemp.IsNull())
	{
		Box3i ib;
		BoxToIBox(btemp, ib);

		int x,y,z;
		for(z=ib.min.z;z<=ib.max.z;++z)
		{
			int bz = z*this->siz.y;
			for(y=ib.min.y;y<=ib.max.y;++y)
			{
				int by = (y+bz)*this->siz.x;
				for(x=ib.min.x;x<=ib.max.x;++x)
				{
					int index1D = by + x;
					if(full2hash.count(index1D) <= 0) //此栅格没有
						continue;
					int the_index = full2hash[index1D];
					for(size_t k=0; k<grid[the_index].indexs.size(); k++)
					{		
						_targets.push_back(grid[the_index].indexs[k]);
						_targets_j.push_back(grid[the_index].indexs_j[k]);
					}
				}
			}
		}
	}
}

//返回点所在栅格
CPoint3i Grid3D::GridP(const CPoint3d & p ) const
{
	CPoint3i pi; 
	PToIP(p, pi);
	return pi;
}
//将点转换为整型点，与GridP函数功能类似
void Grid3D::PToIP(const CPoint3d & p, CPoint3i &pl )const
{
	CPoint3d t1 = p - bbox.min;
	CPoint3d t2 = bbox.max - p;

	if(t1.x >= 0.0 && t2.x >=0.0)
		pl.x = int( t1.x / voxel.x );
	else if(t1.x < 0.0)
      pl.x = siz.x-1;
	else if (t2.x < 0.0)
      pl.x = siz.x-1;
	if(pl.x >= siz.x)
		pl.x = siz.x-1;

	if(t1.y >= 0.0 && t2.y >=0.0)
		pl.y = int( t1.y / voxel.y );
	else if(t1.y < 0.0)
      pl.y = siz.y-1;
	else if (t2.y < 0.0)
      pl.y=siz.y-1;
	if(pl.y >= siz.y)
		pl.y = siz.y-1;

	if(t1.z >= 0.0 && t2.z >=0.0)
		pl.z = int( t1.z / voxel.z );
	else if(t1.z < 0.0)
      pl.z = siz.z-1;
	else if (t2.z < 0.0)
      pl.z= siz.z-1;
	if(pl.z >= siz.z)
		pl.z = siz.z-1;
}
//将整形点转换为三维点
void Grid3D::IPiToPf(const CPoint3i & pl, CPoint3d &p )const
{
	p.x = ((float)pl.x)*voxel.x;
	p.y = ((float)pl.y)*voxel.y;
	p.z = ((float)pl.z)*voxel.z;
	p = p + bbox.min;
}
//得到某整型点所代表的栅格区域的空间边框
void Grid3D::IPiToBox(const CPoint3i & pl, Box3f & b )const
{
	CPoint3d p;
	p.x = ((float)pl.x)*voxel.x;
	p.y = ((float)pl.y)*voxel.y;
	p.z = ((float)pl.z)*voxel.z;
    p = p + bbox.min;
	b.min = p;
	b.max = (p + voxel);
}
//返回整型点所代表栅格的中心点
void Grid3D::IPiToBoxCenter(const CPoint3i & pl, CPoint3d & c )const
{
	CPoint3d p;
	IPiToPf(pl,p);
	c = p + voxel/float(2.0);
}
//将三维空间边框转换为整型类空间边框	
void Grid3D::BoxToIBox( const Box3f & b, Box3i & ib )const
{
	PToIP(b.min, ib.min);
	PToIP(b.max, ib.max);
}
}
