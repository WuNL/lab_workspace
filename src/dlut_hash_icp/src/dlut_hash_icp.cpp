/*************************************************************************
	> File Name: dlut_hash_icp.cpp
	> Author: 吴乃亮
	> Mail: wunailiang@gmail.com 
	> Created Time: Wed 21 May 2014 10:13:53 AM CST
 ************************************************************************/

#include<iostream>
#include"ros/ros.h"
//用于打开pcd
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>



#include "src/voxel.h"
#include "src/icp.h"


#include "src/net_transfor.h"
using namespace std;

typedef boost::unordered_map<unordered_map_voxel,un_key> umap;






int main(int argc,char** argv)
{
    ros::init(argc,argv,"dlut_hash_icp");
    ros::NodeHandle n;

    
    Voxelize* voxelize1;
    voxelize1 = new Voxelize;

    Net_transfor n1(0); 

    //打开两个pcd文件并且分别进行栅格化处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> cloud_fil;
    pcl::PointCloud<pcl::PointXYZ> cloud_fil1;


    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("3.pcd",*cloud);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud,*cloud, indices);
    indices.clear();
    cout <<"cloud.size="<<cloud->size()<<endl;

    boost::unordered_map<unordered_map_voxel,un_key> m1;

    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f,0.01f,0.01f);
    sor.filter(cloud_fil);

    double t1 = ros::Time::now().toSec();
    voxelize1->generateUmap(cloud_fil,0.3,m1);
    double t2 = ros::Time::now().toSec();
    cout <<"划分栅格时间="<<(t2-t1)<<endl;
    cout <<"cloud_fil.size="<<cloud_fil.size()<<endl;



    reader.read<pcl::PointXYZ>("4.pcd",*cloud1);
    pcl::removeNaNFromPointCloud(*cloud1,*cloud1, indices);
    cout <<"cloud1.size="<<cloud1->size()<<endl;
    boost::unordered_map<unordered_map_voxel,un_key> m2;



    voxelize1->generateUmap(*cloud1,0.3,m2);

    /*
    double jd = 30*3.1415927/180;
    double x_shift = 0.1;
    double y_shift = -0.1;
    for(int i = 0;i<cloud.size();i++)
    {
        //cout << cloud[i]<<endl;
        //continue;
        cloud1[i].x = cos(jd)*cloud[i].x-sin(jd)*cloud[i].y+x_shift;
        cloud1[i].y = sin(jd)*cloud[i].x + cos(jd)*cloud[i].y+y_shift;
        cloud1[i].z = cloud[i].z;
    }
    Matrix3d real_R;
    real_R <<cos(jd),-sin(jd),0,sin(jd),cos(jd),0,0,0,1; 
    cout <<"实际的旋转弧度为："<<jd<<endl;
    cout <<"实际旋转矩阵="<<endl << real_R<<endl;
    cout << "实际的平移x,y,z为："<<endl<<x_shift<<"\t"<<y_shift<<"\t"<<0<<endl;
    voxelize1->generateUmap(cloud1,0.3,m2);
    */
    /*


    pair<umap::iterator,bool> search_result;
    pcl::PointCloud<pcl::PointXYZ> cloud_a;//用于存储搜索到的重复区域
    for(umap::iterator iter=m2.begin();iter!=m2.end();iter++)
    {
        if(iter->second.p>0.7)
        {
            search_result = voxelize1->neighbor_search(m1,m2,iter->first);
            if(search_result.second)
            {
                //cout <<"size="<<search_result.first->second.vec_index_point.size()<<endl;

                for(int j = 0 ; j< search_result.first->second.vec_index_point.size(); j++)
                {
                    double x_ = cloud[search_result.first->second.vec_index_point[j]].x;
                    double y_ = cloud[search_result.first->second.vec_index_point[j]].y;
                    double z_ = cloud[search_result.first->second.vec_index_point[j]].z;
                    pcl::PointXYZ temppoint(0,0,0);

                    temppoint.x = x_;
                    temppoint.y = y_;
                    temppoint.z = z_;
                    cloud_a.push_back(temppoint);
                }
            }
        }
    }


		pcl::io::savePCDFileBinary("result.pcd",cloud_a);
    */

    delete voxelize1;
    Icp testicp(m1,m2,cloud_fil,0);
    testicp.icpFit(); 



    /*
    MatrixXd A(2,6);
    VectorXd b(6);
    b <<1,2,3,4,5,6;
    RowVectorXd c = b.transpose();
    cout <<b<<endl;
    cout <<c <<endl;
    for(int i=0;i<7;i++)
    {
    }
    //cout << A <<endl;
    */
/*
MatrixXf A = MatrixXf::Random(3, 2);
cout << "Here is the matrix A:\n" << A << endl;
VectorXf b = VectorXf::Random(3);
cout << "Here is the right hand side b:\n" << b << endl;
cout << "The least-squares solution is:\n"
    << A.jacobiSvd(ComputeThinU | ComputeThinV).solve(b) << endl;
*/
    //cout << sin(3.14)<<endl;
    cout << "hello icp" <<endl;
    return 0;
}
