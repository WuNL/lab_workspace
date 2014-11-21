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
#include "boost/thread.hpp"
#include <queue>
using namespace std;
boost::mutex io_mutex;
typedef boost::unordered_map<unordered_map_voxel,un_key> umap;


void getdata(std::queue<pcl::PointCloud<pcl::PointXYZ> > *p_q)
{
    Net_transfor n1(0);

    while(ros::ok())
    {
        pcl::PointCloud<pcl::PointXYZ> cloud_copy;
        cout <<"in thread\n";
        //cloud->clear();
        if(n1.receive(cloud_copy)==false)
            break;

        //cout <<"cloud_copy.size="<<cloud_copy.size()<<endl;
        {
            boost::mutex::scoped_lock lock(io_mutex);
            p_q->push(cloud_copy);
        }
        cout <<"p_q.size="<<p_q->size()<<endl;
    }
    n1.close_receive();
    cout <<"exit thread\n";
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"dlut_dtbc");
    ros::NodeHandle n;

    Voxelize* voxelize1;
    voxelize1 = new Voxelize;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local(new pcl::PointCloud<pcl::PointXYZ>);//local map. contain a 360' pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_local_temp(new pcl::PointCloud<pcl::PointXYZ>);//local map. contain a 360' pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global(new pcl::PointCloud<pcl::PointXYZ>);//global map
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global_raw(new pcl::PointCloud<pcl::PointXYZ>);//global map
    std::queue<pcl::PointCloud<pcl::PointXYZ> > point_queue;
    boost::thread get_data(boost::bind(getdata,&point_queue));
    int marker = 0;
    int num_of_scan = 0;
    boost::unordered_map<unordered_map_voxel,un_key> m1;
    boost::unordered_map<unordered_map_voxel,un_key> m2;
    Eigen::Matrix4f tf_sum;
    tf_sum<<1,0,0,0,
        0,1,0,0,
        0,0,1,0,
        0,0,0,0;
    while(ros::ok())
    {
        if(point_queue.empty())
        {
            continue;
        }
        //live time
        {
            boost::mutex::scoped_lock lock(io_mutex);
            *cloud = point_queue.front();
            point_queue.pop();
        }

        /*
           std::string filename;
           std::stringstream ss;
           ss << ros::Time::now();
           ss >> filename;
           filename += ".pcd";
           pcl::PCDWriter writer;
           writer.write(filename,*cloud);
           */

        if(marker==0)
        {
            marker=1;

            pcl::copyPointCloud(*cloud,*cloud_global);
            pcl::copyPointCloud(*cloud,*cloud_global_raw);
            swap(*cloud,*cloud_local);
            cloud->clear();
            voxelize1->generateUmap(*cloud_local,0.3,m2);
            continue;
        }
        *cloud_global_raw += *cloud;
        m1.clear();

        /*
         *pcl::transformPointCloud(*cloud,*cloud,tf_sum);
         */

        voxelize1->generateUmap(*cloud,0.3,m1);
        Icp icp(m1,m2,*cloud,0);
        Eigen::Matrix4f tf_mat;
        tf_mat = icp.icpFit();
        tf_sum+=tf_mat;
        //cout <<endl<<"tf_mat = "<<tf_mat <<endl;
        num_of_scan++;
        pcl::transformPointCloud(*cloud,*cloud,tf_mat);
        *cloud_local_temp += *cloud;
        //if(num_of_scan %3 )
        {
            cloud_local->swap(*cloud_local_temp);
            cloud_local_temp->clear();
            m2.clear();
            voxelize1->generateUmap(*cloud_local,0.3,m2);
        }
        *cloud_global += *cloud;

    }

    pcl::PCDWriter writer;
    if(cloud_global->size()!=0)
    {
        writer.write("global_map.pcd",*cloud_global);
        writer.write("global_map_raw.pcd",*cloud_global_raw);
    }
    delete voxelize1;

    /*
       Voxelize* voxelize1;
       voxelize1 = new Voxelize;


    //打开两个pcd文件并且分别进行栅格化处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> cloud_fil;
    pcl::PointCloud<pcl::PointXYZ> cloud_fil1;


    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>("test.pcd",*cloud);

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

    delete voxelize1;
    Icp testicp(m1,m2,cloud_fil,0);
    testicp.icpFit();
    */
    cout << "hello icp" <<endl;
    return 0;
}
