/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
 ** Includes
 *****************************************************************************/

#include <QtGui>
#include <ros/ros.h>
#include <iostream>
#include "../include/pcdviewer/main_window.hpp"
#include <QVTKWidget.h>
#include <QMessageBox>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <vector>

#include <pcl/filters/statistical_outlier_removal.h>

//#include <pcl/registration/ndt.h>
#include <pcl/filters/passthrough.h>

#include <pcl/visualization/pcl_visualizer.h>
//#include <boost/thread/thread.hpp>
#include <pcl/registration/ia_ransac.h>
#include <eigen3/Eigen/Dense>

#include <boost/thread.hpp>


#include <pcl/filters/voxel_grid.h>
//用于boost网络传输
#include <boost/asio.hpp>
#include <boost/thread/thread.hpp>


#include "net_transfor.h"
#include "voxel/voxel.h"
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
/*****************************************************************************
 ** Namespaces
 *****************************************************************************/

namespace pcdviewer {

    using namespace Qt;

    typedef boost::unordered_map<unordered_map_voxel,un_key> umap;

    /*****************************************************************************
     ** Implementation [MainWindow]
     *****************************************************************************/

    MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
        : QMainWindow(parent)
    {
        ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
        //viewer = new pcl::visualization::CloudViewer("Cloud Viewer by WuNL");
        //ui.gridLayout->addWidget(viewer);
        pcdwidget = new PointCloudWidget(this);
        ui.gridLayout_viewer->addWidget(pcdwidget);

    }

    void wait(int sec)
    {
        boost::this_thread::sleep(boost::posix_time::seconds(sec));
    }
    MainWindow::~MainWindow() {
        delete pcdwidget;
    }
    void MainWindow::count(int id)
    {
        while(1)
        {
            cout <<"id=\t"<<id<<endl;
            wait(1);

        }
    }




    void MainWindow::on_pushButton_opnefile_clicked()
    {
        ROS_INFO("filecatkin");
        QString path = QFileDialog::getOpenFileName(this, tr("Open PCD File"), ".", tr("PCD Files(*.PCD)"));
        if(path.length() == 0)
        {
            QMessageBox::information(NULL, tr("Path"), tr("You didn't select any files."));
        }
        else
        {

            std::string file = path.toLatin1().data();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fi (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fil (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);



            std::vector<int> indices;
            pcl::PCDReader reader;

            reader.read<pcl::PointXYZ> (file, *cloud);
            cout<<"cloud->size="<<cloud->size()<<"\t";

            pcl::VoxelGrid<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(0.01f,0.01f,0.03f);
            sor.filter(*cloud_fi);
            cout<<"cloud_fi->size="<<cloud_fi->size()<<endl;

/*
 *
 *            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor1;
 *            sor1.setInputCloud (cloud_fi);
 *            sor1.setMeanK (100);
 *            sor1.setStddevMulThresh (1.5);
 *            sor1.filter (*cloud_fil);
 *
 */
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud (cloud_fi);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (0.1, 2.5);
            //pass.setFilterLimitsNegative (true);
            pass.filter (*cloud_f);


            pcdwidget->showPointCloud(cloud_f);

        }
        /****测试网络传输功能***
          using boost::asio::ip::tcp;
          boost::asio::io_service io_service;

          tcp::resolver resolver(io_service);
          tcp::resolver::query query(tcp::v4(),"192.168.0.170","3456");
          tcp::resolver::iterator iterator = resolver.resolve(query);

          tcp::socket s(io_service);
          s.connect(*iterator);
         **/
    }
    using namespace boost;
    using boost::asio::ip::tcp;
    struct PointCloudBuffers
    {
        //typedef boost::shared_ptr<PointCloudBuffers> Ptr;
        std::vector<short> points;
    };
    void MainWindow::on_pushButton_test_clicked()
    {

        ROS_INFO("TEST");
        Voxelize* voxelize1;
        voxelize1 = new Voxelize;
        QString path = QFileDialog::getOpenFileName(this, tr("Open PCD File"), ".", tr("PCD Files(*.PCD)"));
        if(path.length() == 0)
        {
            QMessageBox::information(NULL, tr("Path"), tr("You didn't select any file."));
        }
        else
        {

            std::string file = path.toLatin1().data();
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ> cloud_fil;
            boost::unordered_map<unordered_map_voxel,un_key> m1;
            std::vector<int> indices;
            pcl::PCDReader reader;
            reader.read<pcl::PointXYZ> (file, *cloud);
            pcl::VoxelGrid<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud);
            sor.setLeafSize(0.01f,0.01f,0.01f);
            sor.filter(cloud_fil);
            voxelize1->generateUmap(cloud_fil,0.3,m1);

            for(umap::iterator iter=m1.begin();iter!=m1.end();iter++)
            {
                if(iter->second.p>0.7)
                {
                    for(int j = 0 ; j< iter->second.vec_index_point.size(); j++)
                    {
                        double x_ = cloud_fil[iter->second.vec_index_point[j]].x;
                        double y_ = cloud_fil[iter->second.vec_index_point[j]].y;
                        double z_ = cloud_fil[iter->second.vec_index_point[j]].z;
                        pcl::PointXYZ temppoint(0,0,0);

                        temppoint.x = x_;
                        temppoint.y = y_;
                        temppoint.z = z_;
                        cloud_f->push_back(temppoint);
                    }
                    //cout<<iter->second.v1<<endl;
                }
            }
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud (cloud_f);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (-3.5, 3.5);
            pass.filter (*cloud_final);
            pcdwidget->showPointCloud(cloud_final);


        }
        delete voxelize1;
    }
};
/*****************************************************************************
 ** Implementation [Slots]
 *****************************************************************************/




