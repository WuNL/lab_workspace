#include "../include/dlut_monitor/getdata.h"
#include "../include/dlut_monitor/t1.h"
//#include "../include/dlut_monitor/main_window.hpp"
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <string>
#include <sstream>
#include <QDebug>
t1::t1():is_save_(false)
{

/*
  double alpha_ = 0.0095;
  double beta_ = 0.0017;
  double gamma_ = -0.0262;
  double length_ = -0.0040;
  */
//new
  double alpha_ = 0;
  double beta_ = 0;
  double gamma_ = 0;
  double length_ = -0.0040;

  h2p_a_ = cos(beta_)*cos(gamma_) - sin(alpha_)*sin(beta_)*sin(gamma_);
  h2p_b_ = -cos(beta_)*sin(gamma_) - sin(alpha_)*sin(beta_)*cos(gamma_);
  h2p_c_ = -cos(alpha_)*sin(beta_);
  h2p_d_ = cos(alpha_)*sin(gamma_);
  h2p_e_ = cos(alpha_)*cos(gamma_);
  h2p_f_ = -sin(alpha_);
  h2p_g_ = sin(beta_)*cos(gamma_) + sin(alpha_)*cos(beta_)*sin(gamma_);
  h2p_h_ = -sin(beta_)*sin(gamma_) + sin(alpha_)*cos(beta_)*cos(gamma_);
  h2p_i_ = cos(alpha_)*cos(beta_);
}
t1::~t1()
{

}
void t1::messageemit()
{
	qDebug("update have send \n");
	Q_EMIT update();
}
void t1::messageemit_people()
{
	qDebug("update_people have send \n");
	Q_EMIT update_people();
}
void t1::showdata()
{
	std::vector<std::vector<float> > temp_rangevec;
	std::vector<double> temp_stampvec;
	temp_rangevec.swap(MyGetDataThread::range_vec_);
	temp_stampvec.swap(MyGetDataThread::timestamp_vec_);
	//MyGetDataThread::timestamp_vec_.clear();
	//MyGetDataThread::range_vec_.clear();
	qDebug("showdata \n");
	static int i1 = 0;
	qDebug("into showdata %d \n",i1);
	i1 ++;
    int height = static_cast<int>(temp_rangevec[0].size()); //the points number of one laserscan
    int width = static_cast<int>(temp_rangevec.size()); //the number of all laserscans
    qDebug("%d,%d\n",height,width);
	float angle_resolution = MyGetDataThread::angle_resolution_;
	float angle_min = MyGetDataThread::angle_min_;

	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = width;
	cloud.height = height;
	cloud.is_dense = false;
	cloud.points.resize(width*height);

	std::vector<pcl::PointXYZ> points_temp;
	points_temp.reserve(width*height);
    
	for (int i = 0; i < width; ++i)
	{
		double angleCurrentScan = temp_stampvec[i];
		//qDebug("angleCurrentScan= %lf\n",angleCurrentScan);
		for (int j = 0; j < height; ++j)
		{
			float range_distance = temp_rangevec[i][j];
			
	        float x_laser = range_distance*cosf(angle_min + j * angle_resolution);
	        float y_laser = range_distance*sinf(angle_min + j * angle_resolution);
	        float z_laser = 0;
	        cloud.points[i*height + j].x = -cos(angleCurrentScan)*y_laser + (0)*sin(angleCurrentScan);
	        cloud.points[i*height + j].y = -sin(angleCurrentScan)*y_laser - (0)*cos(angleCurrentScan);
	        cloud.points[i*height + j].z = x_laser;
	        points_temp.push_back(cloud.points[i*height + j]);
	        /*
	        float a = -cos(angleCurrentScan)*0 + sin(angleCurrentScan)*0.0128996;
	        float b = -cos(angleCurrentScan)*0 + sin(angleCurrentScan)*(-0.999917);
	        float c = -cos(angleCurrentScan)*1 + sin(angleCurrentScan)*0;
	        float d = -sin(angleCurrentScan)*0 + cos(angleCurrentScan)*0.0128996;
	        float e = -sin(angleCurrentScan)*0 + cos(angleCurrentScan)*(-0.999917);
	        float f = -sin(angleCurrentScan)*1 + cos(angleCurrentScan)*0;
	        float g = 1*0.999917 + 0 + 0;
	        float h = 1*0.0128996;
	        //float i = 0;
	        cloud.points[i*height + j].x = (a*x_laser + b*y_laser + (-0.0130)*sin(angleCurrentScan));
	        cloud.points[i*height + j].y = d*x_laser + e*y_laser - (-0.0130)*cos(angleCurrentScan);
	        cloud.points[i*height + j].z = -(g*x_laser + h*y_laser);
			points_temp.push_back(cloud.points[i*height + j]);	
			*/

			

		} 
	}

	MyGetDataThread::point_cloud = points_temp;
	if(is_save_)
	{
		qDebug("is saving!!!\n");
		std::string filename;
		std::stringstream ss;    
		ss << ros::Time::now();
		ss >> filename;
		filename += ".pcd";
		pcl::io::savePCDFileBinary(filename,cloud);
	}

	cloud.clear();
	points_temp.clear();

	messageemit();	

}

void t1::showpeople()
{
	qDebug("slot showpeople \n");
	static int j1 = 0;
	qDebug("into showpeople %d \n",j1);
	j1 ++;
	std::vector<std::vector<float> > temp_rangevec;
	std::vector<double> temp_stampvec;
	temp_rangevec.swap(MyGetDataThread::range_vec_);
	temp_stampvec.swap(MyGetDataThread::timestamp_vec_);
    int height = static_cast<int>(temp_rangevec[0].size()); //the points number of one laserscan
    int width = static_cast<int>(temp_rangevec.size()); //the number of all laserscans
    qDebug("%d,%d\n",height,width);
	float angle_resolution = MyGetDataThread::angle_resolution_;
	float angle_min = MyGetDataThread::angle_min_;

	pcl::PointCloud<pcl::PointXYZ> cloud;
	cloud.width = width;
	cloud.height = height;
	cloud.is_dense = false;
	cloud.points.resize(width*height);

	std::vector<pcl::PointXYZ> points_temp;
	points_temp.reserve(width*height);


	for (int i = 0; i < width; ++i)
	{
		double angleCurrentScan = temp_stampvec[i];
		//qDebug("angleCurrentScan= %lf\n",angleCurrentScan);
		for (int j = 0; j < height; ++j)
		{
			float range_distance = temp_rangevec[i][j];
			
	        float x_laser = range_distance*cosf(angle_min + j * angle_resolution);
	        float y_laser = range_distance*sinf(angle_min + j * angle_resolution);
	        float z_laser = 0;
	        
	        
	 	      float x_p = h2p_a_*x_laser +h2p_b_*y_laser + h2p_c_*z_laser;
	        float y_p = h2p_d_*x_laser +h2p_e_*y_laser + h2p_f_*z_laser;
	        float z_p = h2p_h_*x_laser +h2p_g_*y_laser + h2p_i_*z_laser;       
	        
	        float x_r = -cos(angleCurrentScan)*y_laser + (0.004)*sin(angleCurrentScan);
	        float y_r = -sin(angleCurrentScan)*y_laser - (0.004)*cos(angleCurrentScan);
	        float z_r = x_laser;
	        
	        
	        cloud.points[i*height + j].x = x_r;
	        cloud.points[i*height + j].y = y_r;
	        cloud.points[i*height + j].z = z_r;
	        

	        points_temp.push_back(cloud.points[i*height + j]);
	        /*
	        float a = -cos(angleCurrentScan)*0 + sin(angleCurrentScan)*0.0128996;
	        float b = -cos(angleCurrentScan)*0 + sin(angleCurrentScan)*(-0.999917);
	        float c = -cos(angleCurrentScan)*1 + sin(angleCurrentScan)*0;
	        float d = -sin(angleCurrentScan)*0 + cos(angleCurrentScan)*0.0128996;
	        float e = -sin(angleCurrentScan)*0 + cos(angleCurrentScan)*(-0.999917);
	        float f = -sin(angleCurrentScan)*1 + cos(angleCurrentScan)*0;
	        float g = 1*0.999917 + 0 + 0;
	        float h = 1*0.0128996;
	        //float i = 0;
	        
	        if(j>=200&&j<=520)
	        {
	        	int j2 = 0;
		        cloud.points[i*321 + j2].x = (a*x_laser + b*y_laser + (-0.0130)*sin(angleCurrentScan));
		        cloud.points[i*321 + j2].y = d*x_laser + e*y_laser - (-0.0130)*cos(angleCurrentScan);
		        cloud.points[i*321 + j2].z = -(g*x_laser + h*y_laser);
				points_temp.push_back(cloud.points[i*321 + j2]);
				j2 ++;	
	        }
	        
	        if(j>=200&&j<=520)
	        {
	        	int j2 = 0;
		        cloud.points[i*321 + j2].x = -cos(angleCurrentScan)*y_laser + (0.004)*sin(angleCurrentScan);
		        cloud.points[i*321 + j2].y = -sin(angleCurrentScan)*y_laser - (0.004)*cos(angleCurrentScan);
		        cloud.points[i*321 + j2].z = x_laser;
				points_temp.push_back(cloud.points[i*321 + j2]);
				j2 ++;	
	        }
			*/

		} 
	}

	MyGetDataThread::point_cloud = points_temp;
	if(is_save_)
	{
		qDebug("is saving!!!\n");
		std::string filename;
		std::stringstream ss;    
		ss << ros::Time::now();
		ss >> filename;
		filename += ".pcd";
		pcl::io::savePCDFileASCII(filename,cloud);
	}

	cloud.clear();
	points_temp.clear();
	MyGetDataThread::timestamp_vec_.clear();
	MyGetDataThread::range_vec_.clear();
	messageemit_people();	
}

void t1::run()
{
	qDebug("run \n");
}

bool t1::is_save(bool save)
{
	is_save_ = save;
	return is_save_;
}
