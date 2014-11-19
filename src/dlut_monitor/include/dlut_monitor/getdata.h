#ifndef GETDATA_H
#define GETDATA_H



#include <QThread>
#include <QObject>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "motor.h"
#include <vector>
#include <pcl/point_types.h>
class MyGetDataThread: public QThread
{
	
Q_OBJECT
public:
	MyGetDataThread();
	~MyGetDataThread();
	bool data_switch_fun(bool data);
	bool data_switch_people( bool is_people_detect);
	bool setseq(int seq);
	bool start_data_fun(bool start);
Q_SIGNALS:
	void showdata();
	void showpeople();
public:
	void run();
	static std::vector<double> timestamp_vec_; //save the time stamp of every laser scan
	static std::vector<std::vector<float> > range_vec_; //save the range values of every laser scan
	static std::vector<std::vector<float> > intensity_vec_;//save the intensity values
	
	static std::vector<pcl::PointXYZ> point_cloud;

	static float angle_resolution_;//the angle between two lasers
	static float angle_min_;//the minimal angle
private:
	ros::NodeHandle nh_;
	ros::Subscriber m_subscriber_;

	void getDataCallBack(const sensor_msgs::LaserScan::ConstPtr & ls_data);
	static const double pi = 3.14159265358;



	int freqence;
	int laser_seq;
	int laser_seq_people;
	double motorSpeedUnitDegree_;//motor speed with unit degree/s;
	double motorSpeedUnitRad_;//motor speed with unit rad/s;
	bool data_switch;
	bool is_first_judgment_0;
	bool is_first_judgment_1;
	bool is_first_circle;
	bool _is_people_detect;

	bool start_getdata;
	
};

#endif
