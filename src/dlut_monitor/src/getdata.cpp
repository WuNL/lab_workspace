#include "../include/dlut_monitor/getdata.h"
#include "../include/dlut_monitor/motor.h"
std::vector<double> MyGetDataThread::timestamp_vec_; //save the time stamp of every laser scan
std::vector<std::vector<float> > MyGetDataThread::range_vec_; //save the range values of every laser scan
std::vector<std::vector<float> > MyGetDataThread::intensity_vec_;//save the intensity values

std::vector<pcl::PointXYZ> MyGetDataThread::point_cloud;

float MyGetDataThread::angle_resolution_ = 0.0;
float MyGetDataThread::angle_min_ = 0.0;


MyGetDataThread::MyGetDataThread()
:freqence(0),
laser_seq(0),
laser_seq_people(0),
motorSpeedUnitDegree_(0.0),
motorSpeedUnitRad_(0.0),
data_switch(false),
is_first_judgment_0(false),
is_first_judgment_1(false),
is_first_circle(false),
_is_people_detect(false),
start_getdata(false)
{
  	ROS_INFO("defalt constructor mygetdatathread");
	m_subscriber_ = nh_.subscribe("/scan",1000,&MyGetDataThread::getDataCallBack,this);
}

MyGetDataThread::~MyGetDataThread()
{
  	ROS_INFO("defalt destroyer");
  	ros::shutdown();
	std::vector<double>().swap(timestamp_vec_);
	std::vector<std::vector<float> >().swap(range_vec_);
	std::vector<std::vector<float> >().swap(intensity_vec_);
}
bool MyGetDataThread::data_switch_people(bool is_people_detect)
{
	_is_people_detect = is_people_detect;
	return _is_people_detect;
}
bool MyGetDataThread::data_switch_fun(bool data)
{
	data_switch = data;
	return true;
}
bool MyGetDataThread::start_data_fun(bool start)
{
	start_getdata = start;
	return true;
}

bool MyGetDataThread::setseq(int seq)
{
	laser_seq_people = seq;
	return true;
}

void MyGetDataThread::run()
{
	is_first_circle = false;
	ros::spin();
}

void MyGetDataThread::getDataCallBack(const sensor_msgs::LaserScan::ConstPtr & ls_data)
{
    //std::cout <<"range[0]="<<ls_data->ranges[1080]<<std::endl;
	//if(ls_data->ranges[0]<0.1)
		//return;
	if(data_switch)
	{
		if(!_is_people_detect)
		{
			freqence = dlut_monitor::Motor::m_freq;
			motorSpeedUnitRad_ = 2*pi*freqence/8240.0; // = 2*pi/(8240/f) unit : rad/s

			/*
			//360 scan
			if((ls_data->ranges[700]<0.4) && (is_first_judgment_0 == true))
			{
				qDebug("detect mart 0 ^^\n");
				Q_EMIT showdata();
				is_first_judgment_0 = false;
				is_first_circle = true;
				//timeMotorStart_ = ros::Time::now();
				laser_seq = 1;
			}

			if(ls_data->ranges[700]>0.4)
			{
				is_first_judgment_0 = true;
			}
			


			if(is_first_circle == true)
			{
				angle_resolution_ = ls_data->angle_increment; //get the angle resolution
				angle_min_ = ls_data->angle_min; //get the  minimal angle
				
				double degreeLaserScan = (laser_seq*0.025)*motorSpeedUnitDegree_;//the laserscan degree
				double degreeCurrentFrame = (laser_seq*0.025)*motorSpeedUnitRad_; //the degree which the current scam frame 
				range_vec_ .push_back(ls_data->ranges);
				timestamp_vec_.push_back(degreeCurrentFrame);
				laser_seq++;
			}
			*/
			/*
			//180 scan
			if((ls_data->ranges[700]<0.4) && (is_first_judgment_0 == true))
			{
				qDebug("detect mart 0 ^^\n");
				laser_seq = 0;

				is_first_circle = true;
				is_first_judgment_0 = false;
				Q_EMIT showdata();
			}
      
			if(ls_data->ranges[1]<0.4 && is_first_judgment_1 == true)
			{
				qDebug("detect mart 1 ^^\n");
				//laser_seq = 0;
				is_first_circle = true;
				
				is_first_judgment_1 = false;	
        //range_vec_.clear();
        //timestamp_vec_.clear();
				Q_EMIT showdata();		
			}
      
			if(ls_data->ranges[700]>0.4)
			{
				is_first_judgment_0 = true;
			}
     
			if(ls_data->ranges[1]>0.4)
			{
				is_first_judgment_1 = true;
			}
     

			if(is_first_circle == true)
			{
				angle_resolution_ = ls_data->angle_increment; //get the angle resolution
				angle_min_ = ls_data->angle_min; //get the  minimal angle
				
				double degreeLaserScan = (laser_seq*0.025)*motorSpeedUnitDegree_;//the laserscan degree
				double degreeCurrentFrame = (laser_seq*0.025)*motorSpeedUnitRad_; //the degree which the current scam frame 
				range_vec_ .push_back(ls_data->ranges);
				timestamp_vec_.push_back(degreeCurrentFrame);
				laser_seq++;
			}
			*/

			//180 scan:version 1.0
			if((ls_data->ranges[900]<0.4) && (ls_data->ranges[910]<0.4) && (is_first_judgment_0 == true))
			{
				qDebug("detect marker 0 ^^\n");
				laser_seq = 0;

				is_first_circle = true;
				is_first_judgment_0 = false;
				//Q_EMIT showdata();

				
			}

			if(ls_data->ranges[900]>0.4)
			{
				is_first_judgment_0 = true;
			}

			if(is_first_circle == true)
			{
				angle_resolution_ = ls_data->angle_increment; //get the angle resolution
				angle_min_ = ls_data->angle_min; //get the  minimal angle
				
				//double degreeLaserScan = (laser_seq*0.025)*motorSpeedUnitDegree_;//the laserscan degree
				double degreeCurrentFrame = (laser_seq*0.025)*motorSpeedUnitRad_; //the degree which the current scam frame 
				std::cout <<"degreeCurrentFrame="<<degreeCurrentFrame<<std::endl;
				range_vec_ .push_back(ls_data->ranges);
				timestamp_vec_.push_back(degreeCurrentFrame);
				laser_seq++;
			}
			if(laser_seq == (20*8240/freqence) +1)
			{
				ROS_INFO("laser_seq == %d",(20*8240/freqence)+1);
				Q_EMIT showdata();	
				laser_seq=0;
				is_first_circle = false;
			}
		}
		else
		{
			if(start_getdata)
			{
				std::cout <<"sellp\n";
				sleep(1);
				start_getdata=false;
			}
			freqence = dlut_monitor::Motor::m_freq;
			motorSpeedUnitRad_ = 2*pi*freqence/8240.0; // = 2*pi/(8240/f) unit : rad/s
			//std::cout <<"(20*8240/freqence)="<<(20*8240/freqence)<<std::endl;
			if(laser_seq_people == 0)
			{
				//range_vec_ .push_back(ls_data->ranges);
				//timestamp_vec_.clear();
				//range_vec_.clear();
				//ROS_INFO("seq_people==0 clear the vector");
				
			}
			//if(laser_seq_people < (20*8240/freqence)+1)
			{
				angle_resolution_ = ls_data->angle_increment; //get the angle resolution
				angle_min_ = ls_data->angle_min; //get the  minimal angle
				
				//double degreeLaserScan = (laser_seq_people*0.025)*motorSpeedUnitDegree_;//the laserscan degree
				double degreeCurrentFrame = (laser_seq_people*0.025)*motorSpeedUnitRad_; //the degree which the current scam frame 
				//std::cout <<laser_seq_people<<"\tdegreeCurrentFrame="<<degreeCurrentFrame<<std::endl;
				range_vec_ .push_back(ls_data->ranges);
				timestamp_vec_.push_back(degreeCurrentFrame);
				laser_seq_people++;
			}
			if(laser_seq_people == (20*8240/freqence)+1 )
			{
				Q_EMIT showpeople();
				data_switch = false;
				_is_people_detect = false;
				laser_seq_people = 0;
			}
		}
	}
	
}
