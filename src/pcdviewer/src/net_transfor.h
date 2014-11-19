/*************************************************************************
	> File Name: net_transfor.h
	> Author: 吴乃亮
	> Mail: wunailiang@gmail.com 
	> Created Time: Wed 16 Jul 2014 02:12:45 PM CST
 ************************************************************************/

#include<iostream>
#include <boost/asio.hpp>
#include <vector>
#include <pcl/io/pcd_io.h>
//enum { IP_ADDR = "192.168.0.170",
//     IP_PORT = "3456"};
using namespace std;
using boost::asio::ip::tcp;


class Net_transfor
{
public:
    Net_transfor(int marker_):marker(marker_),
    endpoint(tcp::v4(),static_cast<unsigned int> (3456)),
    acceptor(io_service,endpoint),
    s(io_service),
    m1(true),
    m2(true),
    resolver(io_service),
    query(tcp::v4(),"192.168.0.171","3356"),
    iterator(resolver.resolve(query))
    {
        
    }
    bool send(pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        if(m2)
        {
            s.connect(*iterator);
            m2 = false;
        }
        int num = cloud.size();
        vector<short> points(num*3);
        const int conversion = 1000;
        for(size_t i =0;i<num;++i)
        {
            const pcl::PointXYZ& point = cloud.points[i];
            points[i*3+0] = static_cast<short>(point.x*conversion);
            points[i*3+1] = static_cast<short>(point.y*conversion);
            points[i*3+2] = static_cast<short>(point.z*conversion);
        }
        vector<int> num_of_pt(1);
        num_of_pt[0] = points.size();
        try{
            boost::asio::write(s,boost::asio::buffer(num_of_pt));
            boost::asio::write(s,boost::asio::buffer(points));
        }
        catch(boost::system::system_error const& e)
        {
            cout <<"warning:"<<e.what()<<endl;
        }
        cout <<"send over" <<endl;
    }
    bool receive(pcl::PointCloud<pcl::PointXYZ>& cloud)
    {
        if(m1)
        {
            acceptor.accept(s);
            m1 = false;
        }
        double t0 = ros::Time::now().toSec();
        vector<int> num_of_pt(1);
        boost::asio::read(s,boost::asio::buffer(num_of_pt),boost::asio::transfer_all());
        int num = num_of_pt[0];
        vector<short> buf_pt(num);
        try
        {
            boost::asio::read(s,boost::asio::buffer(buf_pt),boost::asio::transfer_all());
        }
        catch(boost::system::system_error const& e)
        {
            cout <<"warning:"<<e.what()<<endl;
        }
        cloud.clear();
        for(int i=0;i<num;i+=3)
        {
            pcl::PointXYZ temp;
            double x = static_cast<double>(buf_pt[i]);
            double y = static_cast<double>(buf_pt[i+1]);
            double z = static_cast<double>(buf_pt[i+2]);
            temp.x = x/1000;
            temp.y = y/1000;
            temp.z = z/1000;
            cloud.push_back(temp);
        }
        cout <<"received "<<cloud.size()<<"points\n";
        double t1 = ros::Time::now().toSec();
        cout <<"receive spend time ="<<(t1-t0)<<endl;
        pcl::PCDWriter writer;
        writer.write("received.pcd",cloud);

    }


private:
    int marker;
    boost::asio::io_service io_service;
    tcp::endpoint endpoint;
    tcp::acceptor acceptor;
    tcp::socket s;

    bool m1;//用于开启accept
    bool m2;//connect
    tcp::resolver resolver;
    tcp::resolver::query query;
    tcp::resolver::iterator iterator;

};

