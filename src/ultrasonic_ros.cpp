/**
 * @function 读取并发布wit智能的IMU数据
 * 
 * 备注： 
 * 
 * maker:crp
 * 2020-11-13
 ****************************************************/

#include <deque>
#include <ros/ros.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Geometry> 
#include <chrono>
#include <locale>
#include <tuple>
#include <algorithm>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>
#include <boost/assert.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>

extern "C" {
#include <fcntl.h>
#include <getopt.h>
#include <poll.h>
#include <time.h>
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <assert.h>
#include <unistd.h> //  close
#include <string.h> //  strerror
}

#include <serial/serial.h>
#include <std_msgs/String.h>


#include <cmath>
#include <Eigen/Dense>
#include <ultrasonic_ros/ultrasonic.h>
  
using namespace std;
using namespace Eigen;
 

static int data_length = 100;

boost::asio::serial_port* serial_port = 0;
 
static uint8_t data_raw[200];
static std::vector<uint8_t> buffer_;
static std::deque<uint8_t> queue_;
static std::string name, frame_id="/world";
 
//static int fd_ = -1;
static ros::Publisher pub;
//static uint8_t tmp[100];
 
serial::Serial ros_ser;

float ultra_value[12];
 

void analy_ultra_Frame(uint8_t data_raw[], int data_length);
void publish_ultra_Raw_Data(int flag);

int main(int argc,char** argv)
{
	string out_result;
  
	string pub_ultrtopic,dev;
	int buad,time_out,hz;
	ros::init(argc, argv, "ultrasonic_ros");
	ros::NodeHandle n("~");


	n.param<std::string>("dev", dev, "/dev/ttyUSB0");
	n.param<int>("buad", buad, 115200);
	n.param<int>("time_out", time_out, 1000);
	n.param<int>("hz", hz, 200);

 
	ROS_INFO_STREAM("dev:   "<<dev);
	ROS_INFO_STREAM("buad:   "<<buad);
	ROS_INFO_STREAM("time_out:   "<<time_out);
	ROS_INFO_STREAM("hz:   "<<hz);
	  
	ros::Rate loop_rate(hz);
	pub = n.advertise<ultrasonic_ros::ultrasonic>("/ultrasonic/data", 1);
	  
	// 开启串口模块
	 try
	 {
	    ros_ser.setPort(dev);
	    ros_ser.setBaudrate(buad);
	    //ros_serial::Timeout to = serial::Timeout::simpleTimeout(1000);
	    serial::Timeout to = serial::Timeout::simpleTimeout(time_out);
	    ros_ser.setTimeout(to);
	    ros_ser.open();
	    ros_ser.flushInput(); //清空缓冲区数据
	 }
	 catch (serial::IOException& e)
	 {
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}
	if(ros_ser.isOpen())
	{
		ros_ser.flushInput(); //清空缓冲区数据
		ROS_INFO_STREAM("Serial Port opened");
	}
	else
	{
	    return -1;
	}
 
    while(ros::ok())
    { 
		if(ros_ser.available())
		{
			//ROS_INFO_STREAM("Reading from serial port");
			std_msgs::String serial_data;
			
			serial_data.data = ros_ser.read(ros_ser.available());//获取串口数据
			data_length=serial_data.data.size();
			if(data_length<1 || data_length>500)
			{
				ros_ser.flushInput(); //清空缓冲区数据	
				ROS_INFO_STREAM("serial data is too short ,  len: " << serial_data.data.size() );
			}
			else
			{
				for(int i=0;i<data_length;i++)
				{	
					data_raw[i] =serial_data.data.at(i);
				}
				analy_ultra_Frame(data_raw, data_length);
				publish_ultra_Raw_Data(1);
				
			}
		}
 
        ros::spinOnce();
        loop_rate.sleep();
			
    }
   
    std::cout<<" EXIT ..."<<std::endl;
    ros::waitForShutdown();
    ros::shutdown();

    return 1;

}
 

void analy_ultra_Frame(uint8_t data_raw[], int data_length)
{
uint8_t flag=0;
for(int kk = 0; kk < data_length - 1; )
{
 	flag=0;
	if(data_raw[kk] == 0xAE && data_raw[kk + 1] == 0xEA)
	{
		uint8_t len = data_raw[kk+2];
		
		uint8_t sum=0x00;
		for(int i=0;i<len;i++)
		{
		    sum+=data_raw[kk+i];
		}

		if(sum == data_raw[kk + len-3])
		{
			if(data_raw[kk+3] == 0x01 )
			{
				uint8_t index_j=0;
				for(uint8_t index_i=4;index_i<len-3;index_i++)
				    ultra_value[index_j++] = (data_raw[kk + index_i+1]<<8|data_raw[kk + index_i]);	
			}
		 
			flag = 0x01;
			 
		}

	}	 
	else
	{
		flag = 0x00;
	}
	
	if(flag == 0x01)
	{
		kk = kk+11;
	}
	else
	{
		kk = kk+1;
	}
}
}
void publish_ultra_Raw_Data(int flag)
{
	ultrasonic_ros::ultrasonic msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = frame_id;
 	
	for(uint8_t index_i=0;index_i<12;index_i++)
	    msg.data.push_back(ultra_value[index_i]);
	//msg.orientation.w = (double)q.w();
	//msg.orientation.x = (double)q.x();
	//msg.orientation.y = (double)q.y();
	//msg.orientation.z = (double)q.z();

	 
	 
	pub.publish(msg);

	 
}
  
