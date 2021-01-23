/**
 * @function 读取超声波模块上传的数据并发布到ROS topic上
 * 
 * 备注： 发布话题 /ultrasonic/data
 * 
 * maker:crp
 * rupingcen@vip.qq.com
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
 

serial::Serial ros_ser;
ros::Publisher pub;
std::string frame_id="/ultrasonic";
int show_message=0;

uint16_t ultra_value[12];
uint8_t data_raw[200];


void analy_ultra_Frame(uint8_t data_raw[], int data_length);
void publish_ultra_Raw_Data(int flag);

int main(int argc,char** argv)
{
 	string pub_topic_ultra,dev;
	int buad,time_out,hz;
	ros::init(argc, argv, "ultrasonic_ros");
	ros::NodeHandle n("~");

	n.param<std::string>("pub_topic_ultra", pub_topic_ultra, "/ultrasonic/data");
	n.param<std::string>("dev", dev, "/dev/ttyUSB0");
	n.param<int>("buad", buad, 115200);
	n.param<int>("time_out", time_out, 1000);
	n.param<int>("hz", hz, 200);
	n.param<int>("show_message", show_message, 0);
	
	ROS_INFO_STREAM("pub_topic_ultra:   "<<pub_topic_ultra);
	ROS_INFO_STREAM("dev:   "<<dev);
	ROS_INFO_STREAM("buad:   "<<buad);
	ROS_INFO_STREAM("time_out:   "<<time_out);
	ROS_INFO_STREAM("hz:   "<<hz);
	ROS_INFO_STREAM("show_message:   "<<show_message);

	ros::Rate loop_rate(hz);
	pub = n.advertise<ultrasonic_ros::ultrasonic>(pub_topic_ultra, 10);
	  
	 try
	 {
	    ros_ser.setPort(dev);
	    ros_ser.setBaudrate(buad);
	    //ros_serial::Timeout to = serial::Timeout::simpleTimeout(1000);
	    serial::Timeout to = serial::Timeout::simpleTimeout(time_out);
	    ros_ser.setTimeout(to);
	    ros_ser.open();
	    ros_ser.flushInput(); 
	 }
	 catch (serial::IOException& e)
	 {
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}
	if(ros_ser.isOpen())
	{
		ros_ser.flushInput(); 
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
			int data_length;
			serial_data.data = ros_ser.read(ros_ser.available());
			data_length=serial_data.data.size();
			if(data_length<1 || data_length>500)
			{
				ros_ser.flushInput(); 	
				ROS_INFO_STREAM("serial data is too long,  len: " << serial_data.data.size() );
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
			if(data_raw[kk+len-2] == 0xEF && data_raw[kk+len-1] == 0xFE)
			{
				uint8_t sum=0x00;
				for(int i=0;i<len;i++)
				{
					if(i == (len-3))
						continue;

					if((kk+i)>=data_length)
						return ;
					
					sum+=data_raw[kk+i];
				}

				if(sum == data_raw[kk + len-3])
				{
					// ultrasonic measutements 
					if(data_raw[kk+3] == 0xA1 )
					{
						uint8_t m=0;
						for(uint8_t index_i=4;index_i<len-3;index_i+=2)
						{
							if(m>11) 
								continue;
							
							ultra_value[m] = (data_raw[kk + index_i]*256+data_raw[kk + index_i+1]);	
							m++;		
						}
						if(show_message)
						{
							cout<<endl<<"ultrasonic ch1-ch12 value:";
							for(uint8_t index_i=0;index_i<12;index_i++)
							{
								cout<<"  "<<ultra_value[index_i];
							}	
						}	
					}
					flag = 0x01;
				}
				else
				{
					cerr<<"check sum error "<<endl;
				}
			}
			
		}	 
		else
		{
			flag = 0x00;
		}
		
		if(flag == 0x01)
			kk = kk+1;
		else
			kk = kk+1;
	}
}
void publish_ultra_Raw_Data(int flag)
{
	ultrasonic_ros::ultrasonic msg;
	msg.header.stamp = ros::Time::now();
	msg.header.frame_id = frame_id;
 	
	for(uint8_t index_i=0;index_i<12;index_i++)
	{
	    msg.data.push_back(ultra_value[index_i]);
	}
  
	pub.publish(msg);
}
  
