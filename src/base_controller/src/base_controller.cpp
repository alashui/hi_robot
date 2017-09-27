#include <ros/ros.h>
#include <iostream>
#include <iomanip>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include "string.h"
#include <nav_msgs/Odometry.h>
#include <base_controller/base_Speed.h>

using namespace std;
using namespace boost::asio;

union _send_to_STM32_
{
    char send_buf[14];
    
    struct _speed_value_
    {
        float flag ;
        float linear_vel;
        float angular_vel;
        char endcode[2];
    }Float_RAM;

}Data_send;

union _receive_from_STM32_
{
    unsigned char read_buf[12];
    struct _encoder_value_
    {
        float Flag;
        float Vx_speed;
        float Vz_speed;
    }Float_RAM;
}Data_receive;


void cmd_velCallback(const geometry_msgs::Twist &twist_aux)
{
    geometry_msgs::Twist twist = twist_aux;

    Data_send.Float_RAM.flag = (float)15.5;

    Data_send.Float_RAM.endcode[0] = 0x0d;
    Data_send.Float_RAM.endcode[1] = 0x0a;
    Data_send.Float_RAM.linear_vel = twist.linear.x;
    Data_send.Float_RAM.angular_vel = twist.angular.z;
}

int main(int argc,char** argv)
{
    ros::init(argc,argv,"base_controller");
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 50, cmd_velCallback);//cmd_vel_mux/input/teleop
    //ros::Rate loop_rate(10);

	ros::Publisher speed_pub = n.advertise<base_controller::base_Speed>("base_controller_speed",100);

    base_controller::base_Speed speed;		//速度消息
  
    unsigned char check_buf[1];
	
	//串口设置
    io_service iosev;
    serial_port sp(iosev, "/dev/ttyUSB0");         //定义传输的串口
    sp.set_option(serial_port::baud_rate(115200));   
    sp.set_option(serial_port::flow_control());
    sp.set_option(serial_port::parity());
    sp.set_option(serial_port::stop_bits());
    sp.set_option(serial_port::character_size(8));

    while(ros::ok())
    {
        ros::spinOnce();
        ROS_INFO("linear_x is %f",Data_send.Float_RAM.linear_vel);
        ROS_INFO("angular_z is %f",Data_send.Float_RAM.angular_vel);

        read(sp, buffer(Data_receive.read_buf));	//从串口读取数据
        


        if(Data_receive.Float_RAM.Flag = 15.5)
        {
                speed.linear_x = Data_receive.Float_RAM.Vx_speed;
                speed.angular_z = Data_receive.Float_RAM.Vz_speed; 
				speed_pub.publish(speed);
				ros::spinOnce();
				
                ROS_INFO("the vx is %f",speed.linear_x);
                ROS_INFO("the vth is %f",speed.angular_z);
        }
        else
        {    
            read(sp, buffer(check_buf));
        } 

        write(sp, buffer(Data_send.send_buf,14));	//串口发送数据
    
    }

    iosev.run();

    return 0;
}



