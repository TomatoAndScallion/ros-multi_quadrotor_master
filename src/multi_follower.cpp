#include <ros/ros.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include "muti_quadrotor_manager/heartbeat.h"
#include "muti_quadrotor_manager/dt.h"
#include "muti_quadrotor_manager/find_node.h"
#include <stdio.h>
#include <vector>
#include "muti_quadrotor_manager/multi_quadrotor.h"
//#include <sys/time.h>
//#include <errno.h>

bool is_synchro = true;
bool is_group = false;
using namespace std;
ros::Publisher leader_pub; 
ros::Publisher find_node_pub;
ros::Publisher delta_t_pub;
ros::Duration dt = ros::Duration(0,0);
string my_addr;
uint8_t my_level;
int  my_num;
uint8_t heart_count = 0;
vector<relative_position> my_relative;
/*******************************************************
* multi_follower node
* 将本节点进行时间同步。
* 获得本节点编队与其他节点的相关信息。
*******************************************************/
//获得本机ip地址作为本机的编号。 
bool GetIP(const string& vNetType,string& strip)
{
    
    
        for(char c='0';c<='9';++c)
        {
            string strDevice = vNetType + c; //根据网卡类型，遍历wlan网卡
            int fd;
            struct ifreq ifr;
            //使用UDP协议建立无连接的服务
            fd = socket(AF_INET, SOCK_DGRAM, 0);
            strcpy(ifr.ifr_name, strDevice.c_str());       
            //获取IP地址
            if (ioctl(fd, SIOCGIFADDR, &ifr) <  0)
            {
                ::close(fd);
                continue;
            }

            // 将一个IP转换成一个互联网标准点分格式的字符串
            strip = inet_ntoa(((struct sockaddr_in*)&(ifr.ifr_addr))->sin_addr);
            if(!strip.empty())
            {
                ::close(fd);
                return true;
            }
        }
    
    return false;
}
//接收到时间同步话题，计算时间差dt
void leader_pub_callback(const muti_quadrotor_manager::heartbeat& msg)
{
  
  
  ros::Time T4 = ros::Time::now();//TPSN协议 dt = ((T2-T1)-(T4-T3))/2
  ros::Time T3 = msg.header.stamp;
  dt = ((msg.T2_time_stamp - msg.T1_time_stamp) - (T4 - T3));
  dt.sec = dt.sec/2;
  dt.nsec = dt.nsec/2;
  
 
  ROS_INFO("synchro_done");
  
  is_synchro = true;
}
//接收主节点发给的组网请求以及组网确认还有时间同步请求
void follower_find_node_callback(const muti_quadrotor_manager::find_node& msg)
{
    if(msg.clent_addr == my_addr && msg.level == 0)
    {
        is_synchro = false;
    }
    else if(msg.clent_addr == my_addr && msg.level == my_level)
    {
        is_group = true;
    }
    else if(msg.clent_addr == "0.0.0.0" && is_group == false)
    {
        my_level = msg.level+1;
        muti_quadrotor_manager::find_node follower_find_pub;
        follower_find_pub.header.stamp = ros::Time::now();
        follower_find_pub.level  = my_level;
        follower_find_pub.clent_addr = my_addr;
        follower_find_pub.uav_num = my_num;
        find_node_pub.publish(follower_find_pub);
    }

}

int main (int argc, char** argv){
    string vs;//预先定义了几种可能的网卡类型
    //树莓派外置网卡 enxec172fd231f7
    vs= "wlan";//网卡名字，这里选择无线网卡

    if( (GetIP(vs,my_addr)) )
    {
        cout<<"Get my wlan addr:"<<my_addr<<endl;
    }
    string rosname ;
    if (argc >= 2)
    {
        rosname = argv[1];
        sscanf( argv[2], "%d", &my_num );
    }
    else 
    {
        rosname = "uav1";
        my_num = 1;
    }

    
   
    
    ros::init(argc, argv, "follower");
    ros::NodeHandle nh;

    ros::Subscriber leader_sub = nh.subscribe("/leader_pub", 20, leader_pub_callback);
    leader_pub = nh.advertise<muti_quadrotor_manager::heartbeat> ("/leader_sub", 20);
    delta_t_pub = nh.advertise<muti_quadrotor_manager::dt> ("delta_t", 20);
    ros::Publisher heart_beat_send = nh.advertise<muti_quadrotor_manager::find_node>("/heart_send",50);
    ros::Subscriber follower_find_node_sub = nh.subscribe("/leader_find_node",100,follower_find_node_callback); 
    find_node_pub = nh.advertise<muti_quadrotor_manager::find_node>("/leader_load_node",100);
    ros::Rate loop_rate(50);
    while(ros::ok()){

        ros::spinOnce();
        muti_quadrotor_manager::heartbeat pub2;
        muti_quadrotor_manager::dt delta_t_pub2;
        muti_quadrotor_manager::find_node heart_send_pub;
        if(!is_synchro)//发送时间同步话题给主节点
        {
            pub2.header.stamp = ros::Time::now();
            pub2.clent_addr = my_addr;
            
            leader_pub.publish(pub2);
        }
        else if (is_group && is_synchro)//发送与主节点的时间差给所有本地设备运行的节点。
        {
            ros::Time TN = ros::Time::now()+dt;
            //cout<<TN.sec<<'\t'<<TN.nsec<<endl;
            delta_t_pub2.header.stamp = ros::Time::now()+dt;
            delta_t_pub2.clent_addr = my_addr;
            delta_t_pub2.delta_t = dt;
            delta_t_pub.publish(delta_t_pub2);
            heart_count ++;
            if(heart_count >= 50)
            {
                heart_count = 0;
                heart_send_pub.header.stamp = ros::Time::now()+dt;
                heart_send_pub.uav_num = my_num;
                heart_send_pub.clent_addr = my_addr;
                heart_send_pub.level = my_level;
                heart_beat_send.publish(heart_send_pub);
            }

        }          
        
        loop_rate.sleep();
    }
}
