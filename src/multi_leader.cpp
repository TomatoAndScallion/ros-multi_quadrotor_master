#include <ros/ros.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>
#include <vector>
#include <string>
#include "muti_quadrotor_manager/heartbeat.h"
#include "muti_quadrotor_manager/find_node.h"
#include "muti_quadrotor_manager/multi_quadrotor.h"
#include "muti_quadrotor_manager/heartbeats.h"
#include "geometry_msgs/PoseStamped.h" 
#include "geometry_msgs/TwistStamped.h"
#include "muti_quadrotor_manager/com_formation.h"

#define MAX_clent 255
#define My_level 1
using namespace std;
bool find_node_flag = false;
bool synchro_flag = false;
bool synchro_next = false;
bool heart_beat_flag = false;
int circle_count = 0;
int quadrotor_num = 0;
bool start_formation = false;
vector<quadrotor_node> quadrotor_node_list;
ros::Publisher leader_pub; 
ros::Publisher find_leader_pub;
muti_quadrotor_manager::com_formation com_formation_pub;
/*******************************************************
* multi_leader node
* 获得系统中每个编队节点的信息。
* 将每个节点进行时间同步。
*******************************************************/
// 获得本机的ip地址作为编号标识
bool GetIP(const string& vNetType,string& strip)
{
    
    
        for(char c='0';c<='9';++c)
        {
            string strDevice = vNetType + c; //根据网卡类型，遍历wlan网卡
            int fd;
            struct ifreq ifr;
            //使用UDP协议建立无连接的服务
            fd = socket(AF_INET, SOCK_DGRAM, 0);
            strcpy(ifr.ifr_name, strDevice.c_str() );       
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
//send time to follower
void leader_sub_callback(const muti_quadrotor_manager::heartbeat& msg)
{
  
  muti_quadrotor_manager::heartbeat pub1;
  ros::Time T2 = ros::Time::now();
  ros::Time T1 = msg.header.stamp;
  pub1.clent_addr = msg.clent_addr;
  pub1.T1_time_stamp = T1;
  pub1.T2_time_stamp = T2;
  pub1.header.stamp = ros::Time::now();                           
  leader_pub.publish(pub1);
  if(quadrotor_num < quadrotor_node_list.size()&&msg.clent_addr == quadrotor_node_list[quadrotor_num].Get_Addr())
  {
   synchro_next = true;
  }
}
//srv about control node to find other node
bool control_leader_handle(muti_quadrotor_manager::heartbeats::Request  &req,
                           muti_quadrotor_manager::heartbeats::Response &res)
{  
    if(req.master_com == "stop_find_node")
    {
        res.received = true;
        find_node_flag = false;
    }
    else if(req.master_com == "start_find_node")
    {
        res.received = true;
        find_node_flag = true;
    }
    else if(req.master_com == "start_formation")
    {
        res.received = true;
        start_formation = true;
        com_formation_pub.start_time = ros::Time::now();
    }
    else if(req.master_com == "stop_formation")
    {
        res.received = true;
        start_formation = false;
        com_formation_pub.stop_time = ros::Time::now();
    }
    return true;
 }
 //get follower state and save it
void leader_load_callback(const muti_quadrotor_manager::find_node& msg)
{
    if(quadrotor_node_list.size() < MAX_clent)
    {
        for(int i = 0;i<quadrotor_node_list.size();i++)
        {
            if(quadrotor_node_list[i].Is_Addr(msg.clent_addr))
            {
                return;
            }
        }
        quadrotor_node_list.push_back(quadrotor_node(msg.clent_addr,msg.level,msg.uav_num));
        
    }
    find_leader_pub.publish(msg);
}
void beat_sub_callback(const muti_quadrotor_manager::find_node & msg)
{
    for(int i = 0 ; i < quadrotor_node_list.size(); i ++)
    {
        if(quadrotor_node_list[i].Is_Addr(msg.clent_addr));
        {
            quadrotor_node_list[i].T_last = msg.header.stamp;
        }
    }
}
int main (int argc, char** argv){

    ros::init(argc, argv, "multi_leader");
    ros::NodeHandle nh;
    
    ros::Subscriber leader_sub = nh.subscribe("/leader_sub", 20, leader_sub_callback);
    leader_pub = nh.advertise<muti_quadrotor_manager::heartbeat> ("/leader_pub", 20);
    find_leader_pub = nh.advertise<muti_quadrotor_manager::find_node>("/leader_find_node",500);
    ros::Subscriber beat_sub = nh.subscribe("/heart_send",50,beat_sub_callback);
    ros::Subscriber find_leader_sub = nh.subscribe("/leader_load_node",100,leader_load_callback); 

    ros::ServiceServer leader_server = nh.advertiseService("control_leader", control_leader_handle);
    ros::Publisher com_pub = nh.advertise<muti_quadrotor_manager::com_formation>("/com_manager",10);
    string vs;//预先定义了几种可能的网卡类型
    muti_quadrotor_manager::find_node find_node_pub;
    vs= "wlan";//网卡名字，这里选择无线网卡
    
    string my_addr;
    if((GetIP(vs,my_addr)))
    {
        cout<<"Get my wlan addr:"<<my_addr<<endl;
    }
    ros::Rate loop_rate(50);
   
    while(ros::ok()){
     ros::spinOnce();
     if(find_node_flag)//查找节点。
     { 
        if(circle_count < 50)
        {
            find_node_pub.header.stamp = ros::Time::now();
            find_node_pub.clent_addr = "0.0.0.0";
            find_node_pub.level = My_level;
            find_leader_pub.publish(find_node_pub);
            circle_count ++;
        }
        else //初始化时间同步
        {
            synchro_flag = true;
            synchro_next = false;
            quadrotor_num = 0;
        }
        cout<<quadrotor_node_list.size()<<endl;
     }
     else
     {
        if(synchro_flag) //同步时间
        {
            find_node_pub.header.stamp = ros::Time::now();
            find_node_pub.level = 0;
            find_node_pub.clent_addr = quadrotor_node_list[quadrotor_num].Get_Addr();
            find_leader_pub.publish(find_node_pub);
            if(synchro_next == true)
            {
                quadrotor_num++;
                synchro_next = false;
                if(quadrotor_num >= quadrotor_node_list.size())
                {
                    synchro_flag = false;
                    heart_beat_flag =true;
                    cout<<"synchro done"<<endl;
                    for(int i = 0; i < quadrotor_node_list.size();i++)
                    {

                        quadrotor_node_list[i].T_last = ros::Time::now();
                    }
                }
            }
        }

     }
      if(heart_beat_flag)
      {
          ros::Duration dt = ros::Duration(0,0);
          for(int i = 0; i < quadrotor_node_list.size();i++)
          {
                dt = ros::Time::now() - quadrotor_node_list[i].T_last;
                if(dt.sec > 10)
                {
                    quadrotor_node_list[i].Set_alive(false);
                    //cout<<"error"<<endl;
                }
                else
                {
                    quadrotor_node_list[i].Set_alive(true);
                    cout<<dt.sec<<"\t"<<dt.nsec<<endl;
                    //cout<<"OK"<<endl;
                }
          }
          if(start_formation == true)
          {
            com_formation_pub.header.stamp = ros::Time::now();
            com_formation_pub.command = "start";
          }
          else if(start_formation == false)
          {
              com_formation_pub.header.stamp = ros::Time::now();
              com_formation_pub.command = "stop";
          }
          com_pub.publish(com_formation_pub);

      }  
        
       // ros::Time T3 = ros::Time::now();
      // cout<<T3.sec<<'\t'<<T3.nsec<<endl;
        loop_rate.sleep();
    }
}
