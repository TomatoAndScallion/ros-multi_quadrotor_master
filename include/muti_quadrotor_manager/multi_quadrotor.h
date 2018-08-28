#ifndef MULTI_QUAD_H
#define MULTI_QUAD_H

#include <string>
#include <stdint.h>
#include <vector>
#include <ros/ros.h>
using namespace std;
typedef struct 
{
    //相对的xyz的位置
    float x;
    float y;
    float z;
    //相对的yaw角
    float yaw;

} relative_position;
class quadrotor_node
{
public:
    quadrotor_node(string Addr,uint8_t level,uint8_t num);
     ~quadrotor_node();
    bool Set_Addr(string Addr);
    string Get_Addr();
    bool Is_Addr(string Addr);
    bool Set_level(uint8_t lev);
    uint8_t Get_level();
    bool Set_alive(bool temp_alive);
    bool Is_alive();
    bool Set_uav_num(uint8_t num_temp);
    uint8_t Get_uav_num(); 
    vector<relative_position> my_ralative;
    ros::Time T_last;
    


private:
    string Address; //网络编号
    uint8_t level;//等级，目前只有两级，一级leader 一级follower
    bool alive ;//是否能够收到控制消息
    uint8_t uav_num;//飞行器编号初始化的时候赋值
     
    /* data */
};

#endif