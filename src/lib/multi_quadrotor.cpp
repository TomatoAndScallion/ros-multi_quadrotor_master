#include "muti_quadrotor_manager/multi_quadrotor.h"
#include <iostream>
#include <string>
#include <ros/ros.h>
#include <stdint.h>
using namespace std;

quadrotor_node::quadrotor_node(string Addr,uint8_t level,uint8_t num):T_last()
{
  this->Address = Addr;
  this->level = level;
  this->uav_num = num;
  this->alive = true;
  
}
quadrotor_node::~quadrotor_node()
{
   
}
bool quadrotor_node::Set_Addr(string Addr)
{
    this->Address = Addr;
    return true;
}
string quadrotor_node::Get_Addr()
{
    return (this->Address);
}
bool quadrotor_node::Is_Addr(string Addr)
{
    if(Addr == this->Address)
    {
        return true;
    }
    else 
    {
        return false;
    }
}
bool quadrotor_node::Set_level(uint8_t lev)
{
    this->level = lev;
    return true;
}

uint8_t quadrotor_node::Get_level()
{
    return(this->level);
}
bool quadrotor_node::Set_alive(bool temp_alive)
{
    this->alive = temp_alive;
    return true;
}
bool quadrotor_node::Is_alive()
{
   return (this->alive);
}
bool quadrotor_node::Set_uav_num(uint8_t num_temp)
{
    this->uav_num = num_temp;
    return true;
}
uint8_t quadrotor_node::Get_uav_num()
{
    return (this->uav_num);
} 
