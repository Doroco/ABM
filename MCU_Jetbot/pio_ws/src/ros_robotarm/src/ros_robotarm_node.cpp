#include <ros/ros.h>
#include <ros/node_handle.h>
#include "std_msgs/String.h"
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <math.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Point.h>
#include "ros_tutorial_msgs/msgData.h"


#define ADDR_TORQUE_ENABLE              24 //rx-64 & ex-106
#define ADDR_GOAL_POSITION              30 //rx-64 & ex-106
#define ADDR_PRESENT_POSITION           36 //rx-64 & ex-106

#define PROTOCOL_VERSION                1.0

#define ID0                             1 //ex-106_1
#define ID1                             2 //ex-106_2
#define ID2                             3 //rx-64_3
#define BAUDRATE                        1000000 //1M

#define RX_CENTER                       511.5 //0~1023
#define RX_lowest_limit                 0
#define RX_highest_limit                1023
#define EX_CENTER                       2047.5 //0~4095
#define EX_lowest_limit                 0
#define EX_highest_limit                4095

#define PI                              3.141592
#define BASE_HEIGHT                     23 //base to joint1
#define LINK0                           21
#define LINK1                           25
#define LINK2                           30
#define TORQUE_ENABLE                   1




using namespace std;
using namespace dynamixel;
using namespace Eigen;

double theta1=0, theta2=0, theta0=0;

geometry_msgs::Point end_point;

void angleCallback(const geometry_msgs::Point::ConstPtr &data) {
  theta1=data->x;
  theta2=data->y;
  theta0=data->y;
  cout << theta0 <<std::endl;

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_robotarm_node");
  ros::NodeHandle nh;

  ros::Subscriber angle_sub = nh.subscribe("joint_topic",100,angleCallback);

  const char* DEVICENAME;

  std::string port;
  //int baudrate;
  nh.param("ros_robotarm_node/Port",port,std::string("/dev/ttyUSB0"));
  std::cout << port  << std::endl;
  DEVICENAME = port.c_str();
  PortHandler *pohandler0 = PortHandler::getPortHandler(DEVICENAME);
  PortHandler *pohandler1 = PortHandler::getPortHandler(DEVICENAME);
  PortHandler *pohandler2 = PortHandler::getPortHandler(DEVICENAME);
  PacketHandler *pkhandler0 = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  PacketHandler *pkhandler1 = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  PacketHandler *pkhandler2= PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if(pohandler0->openPort() & pohandler1->openPort()& pohandler2->openPort()){
    if(pohandler0->setBaudRate(BAUDRATE) & pohandler1->setBaudRate(BAUDRATE) & pohandler2->setBaudRate(BAUDRATE))
      ROS_INFO("Success!\n");
  }
  else{
    ROS_INFO("Fail!\n");
    return 0;
  }

  // torque enable
  pkhandler0->write1ByteTxRx(pohandler0, ID0, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
  pkhandler1->write1ByteTxRx(pohandler1, ID1, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
  pkhandler2->write1ByteTxRx(pohandler2, ID2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE);
  // first state
  //  pkhandler0->write4ByteTxRx(pohandler0, ID1, ADDR_GOAL_POSITION, EX_CENTER);
  //  pkhandler1->write4ByteTxRx(pohandler1, ID2, ADDR_GOAL_POSITION, EX_CENTER);
  //  pkhandler2->write4ByteTxRx(pohandler2, ID3, ADDR_GOAL_POSITION, 1023);

  ros::Rate loop_rate(100);
    while(ros::ok())
    {
        double theta0_dynamixel, theta1_dynamixel,theta2_dynamixel;

        theta0_dynamixel=(125+theta0)/250*4095;
        theta1_dynamixel=(125+theta1)/250*4095;
        theta2_dynamixel=(150+theta2)*1023/300;

        // write goal position
        pkhandler0->write4ByteTxRx(pohandler0, ID0, ADDR_GOAL_POSITION, theta0_dynamixel);
        pkhandler1->write4ByteTxRx(pohandler1, ID1, ADDR_GOAL_POSITION, theta1_dynamixel);
        pkhandler2->write4ByteTxRx(pohandler2, ID2, ADDR_GOAL_POSITION, theta2_dynamixel);

    //    ROS_INFO("theta(0,1,2) : (%f, %f, %f)",theta0,theta1,theta2);
    //    ROS_INFO("theta(0,1,2) dynamixel : (%f, %f, %f)",theta0_dynamixel,theta1_dynamixel,theta2_dynamixel);

        pohandler0->closePort();
        pohandler1->closePort();
        pohandler2->closePort();

    }

}
