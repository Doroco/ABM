#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "serial/serial.h"
#include "cstdlib"
#include "math.h"
#include "stdio.h"
#include "t_serial.h"

serial::Serial ser;

t_serial g_serial;

int16_t x;
int16_t z;
int16_t checksum;

uint8_t tx_x_L;
uint8_t tx_x_H;
uint8_t tx_z_L;
uint8_t tx_z_H;
uint8_t tx_checksum_L;
uint8_t tx_checksum_H;

char tx_twist[10];
uint8_t tx_twist2[10];

void write_callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO_STREAM("Writing to serial port" << msg->data);

    ser.write(msg->data);
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg )
{

  char s2[17];
  int16_t zero = 85;
  uint8_t zero_H = zero >> 8;
  uint8_t zero_L = zero &0xff;
  char zerostr[10]={1,1,1};
  printf("zero : %d",zero);
  printf("zero_H : %d",zero_H);
  printf("zero_L : %d",zero_L);

  sprintf(zerostr,"%c%x",zero_L,zero_H);
  printf("zero is %s\n",zerostr);
  uint8_t sop = 0x55;
  x = msg->linear.x*1000;
  z = msg->angular.z*1000;
  //  z = z*1000;
  //  x = x*1000;
    std::cout << "linear x " << (int)x;
    std::cout << "angular z " << (int)z;
   // z = (int16_t)z*100;
   // x = (int16_t)x*500;

   // ROS_INFO_STREAM("linear x*100= " << 10934);
   // ROS_INFO_STREAM("angular z*100= " << z);

    tx_x_H = x >>8;
    tx_x_L = x & 0xff;
    tx_z_H = z >>8;
    tx_z_L = z & 0xff;

    checksum = sop+sop+tx_x_H+tx_x_L+tx_z_H+tx_z_L;

    tx_checksum_H = checksum >> 8;
    tx_checksum_L = checksum & 0xff;

    std::cout << "qWtx_x_H " << tx_x_H << std::endl;
    std::cout << "tx_x_L "<< tx_x_L << std::endl;
    std::cout << "tx_z_H "<< tx_z_H << std::endl;
    std::cout << "tx_z_L "<< tx_z_L << std::endl;
    std::cout << "tx_check_H "<< tx_checksum_H << std::endl;
    std::cout << "tx_ehck_L " << tx_checksum_L << std::endl;
//    std::cout << "tx_twist = " << tx_twist << std::endl;

   // sprintf(tx_twist, "%c%c%c%c%c%c%c%c\n",sop,sop,tx_x_H,tx_x_L,tx_z_H,tx_z_L,tx_checksum_H,tx_checksum_L);
    tx_twist[0] = sop;
    tx_twist[1] = sop;
    tx_twist[2] = tx_x_H;
    tx_twist[3] = tx_x_L;
    tx_twist[4] = tx_z_H;
    tx_twist[5] = tx_z_L;
    tx_twist[6] = tx_checksum_H;
    tx_twist[7] = tx_checksum_L;
    tx_twist[8] = '\n';
    tx_twist[9] = '\0';

    //  sprintf(s2, "%lf", );
    std::cout << "tx_twist =  " << tx_twist <<std::endl;
    for(int j  = 0; j<10;j++) {
        tx_twist2[j]=tx_twist[j];
    }
    ser.write(tx_twist2,sizeof(int8_t)*10);
    ser.write(tx_twist2,sizeof(int8_t)*10);
    ser.write(tx_twist2,sizeof(int8_t)*10);
    ser.write(tx_twist2,sizeof(int8_t)*10);

    //    ser.write(tx_twist);
  //  ser.write(tx_twist[0]);

}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "stm_serial");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("write", 100, write_callback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 100);
    ros::Publisher odom_pub = nh.advertise<geometry_msgs::Pose2D>("odom",100);
    ros::Subscriber goal_vel = nh.subscribe("cmd_vel",100,cmd_vel_callback);

    int parsing_i;
    geometry_msgs::Pose2D odom_data;
    uint8_t parsing[12];
    int16_t rx_odom_x;
    int16_t rx_odom_y;
    int16_t rx_odom_yaw;
    int16_t rx_checksum;

    std::string port;
    //int baudrate;
    nh.param("stm_serial/Port",port,std::string("/dev/ttyUSB0"));
    cout << port <<std::endl;
//    if(!g_serial.Open(const_cast<char*>(port.c_str()), baudrate)){
//        cout << "device is not opened! " << endl;
//    }

/*
    uint8_t rx_x_L;
    uint8_t rx_x_H;
    uint8_t rx_y_L;
    uint8_t rx_y_H;
    uint8_t rx_yaw_L;
    uint8_t rx_yaw_H;
    uint8_t rx_checksum_L;
    uint8_t rx_checksum_H;*/

    try
    {
        ser.setPort(port);
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(100);
        ser.setTimeout(to);
        ser.open();


    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen())
    {
        ROS_INFO_STREAM("Serial Port initialized");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(10);

    while(ros::ok())
    {

        ros::spinOnce();

        if(ser.available())
        {
            ROS_INFO_STREAM("Reading from serial port");
            std_msgs::String result;
            result.data = ser.read(ser.available());

            while (!(parsing_i ==11)) {
                parsing[parsing_i]=result.data[parsing_i];
                parsing_i++;
            }
            parsing_i=0;
            rx_checksum = (parsing[8] << 8) | parsing[9] & 0xff;
            std::cout << "checksum = %c\n" << rx_checksum << std::endl;
            //printf("checksum is a  %x\n",rx_checksum);
            std::cout << (parsing[0] + parsing[1] + parsing[2] + parsing[3] + parsing[4] + parsing[5]);
            if (rx_checksum ==(int16_t) parsing[0] + parsing[1] + parsing[2] + parsing[3] + parsing[4] + parsing[5]+parsing[6]+parsing[7]) {
                     std::cout << "그것은 오른쪽\n"<< std::endl;
                     rx_odom_x = (parsing[2] << 8) | parsing[3] & 0xff;
                     rx_odom_y = (parsing[4] << 8) | parsing[5] & 0xff;
                     rx_odom_yaw = (parsing[6] << 8) | parsing[7] & 0xff;
                     std::cout << "rx_odom_x is : %d\n"<< rx_odom_x << std::endl;
                     std::cout << "rx_odom_y is : %d\n" << rx_odom_y << std::endl;
                     std::cout << "rx_odom_yaw is : %d\n" <<rx_odom_yaw<< std::endl;
                     odom_data.x = (double) rx_odom_x/1000;
                     odom_data.y = (double) rx_odom_y/1000;
                     odom_data.theta = (double) rx_odom_yaw/1000;
                     odom_pub.publish(odom_data);
            }
            else {
                ROS_INFO("error : ");
              //  ROS_INFO("rx_odom_x is : %d\n",rx_odom_x);
             //   printf("rx_odom_y is : %d\n", rx_odom_y);
             //   printf("rx_odom_yaw is : %d\n", rx_odom_yaw);
            }
            //ROS_INFO_STREAM("Read: " << result.data);
            read_pub.publish(result);

        }
        loop_rate.sleep();
    }
}
