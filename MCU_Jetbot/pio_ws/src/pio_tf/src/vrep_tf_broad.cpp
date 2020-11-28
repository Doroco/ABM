#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

ros::Publisher vrep_sim_start_pub;
ros::Publisher vrep_sim_stop_pub;
ros::Publisher vrep_sim_step_trigger_pub;
ros::Publisher vrep_sim_enable_syncmode_pub;
sensor_msgs::LaserScan printlaser;
nav_msgs::Odometry odom;
double qua_x;
double qua_y;
double qua_z;
double qua_w;
double pos_x;
double pos_y;
double pos_z;
uint32_t pre_time;
uint32_t now_time;
/*
void poseCallback(const nav_msgs::Odometry::ConstPtr& odomsg ){
   //TF odom => base_link
    static tf::TransformBroadcaster odom_broadcaster;
    odom_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(odomsg->pose.pose.orientation.x,
                                     odomsg->pose.pose.orientation.y,
                                     odomsg->pose.pose.orientation.z,
                                     odomsg->pose.pose.orientation.w),
        tf::Vector3(odomsg->pose.pose.position.x,
                    odomsg->pose.pose.position.y,
                    odomsg->pose.pose.position.z)),
        ros::Time::now(), "odom", "base_footprint"));
      ROS_DEBUG("odometry frame sent");
      ROS_INFO("odometry frame sent");
}
void poseCallback(const geometry_msgs::Pose::ConstPtr& pose){
   //TF odom => base_link
    static tf::TransformBroadcaster odom_broadcaster;
    odom_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(pose->orientation.x,
                                     pose->orientation.y,
                                     pose->orientation.z,
                                     pose->orientation.w),
        tf::Vector3(pose->position.x,
                    pose->position.y,
                    pose->position.z)),
        ros::Time::now(), "odom", "base_footprint"));
      ROS_DEBUG("odometry frame sent");
      ROS_INFO("odometry frame sent");
}
*/
void pointCallback(const geometry_msgs::Point::ConstPtr& point){
       pos_x = point->x;
       pos_y = point->y;
       pos_z = point->z;
    //    ROS_INFO("point rev");
}
void quaternionCallback(const geometry_msgs::Quaternion::ConstPtr& quater){
    qua_x = quater->x;
    qua_y = quater->y;
    qua_z = quater->z;
    qua_w = quater->w;
   // ROS_INFO("quater sent");

}
void linkCallback(const std_msgs::Float32::ConstPtr& msg){
    static tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::Quaternion(0,0,0,1),
                tf::Vector3(0.0, 0.0, msg->data)),
                ros::Time::now(),
                "base_footprint",
                "base_link"));
   // ROS_INFO("base frame sent");
}

void scanCallback(const std_msgs::Float32::ConstPtr& msg){
    static tf::TransformBroadcaster broadcaster;
    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::Quaternion(0,0,0,1),
                tf::Vector3(0.0, 0.0, msg->data)),
                ros::Time::now(),
                "base_link",
                "base_scan"));
    broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(
                tf::Quaternion(qua_x,qua_y,qua_z,qua_w),
                tf::Vector3(pos_x,pos_y,pos_z)),
                ros::Time::now(),
                "odom_combined",
                "base_footprint"));
    //ROS_INFO("scan frame sent");

}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser){
    static ros::NodeHandle n;
    static ros::Publisher laserpub = n.advertise<sensor_msgs::LaserScan>("scan",10);
    now_time = ros::Time::now().sec;
    printlaser.scan_time= now_time - pre_time;
    pre_time = now_time;
    printlaser.header.frame_id = laser->header.frame_id;
    printlaser.angle_min = laser->angle_min;
    printlaser.angle_max = laser->angle_max;
    printlaser.angle_increment = laser->angle_increment;
    printlaser.header.stamp = ros::Time::now();
    printlaser.time_increment =laser->time_increment;
    printlaser.range_min= laser->range_min;
    printlaser.range_max= laser->range_max;
    printlaser.ranges=laser->ranges;
    static ros::Publisher _odom = n.advertise<nav_msgs::Odometry>("odom",10);
    odom.header.frame_id="wheelodom";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.position.z = pos_z;
    odom.pose.pose.orientation.w = qua_w;
    odom.pose.pose.orientation.x = qua_x;
    odom.pose.pose.orientation.y = qua_y;
    odom.pose.pose.orientation.z = qua_z;
    _odom.publish(odom);
    laserpub.publish(printlaser);


}

void vrepStart()
{
  ROS_INFO("Starting V-REP Simulation");
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_start_pub.publish(msg);
}
void vrepStop()
{
  ROS_INFO("Stopping V-REP Simulation");
  std_msgs::Bool msg;
  msg.data = true;
  vrep_sim_stop_pub.publish(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pio_vrep_tf");
    ros::NodeHandle nh;
    int cnt;
    ros::Subscriber link_sub = nh.subscribe<std_msgs::Float32>("/base2link",10, linkCallback);
    ros::Subscriber scan_sub = nh.subscribe<std_msgs::Float32>("/link2scan",10, scanCallback);

    ros::Subscriber point_sub = nh.subscribe<geometry_msgs::Point>("/point", 10, pointCallback);
    ros::Subscriber quater_sub = nh.subscribe<geometry_msgs::Quaternion>("/quaternion", 10, quaternionCallback);

    ros::Subscriber laser_sub = nh.subscribe<sensor_msgs::LaserScan>("/fakescan",5, laserCallback);

    vrep_sim_start_pub = nh.advertise<std_msgs::Bool>("/startSimulation", 5);
    vrep_sim_stop_pub = nh.advertise<std_msgs::Bool>("/stopSimulation", 5);
    vrep_sim_step_trigger_pub = nh.advertise<std_msgs::Bool>("/triggerNextStep", 100);
    vrep_sim_enable_syncmode_pub = nh.advertise<std_msgs::Bool>("/enableSyncMode", 5);

    tf::TransformBroadcaster broadcaster;

    ros::spin();

    ros::Rate loop_rate(10);
    vrepStart();
    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}
