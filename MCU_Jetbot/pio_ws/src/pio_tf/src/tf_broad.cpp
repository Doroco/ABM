#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <sensor_msgs/Imu.h>
#include <ros_tutorial_msgs/msgData.h>

double odom_x;
double odom_y;
double odom_theta;
double qua_w;
double qua_x;
double qua_y;
double qua_z;


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
void pointCallback(const geometry_msgs::Pose2D::ConstPtr& point){
       odom_x = point->x;
       odom_y = point->y;
       odom_theta = point->theta;
        ROS_INFO("point rev");
        static tf::TransformBroadcaster broadcaster;
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(
                    tf::Quaternion(qua_x,qua_y,qua_z,qua_w),
                    tf::Vector3(odom_x,odom_y,odom_theta)),
                    ros::Time::now(),
                    "odom",
                    "base_footprint"));
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(
                    tf::Quaternion(0,0,0,1),
                    tf::Vector3(0.0, 0.0, 0.075)),
                    ros::Time::now(),
                    "base_footprint",
                    "base_link"));
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(
                    tf::Quaternion(0,0,0,1),
                    tf::Vector3(0.0, 0.0,0.16)),
                    ros::Time::now(),
                    "base_link",
                    "base_scan"));

}
void quaternionCallback(const sensor_msgs::Imu::ConstPtr& data){
    qua_w = data->orientation.w;
    qua_x = data->orientation.x;
    qua_y = data->orientation.y;
    qua_z = data->orientation.z;

    ROS_INFO("quater sent");

}
void linkCallback(const std_msgs::Float32::ConstPtr& msg){
    static tf::TransformBroadcaster broadcaster;

   // ROS_INFO("base frame sent");
}

void scanCallback(const std_msgs::Float32::ConstPtr& msg){

    //ROS_INFO("scan frame sent");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pio_tf");
    ros::NodeHandle nh;
    int cnt;

    ros::Subscriber odom_sub = nh.subscribe<geometry_msgs::Pose2D>("/odom", 10, pointCallback);
    ros::Subscriber quater_sub = nh.subscribe<sensor_msgs::Imu>("/imu_data", 10, quaternionCallback);


    tf::TransformBroadcaster broadcaster;

    ros::spin();

    ros::Rate loop_rate(10);
    while (ros::ok())
    {

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}
