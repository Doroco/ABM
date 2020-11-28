/**
* @brief Franka robot controller in V-REP
* @detail Get joint angle
* @author JiminLee
* @date 2018-07-27
* @version 0.0.1
*/
#ifndef SIMULATION_INTERFACE_H
#define SIMULATION_INTERFACE_H

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "ros_tutorial_msgs/msgData.h"


#include <math.h>
#include <vector>

//#include "vrep_franka_tutorial/moveitcontroller.h"

//-------------------Parameters------------------------------------------------
#define PI 3.14159265359
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((rad) * 180.0 / PI)

#define MIN_OMEGA deg2rad(-15)// rad/s
#define MAX_OMEGA deg2rad(15) // rad/s

#define MIN_VELOCITY -0.05//s // m/s
#define MAX_VELOCITY 0.18 // m/s

#define MAX_VEL_ACCELATION 0.04 //m/s^2
#define MAX_OMEGA_ACCELATION deg2rad(15)  //m/s^2

#define MAX_DIST 0.6

#define VELOCITY_RESOLUTION 0.005
#define YAWRATE_RESOLUTION deg2rad(0.1)
#define SAMPlE_HZ    10// Hz
#define PREDICT_TIME 1.0

//--------------Gain---------------------------------------
#define HEADING_COST_GAIN_pole 0.008
#define HEADING_COST_GAIN_zero 0.83
#define DIST_COST_GAIN 1.50
#define SPEED_COST_GAIN 0.5
#define OBSTACLE_COST_GAIN 1.3

class DWA_Planner
{
public:
    DWA_Planner();

    class State
    {
    public:
        State(double _x, double _y, double _theata, double _v,double _w);

        double x;
        double y;
        double theata;
        double v;
        double w;
    private:
    };

    class Window
    {
    public:
        Window();
        Window(const double _v_min,const double _v_max,const double _w_min,const double _w_max);
        double v_min;
        double v_max;
        double w_min;
        double w_max;
    private:
    };

    //Call Back Method
    void scan_callback(const sensor_msgs::LaserScanConstPtr& msg);
    void goal_callback(const geometry_msgs::PointConstPtr& msg);
    void cur_pos_callback(const geometry_msgs::Pose2DConstPtr& msg);

    void process(void);
    void set(int Mode){ this->Mode = Mode;}
    int get(int Mode){ return this->Mode;}
    //Map Imforamtion
    std::vector<Eigen::Vector2f> scan_to_obs();

    //Navigation Methods
    DWA_Planner::Window calc_dynamic_window(const geometry_msgs::Twist& current_velocity);
    void motion(State& state, const double v, const double w);
    std::vector<State> Find_BestTrajectory(Window window,Eigen::Vector2f goal,std::vector<Eigen::Vector2f> obs_list);

    //Obj Function
    float safe_acos(float value);
    float calc_to_goal_cost(const std::vector<State>& traj, Eigen::Vector2f goal);
    float calc_speed_cost(const std::vector<State>& traj, const float target_velocity);
    float calc_obstacle_cost(const std::vector<State>& traj, const std::vector<Eigen::Vector2f>& Obsist);

    void modecallback (const ros_tutorial_msgs::msgData::ConstPtr& data);
private:
    ros::NodeHandle nh;

    ros::Publisher twist_pub;

    ros::Subscriber Scan_sub;
    ros::Subscriber goal_pose_sub;
    ros::Subscriber Current_Pose_sub;
    ros::Subscriber mode_sub;

    geometry_msgs::Twist current_velocity;
    geometry_msgs::Twist best_velocity;
    geometry_msgs::Pose2D Current_Pose;
    sensor_msgs::LaserScan scan;
    geometry_msgs::Point goal;

    Eigen::Vector3d Cur_pose;
    Eigen::Vector2f goal_area;

    State cur_state = State(0.0,0.0,0.0,0.0,0.0);

    bool scan_updated;
    bool position_updated;
    bool goal_updated;
    bool twist_updated;
    uint8_t  Mode;

    double dt;
};

#endif
