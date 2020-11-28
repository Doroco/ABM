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
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include "ros_tutorial_msgs/msgData.h"

//#include "vrep_franka_tutorial/moveitcontroller.h"

#define E_OMEAG 0.05
#define E_VEL   0.2
#define DAMPING_CONSTRAINTS 0.03
#define TOTAL_DOF 3 // Open-chain Serial Robot
#define SIM_DT    0.01 // 10ms
#define PI 3.14159265359
#define deg2rad(deg)  ((deg) * PI / 180.0)
#define rad2deg(rad)  ((rad) * 180.0 / PI)
#define HZ 10
double ctrl_hz = 200;
int mode = 0;

const std::string JOINT_NAME[TOTAL_DOF] = {"joint1","joint2","joint3"};


    class controller_interface
    {

    public:
      controller_interface();
      ~controller_interface();


    /**
    * @brief joint value callback function
    * @detail get joint value from vrep simulator
    * @param msg : sensor_msgs::JointState
    */
    void joint_cb(const sensor_msgs::JointStateConstPtr& msg);
    /**
    * @brief simulation callback function
    * @detail get simulation status from vrep simulator(0:sim not running, 1: sim paused, 2:sim stopped, 3:sim running)
    * @param msg : std_msgs::Int32
    */

    /**
    * @brief This is our joint control framework for panda manipulator
    * @param read_vrep : Read sensor from V-REP. Do spinonce to update callback function.
    * @param compute : function for compute algorithm. Update Target joint value from current joint value to desired joint value(cubic spline).
    * @param write_vrep : Changing our joint value type to V-REP form. Usually Triggering is done by this function.
    * @param wait : For syncmode... this function is nessesary to sync controller and V-REP simulation environment.
    */
    //void set_desired_q(std::vector<float> dq);
    void set_desired_T(std::vector<float> P);

    //Modern Robotics Algorithm

    Eigen::MatrixXd Normalize(Eigen::MatrixXd V);

    // x --> [x] busket Notation
    Eigen::Matrix3d VecToso3(const Eigen::Vector3d& omg);

    // [x] --> x  busket Notation to unit Omega
    Eigen::Vector3d so3ToVec(const Eigen::MatrixXd& so3mat);

    //Calc V -->[adV]
    Eigen::MatrixXd ad(Eigen::VectorXd V);

    //[w]*theata --> [x y z] theata
    Eigen::Vector4d AxisAng3(const Eigen::Vector3d& expc3);

    // Combine R and P to T
    Eigen::MatrixXd RpToTrans(const Eigen::Matrix3d& R, const Eigen::Vector3d& p);

    // Decompose T to R and p
    std::vector<Eigen::MatrixXd> TransToRp(const Eigen::MatrixXd& T);

    // V --> [V]    [w]  v
    //               0   0
    Eigen::MatrixXd VecTose3(const Eigen::VectorXd& V);

    //[V] --> V    [ w1 w2 w3 v1 v2 v3]
    Eigen::VectorXd se3ToVec(const Eigen::MatrixXd& T);

    //T --> adT   R     0
    //            [p]R  R
    Eigen::MatrixXd Adjoint(const Eigen::MatrixXd& T);

    //T --> inv(T)
    Eigen::MatrixXd TransInv(const Eigen::MatrixXd& transform);

    //R to trans(R)
    Eigen::MatrixXd RotInv(const Eigen::MatrixXd& rotMatrix);

    //Lodriguess Formulation
    Eigen::Matrix3d MatrixExp3(const Eigen::Matrix3d& so3mat);

    //Screw Axies to T  e[s]*theata
    Eigen::MatrixXd MatrixExp6(const Eigen::MatrixXd& se3mat);

    // Matrix Logarithm
    Eigen::Matrix3d MatrixLog3(const Eigen::Matrix3d& R);

    // T -> [s]  [w] v
    //            0  0
    Eigen::MatrixXd MatrixLog6(const Eigen::MatrixXd& T);

    //Solve FK in Space Frame
    Eigen::MatrixXd FKinSpace(const Eigen::MatrixXd& M, const Eigen::MatrixXd& Slist, const Eigen::VectorXd& thetaList);

    //Find Jacobian throw Screw Axies in Space Frame
    Eigen::MatrixXd JacobianSpace(const Eigen::MatrixXd& Slist, const Eigen::MatrixXd& thetaList);

    void IKsolver();

    /**
    * @brief controller to vrep simulator publisher message lists
    * @param vrep_joint_set_pub_(sensor_msgs::JointState) : controller joint value --> vrep simulator robot joint value
    * @param vrep_sim_start_pub_(std_msgs::Bool) : vrep simulation start command
    * @param vrep_sim_stop_pub_(std_msgs::Bool) : vrep simulation stop command
    * @param vrep_sim_step_trigger_pub_(std_msgs::Bool) : vrep simulation update one step command
    * @param vrep_sim_enable_syncmode_pub_(std_msgs::Bool) : vrep simulation enable syncmode command
    */
    public:
//    ros::Publisher joint_set_pub_;
    //ros::Publisher vrep_sim_start_pub_;
    //ros::Publisher vrep_sim_stop_pub_;
    //ros::Publisher vrep_sim_step_trigger_pub_;
    //ros::Publisher vrep_sim_enable_syncmode_pub_;

    /**
    * @brief controller to vrep simulator subscriber message lists
    * @param vrep_joint_state_sub_(sensor_msgs::JointState) : get vrep simulator robot current joint value
    * @param vrep_sim_step_done_sub_(std_msgs::Bool) : get simulation trigger step done signal
    * @param vrep_sim_time_sub_(std_msgs::Float32) : get vrep simulator time tick
    * @param vrep_sim_status_sub_(std_msgs::Int32) : get vrep simulation status(0:sim not running, 1: sim paused, 2:sim stopped, 3:sim running)
    */
    //ros::Subscriber vrep_sim_step_done_sub_;
    //ros::Subscriber vrep_sim_time_sub_;
    //ros::Subscriber vrep_sim_status_sub_;

    /**
    * @brief Other parameters for using ros interface and control    
    */


//    bool sim_step_done_;
//    float sim_time_; // from v-rep simulation time
//    int tick;

    //Robot Parameter
    sensor_msgs::JointState joint_cmd_;

    //Screw Axies
    Eigen::MatrixXd Slist;
    Eigen::Matrix4d M;

    //Rec Information
    Eigen::Matrix4d desired_T;
    Eigen::VectorXd q_;
    Eigen::VectorXd q_pre;

    float Kp;
    float damping_constraints;



    bool is_first_run;
    };


#endif
