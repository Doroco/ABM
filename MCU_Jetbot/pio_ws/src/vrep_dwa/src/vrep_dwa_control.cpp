#include "vrep_dwa/vrep_dwa_control.hpp"

DWA_Planner::DWA_Planner(void)
  :scan_updated(false), position_updated(false), goal_updated(false),twist_updated(false)
{
  dt = 1.0/SAMPlE_HZ;
  mode = 1;
  current_velocity.linear.x = 0.0;
  current_velocity.angular.z = 0.0;
  twist_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",10);
  Scan_sub = nh.subscribe<sensor_msgs::LaserScan>("scan",10,&DWA_Planner::scan_callback,this);
  goal_pose_sub = nh.subscribe<geometry_msgs::Point>("/set_goal",10,&DWA_Planner::goal_callback,this);
  Current_Pose_sub = nh.subscribe<geometry_msgs::Pose2D>("/Odom_update",10,&DWA_Planner::cur_pos_callback,this);
}

DWA_Planner::State::State(double _x, double _y, double _theata, double _v,double _w)
  :x(_x) ,y(_y), theata(_theata),v(_v),w(_w)
{

}

DWA_Planner::Window::Window(void)
 :v_min(0.0), v_max(0.0), w_min(0.0),w_max(0.0)
{

}

DWA_Planner::Window::Window(const double _v_min,const double _v_max,const double _w_min,const double _w_max)
 :v_min(_v_min), v_max(_v_max),w_min(_w_min),w_max(_w_max)
{

}

void DWA_Planner::scan_callback(const sensor_msgs::LaserScanConstPtr& msg)
{
  scan = *msg;
  scan_updated = true;
}

void DWA_Planner::goal_callback(const geometry_msgs::PointConstPtr &msg)
{
  goal = *msg;
  goal_area.x() = goal.x;
  goal_area.y() = goal.y;
  goal_updated = true;
}

void DWA_Planner::cur_pos_callback(const geometry_msgs::Pose2DConstPtr &msg)
{
  Current_Pose = *msg;
  Cur_pose.x() = Current_Pose.x;
  Cur_pose.y() = Current_Pose.y;
  Cur_pose.z() = Current_Pose.theta;

  cur_state.x = Current_Pose.x;
  cur_state.y = Current_Pose.y;
  cur_state.theata = Current_Pose.theta;

  position_updated = true;
}

DWA_Planner::Window DWA_Planner::calc_dynamic_window(const geometry_msgs::Twist &current_velocity)
{
  Window window(MIN_VELOCITY,MAX_VELOCITY,MIN_OMEGA,MAX_OMEGA);
  window.v_min = std::max(current_velocity.linear.x-MAX_VEL_ACCELATION*dt,MIN_VELOCITY);
  window.v_max = std::min(current_velocity.linear.x+MAX_VEL_ACCELATION*dt,MAX_VELOCITY);
  window.w_min = std::max(current_velocity.angular.z-MAX_OMEGA_ACCELATION*dt,-MAX_OMEGA);
  window.w_max = std::min(current_velocity.angular.z+MAX_OMEGA_ACCELATION*dt,MAX_OMEGA);
  return window;
}

void DWA_Planner::motion(State &state, const double v, const double w)
{
  state.theata += w*dt;
  state.x += v* std::cos(state.theata)*dt;
  state.y += v* std::sin(state.theata)*dt;
  state.v = v;
  state.w = w;
}

std::vector<Eigen::Vector2f> DWA_Planner::scan_to_obs()
{
  std::vector<Eigen::Vector2f> obs_list;
  float angle = scan.angle_min;
  for(auto r : scan.ranges){
    float x =  r *std::cos(angle);
    float y =  r *std::sin(angle);
    Eigen::Vector2f Obs_state ={x, y};
    obs_list.emplace_back(Obs_state);
    angle += scan.angle_increment;
  }
  return obs_list;
}

float DWA_Planner::calc_to_goal_cost(const std::vector<State> &traj, Eigen::Vector2f goalpos)
{
  //float traj_mag = sqrtf(powf(cur_state.x+traj.back().x,2)+powf(cur_state.y+traj.back().y,2));
  //float goal_mag = sqrtf(powf(goal.x,2)+powf(goal.y,2));
  //float dot_product = (  (cur_state.x+traj.back().x )*goal.x )+( (cur_state.y+traj.back().y)*goal.y );
  //float dist = sqrtf(powf(cur_state.x+traj.back().x-goal.x,2)+powf(cur_state.y+traj.back().y-goal.y,2));
  //ROS_INFO("traj_mag : %f",traj_mag);
  //ROS_INFO("goal_mag : %f",goal_mag);
  //ROS_INFO("dot_product : %f",dot_product);
  //ROS_INFO("result : %f",safe_acos(dot_product/goal_mag*traj_mag));
  //ROS_INFO("ang : %f",dist+abs((float)safe_acos(dot_product/goal_mag*traj_mag)));
  //float theata = (float)acos(dot_product/goal_mag*traj_mag);
  float goal_theata = atan2f(goalpos.y()-Cur_pose.y(),goalpos.x()-Cur_pose.x());
  float traj_theata = traj.back().theata;
  float head_error = goal_theata - traj_theata;

  if(head_error< 0)
  {
    head_error = -head_error;
  }
  std::cout << head_error << std::endl;
  float cost_head = HEADING_COST_GAIN_zero*(1.0/(cost_head + HEADING_COST_GAIN_pole));
  /*
  if(goal_theata < 0)
  {
    goal_theata += 2*PI;
  }

  if( goal_theata <= deg2rad(90))
  {
    if( odm >= PI || odm < goal_theata)
    {
      odm = 2*PI+goal_theata-odm;
      heading = goal_theata-odm;
    }
    else {
      heading = odm-goal_theata;
    }
  }
  else {
    if(){
    }
    else {

    }

  }
  */

  //float theata = rad2deg(cur_state.theata+traj.back().theata);
  //double target_theata = 0;
  //if(goal_theata < theata)
  //    target_theata = goal_theata-theata;
  //else {
  //    target_theata = theata-goal_theata;
 // }
  //target_theata = 180- target_theata;
  //target_theata = deg2rad(target_theata);


  return  cost_head;//DIST_COST_GAIN*(dist);//abs((float)acosf(dot_product/traj_mag*goal_mag));// + HEADING_COST_GAIN*(target_theata);//abs((float)acosf(dot_product/traj_mag*goal_mag));//*dist;
}

float DWA_Planner::calc_speed_cost(const std::vector<State> &traj, const float target_velocity)
{
  float cost = fabsf(target_velocity-traj.back().v);
  return cost;
}

float DWA_Planner::calc_obstacle_cost(const std::vector<State> &traj, const std::vector<Eigen::Vector2f> &Obs_list)
{
  float cost = 0.0f;
  float min_dist = 1e3f;
  for(const auto& state : traj)
  {
    for(const auto& obs : Obs_list)
    {
      float dist = sqrt(powf(state.x - obs.x(),2)+powf(state.y - obs.y(),2));
      if(dist < MAX_DIST){
        cost = 1e6f;
        return cost;
      }
      min_dist = std::min(min_dist,dist);
    }
  }
  cost = 1.0/min_dist;
  return cost;
}

std::vector<DWA_Planner::State> DWA_Planner::Find_BestTrajectory(Window window,Eigen::Vector2f goal,std::vector<Eigen::Vector2f> obs_list)
{
  float min_cost = 1e6;
  float goal_cost;
  float speed_cost;
  float obstacle_cost;
  std::vector<State> best_traj;

  for(float v = window.v_min ; v <= window.v_max ; v += VELOCITY_RESOLUTION)
  {
    for(float w = window.w_min; w <= window.w_max ; w += YAWRATE_RESOLUTION)
    {
      //position Drived from LaserScan Data
      State state(0,0,0,current_velocity.linear.x,current_velocity.angular.z);
      std::vector<State> traj;
      for(float t =0; t <= PREDICT_TIME ; t += dt)
      {
        motion(state,v,w);
        traj.emplace_back(state);
        t += dt;
      }

      goal_cost = calc_to_goal_cost(traj,goal);
      speed_cost = calc_speed_cost(traj,MAX_VELOCITY);
      obstacle_cost = calc_obstacle_cost(traj, obs_list);

      float cost = goal_cost+SPEED_COST_GAIN*speed_cost+OBSTACLE_COST_GAIN*obstacle_cost;

      if(min_cost > cost)
      {
        min_cost = cost;
        best_traj = traj;
        best_velocity.linear.x = v;
        best_velocity.angular.z = w;
      }
    }
    std::cout << goal_cost << ',' << speed_cost << ',' <<obstacle_cost << std::endl;
  }
  float dist = sqrt(powf(goal.x()-cur_state.x,2)+powf(goal.y()-cur_state.y,2));
  if(min_cost == 1e6 || dist < 0.4){
      std::vector<State> traj;
      State state(Cur_pose.x(),Cur_pose.y(),Cur_pose.z(), 0.0, 0.0);
      traj.push_back(state);
      best_traj = traj;
      if(mode == 1)
      {
        best_velocity.linear.x = 0;
        best_velocity.angular.z = ;
      }
      else {
        best_velocity.linear.x = 0;
        best_velocity.angular.z = 0;
      }
  }


  return best_traj;
}

void DWA_Planner::process()
{
  ros::Rate loop_rate(SAMPlE_HZ);
  State preState = State(0.0,0.0,0.0,0.0,0.0);
  while(ros::ok()){
    if(scan_updated && goal_updated && position_updated)
    {
      double start = ros::Time::now().toSec();
      double dx = cur_state.x-preState.x;
      double dy = cur_state.y-preState.y;
      double instanueous_vel = sqrt(powf(dx,2)+powf(dy,2));
      double instnaueous_w = (cur_state.theata - preState.theata);

      //ROS_INFO("vel : %f",instanueous_vel);
      //ROS_INFO("w : %f",instnaueous_w);

      double dter = atan2(dy,dx);

      if( abs(dter -cur_state.theata) < deg2rad(80))
      {
        current_velocity.linear.x = instanueous_vel;
      }
      else {
        current_velocity.linear.x = -instanueous_vel;
      }
      current_velocity.angular.z = instnaueous_w;

      Window dynamic_window = calc_dynamic_window(current_velocity);
      //ROS_INFO(" min_Vel : %f",dynamic_window.v_min);
      //ROS_INFO(" max_Vel : %f",dynamic_window.v_max);
      //ROS_INFO(" min_w : %f",dynamic_window.w_min);
      //ROS_INFO(" max_w : %f",dynamic_window.w_max);
      std::vector<Eigen::Vector2f> obs_list;

      obs_list = scan_to_obs();

      std::vector<State> best_traj =Find_BestTrajectory(dynamic_window,goal_area,obs_list);

      geometry_msgs::Twist robot_vel;
      robot_vel.linear.x = best_velocity.linear.x;
      robot_vel.angular.z = best_velocity.angular.z;

      //ROS_INFO("Published Vel : %f",robot_vel.linear.x);
      //ROS_INFO("Published w : %f",robot_vel.angular.z);
      twist_pub.publish(robot_vel);
      cur_state.v = best_velocity.linear.x;
      cur_state.w = best_velocity.angular.z;

      preState = cur_state;

      position_updated = false;
      twist_updated = false;
      scan_updated = false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

float DWA_Planner::safe_acos(float value)
{
    if (value<=-1.0) {
        return PI;
    } else if (value>=1.0) {
        return 0;
    } else {
        return acos(value);
    }
}
