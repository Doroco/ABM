#include "vrep_dwa/vrep_dwa_control.hpp"

int main(int argc, char** argv)
{
  ros::init(argc,argv,"vrep_dwa_main");
  DWA_Planner sim_dwa;
  sim_dwa.process();
  return 0;
}
