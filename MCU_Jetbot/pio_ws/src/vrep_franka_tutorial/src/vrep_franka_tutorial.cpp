#include "vrep_franka_tutorial/franka_controller.hpp"

controller_interface obj;

void end_pose_callback(const geometry_msgs::Point::ConstPtr& pose ) {
    std::vector<float> Point;
    Point.push_back(pose->x);
    Point.push_back(pose->y);
    Point.push_back(pose->z);
    //std::cout << "x  :"<<Point.at(0) << std::endl;
    //std::cout << Point.at(1) << std::endl;
    //std::cout << Point.at(2) << std::endl;
    obj.set_desired_T(Point);
    mode = 1;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vrep_franka_tutorial");
  ros::NodeHandle nh;

  ros::Subscriber end_pose_sub = nh.subscribe<geometry_msgs::Point>("end_point",10,end_pose_callback);
  ros::Publisher joint_pub = nh.advertise<geometry_msgs::Point>("joint_topic",100);


  //float j_val[3];
  ros_tutorial_msgs::msgData j_angle;
  geometry_msgs::Point angle;
  angle.x = 0;
  angle.y = 0;
  angle.z = 0;

//  nh.getParam("j1",j_val[0]);
//  nh.getParam("j2",j_val[1]);
//  nh.getParam("j3",j_val[2]);

//  std::vector<float> desired_q;
//  for(size_t i=0;i<TOTAL_DOF;i++)
//  {
//    desired_q.push_back(j_val[i]);
//    std::cout << j_val[i];
//  }
//    std::cout << std::endl;


//    ros::spin();
    ros::Rate loop_rate(HZ);
  while(ros::ok())
  {
      if(mode == 1) {
          std::cout << "mode = 1"<< std::endl;
          obj.IKsolver();
        //  int length = obj.q_.size();
        //  for (int i = 0; i < length ; i++) {
        //        j_angle.angle[i]=obj.q_[i];
        //  }
          angle.x = obj.q_[0];
          angle.y = obj.q_[1];
          angle.z = obj.q_[2];

          joint_pub.publish(angle);

      }

      ros::spinOnce();
      loop_rate.sleep();

  }

  return 0;

}
