#include "vrep_franka_tutorial/franka_controller.hpp"


controller_interface::controller_interface()
{
    is_first_run = true;
//    tick = 0;
//    sim_step_done_ = false;
//    sim_time_ = 0.0f;

    q_.resize(TOTAL_DOF);
    q_.setZero();
    q_[0] = 0;
    q_[1] = 0;
    q_[2] = deg2rad(150);
    q_pre.resize(TOTAL_DOF);
    q_pre.setZero();

    joint_cmd_.name.resize(TOTAL_DOF);
    joint_cmd_.position.resize(TOTAL_DOF);

    for(size_t i=0; i<TOTAL_DOF; i++)
    {
       joint_cmd_.name[i]= JOINT_NAME[i];
    }

    //Init M Matrix
    M= Eigen::MatrixXd::Identity(4,4);
    M <<  1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1, 99,
          0, 0, 0, 1;

    Slist.resize(6,3);
    Slist.setZero();
    Slist <<  -1,  -1,  -1,
              0,  0,  0,
              0,  0,  0,
              0,  0,  0,
             -23, -44, -69,
              0,  0,  0;

    desired_T= Eigen::MatrixXd::Identity(4,4);

    Kp = 1;
    damping_constraints = 0.03;
    //vrep_sim_start_pub_ = nh_.advertise<std_msgs::Bool>("/startSimulation", 5);
   // vrep_sim_stop_pub_ = nh_.advertise<std_msgs::Bool>("/stopSimulation", 5);
   // vrep_sim_step_trigger_pub_ = nh_.advertise<std_msgs::Bool>("/triggerNextStep", 100);
   // vrep_sim_enable_syncmode_pub_ = nh_.advertise<std_msgs::Bool>("/enableSyncMode", 5);
    //joint_set_pub_ = nh_.advertise<sensor_msgs::JointState>("/panda/joint_set", 10);

   // joint_state_sub_ = nh_.subscribe("/panda/joint_states", 100, &controller_interface::joint_cb, this);
    //vrep_sim_step_done_sub_ = nh_.subscribe("/simulationStepDone", 100, &controller_interface::sim_step_done_cb, this);
    //vrep_sim_time_sub_ = nh_.subscribe("/simulationTime",100,&controller_interface::sim_time_cb,this);
    //vrep_sim_status_sub_ = nh_.subscribe("/simulationState",100,&controller_interface::sim_status_cb,this);
  }
  controller_interface::~controller_interface()
  {       
  }


  //Modern Robotics Algorithm///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Eigen::MatrixXd controller_interface::Normalize(Eigen::MatrixXd V)
  {
    V.normalize();
    return V;
  }

  Eigen::Matrix3d controller_interface::VecToso3(const Eigen::Vector3d &omg)
  {
    Eigen::Matrix3d m_ret;
    m_ret << 0, -omg(2), omg(1),
      omg(2), 0, -omg(0),
      -omg(1), omg(0), 0;
    return m_ret;
  }

  Eigen::Vector3d controller_interface::so3ToVec(const Eigen::MatrixXd &so3mat)
  {
    Eigen::Vector3d v_ret;
    v_ret << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
    return v_ret;
  }

  //
  Eigen::MatrixXd controller_interface::ad(Eigen::VectorXd V)
  {
    Eigen::Matrix3d omgmat = VecToso3(Eigen::Vector3d(V(0), V(1), V(2)));

    Eigen::MatrixXd result(6, 6);
    result.topLeftCorner<3, 3>() = omgmat;
    result.topRightCorner<3, 3>() = Eigen::Matrix3d::Zero(3, 3);
    result.bottomLeftCorner<3, 3>() = VecToso3(Eigen::Vector3d(V(3), V(4), V(5)));
    result.bottomRightCorner<3, 3>() = omgmat;
    return result;
  }

  Eigen::Vector4d  controller_interface::AxisAng3(const Eigen::Vector3d &expc3)
  {
    Eigen::Vector4d v_ret;
    v_ret << Normalize(expc3), expc3.norm();
    return v_ret;
  }

  Eigen::MatrixXd controller_interface::RpToTrans(const Eigen::Matrix3d &R, const Eigen::Vector3d &p)
  {
    Eigen::MatrixXd m_ret(4, 4);
    m_ret << R, p,
      0, 0, 0, 1;
    return m_ret;
  }

  std::vector<Eigen::MatrixXd> controller_interface::TransToRp(const Eigen::MatrixXd &T)
  {
    std::vector<Eigen::MatrixXd> Rp_ret;
    Eigen::Matrix3d R_ret;
    // Get top left 3x3 corner
    R_ret = T.block<3, 3>(0, 0);

    Eigen::Vector3d p_ret(T(0, 3), T(1, 3), T(2, 3));

    Rp_ret.push_back(R_ret);
    Rp_ret.push_back(p_ret);

    return Rp_ret;
  }

  Eigen::MatrixXd controller_interface::VecTose3(const Eigen::VectorXd &V)
  {
    // Separate angular (exponential representation) and linear velocities
    Eigen::Vector3d exp(V(0), V(1), V(2));
    Eigen::Vector3d linear(V(3), V(4), V(5));

    // Fill in values to the appropriate parts of the transformation matrix
    Eigen::MatrixXd m_ret(4, 4);
    m_ret << VecToso3(exp), linear,
      0, 0, 0, 0;

    return m_ret;
  }

  Eigen::VectorXd  controller_interface::se3ToVec(const Eigen::MatrixXd &T)
  {
    Eigen::VectorXd m_ret(6);
    m_ret << T(2, 1), T(0, 2), T(1, 0), T(0, 3), T(1, 3), T(2, 3);

    return m_ret;
  }

  Eigen::MatrixXd controller_interface::Adjoint(const Eigen::MatrixXd &T)
  {
    std::vector<Eigen::MatrixXd> R = TransToRp(T);
    Eigen::MatrixXd ad_ret(6, 6);
    ad_ret = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd zeroes = Eigen::MatrixXd::Zero(3, 3);
    ad_ret << R[0], zeroes,
      VecToso3(R[1]) * R[0], R[0];
    return ad_ret;
  }

  Eigen::MatrixXd controller_interface::TransInv(const Eigen::MatrixXd &transform)
  {
    auto rp = TransToRp(transform);
    auto Rt = rp.at(0).transpose();
    auto t = -(Rt * rp.at(1));
    Eigen::MatrixXd inv(4, 4);
    inv = Eigen::MatrixXd::Zero(4,4);
    inv.block(0, 0, 3, 3) = Rt;
    inv.block(0, 3, 3, 1) = t;
    inv(3, 3) = 1;
    return inv;
  }

  Eigen::MatrixXd controller_interface::RotInv(const Eigen::MatrixXd &rotMatrix)
  {
    return rotMatrix.transpose();
  }


  Eigen::Matrix3d controller_interface::MatrixExp3(const Eigen::Matrix3d &so3mat)
  {
    Eigen::Vector3d omgtheta = so3ToVec(so3mat);

    Eigen::Matrix3d m_ret = Eigen::Matrix3d::Identity();
    if (so3mat.norm() < 0.00001) {
      return m_ret;
    }
    else {
      double theta = (AxisAng3(omgtheta))(3);
      Eigen::Matrix3d omgmat = so3mat * (1 / theta);
      return m_ret + std::sin(theta) * omgmat + ((1 - std::cos(theta)) * (omgmat * omgmat));
    }
  }

  Eigen::MatrixXd controller_interface::MatrixExp6(const Eigen::MatrixXd &se3mat)
  {
    Eigen::Matrix3d se3mat_cut = se3mat.block<3, 3>(0, 0);
    Eigen::Vector3d omgtheta = so3ToVec(se3mat_cut);

    Eigen::MatrixXd m_ret(4, 4);

    // If negligible rotation, m_Ret = [[Identity, angular velocty ]]
    //									[	0	 ,		1		   ]]
    if ((omgtheta.norm() < 0.00001)) {
      // Reuse previous variables that have our required size
      se3mat_cut = Eigen::MatrixXd::Identity(3, 3);
      omgtheta << se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
      m_ret << se3mat_cut, omgtheta,
        0, 0, 0, 1;
      return m_ret;
    }
    // If not negligible, MR page 105
    else {
      double theta = (AxisAng3(omgtheta))(3);
      Eigen::Matrix3d omgmat = se3mat.block<3, 3>(0, 0) / theta;
      Eigen::Matrix3d expExpand = Eigen::MatrixXd::Identity(3, 3) * theta + (1 - std::cos(theta)) * omgmat + ((theta - std::sin(theta)) * (omgmat * omgmat));
      Eigen::Vector3d linear(se3mat(0, 3), se3mat(1, 3), se3mat(2, 3));
      Eigen::Vector3d GThetaV = (expExpand*linear) / theta;
      m_ret << MatrixExp3(se3mat_cut), GThetaV,
        0, 0, 0, 1;
      return m_ret;
    }
  }


  Eigen::Matrix3d  controller_interface::MatrixLog3(const Eigen::Matrix3d &R)
  {
    double acosinput = (R.trace() - 1) / 2.0;
    Eigen::MatrixXd m_ret = Eigen::MatrixXd::Zero(3, 3);
    if (acosinput >= 1)
      return m_ret;
    else if (acosinput <= -1) {
      Eigen::Vector3d omg;
      if (!(1 + R(2, 2) <0.00001))
        omg = (1.0 / std::sqrt(2 * (1 + R(2, 2))))*Eigen::Vector3d(R(0, 2), R(1, 2), 1 + R(2, 2));
      else if (!(1 + R(1, 1) <0.0001))
        omg = (1.0 / std::sqrt(2 * (1 + R(1, 1))))*Eigen::Vector3d(R(0, 1), 1 + R(1, 1), R(2, 1));
      else
        omg = (1.0 / std::sqrt(2 * (1 + R(0, 0))))*Eigen::Vector3d(1 + R(0, 0), R(1, 0), R(2, 0));
      m_ret = VecToso3(M_PI * omg);
      return m_ret;
    }
    else {
      double theta = std::acos(acosinput);
      m_ret = theta / 2.0 / sin(theta)*(R - R.transpose());
      return m_ret;
    }
  }

  Eigen::MatrixXd controller_interface::MatrixLog6(const Eigen::MatrixXd &T)
  {
    Eigen::MatrixXd m_ret(4, 4);
    auto rp = TransToRp(T);
    Eigen::Matrix3d omgmat = MatrixLog3(rp.at(0));
    Eigen::Matrix3d zeros3d = Eigen::Matrix3d::Zero(3, 3);
    if ((omgmat.norm()<.00001)) {
      m_ret << zeros3d, rp.at(1),
        0, 0, 0, 0;
    }
    else {
      double theta = std::acos((rp.at(0).trace() - 1) / 2.0);
      Eigen::Matrix3d logExpand1 = Eigen::MatrixXd::Identity(3, 3) - omgmat / 2.0;
      Eigen::Matrix3d logExpand2 = (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2)*omgmat*omgmat / theta;
      Eigen::Matrix3d logExpand = logExpand1 + logExpand2;
      m_ret << omgmat, logExpand*rp.at(1),
        0, 0, 0, 0;
    }
    return m_ret;
  }

  Eigen::MatrixXd controller_interface::FKinSpace(const Eigen::MatrixXd &M, const Eigen::MatrixXd &Slist, const Eigen::VectorXd &thetaList)
  {
    Eigen::MatrixXd T = M;
    for (int i = (thetaList.size() - 1); i > -1; i--) {
      T = MatrixExp6(VecTose3(Slist.col(i)*thetaList(i))) * T;
    }
    return T;
  }

  Eigen::MatrixXd controller_interface::JacobianSpace(const Eigen::MatrixXd &Slist, const Eigen::MatrixXd &thetaList)
  {
    Eigen::MatrixXd Js = Slist;
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(4, 4);
    Eigen::VectorXd sListTemp(Slist.col(0).size());
    for (int i = 1; i < thetaList.size(); i++) {
      sListTemp << Slist.col(i - 1) * thetaList(i - 1);
      T = T * MatrixExp6(VecTose3(sListTemp));
      // std::cout << "array: " << sListTemp << std::endl;
      Js.col(i) = Adjoint(T) * Slist.col(i);
    }

    return Js;
  }

  void controller_interface::IKsolver()
  {
    Eigen::MatrixXd Tfk = FKinSpace(M, Slist, q_);

    std::cout<<"T:\n"<<Tfk<<std::endl;

    // Body Frame to Disired T
    Eigen::MatrixXd Tdiff = TransInv(Tfk)*desired_T;

    //Twist Transform Body to Space  MatrixLog6--> T to [V]  to se3ToVec V to Adjoint(transform Basepoint)
    Eigen::VectorXd Vs = Adjoint(Tfk)*se3ToVec(MatrixLog6(Tdiff));
    Eigen::Vector3d angular(Vs(0), Vs(1), Vs(2));
    Eigen::Vector3d linear(Vs(3), Vs(4), Vs(5));

    //Allowable Error about Angluar Vel and Linear Vel
    bool err = (angular.norm() < E_OMEAG || linear.norm() < E_VEL);

    std::cout<<"E-omga :"<<angular.norm()<<std::endl;
    std::cout<<"E-vel  :"<<linear.norm()<<std::endl;

    if(err)
    {
       return;
    }
    else {
      Eigen::MatrixXd Js;
      Eigen::MatrixXd Js_Sudo;

      Js = JacobianSpace(Slist, q_);


      Js_Sudo = Js.transpose()*Js;

      float ddamping_constraints = damping_constraints +linear.norm()*16.2+angular.norm()*7;

      Eigen::MatrixXd damping = Eigen::MatrixXd::Identity(Js_Sudo.rows(),Js_Sudo.cols())*(ddamping_constraints*ddamping_constraints);

      // In 3-Dimension N 6DOF Our System is 3DOF (cuz is open-chain Serial Robot)
      // So J is tall --> pinv(J) -- > ( trans(J)*J )*trans(J)
      q_ +=  ((Js_Sudo+damping).inverse() * Js.transpose()*Vs )*(HZ);
      for(int i ; i < q_.size(); i++)
      {
        if(q_[i] < -150 ){
            q_[i] = -150;
            std::cout << "saturation1" <<std::endl;
        }
        if(q_[i] > 0) {
            q_[i] = 0;
            std::cout << "saturation2" <<std::endl;
        }
      }
      //q_ +=  Js.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Vs);//*HZ;
      q_pre = q_;
      }
  }

  //Modern Robotics Algorithm///////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  void controller_interface::set_desired_T(std::vector<float> P)
  {
    if( P.size() == 3)
    {//cosf(deg2rad(160))
      desired_T <<  1.0, 0,0,P[0],
                    0, cosf(deg2rad(147)), sinf(deg2rad(147)), P[1],
                    0, -sinf(deg2rad(147)), cosf(deg2rad(147)), -0.5,
                    0,0, 0, 1;
      std::cout<<desired_T<<std::endl;
    }
    else
    {
      desired_T << 1,0,0,0,
                   0,1,0,0,
                   0,0,1,0,
                   0,0,0,1;
    }
  }

  void controller_interface::joint_cb(const sensor_msgs::JointStateConstPtr& msg)
  {

    if(msg->name.size() == TOTAL_DOF)
    {
      for(size_t i=0; i< msg->name.size(); i++)
      {
        q_[i] = msg->position[i];
     //   IKsolver(q_);
        //q_target[i] = msg->velocity[i];
      }
    }
    else
    {
      ROS_INFO("Controller's total Dof and JointStates from VREP is not same size!!");
    }
  }

