/*
Copyright (c) 2020, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*#include <ros/ros.h>
#include <configuration_msgs/StartConfiguration.h>
#include <configuration_msgs/StopConfiguration.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <subscription_notifier/subscription_notifier.h>
#include <ros_myo/MyoPose.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_teleop_node");
  ros::NodeHandle nh;

  double st=0.01;
  ros::WallRate lp(1.0/st);



  ros::ServiceClient configuration_client=nh.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
  ros::ServiceClient stop_configuration_client=nh.serviceClient<configuration_msgs::StopConfiguration>("/configuration_manager/stop_configuration");

  configuration_msgs::StartConfiguration srv_start;
  srv_start.request.start_configuration="cart_teleop";
  srv_start.request.strictness=1;

  configuration_msgs::StopConfiguration srv_stop;
  srv_stop.request.strictness=1;

  ros_helper::SubscriptionNotifier<sensor_msgs::JointState> js_sub(nh,"/manipulator/joint_states",1);
  if (!js_sub.waitForANewData(ros::Duration(10)))
  {
    ROS_ERROR("No topic received");
    return 0;
  }

  ros_helper::SubscriptionNotifier<geometry_msgs::TwistStamped> imu_sub(nh,"/myo_raw/acc_twist_global",1);
  if (!imu_sub.waitForANewData(ros::Duration(10)))
  {
    ROS_ERROR("No topic received");
    return 0;
  }
  ros_helper::SubscriptionNotifier<ros_myo::MyoPose> myo_pose_sub(nh,"/myo_raw/myo_gest",1);
  if (!myo_pose_sub.waitForANewData(ros::Duration(60)))
  {
    ROS_ERROR("No topic received");
    return 0;
  }
  
  ros_helper::SubscriptionNotifier<std_msgs::Bool> bool_sub(nh,"/moveit_planning/active",1);
  if (!bool_sub.waitForANewData(ros::Duration(10)))
  {
    ROS_ERROR("No topic received");
    return 0;
  }

  ros::Publisher jteleop_pub=nh.advertise<sensor_msgs::JointState>("/planner_hw/joint_teleop/target_joint_teleop",1);
  ros::Publisher cteleop_pub=nh.advertise<geometry_msgs::TwistStamped>("/planner_hw/cart_teleop/target_cart_teleop",1);
  ros::Publisher vibration_pub=nh.advertise<std_msgs::UInt8>("/myo_raw/vibrate",1);

  double tau=10*st;
  double coeff=0.5;
  double vel_max=0.1;
  double noise=0.2;
  double vel_min=-0.1;
  double a=std::exp(-st/tau);
  double rotz_angle=0;


  if (!nh.getParam("rotz_angle",rotz_angle))
  {
    ROS_WARN("rotz_angle not set, using 0");
    rotz_angle=0;
  }

  if (!nh.getParam("scale_coef",coeff))
  {
    ROS_WARN("scale_coef not set, using 0.01");
    coeff=0.01;
  }
  Eigen::Vector3d acc_in_g; // g global frame of imu
  Eigen::Vector3d acc_in_g_filt;
  Eigen::Vector3d acc_in_g_satured;
  Eigen::Vector3d acc_in_b_satured; //b base frame of robot
  Eigen::Vector3d acc_in_b_satured_old; //b base frame of robot
  Eigen::Vector3d vel_in_b;
  Eigen::Vector3d vel_in_b_old;
  Eigen::Vector3d pos_in_b;

  acc_in_g.setZero();
  acc_in_g_filt.setZero();
  acc_in_g_satured.setZero();
  acc_in_b_satured.setZero();
  acc_in_b_satured_old.setZero();
  vel_in_b.setZero();
  vel_in_b_old.setZero();
  pos_in_b.setZero();

  Eigen::Affine3d T_b_g;
  T_b_g.setIdentity();
  Eigen::AngleAxisd rotz(rotz_angle,Eigen::Vector3d::UnitZ());
  T_b_g=rotz;

  bool do_disable=false;
  bool disabled=false;
  bool first_cicle=true;
  std_msgs::UInt8 msgs_vibr;

  while (ros::ok())
  {
    ros::spinOnce();
    ros_myo::MyoPose gest=myo_pose_sub.getData();
    std_msgs::Bool msgs_bool=bool_sub.getData();
    bool activation=msgs_bool.data;

    if (gest.pose==ros_myo::MyoPose::FIST && !activation)
    {
     if (first_cicle)
     {
       //stop_configuration_client.call(srv_stop);
       configuration_client.call(srv_start);
       ros::Duration(2).sleep();

       msgs_vibr.data=2;
       vibration_pub.publish(msgs_vibr);
     }

      do_disable=false;
      disabled=false;
      first_cicle=false;
      ROS_INFO_THROTTLE(1,"actived node vel");
      geometry_msgs::TwistStamped imu=imu_sub.getData();
      acc_in_g(0)=imu.twist.linear.x;
      acc_in_g(1)=imu.twist.linear.y;
      acc_in_g(2)=imu.twist.linear.z;
      acc_in_g_filt=a*acc_in_g_filt+(1-a)*acc_in_g;
      for (int idx=0;idx<3;idx++)
      {
        if ((std::abs(acc_in_g_filt(idx))<noise) && (std::abs(acc_in_g(idx))<noise))
          acc_in_g_satured(idx)=0;
        else
          acc_in_g_satured(idx)=acc_in_g_filt(idx);
      }

      acc_in_b_satured=T_b_g*acc_in_g_satured;
      vel_in_b=vel_in_b+coeff*st*0.5*(acc_in_b_satured+acc_in_b_satured_old);
      for (int idx=0;idx<3;idx++)
      {
        if (vel_in_b(idx)>vel_max)
          vel_in_b(idx)=vel_max;
        else if (vel_in_b(idx)<-vel_max)
          vel_in_b(idx)=-vel_max;
      }
      pos_in_b=pos_in_b+coeff*st*0.5*(vel_in_b+vel_in_b_old);

      acc_in_b_satured_old=acc_in_b_satured;
      vel_in_b_old=vel_in_b;
    }
    else
    {
      ROS_INFO_THROTTLE(1,"disable node vel");
      acc_in_g.setZero();
      acc_in_g_filt.setZero();
      acc_in_g_satured.setZero();
      acc_in_b_satured.setZero();
      acc_in_b_satured_old.setZero();
      vel_in_b.setZero();
      do_disable=true;
      first_cicle=true;
    }

    if (!disabled)
    {
      geometry_msgs::TwistStamped twist;
      twist.header.frame_id="BASE";
      twist.header.stamp=ros::Time::now();
      twist.twist.linear.x=vel_in_b(0);
      twist.twist.linear.y=vel_in_b(1);
      twist.twist.linear.z=0;//vel_in_b(2);
      cteleop_pub.publish(twist);
    }
    if (do_disable)
      disabled=true;

    lp.sleep();
  }
  return 0;
}*/


