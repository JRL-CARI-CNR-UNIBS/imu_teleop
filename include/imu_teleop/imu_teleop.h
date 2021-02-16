#pragma once

#include <ros/ros.h>
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

namespace imu_teleop {

enum State {NONE, TEACH, EXECUTION, VEL_CONTROL, DIRECTION_CONTROL, EXECUTE_DIRECTION};

class ImuConfig
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuConfig();

    Eigen::Vector3d acc_in_g_; // g global frame of imu
    Eigen::Vector3d acc_in_g_filt_;
    Eigen::Vector3d acc_in_g_satured_;
    Eigen::Vector3d acc_in_b_satured_; //b base frame of robot
    Eigen::Vector3d acc_in_b_satured_old_; //b base frame of robot
    Eigen::Vector3d vel_in_b_;
    Eigen::Vector3d vel_in_b_old_;
    Eigen::Vector3d position_;
    Eigen::Vector3d versor_;


    void velocityConfiguration(double a, double noise,Eigen::Affine3d T_b_g, double vel_max,double coeff, double st);
    void filter(double a);
    void saturation(double noise, Eigen::Affine3d T_b_g);
    void safetyLimits(double max);
    void integralVelocity(double coeff, double st);
    void integralPosition(double st);
    Eigen::Vector3d getVersor();
    void vectorReset();
    void generateVersor();
};

inline void ImuConfig::filter(double a)
{
 acc_in_g_filt_=a*acc_in_g_filt_+(1-a)*acc_in_g_;
}

inline void ImuConfig::saturation(double noise,Eigen::Affine3d T_b_g)
{
    for (int idx=0;idx<3;idx++)
    {
      if ((std::abs(acc_in_g_filt_(idx))<noise) && (std::abs(acc_in_g_(idx))<noise))
        acc_in_g_satured_(idx)=0;
      else
        acc_in_g_satured_(idx)=acc_in_g_filt_(idx);
    }
    acc_in_b_satured_=T_b_g*acc_in_g_satured_;
}

inline void ImuConfig::safetyLimits(double vel_max)
{
    for (int idx=0;idx<3;idx++)
    {
      if (vel_in_b_(idx)>vel_max)
        vel_in_b_(idx)=vel_max;
      else if (vel_in_b_(idx)<-vel_max)
        vel_in_b_(idx)=-vel_max;
    }
}

inline void ImuConfig::vectorReset()
{
    acc_in_g_.setZero();
    acc_in_g_filt_.setZero();
    acc_in_g_satured_.setZero();
    acc_in_b_satured_.setZero();
    acc_in_b_satured_old_.setZero();
    vel_in_b_.setZero();
    vel_in_b_old_.setZero();
    position_.setZero();
    versor_.setZero();
}

inline void ImuConfig::integralVelocity(double coeff, double st)
{
   vel_in_b_=vel_in_b_+(coeff*st*0.5*(acc_in_b_satured_+acc_in_b_satured_old_));
}

inline void ImuConfig::integralPosition(double st)
{
   position_=position_+(st*0.5*(vel_in_b_+vel_in_b_old_));
}

inline void ImuConfig::generateVersor()
{
  double norma=sqrt((position_(0)*position_(0))+(position_(1)*position_(1))+(position_(2)*position_(2)));
  versor_=position_/norma;
}

inline void ImuConfig::velocityConfiguration(double a, double noise,Eigen::Affine3d T_b_g, double vel_max,double coeff, double st)
{
    filter(a);
    saturation(noise,T_b_g);
    integralVelocity(coeff, st);
    safetyLimits(vel_max);
    integralPosition(st);
    generateVersor();
    acc_in_b_satured_old_=acc_in_b_satured_;
    vel_in_b_old_=vel_in_b_;
}

inline Eigen::Vector3d ImuConfig::getVersor()
{
    return versor_;
}

inline ImuConfig::ImuConfig()
{
 vectorReset();
}

}



