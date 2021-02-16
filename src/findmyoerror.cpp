#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <ros_myo/MyoPose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <imu_teleop/imu_teleop.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ros/console.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "findmyoerror");
    ros::NodeHandle nh;

    double st=0.01;
    ros::WallRate lp(1.0/st);

    ros_helper::SubscriptionNotifier<geometry_msgs::TwistStamped> imu_sub(nh,"/myo_raw/acc_twist_global",1);
    if (!imu_sub.waitForANewData(ros::Duration(10)))
    {
      ROS_ERROR("No topic received");
      return 0;
    }

    Eigen::Vector3d acc_in_g;
    Eigen::Vector3d error;
    acc_in_g.setZero();
    error.setZero();
    int i=0;

    ros::Time t0=ros::Time::now();
    while ((ros::Time::now()-t0).toSec()<15)
    {
        if(imu_sub.isANewDataAvailable())
        {
          geometry_msgs::TwistStamped imu=imu_sub.getData();
          acc_in_g(0)=imu.twist.linear.x;
          acc_in_g(1)=imu.twist.linear.y;
          acc_in_g(2)=imu.twist.linear.z;
          i+=1;

          error+=acc_in_g/i;
        }
    }

    ROS_INFO_STREAM("error");
    ROS_INFO_STREAM(error);

}
