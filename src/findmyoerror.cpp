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
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Core>
#include <tf_conversions/tf_eigen.h>



int main(int argc, char **argv)
{
    ros::init(argc, argv, "findmyoerror");
    ros::NodeHandle nh;

    double st=0.01;
    ros::WallRate lp(1.0/st);
    tf::TransformListener listener;

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
    ROS_INFO_STREAM("error norm");
    ROS_INFO_STREAM(error.norm());
    //system("rosparam dump ~/.ros/err_param.yaml /myo_error");

    tf::StampedTransform transform;
    for (int itrial=0;itrial<10;itrial++)
    {
        try{
           listener.lookupTransform("/imu_global", "/myo_raw",
                                    ros::Time(0), transform);
           ROS_INFO("ok");
           break;
         }
         catch (tf::TransformException ex){
           ROS_ERROR("%s",ex.what());
           ros::Duration(1.0).sleep();
         }
    }
     Eigen::Affine3d T_gs;

     tf::transformTFToEigen(transform,T_gs);

     Eigen::Vector3d z_g(0,0,1);

     ROS_INFO_STREAM("Zs=" <<T_gs.linear().col(2));
     Eigen::Vector3d Zs=T_gs.linear().col(2);
     Eigen::Vector3d Ys=T_gs.linear().col(1);
     double cos_theta=Zs.dot(z_g);
     double cos_gamma=Ys.dot(z_g);
     double theta=acos(cos_theta);
     double gamma=acos(cos_gamma);
     double media=0.0593;
     double amplitude=0.4531;

     ROS_INFO("gamma=");
     ROS_INFO_STREAM(gamma);
     if(gamma>(3.1416/2.0))
     {
       theta=-theta;
     }
     ROS_INFO("theta=");
     ROS_INFO_STREAM(theta);
    double offset=media-amplitude*sin(theta+(3.1416*0.5)-0.25);
    ROS_INFO("offset=");
    ROS_INFO_STREAM(offset);
}


