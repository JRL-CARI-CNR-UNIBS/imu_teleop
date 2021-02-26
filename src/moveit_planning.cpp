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

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <configuration_msgs/StartConfiguration.h>
#include <configuration_msgs/StopConfiguration.h>
#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <ros_myo/MyoPose.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float64MultiArray.h>
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

void change_config(std::string new_config, ros::ServiceClient configuration_client_channel)
{
    configuration_msgs::StartConfiguration srv;
    srv.request.start_configuration=new_config;
    srv.request.strictness=1;
    configuration_client_channel.call(srv);
    ros::Duration(2).sleep();
}

double findMyoError(tf::TransformListener& listener)
{
  ros::Duration(0.01).sleep();
  double media=0.0593;
  double amplitude=0.4531;
  tf::StampedTransform transform;
  Eigen::Affine3d T_gs;
  Eigen::Vector3d z_g(0,0,1);

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

   tf::transformTFToEigen(transform,T_gs);

   Eigen::Vector3d Zs=T_gs.linear().col(2);
   Eigen::Vector3d Ys=T_gs.linear().col(1);
   double cos_theta=Zs.dot(z_g);
   double cos_gamma=Ys.dot(z_g);
   double theta=acos(cos_theta);
   double gamma=acos(cos_gamma);

   if(gamma>(3.1416/2.0))
   {
     theta=-theta;
   }
   double offset=media-amplitude*sin(theta+(3.1416*0.5)-0.25);
   return offset;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  //("rosparam dump ~/.ros/err_param.yaml /myo_error");

  double st=0.01;
  ros::WallRate lp(1.0/st);

  ros::ServiceClient configuration_client=nh.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
  ros::ServiceClient stop_configuration_client=nh.serviceClient<configuration_msgs::StopConfiguration>("/configuration_manager/stop_configuration");

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

  ros::Publisher activation_pub=nh.advertise<std_msgs::Bool>("/moveit_planning/active",1);
  ros::Publisher vibration_pub=nh.advertise<std_msgs::UInt8>("/myo_raw/vibrate",1);
  ros::Publisher jteleop_pub=nh.advertise<sensor_msgs::JointState>("/planner_hw/joint_teleop/target_joint_teleop",1);
  ros::Publisher cteleop_pub=nh.advertise<geometry_msgs::TwistStamped>("/planner_hw/cart_teleop/target_cart_teleop",1);
  ros::Publisher control_vel=nh.advertise<geometry_msgs::TwistStamped>("velocity",1);
  ros::Publisher control_pos=nh.advertise<geometry_msgs::PoseStamped>("position",1);

  std::string realFake_configuration;
  if (!nh.getParam("teaching_configuration",realFake_configuration))
  {
    ROS_INFO("Configuration is not defined");
  }

  std::string group_name;
  if (!nh.getParam("group_name",group_name))
  {
    ROS_INFO("Group name is not defined, using 'manipulator' as default value");
    group_name="manipulator";
  }

  moveit::planning_interface::MoveGroupInterface group(group_name);
  if (!group.startStateMonitor(10))
  {
    ROS_ERROR("Unable to get the current state of the move group %s",group_name.c_str());
    return 0;
  }
  group.setStartState(*group.getCurrentState());

  moveit::core::RobotState trj_state = *group.getCurrentState();
  std::vector<double> current_joint_configuration;
  trj_state.copyJointGroupPositions(group_name,current_joint_configuration);

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(group_name);
  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  //enum State {NONE, TEACH, EXECUTION, VEL_CONTROL, DIRECTION_CONTROL, EXECUTE_DIRECTION};

  tf::TransformListener listener;

  std::vector<std::vector<double>> waypoints;
  //int state=NONE;
  imu_teleop::State state=imu_teleop::State::NONE;
  int gest_prec=0;
  bool active=false;
  int lenght=waypoints.size();
  std_msgs::UInt8 msgs_vibr;
  std_msgs::Bool msgs_bool;
  geometry_msgs::TwistStamped vel;
  geometry_msgs::PoseStamped pos;

  Eigen::Vector3d final_pos;
  Eigen::Vector3d versor;
  double movement_lenght=0.1;
  imu_teleop::ImuConfig imuData;
  bool execute_end=false;
  bool direction_taken=false;

  double tau;
  !nh.getParam("tau",tau);
  double coeff;
  !nh.getParam("coeff",coeff);
  double vel_max;
  !nh.getParam("vel_max",vel_max);
  double noise;
  !nh.getParam("noise",noise);
  double rotz_angle;
  !nh.getParam("rotz_angle",rotz_angle);
  double vel_min=-vel_max;
  double a=std::exp(-st/tau);
  //double rotz_angle=0.0;
  final_pos.setZero();
  versor.setZero();

  Eigen::Affine3d T_b_g;
  T_b_g.setIdentity();
  Eigen::AngleAxisd rotz(rotz_angle,Eigen::Vector3d::UnitZ());
  T_b_g=rotz;
  ros::Time t0=ros::Time::now();
  ros::Time t0_direction=ros::Time::now();

  while (ros::ok())
  {
      ros::spinOnce();
      lp.sleep();
      ros_myo::MyoPose gest=myo_pose_sub.getData();

      switch (state) { //switch degli stati
      case imu_teleop::State::TEACH:{
          if (gest.pose==3 && gest_prec!=3)
          {
            moveit::core::RobotState trj_state = *group.getCurrentState();
            std::vector<double> current_joint_configuration;
            trj_state.copyJointGroupPositions(group_name,current_joint_configuration);

            waypoints.push_back(current_joint_configuration); // così aggiungi le configurazioni che vuoi al vettore dei punti.
            lenght=waypoints.size();

            msgs_vibr.data=1;
            vibration_pub.publish(msgs_vibr);
          }
      } break;

      case imu_teleop::State::EXECUTION:{
          change_config("trj_tracker",configuration_client);

          for (unsigned int iw=0;iw<waypoints.size();iw++)
          {
            ROS_INFO("go to waypoint %u",iw);
            group.setStartState(*group.getCurrentState());
            group.setJointValueTarget(waypoints.at(iw));

            moveit::planning_interface::MoveGroupInterface::Plan plan;

            bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
            if (!success)
            {
              ROS_ERROR("Planning failed computing waypoint %u", iw);
              return 0;
            }
            group.execute(plan);
            ros::Duration(2).sleep();
           }

          active=false;
      } break;

      case imu_teleop::State::VEL_CONTROL:{
        geometry_msgs::TwistStamped imu=imu_sub.getData();
        double offsetZ=findMyoError(listener);
        imuData.acc_in_g_(0)=imu.twist.linear.x;
        imuData.acc_in_g_(1)=imu.twist.linear.y;
        imuData.acc_in_g_(2)=imu.twist.linear.z-offsetZ;

        ROS_INFO_STREAM("acc_in_g");
        ROS_INFO_STREAM(imuData.acc_in_g_);

        imuData.velocityConfiguration(a,noise,T_b_g,vel_max,coeff,st);
        ROS_INFO_STREAM("acc_in_g_filt");
        ROS_INFO_STREAM(imuData.acc_in_g_filt_);
        ROS_INFO_STREAM("acc_in_g_satured_");
        ROS_INFO_STREAM(imuData.acc_in_g_satured_);
        ROS_INFO_STREAM("aacc_in_b_satured_");
        ROS_INFO_STREAM(imuData.acc_in_b_satured_);
        ROS_INFO_STREAM("vel_in_b_");
        ROS_INFO_STREAM(imuData.vel_in_b_);

        geometry_msgs::TwistStamped twist;
        twist.header.frame_id="BASE";
        twist.header.stamp=ros::Time::now();
        twist.twist.linear.x=imuData.vel_in_b_(0);
        twist.twist.linear.y=imuData.vel_in_b_(1);
        twist.twist.linear.z=imuData.vel_in_b_(2);
        cteleop_pub.publish(twist);

      } break;

      case imu_teleop::State::DIRECTION_CONTROL:{
        if(gest.pose==2)
        {
          geometry_msgs::TwistStamped imu=imu_sub.getData();
          double offsetZ=findMyoError(listener);
          imuData.acc_in_g_(0)=imu.twist.linear.x;
          imuData.acc_in_g_(1)=imu.twist.linear.y;
          imuData.acc_in_g_(2)=imu.twist.linear.z-offsetZ;

          imuData.velocityConfiguration(a,noise,T_b_g,10000.0,coeff,st);
          vel.twist.linear.x=imuData.vel_in_b_(0);
          vel.twist.linear.y=imuData.vel_in_b_(1);
          vel.twist.linear.z=imuData.vel_in_b_(2);
          vel.header.stamp=ros::Time::now();
          vel.header.frame_id=imu.header.frame_id;
          pos.pose.position.x=imuData.position_(0);
          pos.pose.position.y=imuData.position_(1);
          pos.pose.position.z=imuData.position_(2);
          pos.header.stamp=ros::Time::now();
          pos.header.frame_id=imu.header.frame_id;
          control_vel.publish(vel);
          control_pos.publish(pos);
          direction_taken=true;
        }
      } break;

      case imu_teleop::State::EXECUTE_DIRECTION:{
        versor=imuData.getVersor();
        ROS_INFO_THROTTLE(1,"versor\n");
        ROS_INFO_STREAM(versor);
        ROS_INFO_THROTTLE(1,"position\n");
        ROS_INFO_STREAM(imuData.position_);

        moveit::core::RobotStatePtr kinematic_state=group.getCurrentState();
        std::vector<double> joint_values;

        kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
        ROS_INFO_THROTTLE(1,"actual joint");
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
          ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
        geometry_msgs::PoseStamped T_w_t=group.getCurrentPose();
        geometry_msgs::Pose gripperPose=T_w_t.pose;
        ROS_INFO_THROTTLE(1,"current pose\n");
        ROS_INFO_STREAM( T_w_t);

        gripperPose.position.x+=(versor(0)*movement_lenght);
        gripperPose.position.y+=(versor(1)*movement_lenght);
        gripperPose.position.z+=(versor(2)*movement_lenght);
        ROS_INFO_THROTTLE(1,"target pose\n");
        ROS_INFO_STREAM(gripperPose);

        double timeout = 0.1;
        bool found_ik = kinematic_state->setFromIK(joint_model_group, gripperPose, timeout);
        if (found_ik)
        {
          ROS_INFO_THROTTLE(1,"cinematica inversa");
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }

        group.setStartState(*group.getCurrentState());
        group.setJointValueTarget(*kinematic_state);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        ROS_INFO_THROTTLE(1,"plan");

        bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ros::Duration(2).sleep();
        if (!success)
        {
          ROS_ERROR("Planning failed computing waypoint");
          return 0;
        }
        ROS_INFO_THROTTLE(1,"group plan");
        group.execute(plan);
        ROS_INFO_THROTTLE(1,"execute plan");
        ros::Duration(2).sleep();

        execute_end=true;

        /*versor=imuData.getVersor();
        ROS_INFO_THROTTLE(1,"versor\n");
        ROS_INFO_STREAM(versor);
        geometry_msgs::PoseStamped T_w_t=group.getCurrentPose();
        ROS_INFO_THROTTLE(1,"current pose\n");
        ROS_INFO_STREAM( T_w_t);
        T_w_t.pose.position.x+=(versor(0)*movement_lenght);
        T_w_t.pose.position.y=T_w_t.pose.position.y+(versor(1)*movement_lenght);
        T_w_t.pose.position.z=T_w_t.pose.position.z+versor(2)*movement_lenght;
        ROS_INFO_THROTTLE(1,"target pose\n");
        ROS_INFO_STREAM(T_w_t);
        group.setStartState(*group.getCurrentState());
        group.setPoseTarget(T_w_t);
        ROS_INFO_THROTTLE(1,"target set");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        ROS_INFO_THROTTLE(1,"plan");

        bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ros::Duration(2).sleep();
        if (!success)
        {
          ROS_ERROR("Planning failed computing waypoint");
          return 0;
        }
        ROS_INFO_THROTTLE(1,"group plan");
        group.execute(plan);
        ROS_INFO_THROTTLE(1,"execute plan");
        ros::Duration(2).sleep();
        execute_end=true;*/
      } break;

      default: {
      } break;
      }

      switch (state) { //switch delle transizioni
      case imu_teleop::State::TEACH: {
          if(gest.pose==4)
          {
              if ((ros::Time::now()-t0).toSec()>5)
              {
               state=imu_teleop::State::EXECUTION;
               t0=ros::Time::now();

               ROS_INFO_THROTTLE(1,"deactive TEACH");

               msgs_vibr.data=2;
               vibration_pub.publish(msgs_vibr);
              }
          }
          else
          {
            t0=ros::Time::now();
          }

      } break;

      case imu_teleop::State::EXECUTION: {
          if (!active)
          {
            msgs_vibr.data=3;
            vibration_pub.publish(msgs_vibr);

            state=imu_teleop::State::NONE;
            ROS_INFO_THROTTLE(1,"deactive EXECUTION");
          }

      } break;

      case imu_teleop::State::NONE: {
          if(gest.pose==4)
          {
           if ((ros::Time::now()-t0).toSec()>5)
           {

            msgs_vibr.data=2;
            vibration_pub.publish(msgs_vibr);

            state=imu_teleop::State::TEACH;
            active=true;
            t0=ros::Time::now();
            change_config(realFake_configuration,configuration_client);

            ROS_INFO_THROTTLE(1,"actived TEACH");
           }
          }
          else
          {
            t0=ros::Time::now();
            //ROS_INFO_THROTTLE(1,"time reset");
          }

          if (gest.pose==5)
          {
              state=imu_teleop::State::EXECUTION;
              active=true;

              ROS_INFO_THROTTLE(1,"active EXECUTION");

              msgs_vibr.data=3;
              vibration_pub.publish(msgs_vibr);
          }

          if (gest.pose==2)
          {
            imuData.vectorReset();

            msgs_vibr.data=2;
            vibration_pub.publish(msgs_vibr);

            change_config("cart_teleop",configuration_client);
            state=imu_teleop::State::VEL_CONTROL;

            ROS_INFO_THROTTLE(1,"active VEL CONTROL");

          }

          if(gest.pose==3)
          {
           if ((ros::Time::now()-t0_direction).toSec()>5)
           {
            imuData.vectorReset();

            msgs_vibr.data=2;
            vibration_pub.publish(msgs_vibr);

            change_config("cart_teleop",configuration_client);
            state=imu_teleop::State::DIRECTION_CONTROL;

            ROS_INFO_THROTTLE(1,"actived DIRECTION CONTROL");

            t0_direction=ros::Time::now();
           }
          }
          else
          {
            t0_direction=ros::Time::now();
            //ROS_INFO_THROTTLE(1,"time reset");
          }
      } break;

      case imu_teleop::State::VEL_CONTROL: {
          if (gest.pose!=2)
            {
             imuData.vectorReset();
             geometry_msgs::TwistStamped twist;
             twist.header.frame_id="BASE";
             twist.header.stamp=ros::Time::now();
             twist.twist.linear.x=imuData.vel_in_b_(0);
             twist.twist.linear.y=imuData.vel_in_b_(1);
             twist.twist.linear.z=0;//imuData.vel_in_b_(2);
             cteleop_pub.publish(twist);

             state=imu_teleop::State::NONE;
             msgs_vibr.data=2;

             vibration_pub.publish(msgs_vibr);
             ROS_INFO_THROTTLE(1,"deactive VEL CONTROL");


            }
      } break;

      case imu_teleop::State::DIRECTION_CONTROL: {
          if(gest.pose==3)
          {
           if ((ros::Time::now()-t0_direction).toSec()>5)
           {
             msgs_vibr.data=2;
             vibration_pub.publish(msgs_vibr);
            state=imu_teleop::State::NONE;
            ROS_INFO_THROTTLE(1,"deactive DIRECTION CONTROL");

            imuData.vectorReset();
            t0_direction=ros::Time::now();
           }
          }
          else
          {
            t0_direction=ros::Time::now();
            //ROS_INFO_THROTTLE(1,"time reset");
          }

          if(gest.pose!=2 && direction_taken)
          {
            state=imu_teleop::State::EXECUTE_DIRECTION;
            msgs_vibr.data=3;
            vibration_pub.publish(msgs_vibr);
            change_config("trj_tracker",configuration_client);
            ROS_INFO_THROTTLE(1,"active EXECUTE DIRECTION");
            direction_taken=false;
          }
      } break;

      case imu_teleop::State::EXECUTE_DIRECTION: {
        if (execute_end)
        {
          imuData.vectorReset();
          msgs_vibr.data=3;

          state=imu_teleop::State::DIRECTION_CONTROL;
          ROS_INFO_THROTTLE(1,"deactive EXECUTE DIRECTION");
          execute_end=false;
        }
      } break;


      }

      msgs_bool.data=active;
      activation_pub.publish(msgs_bool);
      gest_prec=gest.pose;

  }

  }

  /* std::vector<std::vector<double>> waypoints;
   bool active=false;
   bool repetition=false;
   ros::Time t0=ros::Time::now();
   std_msgs::Bool msgs_bool;
   std_msgs::UInt8 msgs_vibr;
   int lenght=waypoints.size();
   int gest_prec=0;

    while (ros::ok())
    {
     ros::spinOnce();
     ros_myo::MyoPose gest=myo_pose_sub.getData();

     if (active && !repetition)
     {
         if (gest.pose==3 && gest_prec!=3)
         {
           moveit::core::RobotState trj_state = *group.getCurrentState();
           std::vector<double> current_joint_configuration;
           trj_state.copyJointGroupPositions(group_name,current_joint_configuration);

            waypoints.push_back(current_joint_configuration); // così aggiungi le configurazioni che vuoi al vettore dei punti.
            lenght=waypoints.size();

            msgs_vibr.data=1;
            vibration_pub.publish(msgs_vibr);
         }

         if (gest.pose==4)
         {
             if((ros::Time::now()-t0).toSec()>5)
              {
                 msgs_vibr.data=2;
                 vibration_pub.publish(msgs_vibr);

                 srv.request.start_configuration="trj_tracker";
                 configuration_client.call(srv);
                 ros::Duration(2).sleep();

                 active=false;
                 t0=ros::Time::now();
                 ROS_INFO_THROTTLE(1,"deactive node pose");
                 for (unsigned int iw=0;iw<waypoints.size();iw++)
                 {
                   ROS_INFO("go to waypoint %u",iw);
                   group.setStartState(*group.getCurrentState());
                   group.setJointValueTarget(waypoints.at(iw));


                   moveit::planning_interface::MoveGroupInterface::Plan plan;

                   bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
                   if (!success)
                   {
                     ROS_ERROR("Planning failed computing waypoint %u", iw);
                     return 0;
                   }
                   group.execute(plan);
                 }

                 msgs_vibr.data=2;
                 vibration_pub.publish(msgs_vibr);
              }
         }
         else
         {
             t0=ros::Time::now();
         }
     }
     else
     {
         if (repetition)
         {
             srv.request.start_configuration="trj_tracker";
             configuration_client.call(srv);
             ros::Duration(2).sleep();

             for (unsigned int iw=0;iw<waypoints.size();iw++)
             {
               ROS_INFO("go to waypoint %u",iw);
               group.setStartState(*group.getCurrentState());
               group.setJointValueTarget(waypoints.at(iw));


               moveit::planning_interface::MoveGroupInterface::Plan plan;

               bool success = (group.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
               if (!success)
               {
                 ROS_ERROR("Planning failed computing waypoint %u", iw);
                 return 0;
               }
               group.execute(plan);
             }
             active=false;
             repetition=false;

             msgs_vibr.data=3;
             vibration_pub.publish(msgs_vibr);

             ROS_INFO_THROTTLE(1,"deactive repetition");
         }
         else
         {
             if (gest.pose==5)
             {
              active=true;
              repetition=true;

              msgs_vibr.data=3;
              vibration_pub.publish(msgs_vibr);

              ROS_INFO_THROTTLE(1,"repetition active");
             }

             if (gest.pose==4)
             {
                 if((ros::Time::now()-t0).toSec()>5)
                  {
                     active=true;
                     t0=ros::Time::now();
                     //stop_configuration_client.call(srv_stop);
                     srv.request.start_configuration="trj_tracker";
                     configuration_client.call(srv);
                     ros::Duration(2).sleep();

                     ROS_INFO_THROTTLE(1,"active node pos");
                     msgs_vibr.data=2;
                     vibration_pub.publish(msgs_vibr);
                  }
             }
             else
             {
                 t0=ros::Time::now();
                 ROS_INFO_THROTTLE(1,"time reset");
             }
         }
     }



     msgs_bool.data=active;
     activation_pub.publish(msgs_bool);
     gest_prec=gest.pose;

    }


   return 0;
 }*/
