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

#include <configuration_msgs/StartConfiguration.h>
#include <configuration_msgs/StopConfiguration.h>
#include <ros/ros.h>
#include <subscription_notifier/subscription_notifier.h>
#include <ros_myo/MyoPose.h>
#include <std_msgs/Bool.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  double st=0.01;
  ros::WallRate lp(1.0/st);

  ros::ServiceClient configuration_client=nh.serviceClient<configuration_msgs::StartConfiguration>("/configuration_manager/start_configuration");
  ros::ServiceClient stop_configuration_client=nh.serviceClient<configuration_msgs::StopConfiguration>("/configuration_manager/stop_configuration");


  configuration_msgs::StartConfiguration srv;
  srv.request.start_configuration='trj_tracker';
  srv.request.strictness=1;

  configuration_msgs::StopConfiguration srv_stop;
  srv_stop.request.strictness=1;

  ros_helper::SubscriptionNotifier<ros_myo::MyoPose> myo_pose_sub(nh,"/myo_raw/myo_gest",1);
  if (!myo_pose_sub.waitForANewData(ros::Duration(60)))
  {
    ROS_ERROR("No topic received");
    return 0;
  }

  ros::Publisher activation_pub=nh.advertise<std_msgs::Bool>("/moveit_planning/active",1);

  std::string group_name;
  if (!nh.getParam("group_name",group_name))
  {
    ROS_INFO("Group name is not defined, using 'manipulator' as default value");
    group_name="manipulator";
  }

  moveit::planning_interface::MoveGroupInterface group(group_name);
  group.setStartState(*group.getCurrentState());

  if (!group.startStateMonitor(10))
  {
    ROS_ERROR("Unable to get the current state of the move group %s",group_name.c_str());
    return 0;
  }

  moveit::core::RobotState trj_state = *group.getCurrentState();
  std::vector<double> current_joint_configuration;
  trj_state.copyJointGroupPositions(group_name,current_joint_configuration);


  std::vector<std::vector<double>> waypoints;
  bool active=false;
  ros::Time t0=ros::Time::now();
  std_msgs::Bool msgs;
  int lenght=waypoints.size();
  int gest_prec=0;

   while (ros::ok())
   {
    ros::spinOnce();
    ros_myo::MyoPose gest=myo_pose_sub.getData();

    if (active)
    {
        if (gest.pose==3 && gest_prec!=3)
        {
          moveit::core::RobotState trj_state = *group.getCurrentState();
          std::vector<double> current_joint_configuration;
          trj_state.copyJointGroupPositions(group_name,current_joint_configuration);

           waypoints.push_back(current_joint_configuration); // così aggiungi le configurazioni che vuoi al vettore dei punti.
           lenght=waypoints.size();
        }

        if (gest.pose==4)
        {
            if((ros::Time::now()-t0).toSec()>5)
             {
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
             }
        }
        else
        {
            t0=ros::Time::now();
        }
    }
    else
    {
        if (gest.pose==4)
        {
            if((ros::Time::now()-t0).toSec()>5)
             {
                active=true;
                t0=ros::Time::now();
                //stop_configuration_client.call(srv_stop);
                //configuration_client.call(srv);
                ROS_INFO_THROTTLE(1,"active node pos");
             }
        }
        else
        {
            t0=ros::Time::now();
            ROS_INFO_THROTTLE(1,"time reset");
        }
    }

    msgs.data=active;
    activation_pub.publish(msgs);
    gest_prec=gest.pose;

   }


  return 0;
}



