/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan, Ridhwan Luthra*/

// ROS
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "nav_msgs/Odometry.h"

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

///// Tabelle mit Korrespondenz zwischen Spielfeldnummer und Effektor-Position
float table_coord_place [9][3] = {{-0.120, 0.450, 0.025}, \
                                  {0, 0.450, 0.025}, \
                                  {0.120, 0.450, 0.025}, \
                                  {-0.120, 0.570, 0.025}, \
                                  {0, 0.570, 0.025}, \
                                  {0.120, 0.570, 0.025}, \
                                  {-0.120, 0.690, 0.025}, \
                                  {0, 0.690, 0.025}, \
                                  {0.120, 0.690, 0.025}, \
                                };
bool msg_valid = false;
nav_msgs::Odometry msg_coord;

void ComputeCoordCallback(const std_msgs::Int8::ConstPtr& pawn_pos)
{
  //Bei Validierung der Eingabe bekommt man ein Int8 zwischen 1 und 9. Diese Zahl
  //entspricht der Position des Spielsteins auf dem Spielfeld
  //ROS_INFO("Spielstein-Position bekommen");

  //Berechnung der Koordinaten des Effektors zur Platzierung des Spielsteins
  int pos = pawn_pos->data;
  float temp_coord [3] = {table_coord_place[pos-1][0], table_coord_place[pos-1][1], table_coord_place[pos-1][2]};

  msg_coord.pose.pose.position.x = temp_coord[0];
  msg_coord.pose.pose.position.y = temp_coord[1];
  msg_coord.pose.pose.position.z = temp_coord[2];

  msg_valid = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compute_pose");
  ros::NodeHandle nh;
  ros::Publisher pub_coord_robot_effector = nh.advertise<nav_msgs::Odometry>("coord_robot_effector",100);
  ros::Subscriber sub_valid_next_move = nh.subscribe("valid_next_move", 100, ComputeCoordCallback);

  ros::Rate loop_rate(40);

  int count = 0;
  while (ros::ok())
  {
    //Ausgabe der Koordinaten auf dem Topic "coord_robot_effector"
    if (msg_valid)
    {
      pub_coord_robot_effector.publish(msg_coord);
      //ROS_INFO("Koordinaten gesendet");
      msg_valid = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
