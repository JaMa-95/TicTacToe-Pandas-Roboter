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
#include "std_msgs/Int8MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Bool.h"

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <string>

// Tabelle, die die bereits platzierten Spielsteine erhält
int CurrentPawnPos [9] = {0,0,0, 0,0,0, 0,0,0};

int end_move_true = 1;
int msg_valid = -1;
std_msgs::Int8 msg_to_send;

void CheckPositionCallback(const std_msgs::Int8MultiArray::ConstPtr& next_move)
{
  //Hole Spielfeldnummer
  ROS_INFO("Info von Spieler erhalten - Pruefung Position");
  int8_t position = next_move->data[0];
  //Prüfe Gültigkeit der Spielfeldnummer
  if ((0 < position) and (position < 10))
  {
    //Gültige Spielfeldnummer
    if (CurrentPawnPos[position-1] == 0)
    {
      ROS_INFO("Kein Stein auf Feld");
      //Spielzug ist ok da bereits kein Spielstein auf Spielfeld
      msg_to_send.data = position;
      msg_valid = 1;
      //hole Spielernummer
      int player = next_move->data[1];

      if (player == 1)
      {
        CurrentPawnPos[position-1] = 1;
      }
      else if (player == 2)
      {
        CurrentPawnPos[position-1] = 2;
      }
      else
      {
        //Spielzug ist doch nicht ok, da keine Spielerzuordnung stattfinden kann
        msg_valid = -1;
      }
    }
    else
    {
      //Es ist bereits ein Spielstein auf dem Spielfeld
      ROS_INFO ("Stein bereits auf dem Feld");
      msg_valid = 2;
    }
  }
  else {
    //Ungültige Spielfeldnummer
    msg_valid = 2;
  }


}

void CheckRobot(const std_msgs::Bool::ConstPtr& end_move)
{
	bool end = end_move->data;

	if (end == true)
	{
	end_move_true = 1;
	end = false;
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "eval_game");
  ros::NodeHandle nh;
  ros::Publisher pub_valid_next_move = nh.advertise<std_msgs::Int8>("valid_next_move",10);
  ros::Publisher pub_end_eval_game = nh.advertise<std_msgs::Int8>("end_eval_game", 1000);
  ros::Subscriber sub_next_move = nh.subscribe("next_move", 100, CheckPositionCallback);
  ros::Subscriber sub_end_move = nh.subscribe("end_move_robot", 100, CheckRobot);

  ros::Rate loop_rate(40);

  int count = 0;
  while (ros::ok())
  {
	//Möglichkeiten für den Sieg
	int win1 = CurrentPawnPos[0]*CurrentPawnPos[1]*CurrentPawnPos[2];
	int win2 = CurrentPawnPos[3]*CurrentPawnPos[4]*CurrentPawnPos[5];
	int win3 = CurrentPawnPos[6]*CurrentPawnPos[7]*CurrentPawnPos[8];
	int win4 = CurrentPawnPos[0]*CurrentPawnPos[3]*CurrentPawnPos[6];
	int win5 = CurrentPawnPos[1]*CurrentPawnPos[4]*CurrentPawnPos[7];
	int win6 = CurrentPawnPos[2]*CurrentPawnPos[5]*CurrentPawnPos[8];
	int win7 = CurrentPawnPos[0]*CurrentPawnPos[4]*CurrentPawnPos[8];
	int win8 = CurrentPawnPos[2]*CurrentPawnPos[4]*CurrentPawnPos[6];
	int draw = CurrentPawnPos[0]+CurrentPawnPos[1]+CurrentPawnPos[2]+CurrentPawnPos[3]+CurrentPawnPos[4]
		  +CurrentPawnPos[5]+CurrentPawnPos[6]+CurrentPawnPos[7]+CurrentPawnPos[8];

	//Nachricht an communication wenn Ende von robot_move

	if (end_move_true ==1)
	{
	//Sende Nachricht gewonnen auf end_eval_game
	if ((win1 == 1 || win2 == 1 || win3 == 1 || win4 == 1 || win5 == 1 || win6 == 1 || win7 == 1 || win8 == 1) )
	{
		std_msgs::Int8 msg_fault;
		int winner = 1;
		msg_fault.data = 3;
		//ROS_INFO("WINNER1");
      		pub_end_eval_game.publish(msg_fault);
		end_move_true = 0;
		for (int i =0; i<10; i++)
		{
		CurrentPawnPos[i] = 0;
		}
	}
	if ((win1 == 8 || win2 == 8 || win3 == 8 || win4 == 8 || win5 == 8 || win6 == 8 || win7 == 8 || win8 == 8))
	{
		std_msgs::Int8 msg_fault;
		int winner = 2;
		msg_fault.data = 4;
		//ROS_INFO("WINNER2");
      		pub_end_eval_game.publish(msg_fault);
		end_move_true = 0;
		for (int i =0; i<10; i++)
		{
		CurrentPawnPos[i] = 0;
		}
	}
	if (draw == 13)
	{
		std_msgs::Int8 msg_fault;
		msg_fault.data = 5;
		pub_end_eval_game.publish(msg_fault);
		end_move_true = 0;
		for (int i =0; i<10; i++)
		{
		CurrentPawnPos[i] = 0;
		}
	}
	}
	if  (end_move_true ==1)
   	 {
 	 std_msgs::Int8 msg_fault;
	 msg_fault.data = 1;
         pub_end_eval_game.publish(msg_fault);
         end_move_true = -1;
         }

    //Sende Nachricht auf dem Topic "valid_next_move"
    if (msg_valid == 1)
    {
      //Sende die Position des nächsten Spielsteins
      pub_valid_next_move.publish(msg_to_send);
      //ROS_INFO("OK: %d", msg_to_send);
      msg_valid =-1;
    }

    //Sende Nachricht auf dem Topuc "end_eval_game"
    else if (msg_valid == 2)
    {
      //Ungültige Spielerangabe
      std_msgs::Int8 msg_fault;
      msg_fault.data = 2;
      pub_end_eval_game.publish(msg_fault);
      //ROS_INFO("Ungueltig");
      msg_valid = -1;
    }

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
