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
#include "std_msgs/Bool.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.035;
  posture.points[0].positions[1] = 0.035;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.025;
  posture.points[0].positions[1] = 0.025;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick(moveit::planning_interface::MoveGroupInterface& move_group, int idx, float pickX, float pickY, float pickZ)
{
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  // Setting grasp pose
  // ++++++++++++++++++++++
  // This is the pose of panda_link8.
  // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0
  // (half of the length of the cube).
  // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and
  // palm of eef - some extra padding)
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(-M_PI, 0, -3*M_PI /4);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = pickX;
  grasps[0].grasp_pose.pose.position.y = pickY;
  grasps[0].grasp_pose.pose.position.z = pickZ+0.075; //pose:pickZ - palm of endeffector: pickZ-0.075

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive x axis */
  grasps[0].pre_grasp_approach.direction.vector.z = -1.0;
  grasps[0].pre_grasp_approach.min_distance = 0.095;
  grasps[0].pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive z axis */
  grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
  grasps[0].post_grasp_retreat.min_distance = 0.1;
  grasps[0].post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  openGripper(grasps[0].pre_grasp_posture);


  // Setting posture of eef during grasp
  closedGripper(grasps[0].grasp_posture);


  // Call pick to pick up the object using the grasps given
  if (idx==0) {
      move_group.pick("object1", grasps);
  }
  else if (idx==1) {
      move_group.pick("object2", grasps);
  }
  else if (idx==2) {
      move_group.pick("object3", grasps);
  }
  else if (idx==3) {
      move_group.pick("object4", grasps);
  }
  else if (idx==4) {
      move_group.pick("object5", grasps);
  }
  else if (idx==5) {
      move_group.pick("object6", grasps);
  }
  else if (idx==6) {
      move_group.pick("object7", grasps);
  }
  else if (idx==7) {
      move_group.pick("object8", grasps);
  }
  else if (idx==8) {
      move_group.pick("object9", grasps);
  }
  else if (idx==9) {
      move_group.pick("object10", grasps);
  }

}

std::vector<moveit_msgs::CollisionObject> collision_objects;
std::vector<moveit_msgs::ObjectColor> mycolorobj;
void place(moveit::planning_interface::MoveGroupInterface& group, int idx, float placeX, float placeY, float placeZ)
{
  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = placeX;
  place_location[0].place_pose.pose.position.y = placeY;
  place_location[0].place_pose.pose.position.z = placeZ;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive z axis */
  place_location[0].post_place_retreat.direction.vector.z = 1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;


  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);


  // Call place to place the object using the place locations given.

  if (idx==0) {
      group.place("object1", place_location);
      collision_objects[idx].primitive_poses[idx].position.x = placeX;
      collision_objects[idx].primitive_poses[idx].position.y = placeY;
      collision_objects[idx].primitive_poses[idx].position.z = placeZ;
  }
  else if (idx==1) {
      collision_objects[1].primitive_poses[1].position.x = placeX;
      collision_objects[1].primitive_poses[1].position.y = placeY;
      collision_objects[1].primitive_poses[1].position.z = placeZ;
      group.place("object2", place_location);
  }
  else if (idx==2) {
      group.place("object3", place_location);
  }
  else if (idx==3) {
      group.place("object4", place_location);
  }
  else if (idx==4) {
      group.place("object5", place_location);
  }
  else if (idx==5) {
      group.place("object6", place_location);
      collision_objects[5].primitive_poses[5].position.x = placeX;
      collision_objects[5].primitive_poses[5].position.y = placeY;
      collision_objects[5].primitive_poses[5].position.z = placeZ;
  }
  else if (idx==6) {
      group.place("object7", place_location);
      collision_objects[idx].primitive_poses[idx].position.x = placeX;
      collision_objects[idx].primitive_poses[idx].position.y = placeY;
      collision_objects[idx].primitive_poses[idx].position.z = placeZ;
  }
  else if (idx==7) {
      group.place("object8", place_location);
  }
  else if (idx==8) {
      group.place("object9", place_location);
  }
  else if (idx==9) {
      group.place("object10", place_location);
  }


}

void move_wait_pos(moveit::planning_interface::MoveGroupInterface& group)
{
  geometry_msgs::Pose target_pose1;
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, -3*M_PI/4);
  target_pose1.orientation = tf2::toMsg(orientation);
  target_pose1.position.x = 0.500;
  target_pose1.position.y = 0;
  target_pose1.position.z = 1.0;
  group.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  group.plan(my_plan);
  group.move();
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  //************ AUFBAU DER GESAMTEN SPIELUMGEBUNG ************//
  //
  // Deklaration von einem Vektor für 14 Objekte

  collision_objects.resize(14);

  //******** AUFBAU DER SPIELSTEINE DES SPIELERS 1 ********//

  //*** Spielstein 1 - Spieler 1 ***//
  collision_objects[0].header.frame_id = "panda_link0";
  collision_objects[0].id = "object1";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.050;
  collision_objects[0].primitives[0].dimensions[1] = 0.025;
  //collision_objects[0].primitives[0].dimensions[2] = 0.050;

  /* Define the pose of the object. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.500;
  collision_objects[0].primitive_poses[0].position.y = -0.200;
  collision_objects[0].primitive_poses[0].position.z = 0.025;

  collision_objects[0].operation = collision_objects[0].ADD;


  //*** Spielstein 2 - Spieler 1 ***//
  collision_objects[1].header.frame_id = "panda_link0";
  collision_objects[1].id = "object2";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.050;
  collision_objects[1].primitives[0].dimensions[1] = 0.025;
  //collision_objects[1].primitives[0].dimensions[2] = 0.050;

  /* Define the pose of the object. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.500;
  collision_objects[1].primitive_poses[0].position.y = -0.100;
  collision_objects[1].primitive_poses[0].position.z = 0.025;

  collision_objects[1].operation = collision_objects[1].ADD;


  //*** Spielstein 3 - Spieler 1 ***//
  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "object3";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].CYLINDER;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.050;
  collision_objects[2].primitives[0].dimensions[1] = 0.025;
  //collision_objects[2].primitives[0].dimensions[2] = 0.050;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.500;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.025;

  collision_objects[2].operation = collision_objects[2].ADD;


  //*** Spielstein 4 - Spieler 1 ***//
  collision_objects[3].header.frame_id = "panda_link0";
  collision_objects[3].id = "object4";

  /* Define the primitive and its dimensions. */
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[2].primitives[0].CYLINDER;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 0.050;
  collision_objects[3].primitives[0].dimensions[1] = 0.025;
  //collision_objects[3].primitives[0].dimensions[2] = 0.050;

  /* Define the pose of the object. */
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0.500;
  collision_objects[3].primitive_poses[0].position.y = 0.100;
  collision_objects[3].primitive_poses[0].position.z = 0.025;

  collision_objects[3].operation = collision_objects[3].ADD;


  //*** Spielstein 5 - Spieler 1 ***//
  collision_objects[4].header.frame_id = "panda_link0";
  collision_objects[4].id = "object5";

  /* Define the primitive and its dimensions. */
  collision_objects[4].primitives.resize(1);
  collision_objects[4].primitives[0].type = collision_objects[3].primitives[0].CYLINDER;
  collision_objects[4].primitives[0].dimensions.resize(3);
  collision_objects[4].primitives[0].dimensions[0] = 0.050;
  collision_objects[4].primitives[0].dimensions[1] = 0.025;
  //collision_objects[4].primitives[0].dimensions[2] = 0.050;

  /* Define the pose of the object. */
  collision_objects[4].primitive_poses.resize(1);
  collision_objects[4].primitive_poses[0].position.x = 0.500;
  collision_objects[4].primitive_poses[0].position.y = 0.200;
  collision_objects[4].primitive_poses[0].position.z = 0.025;

  collision_objects[4].operation = collision_objects[4].ADD;


  //******** AUFBAU DER SPIELSTEINE DES SPIELERS 2 ********//

  //*** Spielstein 1 - Spieler 2 ***//
  collision_objects[5].header.frame_id = "panda_link0";
  collision_objects[5].id = "object6";

  /* Define the primitive and its dimensions. */
  collision_objects[5].primitives.resize(1);
  collision_objects[5].primitives[0].type = collision_objects[4].primitives[0].BOX;
  collision_objects[5].primitives[0].dimensions.resize(3);
  collision_objects[5].primitives[0].dimensions[0] = 0.050;
  collision_objects[5].primitives[0].dimensions[1] = 0.050;
  collision_objects[5].primitives[0].dimensions[2] = 0.050;

  /* Define the pose of the object. */
  collision_objects[5].primitive_poses.resize(1);
  collision_objects[5].primitive_poses[0].position.x = 0.600;
  collision_objects[5].primitive_poses[0].position.y = -0.200;
  collision_objects[5].primitive_poses[0].position.z = 0.025;

  collision_objects[5].operation = collision_objects[5].ADD;


  //*** Spielstein 2 - Spieler 2 ***//
  collision_objects[6].header.frame_id = "panda_link0";
  collision_objects[6].id = "object7";

  /* Define the primitive and its dimensions. */
  collision_objects[6].primitives.resize(1);
  collision_objects[6].primitives[0].type = collision_objects[5].primitives[0].BOX;
  collision_objects[6].primitives[0].dimensions.resize(3);
  collision_objects[6].primitives[0].dimensions[0] = 0.050;
  collision_objects[6].primitives[0].dimensions[1] = 0.050;
  collision_objects[6].primitives[0].dimensions[2] = 0.050;

  /* Define the pose of the object. */
  collision_objects[6].primitive_poses.resize(1);
  collision_objects[6].primitive_poses[0].position.x = 0.600;
  collision_objects[6].primitive_poses[0].position.y = -0.100;
  collision_objects[6].primitive_poses[0].position.z = 0.025;

  collision_objects[6].operation = collision_objects[6].ADD;


  //*** Spielstein 3 - Spieler 2 ***//
  collision_objects[7].header.frame_id = "panda_link0";
  collision_objects[7].id = "object8";

  /* Define the primitive and its dimensions. */
  collision_objects[7].primitives.resize(1);
  collision_objects[7].primitives[0].type = collision_objects[6].primitives[0].BOX;
  collision_objects[7].primitives[0].dimensions.resize(3);
  collision_objects[7].primitives[0].dimensions[0] = 0.050;
  collision_objects[7].primitives[0].dimensions[1] = 0.050;
  collision_objects[7].primitives[0].dimensions[2] = 0.050;

  /* Define the pose of the object. */
  collision_objects[7].primitive_poses.resize(1);
  collision_objects[7].primitive_poses[0].position.x = 0.600;
  collision_objects[7].primitive_poses[0].position.y = 0;
  collision_objects[7].primitive_poses[0].position.z = 0.025;

  collision_objects[7].operation = collision_objects[7].ADD;


  //*** Spielstein 4 - Spieler 2 ***//
  collision_objects[8].header.frame_id = "panda_link0";
  collision_objects[8].id = "object9";

  /* Define the primitive and its dimensions. */
  collision_objects[8].primitives.resize(1);
  collision_objects[8].primitives[0].type = collision_objects[7].primitives[0].BOX;
  collision_objects[8].primitives[0].dimensions.resize(3);
  collision_objects[8].primitives[0].dimensions[0] = 0.050;
  collision_objects[8].primitives[0].dimensions[1] = 0.050;
  collision_objects[8].primitives[0].dimensions[2] = 0.050;

  /* Define the pose of the object. */
  collision_objects[8].primitive_poses.resize(1);
  collision_objects[8].primitive_poses[0].position.x = 0.600;
  collision_objects[8].primitive_poses[0].position.y = 0.100;
  collision_objects[8].primitive_poses[0].position.z = 0.025;

  collision_objects[8].operation = collision_objects[8].ADD;


  //*** Spielstein 5 - Spieler 2 ***//
  collision_objects[9].header.frame_id = "panda_link0";
  collision_objects[9].id = "object10";

  /* Define the primitive and its dimensions. */
  collision_objects[9].primitives.resize(1);
  collision_objects[9].primitives[0].type = collision_objects[8].primitives[0].BOX;
  collision_objects[9].primitives[0].dimensions.resize(3);
  collision_objects[9].primitives[0].dimensions[0] = 0.050;
  collision_objects[9].primitives[0].dimensions[1] = 0.050;
  collision_objects[9].primitives[0].dimensions[2] = 0.050;

  /* Define the pose of the object. */
  collision_objects[9].primitive_poses.resize(1);
  collision_objects[9].primitive_poses[0].position.x = 0.600;
  collision_objects[9].primitive_poses[0].position.y = 0.200;
  collision_objects[9].primitive_poses[0].position.z = 0.025;

  collision_objects[9].operation = collision_objects[9].ADD;



  //******** AUFBAU DER SPIELWÄNDE ******** //
  //*** Wand 1 ***//
  collision_objects[10].header.frame_id = "panda_link0";
  collision_objects[10].id = "object11";

  /* Define the primitive and its dimensions. */
  collision_objects[10].primitives.resize(1);
  collision_objects[10].primitives[0].type = collision_objects[9].primitives[0].BOX;
  collision_objects[10].primitives[0].dimensions.resize(3);
  collision_objects[10].primitives[0].dimensions[0] = 0.320;
  collision_objects[10].primitives[0].dimensions[1] = 0.010;
  collision_objects[10].primitives[0].dimensions[2] = 0.020;

  /* Define the pose of the object. */
  collision_objects[10].primitive_poses.resize(1);
  collision_objects[10].primitive_poses[0].position.x = 0.000;
  collision_objects[10].primitive_poses[0].position.y = 0.510;
  collision_objects[10].primitive_poses[0].position.z = 0.010;

  collision_objects[10].operation = collision_objects[10].ADD;


  //*** Wand 2 ***//
  collision_objects[11].header.frame_id = "panda_link0";
  collision_objects[11].id = "object12";

  /* Define the primitive and its dimensions. */
  collision_objects[11].primitives.resize(1);
  collision_objects[11].primitives[0].type = collision_objects[10].primitives[0].BOX;
  collision_objects[11].primitives[0].dimensions.resize(3);
  collision_objects[11].primitives[0].dimensions[0] = 0.320;
  collision_objects[11].primitives[0].dimensions[1] = 0.010;
  collision_objects[11].primitives[0].dimensions[2] = 0.020;

  /* Define the pose of the object. */
  collision_objects[11].primitive_poses.resize(1);
  collision_objects[11].primitive_poses[0].position.x = 0.000;
  collision_objects[11].primitive_poses[0].position.y = 0.630;
  collision_objects[11].primitive_poses[0].position.z = 0.010;

  collision_objects[11].operation = collision_objects[11].ADD;


  //*** Wand 3 ***//
  collision_objects[12].header.frame_id = "panda_link0";
  collision_objects[12].id = "object13";

  /* Define the primitive and its dimensions. */
  collision_objects[12].primitives.resize(1);
  collision_objects[12].primitives[0].type = collision_objects[11].primitives[0].BOX;
  collision_objects[12].primitives[0].dimensions.resize(3);
  collision_objects[12].primitives[0].dimensions[0] = 0.010;
  collision_objects[12].primitives[0].dimensions[1] = 0.320;
  collision_objects[12].primitives[0].dimensions[2] = 0.020;

  /* Define the pose of the object. */
  collision_objects[12].primitive_poses.resize(1);
  collision_objects[12].primitive_poses[0].position.x = -0.060;
  collision_objects[12].primitive_poses[0].position.y = 0.570;
  collision_objects[12].primitive_poses[0].position.z = 0.010;

  collision_objects[12].operation = collision_objects[12].ADD;


  //*** Wand 4 ***//
  collision_objects[13].header.frame_id = "panda_link0";
  collision_objects[13].id = "object14";

  /* Define the primitive and its dimensions. */
  collision_objects[13].primitives.resize(1);
  collision_objects[13].primitives[0].type = collision_objects[12].primitives[0].BOX;
  collision_objects[13].primitives[0].dimensions.resize(3);
  collision_objects[13].primitives[0].dimensions[0] = 0.010;
  collision_objects[13].primitives[0].dimensions[1] = 0.320;
  collision_objects[13].primitives[0].dimensions[2] = 0.020;

  /* Define the pose of the object. */
  collision_objects[13].primitive_poses.resize(1);
  collision_objects[13].primitive_poses[0].position.x = 0.060;
  collision_objects[13].primitive_poses[0].position.y = 0.570;
  collision_objects[13].primitive_poses[0].position.z = 0.010;

  collision_objects[13].operation = collision_objects[13].ADD;


  //******** DEFINITION DER OBJEKTFARBEN ********//

  mycolorobj.resize(14);

  //*** Definition der benutzten Farben ***//
  std_msgs::ColorRGBA red;
  red.r=255;
  red.g=0;
  red.b=0;
  red.a=1;
  std_msgs::ColorRGBA blue;
  blue.r=0;
  blue.g=0;
  blue.b=1;
  blue.a=1;
  //std_msgs::ColorRGBA brown = rviz_visual_tools::RvizVisualTools.getColor(rviz_visual_tools::BROWN);
  std_msgs::ColorRGBA brown;
  brown.r=0.597;
  brown.g=0.296;
  brown.b=0.0;
  brown.a=1;

  //*** Verändern der Farbe der Objekte ***//
  for (int i=0; i<14; i++)
  {
    std::string name = "object" + boost::lexical_cast<std::string>(i+1);
    if (i<5)
    {
      mycolorobj[i].id = name;
      mycolorobj[i].color = blue;
    }
    else if (i<10)
    {
      mycolorobj[i].id = name;
      mycolorobj[i].color = red;
    }
    else if (i<14)
    {
      mycolorobj[i].id = name;
      mycolorobj[i].color = brown;
    }
  }

  planning_scene_interface.applyCollisionObjects(collision_objects, mycolorobj);

}

float place_next [3] = {0.0,0.450,0.025};
bool msg_valid = false;
void moveCallback(const nav_msgs::Odometry::ConstPtr& coord)
{
  //Lese Effektor-Koordinaten für die Platzierung des nächsten Spielsteins
  //ROS_INFO("Koordinaten empfangen");

  place_next[0] = coord->pose.pose.position.x;
  place_next[1] = coord->pose.pose.position.y;
  place_next[2] = coord->pose.pose.position.z;

  //Validiere die Bewegung und das Schicken der passenden Nachricht
  msg_valid = true;
}

//Pick-Tabelle definieren, in der die Position der Spielsteine im Anfangszustand gespeichert sind
//Jeder Spieler besitzt 5 Spielsteine, die mit p1 bzw. p2 bezeichnet sind
//Spieler 1
float pickX_p1[5] = {0.500, 0.500, 0.500, 0.500, 0.500};
float pickY_p1[5] = {-0.200, -0.100, 0.0, 0.100, 0.200};
float pickZ_p1[5] = {0.050, 0.050, 0.050, 0.050, 0.050};

//Spieler 2
float pickX_p2[5] = {0.600, 0.600, 0.600, 0.600, 0.600};
float pickY_p2[5] = {-0.200, -0.100, 0.0, 0.100, 0.200};
float pickZ_p2[5] = {0.050, 0.050, 0.050, 0.050, 0.050};


std::vector<moveit_msgs::CollisionObject> replace_object;
std::vector<moveit_msgs::ObjectColor> replace_color;
void ChangeObjectColor(int idx, float place_next[3], int player)
{
  std::string name = "object" + boost::lexical_cast<std::string>(idx+1) + "bis";

  replace_object.resize(1);
  replace_object[0].header.frame_id = "panda_link0";
  replace_object[0].id = name;
  /* Define the primitive and its dimensions. */
  replace_object[0].primitives.resize(1);
  if (player == 1)
  {
    replace_object[0].primitives[0] = collision_objects[0].primitives[0];
  }
  else if (player == 2)
  {
    replace_object[0].primitives[0] = collision_objects[5].primitives[0];
  }
  /* Define the pose of the object. */
  replace_object[0].primitive_poses.resize(1);
  replace_object[0].primitive_poses[0].position.x = place_next[0];
  replace_object[0].primitive_poses[0].position.y = place_next[1];
  replace_object[0].primitive_poses[0].position.z = place_next[2];
  /* Add object */
  replace_object[0].operation = replace_object[0].ADD;

  /* Define the color */
  replace_color.resize(1);
  std_msgs::ColorRGBA blue;
  blue.r=0;
  blue.g=0;
  blue.b=1;
  blue.a=1;
  std_msgs::ColorRGBA red;
  red.r=1;
  red.g=0;
  red.b=0;
  red.a=1;
  replace_color[0].id = name;
  if (player == 1)
  {
    replace_color[0].color = blue;
  }
  else if (player == 2)
  {
    replace_color[0].color = red;
  }

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_robot");
  ros::NodeHandle nh;
  ros::Publisher pub_end_move_robot = nh.advertise<std_msgs::Bool>("end_move_robot",10);
  ros::Subscriber sub_coord_robot_effector = nh.subscribe("coord_robot_effector", 100, moveCallback);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Rate loop_rate(40);

  //ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface group("panda_arm");
  group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);
  ROS_INFO("Startet world");

  int count = 0;
  int player = 1;
  int move_nb = 0;
  while (ros::ok())
  {
    if (msg_valid)
    {
      //Starte Bewegung von Spielstein
      if (player == 1)
      {
        ROS_INFO("Player1 Move");
        //ROS_INFO("Koordinaten: X:%f - Y:%f - Z:%f", place_next[0], place_next[1], place_next[2]);
        int idx = move_nb;

        pick(group,idx,pickX_p1[move_nb],pickY_p1[move_nb],pickZ_p1[move_nb]);
        ros::WallDuration(0.5).sleep();
        place(group,idx,place_next[0],place_next[1],place_next[2]);
        //planning_scene_interface.applyCollisionObjects(collision_objects, mycolorobj);

        //Suppress old object and define a new one at the place position
        std::vector<std::string> object_ids;
        object_ids.push_back(collision_objects[idx].id);
        planning_scene_interface.removeCollisionObjects(object_ids);
        ChangeObjectColor(idx, place_next, player);
        planning_scene_interface.addCollisionObjects(replace_object, replace_color);

        //Moving robot to wait poisition
        move_wait_pos(group);

        player = 2;
      }
      else if (player == 2)
      {
        ROS_INFO("Player2 Move");
        int idx = 5 + move_nb;

        pick(group,idx,pickX_p2[move_nb],pickY_p2[move_nb],pickZ_p2[move_nb]);
        ros::WallDuration(0.5).sleep();
        place(group,idx,place_next[0],place_next[1],place_next[2]);

        //Suppress old object and define a new one at the place position
        std::vector<std::string> object_ids;
        object_ids.push_back(collision_objects[idx].id);
        planning_scene_interface.removeCollisionObjects(object_ids);
        ChangeObjectColor(idx, place_next, player);
        planning_scene_interface.addCollisionObjects(replace_object, replace_color);

        //Moving robot to wait poisition
        move_wait_pos(group);

        player = 1;
        move_nb++;
      }

      //Sende Nachricht auf Topic "end_move_robot"
      std_msgs::Bool msg_ok;
      msg_ok.data = true;
      pub_end_move_robot.publish(msg_ok);
      ROS_INFO("OK Ende der Bewegung");
      msg_valid = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}
