// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#include "spring_framework/spring_display.h"
#include <math.h>
#include <tf/tf.h>

using namespace spring_framework;
/*********************************************************************
* Comment
*********************************************************************/
SpringDisplay::SpringDisplay(string topic_name, string frame_name)
{
  topic_name_ = topic_name;
  frame_name_ = frame_name;

  setAlpha(0.6);
  setWidth(0.035);
  setDuration(1.0);
}
/*********************************************************************
* Comment
*********************************************************************/
SpringDisplay::~SpringDisplay()
{

}
/*********************************************************************
* Starts the publisher
*********************************************************************/
void SpringDisplay::start(ros::NodeHandle &nh)
{
  // create the visualization_msgs publisher for markers
  vis_pub_ = nh.advertise<visualization_msgs::MarkerArray>(topic_name_+"_pub", 0);

}
/*********************************************************************
* Publish a spring marker between given points.
* Marker parameters can be adjusted with class setter methods.
*********************************************************************/
void SpringDisplay::updateSpring(const int spring_no, const double l_rest, const KDL::Vector &p1, const KDL::Vector &p2){

  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_name_;
  marker.header.stamp = ros::Time();
  marker.ns = topic_name_+"_pub";

  marker.id = spring_no;

  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.mesh_resource = "package://spring_framework/meshes/spring.stl";

  // show mesh in the middle of points
  marker.pose.position.x = (p1.x() + p2.x())/2;
  marker.pose.position.y = (p1.y() + p2.y())/2;
  marker.pose.position.z = (p1.z() + p2.z())/2;

  // calculate the orientation
  KDL::Vector p_init(1.0, 0.0, 0.0);
  KDL::Vector p_dif = p1-p2;
  double l_spring = p_dif.Norm();

  double dot_pr = KDL::dot(p_init, p_dif);

  double angle = acos(dot_pr/(p_init.Norm()*l_spring));
  KDL::Vector axis = p_init * p_dif;

  KDL::Rotation rot = KDL::Rotation::Rot(axis, angle);

  // assing to message
  rot.GetQuaternion(
    marker.pose.orientation.x,
    marker.pose.orientation.y,
    marker.pose.orientation.z,
    marker.pose.orientation.w);

  marker.scale.x = width_ * l_spring;
  marker.scale.y = width_ * 0.05;
  marker.scale.z = width_ * 0.06;

  // color gets red when stretched or shrunk
  double l_ratio = l_rest/l_spring;
  if(l_ratio < 1.0) l_ratio = 1.0 / l_ratio;
  l_ratio = min(l_ratio, 2.0) - 1.0;

  marker.color.r = 0.4 + l_ratio * 0.6;
  marker.color.g = 1.0 - l_ratio * 0.6;
  marker.color.b = 0.4;
  marker.color.a = alpha_;
  
  marker.lifetime = ros::Duration(duration_);
  marker.frame_locked = true;

  marker_arr_.markers.push_back(marker);
}
void SpringDisplay::updateSpring(const int spring_no, const VirtualSpring &vs){
  vector<KDL::Frame> fs_vec = vs.getFrames();
  updateSpring(spring_no, vs.getRestLength(), fs_vec[1].p, fs_vec[0].p);
}
/*********************************************************************
* convenience function to display all springs in a network.
*********************************************************************/
void SpringDisplay::updateSprings(const SpringNetwork &sn){
  int spring_count = sn.getSize();
  for(int si=0; si<spring_count; si++){
    VirtualSpring spring;
    sn.getSpring(si, spring);
    updateSpring(si, spring);
  }
}
/*********************************************************************
* Publish a line marker between the points of an array.
* Marker parameters can be adjusted with class setter methods.
*********************************************************************/
void SpringDisplay::publishMarkers(){

  // publish it
  vis_pub_.publish( marker_arr_ );

  // clean for the next round
  marker_arr_.markers.clear();
}
/*********************************************************************
* Deletes all the markers created by this object.
*********************************************************************/
void SpringDisplay::cleanMarkers(){
  // create the common part of the message
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_name_;
  marker.header.stamp = ros::Time();
  marker.ns = topic_name_ + "_pub";

  marker.action = visualization_msgs::Marker::DELETEALL;

  // add cleaner marker to array
  marker_arr_.markers.push_back(marker);

  // force publish iminently
  publishMarkers();
}
/*********************************************************************
* Setters
*********************************************************************/
void SpringDisplay::setAlpha(float a){
  alpha_ = a;
}
void SpringDisplay::setWidth(float w){
  width_ = w;
}
// Zero for persistent markers
void SpringDisplay::setDuration(float d){
  duration_ = d;
}
