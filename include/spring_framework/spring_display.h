// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#ifndef SPRING_DISPLAY_H
#define SPRING_DISPLAY_H

#include <ros/ros.h>

#include <spring_framework/spring_network.h>
#include <spring_framework/virtual_spring.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/ColorRGBA.h>

#include <kdl/jntarray.hpp>


using namespace std;

namespace spring_framework
{

class SpringDisplay {

protected:

  string topic_name_;
  string frame_name_;

  ros::Publisher vis_pub_;

  visualization_msgs::MarkerArray marker_arr_;

  float alpha_;
  float width_;
  float duration_;

public:

  SpringDisplay(string topic_name, string frame_name="hand_root");
  ~SpringDisplay();

  void setAlpha(float a=0.6);
  void setWidth(float w);
  void setDuration(float d);

  void start(ros::NodeHandle &nh);

  // update a spring display with new edge points
  void updateSpring(const int spring_no, const double l_rest, const KDL::Vector &p1, const KDL::Vector &p2);
  void updateSpring(const int spring_no, const VirtualSpring &vs);
  // convenience function to display all springs in a network
  void updateSprings(const SpringNetwork &sn);
  void publishMarkers();
  void cleanMarkers();


};


}// end namespace spring_framework

#endif // SPRING_DISPLAY_H
