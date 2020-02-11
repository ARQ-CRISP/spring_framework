// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#include "spring_framework/virtual_spring.h"

using namespace std;
using namespace spring_framework;

/*********************************************************************
* VirtualSpring defines a spring object between two cartesian frames.
* Behavior of the spring is characterized by the rest length and
* stiffness parameters
*********************************************************************/
VirtualSpring::VirtualSpring(){

  setRestLength(0);
  setStiffness(0);
}
VirtualSpring::VirtualSpring(const double l_rest, const double k){

  setRestLength(l_rest);
  setStiffness(k);
}
VirtualSpring::VirtualSpring(const VirtualSpring &vs){

  k_ = vs.getStiffness();
  l_ = vs.getRestLength();
  frame_vec_ = vs.getFrames();
}
/*********************************************************************
* Destructor
*********************************************************************/
VirtualSpring::~VirtualSpring(){

}
/*********************************************************************
* Updates the frame poses on spring's ends.
* This should be called every time the spring end poses change.
*********************************************************************/
void VirtualSpring::updateFrames(const vector<KDL::Frame> &frame_vec){
  // size safety
  if(frame_vec.size() != 2) {
    ROS_WARN("VirtualSpring: should have 2 frames.");
  }

  frame_vec_ = frame_vec;
}
/*********************************************************************
* calculates the current force the spring applies
* 'end' argument indicates the end frame index (0 or 1)
* where the force is applied.
*********************************************************************/
void VirtualSpring::calcForce(const int end, KDL::Vector &f_out){

  // wrong end index
  if (end > 1 || end < 0) {
    f_out = KDL::Vector::Zero();
    ROS_WARN("VirtualSpring: end argument is either 0 or 1.");
    return;
  }

  // delta p: distance between spring end frames
  KDL::Vector p_dif = frame_vec_[1-end].p - frame_vec_[end].p;

  // force = amplitude * direction
  f_out = (k_ * (p_dif.Norm() - l_)) * (p_dif / p_dif.Norm());

  // convert to meters
  f_out = f_out * 100.0;

}
/*********************************************************************
* return a message equivalent to this spring
*********************************************************************/
VirtualSpringMsg VirtualSpring::toMsg() const{
  VirtualSpringMsg msg;

  msg.k = k_;
  msg.l = l_;

  // convert kdl frame to geometry_msgs pose
  tf::poseKDLToMsg(frame_vec_[0], msg.f1);
  tf::poseKDLToMsg(frame_vec_[1], msg.f2);

  return msg;
}
/*********************************************************************
* Getters and setters
*********************************************************************/
// *********** getters
double VirtualSpring::getRestLength() const{
  return l_; }
double VirtualSpring::getStiffness() const{
  return k_; }
vector<KDL::Frame> VirtualSpring::getFrames() const{
  return frame_vec_; }
// *********** setters
void VirtualSpring::setRestLength(const double l_rest){
  l_ = l_rest; }
void VirtualSpring::setStiffness(const double k){
  k_ = k; }
