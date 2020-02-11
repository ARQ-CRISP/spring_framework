// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#include "spring_framework/spring_network.h"

using namespace std;
using namespace spring_framework;


/*********************************************************************
* Comment
*********************************************************************/
SpringNetwork::SpringNetwork(){

}
SpringNetwork::~SpringNetwork(){

}
/*********************************************************************
* sets rest lengths to the current distances between end frames.
* optional parameter 'scaler' is applied on the rest lengths.
* applied to all springs.
*********************************************************************/
void SpringNetwork::resetRestLengths(const double scaler){

  int spring_count = spring_vec_.size();

  for(int si=0; si < spring_count; si++){
    resetRestLength(si, scaler);
  }
}
/*********************************************************************
* sets rest lengths to the current distances between end frames.
* optional parameter 'scaler' is applied on the rest lengths.
*********************************************************************/
void SpringNetwork::resetRestLength(const int spring_no, const double scaler ){

  // frame indices for this spring
    int s_offset = spring_no*2;
  int fi_0 = network_vec_[s_offset];
  int fi_1 = network_vec_[s_offset+1];

  spring_vec_[spring_no].setRestLength(
      (frame_vec_[fi_0].p - frame_vec_[fi_1].p).Norm() * scaler);
}
/*********************************************************************
* add existing spring to network
*********************************************************************/
void SpringNetwork::addSpring(VirtualSpring &spring){
  spring_vec_.push_back(spring);
}
/*********************************************************************
* create a spring and add to network
* fi_1: first frame index
* fi_2: second frame index
* l: rest length
* k: stiffness parameter
*********************************************************************/
void SpringNetwork::createSpring(const int fi_1, const int fi_2, const double l, const double k){
  // add indices to network
  network_vec_.push_back(fi_1);
  network_vec_.push_back(fi_2);
  // create the spring
  VirtualSpring vs(l,k);
  spring_vec_.push_back(vs);
}
/*********************************************************************
* create multiple springs, defined by a list of pairs.
* pairs are the indices of frames between which the spring is connected.
*********************************************************************/
void SpringNetwork::createNetwork(const vector<int> &network_vec){
  // divided by two since 2 indices make one spring
  int spring_count = network_vec.size()/2;
  // create a spring for each pair of entries
  for(int si=0; si<spring_count; si++) createSpring(network_vec[si*2], network_vec[si*2+1]);
}
/*********************************************************************
* updates the frames, this should be called every time the frames move
*********************************************************************/
void SpringNetwork::updateFrames(const vector<KDL::Frame> &frame_vec){

  frame_vec_ = frame_vec;

  int spring_count = getSize();
  // update frames of particular springs according to the indices
  for(int si=0; si<spring_count; si++){
    vector<KDL::Frame> spring_frames;

    int fi_1 = network_vec_[si*2];
    int fi_2 = network_vec_[(si*2)+1];

    spring_frames.push_back( frame_vec_[fi_1]);
    spring_frames.push_back( frame_vec_[fi_2]);

    spring_vec_[si].updateFrames(spring_frames);

  }
}
/*********************************************************************
* return a message equivalent to this network
*********************************************************************/
SpringNetworkMsg SpringNetwork::toMsg() const{
  SpringNetworkMsg msg;

  // add all springs to msg
  int spring_count = getSize();
  for(int si=0; si<spring_count; si++){
    msg.springs.push_back(spring_vec_[si].toMsg());
  }

  return msg;
}
/*********************************************************************
* Getters and setters
*********************************************************************/
// *********** getters
int SpringNetwork::getSize() const{
  return spring_vec_.size();
}
vector<KDL::Frame> SpringNetwork::getFrames() const{
  return frame_vec_;
}
void SpringNetwork::getSpring(const int spring_no, VirtualSpring &spring) const{
  spring = spring_vec_[spring_no];}
double SpringNetwork::getRestLength(const int spring_no) const{
  return spring_vec_[spring_no].getRestLength(); }
vector<double> SpringNetwork::getStiffness() const{
  int spring_count = getSize();
  vector<double> l_rest_vec(spring_count);
  for(int si=0; si < spring_count; si++) l_rest_vec[si] = spring_vec_[si].getStiffness();
  return l_rest_vec;
}
double SpringNetwork::getStiffness(const int spring_no) const{
  return spring_vec_[spring_no].getStiffness(); }
int SpringNetwork::getSpringFrame(const int spring_no, const int end) const{
  return network_vec_[spring_no*2+end]; }
void SpringNetwork::getSpringForce(const int spring_no, KDL::Vector &f_out){
  getSpringForce(spring_no, 0, f_out); }
void SpringNetwork::getSpringForce(const int spring_no, const int dir, KDL::Vector &f_out){
  spring_vec_[spring_no].calcForce(dir, f_out); }
// *********** setters
void SpringNetwork::setRestLength(const int spring_no, const double l_rest){
  spring_vec_[spring_no].setRestLength(l_rest); }
void SpringNetwork::setRestLengths(const vector<double> l_rest_vec){
  int spring_count = getSize();
  for(int si=0; si < spring_count; si++) spring_vec_[si].setRestLength(l_rest_vec[si]);
}
void SpringNetwork::setStiffness(const double k){
    int spring_count = getSize();
    for(int si=0; si < spring_count; si++) spring_vec_[si].setStiffness(k);
}
void SpringNetwork::setStiffness(const int spring_no, const double k){
  spring_vec_[spring_no].setStiffness(k); }
