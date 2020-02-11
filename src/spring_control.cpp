// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)

#include "spring_framework/spring_control.h"

using namespace std;
using namespace spring_framework;

/*********************************************************************
* SpringController class defines a virtual object w.r.t. contact positions.
* contact_count defines the number of contacts made with an object,
* typically the number of fingers.
*********************************************************************/
SpringController::SpringController(int contact_count, shared_ptr<SpringNetwork> network){

  contact_count_ = contact_count;

  network_ = network;

  // empty vf
  frame_vec_.resize(1);

  // defaults
  k_rot_ = 30.0;
  k_pos_ = 75.0;
  k_vel_ = 6.0;

  // set state to 'fresh'
  fresh_ = true;
}
SpringController::~SpringController()
{
}
/*********************************************************************
* calculates the new virtual frame pose using the contact point frames.
* see (Li et al. 2016) & (Tahara et al. 2012)
* also keeps track of time and velocity
*********************************************************************/
void SpringController::updateState(const vector<KDL::Frame> &frame_vec) {

  // keep previous state for calculations
  KDL::Frame vf_prev(frame_vec_[0]);

  // update frame vector
  frame_vec_ = frame_vec;

  // calculate the new VF using new frame_vec_
  KDL::Frame vf = computeVirtualFrame(frame_vec_, contact_count_);
  // push_front vf into frame_vec_
  frame_vec_.insert(frame_vec_.begin(), vf);

  // if not "fresh" also calculate the velocity
  if(!fresh_){

    // Calc time since last control
    ros::Time t_now = ros::Time::now();
    double dt_update = (t_now - t_update_).sec + 1e-9 * (t_now - t_update_).nsec;

    // differentiate vf
    KDL::Vector vel = (frame_vec_[0].p - vf_prev.p) / dt_update;

      // smoothing
    kdl_control_tools::smoothVector(vel, vel_que_, 3);
  }

  fresh_ = false;

  // update spring end frames
  if(network_)
    network_->updateFrames(frame_vec_);

  t_update_ = ros::Time::now();
}
/*********************************************************************
* calculates the force for a spring
* it uses the internal state variables,
* so the state should be updated using updateState().
*********************************************************************/
void SpringController::calcForce(const int spring_no, KDL::Vector &f_out){
  // size safety
  int spring_count = network_->getSize();
  if(spring_no >= spring_count) {
    f_out = KDL::Vector::Zero();
    ROS_WARN("SpringController: spring no %d out of bounds (%d).",
          spring_no, spring_count);
    return;
  }

  // calculate spring force
  // order matters; force will be applied on the first(0th) frame.
  network_->getSpringForce(spring_no, f_out);
}
/*********************************************************************
* calculates the force for all springs.
* it uses the internal state variables,
* so the state should be updated using updateState().
*********************************************************************/
void SpringController::calcSpringForces(vector<KDL::Vector> &f_outs){
  // resize force vector to spring count
  int spring_count = network_->getSize();
  f_outs.resize(spring_count);

  for(int si=0; si < spring_count; si++){
    // calculate spring force
    // order matters; force will be applied on the first(0th) frame.
    network_->getSpringForce(si, f_outs[si]);
  }
}
/*********************************************************************
* calculates the force for all frames.
* it uses the internal state variables,
* so the state should be updated using updateState().
*********************************************************************/
void SpringController::calcFrameForces(vector<KDL::Vector> &f_outs, bool double_sided){
  // return a force for each frame
  int f_size = frame_vec_.size();
  f_outs.resize(f_size);
  // spring size may be different than the frame size
  int s_size = network_->getSize();

  for(int si=0; si<s_size; si++){
    // calc spring force
    KDL::Vector f_spring;
    network_->getSpringForce(si, f_spring);
    // get the primary frame for this spring
    int fi_1 = network_->getSpringFrame(si, 0);
    // accumulate forces for corresponding frame
    f_outs[fi_1] +=   f_spring;

    // if double sided do the same for the opposite frame
    if(double_sided){
      int fi_2 = network_->getSpringFrame(si, 1);
      f_outs[fi_2] += (-f_spring);
    }
  }
}
/*********************************************************************
* Calculate the force to drive the current vf position towards
* the desired position x_des. Also maintain the desired velocity xd_des.
*********************************************************************/
void SpringController::calcPositionForce(const KDL::Vector x_des, const KDL::Vector xd_des, KDL::Vector &f_out)
{
  // *** Object manipulation force
  // ** Position control
  // delta x: distance between the current object frame and the desired
  KDL::Vector x_dif = x_des - frame_vec_[0].p;
  // delta xd: error between the current velocity and the desired
  KDL::Vector xd_dif = xd_des - vel_que_.front();

  // debug
  if(p_logger_){
    p_logger_->log("e_pos", x_dif.Norm());
    p_logger_->log("e_vel", xd_dif.Norm());
  }

  // force = spring + damping
  f_out = (k_pos_ * x_dif) + (k_vel_ * xd_dif);
}
/*********************************************************************
* Calculate the force to drive the current vf orientation
* towards the desired orientation x_des.
*********************************************************************/
void SpringController::calcRotationForce(const KDL::Rotation x_des, vector<KDL::Vector> &f_outs)
{
  // *** Object manipulation force
  // ** Orientation control (Tahara et al. 2012)

  // rotational error between current object frame and the desired
  KDL::Vector r_dif_kdl =
    k_rot_ *
    (frame_vec_[0].M.UnitX()*x_des.UnitX() +
    frame_vec_[0].M.UnitY()*x_des.UnitY() +
    frame_vec_[0].M.UnitZ()*x_des.UnitZ());

  // convert to eigen for matrix operations
  Eigen::Vector3d r_dif;
  tf::vectorKDLToEigen(r_dif_kdl, r_dif);

  // debug
  if(p_logger_)
    p_logger_->log("e_rot", (r_dif_kdl / (k_rot_*k_rot_)).Norm());

  // projection matrices to decompose r_dif
  Eigen::Vector3d r_z;
  tf::vectorKDLToEigen(frame_vec_[0].M.UnitZ(), r_z);
  Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

  Eigen::Matrix3d A_n = (r_z * r_z.transpose()) / r_z.squaredNorm();
  Eigen::Matrix3d A_t = I3 - A_n;

  // create a force for each contact
  f_outs.resize(contact_count_);
  // calculate the rotation control force for each contact
  for(int ci=0; ci < contact_count_; ci++){
    // finger and object positions
    Eigen::Vector3d x_i;
    Eigen::Vector3d x_o;
    tf::vectorKDLToEigen(frame_vec_[ci].p, x_i);
    tf::vectorKDLToEigen(frame_vec_[0].p, x_o);

    // rotation axis vectors
    Eigen::Vector3d ax_n = A_n * r_dif;
    Eigen::Vector3d ax_t = A_t * r_dif;

    // moment vector from finger to rotation pivot
    Eigen::Vector3d v_n = x_o - x_i;

    // add rotation control force
    Eigen::Vector3d f_rot = v_n.cross(ax_n);

    if(ax_t != Eigen::Vector3d::Zero()){
        // moment vector from finger to rotation pivot
        Eigen::Vector3d rz_axt = r_z.cross(ax_t);
        double nom = (v_n.transpose() * rz_axt);
        Eigen::Vector3d v_t =
          ( nom / ax_t.squaredNorm()) * rz_axt;

        // add rotation control force
        f_rot += v_t.cross(ax_t);

    }// else no force along ax_t

    // convert to kdl and add to the vector
    tf::vectorEigenToKDL(f_rot, f_outs[ci]);
  }
}
/*********************************************************************
* sets rest lengths to the current distances between end frames.
* optional parameter 'scaler' is applied on the rest lengths.
*********************************************************************/
void SpringController::resetRestLengths(const double scaler){

  network_->resetRestLengths(scaler);
}
/*********************************************************************
* Getters and setters
*********************************************************************/
// *********** getters
double SpringController::getRestLength(const int spring_no) const{
  return network_->getRestLength(spring_no); }
vector<double> SpringController::getStiffness() const{
  int spring_count = network_->getSize();
  vector<double> l_rest_vec(spring_count);
  for(int si=0; si < spring_count; si++) l_rest_vec[si] = network_->getStiffness(si);
  return l_rest_vec;
}
double SpringController::getStiffness(const int spring_no) const{
  return network_->getStiffness(spring_no); }
double SpringController::getPositionGain() const{
  return k_pos_; }
double SpringController::getVelocityGain() const{
  return k_vel_; }
double SpringController::getRotationGain() const{
  return k_rot_; }
void SpringController::getVirtualFrame(KDL::Frame& vf) const{
  vf = KDL::Frame(frame_vec_[0]); //copy
}
KDL::Frame SpringController::getVirtualFrame() const{
 return frame_vec_[0]; //copy
}
vector<KDL::Frame> SpringController::getFrames() const{
  return frame_vec_;
}
// *********** setters
void SpringController::setRestLength(const int spring_no, const double l_rest){
  network_->setRestLength(spring_no, l_rest); }
void SpringController::setRestLengths(const vector<double> l_rest_vec){
  network_->setRestLengths(l_rest_vec);
}
void SpringController::setStiffness(const double k){ // set same constant for all
  network_->setStiffness(k);
}
void SpringController::setStiffness(const int spring_no, const double k){
  network_->setStiffness(spring_no, k); }
void SpringController::setPositionGain(const double k){
  k_pos_ = k; }
void SpringController::setVelocityGain(const double k){
  k_vel_ = k; }
void SpringController::setRotationGain(const double k){
  k_rot_ = k; }
/*********************************************************************
* Connect an external ProgressLogger and log to it
*********************************************************************/
void SpringController::connectLogger(shared_ptr<kdl_control_tools::ProgressLogger> logger){
  p_logger_ = logger;
}
/*********************************************************************
* Static functions
*********************************************************************/
/*********************************************************************
* calculates the new virtual frame pose using the contact shared_ptr<frames>
 (first contact_count elements of frame_vec).
* see (Li et al. 2016) & (Tahara et al. 2012)
*********************************************************************/
KDL::Frame SpringController::computeVirtualFrame(vector<KDL::Frame>& frame_vec, int contact_count) {
  // if not specified, take all frames as contact
  if(contact_count < 0)
    contact_count = frame_vec.size();
  // constants
  const int i_thumb = contact_count - 1;

  // calculate VF position
  KDL::Frame vf = KDL::Frame::Identity(); // I rot, zero pos
  for(int ci=0; ci < contact_count; ci++){

    // thumb has more weight
    double weight = 1.0;
    if(ci==(i_thumb)) weight = 3.0;

    vf.p += frame_vec[ci].p * weight / (double)(contact_count + 2.0); // -1+3
  }

  // calculate VF rotation
    // r_x
  KDL::Vector r_x = frame_vec[i_thumb].p - frame_vec[0].p;
  r_x = r_x / r_x.Norm();
    // r_z
  KDL::Vector p_mid; // average all except first and last contacts
  for(int ci=1; ci < i_thumb; ci++)
     p_mid += frame_vec[ci].p/(contact_count-2);

  KDL::Vector r_z = (p_mid - frame_vec[0].p) * r_x;
  r_z = r_z / r_z.Norm();
    // r_y
  KDL::Vector r_y = r_z * r_x;
    // create the rotation matrix
  vf.M = KDL::Rotation(r_x, r_y, r_z);

  return vf;
}
