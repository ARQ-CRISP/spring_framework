// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


#ifndef VIRTUAL_SPRING_H
#define VIRTUAL_SPRING_H

#include <ros/ros.h>

#include <kdl/kdl.hpp>

#include <kdl_conversions/kdl_msg.h>
#include <eigen_conversions/eigen_kdl.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// msgs
#include <spring_framework/VirtualSpringMsg.h>


using namespace std;

namespace spring_framework
{

// VirtualSpring defines a spring object between two cartesian frames.
// Behavior of the spring is characterized by the rest length and
// stiffness parameters.
class VirtualSpring
{
  private:

    // stiffness
    double k_;
    // rest length
    double l_;

    // state
    vector<KDL::Frame> frame_vec_;

  public:

    VirtualSpring();
    VirtualSpring(const double l_rest, const double k);
    VirtualSpring(const VirtualSpring &vs);
    ~VirtualSpring();

    // updates the frame poses on spring's ends
    void updateFrames(const vector<KDL::Frame> &frame_vec);

    // calculates the current force the spring applies
    // 'end' argument indicates the end frame index (0 or 1)
    // where the force is applied.
    void calcForce(const int end, KDL::Vector &f_out);

    // return a message equivalent to this spring
    VirtualSpringMsg toMsg() const;

    // getters and setters
      // getters
    double getRestLength() const;
    double getStiffness() const;
    vector<KDL::Frame> getFrames() const;
      // setters
    void setRestLength(const double l_rest);
    void setStiffness(const double k);
};

} // end namespace spring_framework

#endif // VIRTUAL_SPRING_H
