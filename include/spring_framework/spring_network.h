// Copyright (C) 2020  Gokhan Solak, CRISP, Queen Mary University of London
// This code is licensed under the BSD 3-Clause license (see LICENSE for details)


#ifndef SPRING_NETWORK_H
#define SPRING_NETWORK_H

#include <ros/ros.h>

#include <spring_framework/virtual_spring.h>

#include <kdl/kdl.hpp>

#include <kdl_conversions/kdl_msg.h>

// messages
#include <spring_framework/SpringNetworkMsg.h>

using namespace std;

namespace spring_framework
{

// SpringNetwork class defines a spring structure.
// Each spring is defined as a pair of frame indices
// among which the spring is connected.
class SpringNetwork
{
  private:

    // a set of frames are used to define the springs.
    // typically fingertip frames, but can be anything.
    // these frames are then referred with their index.
    vector<KDL::Frame> frame_vec_;

    // set of indices indicate the spring network.
    // every two indices define one spring.
    // order matters; force will be applied on the first frame.
    vector<int> network_vec_;

    // virtual spring objects are stored in this vector
    vector<VirtualSpring> spring_vec_;

  public:

    SpringNetwork();
    ~SpringNetwork();

    // add existing spring to network
    void addSpring(VirtualSpring &spring);

    // create a spring and add to network
    void createSpring(const int fi_1, const int fi_2=0, const double l=0, const double k=0);

    // create multiple springs, defined by a list of pairs
    void createNetwork(const vector<int> &network_vec);

    // updates the frames
    void updateFrames(const vector<KDL::Frame> &frame_vec);

    // sets rest lengths to the current distances between end points
    void resetRestLengths(const double scaler = 1.0);
    void resetRestLength(const int spring_no, const double scaler = 1.0);

    // return a message equivalent to this network
    SpringNetworkMsg toMsg() const;

    // getters and setters
      // getters
    int getSize() const;
    void getSpring(const int spring_no, VirtualSpring &spring) const;
    double getRestLength(const int spring_no) const;
    vector<double> getStiffness() const;
    double getStiffness(const int spring_no) const;
    int getSpringFrame(const int spring_no, const int end=0) const;
    void getSpringForce(const int spring_no, KDL::Vector &f_out);
    void getSpringForce(const int spring_no, const int dir, KDL::Vector &f_out);
    vector<KDL::Frame> getFrames() const;
      // setters
    void setRestLength(const int spring_no, const double l_rest);
    void setRestLengths(const vector<double> l_rest_vec);
    void setStiffness(const double k);
    void setStiffness(const int spring_no, const double k);

};

} // end namespace spring_framework

#endif // SPRING_NETWORK_H
