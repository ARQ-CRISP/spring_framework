# Virtual Spring Framework
_A ROS integrated virtual spring implementation_

Virtual Spring Framework package enables creating virtual springs between arbitrary frames to achieve impedance control. It uses KDL and Eigen libraries and works with ROS-Kinetic.

Spring definition is independent from the physical system of application. Some practical applications may be robot hand control and bimanual control.

## Installation
*Tested on Ubuntu 16.04, ROS-Kinetic*

* **Install Eigen3:** `sudo apt install libeigen3-dev`
* **Install KDL:** `sudo apt install ros-kinetic-orocos-kdl`
* **Install ROS packages:** Clone [*kdl_control_tools*](https://github.com/ARQ-CRISP/kdl_control_tools) and  [*spring_framework*](https://github.com/ARQ-CRISP/spring_framework) into `your-catkin-workspace/src` directory. Build the workspace with `catkin_make`.

## Usage

Spring framework can be used in different levels. On the lowest-level, you can create and use individual _virtual springs_. On the highest-level you can use a _spring server_ that creates a set of springs given an array specification, handles ROS communication to update the springs and publish forces applied by the springs.

On the lowest-level, you can create `VirtualSpring` objects to define individual springs between pairs of frames. A `VirtualSpring` provides an interface to set spring parameters, update its frames and calculate the force applied by the spring in Cartesian space.

```c++
spring_framework::VirtualSpring vs(rest_length, stiffness);
vs.updateFrames(frames); // frames.size() == 2
// calculate forces applied on both ends
KDL::Vector force_left, force_right;
vs.calcForce(0, force_left);
vs.calcForce(1, force_right);

```

You can define a network of springs using a `SpringNetwork` object. This class provides an interface to create a set of springs and update their frames as a batch. A spring network can be specified using a vector of frame pairs. Each frame is represented by its index in the frames vector. The following code shows the usage of a network of 3 springs defined between 3 frames:

```c++
// create network
spring_framework::SpringNetwork snetwork;
snetwork.createSpring(0, 1);
snetwork.createSpring(0, 2);
snetwork.createSpring(1, 2);
// set parameters
snetwork.setStiffness(s); // sets stiffness of all to s
snetwork.setStiffness(0, s0); // sets stiffness of 0th spring to s0
// update frames (ordered)
snetwork.updateFrames(frames); // frames.size() >= 3
// calculate current force of 2nd spring on its 0th frame
KDL::Vector f2; // output force
snetwork.getSpringForce(2, 0, f2);
```

The framework also contains the `SpringController` class, to use the virtual springs at the object-control level. A `SpringController` assumes that the 0th frame is the object frame, pose of which is approximated w.r.t. the other frames.


At the highest level, `SpringServer` runs a node in a separate thread, subscribes to default topics to update the spring frames, publishes the resulting forces periodically. It takes an existing `SpringNetwork` object at construction. Its execution can be controlled with start / stop methods. It allows specification of callback functions to customize updating behaviour.

```c++
// create the spring server using an existing network with 3 contact frames
sserver = new spring_framework::SpringServer(3, &snetwork);
// set a callback that is called at each update of spring server
function<void(const SpringServer*, const vector<KDL::Frame>&)> f = springUpdateCallback;
sserver->setUpdateEnterHandler(f);
// initiate the spring server node
sserver->start(node_handle);
```

A spring server optionally publishes the spring network information as a `SpringNetworkMsg` message. This message contains the latest info of each spring (K, L, F1, F2). By default it is published, but it can be avoided using the constructor argument or the `SpringServer::togglePublishSprings` method.
