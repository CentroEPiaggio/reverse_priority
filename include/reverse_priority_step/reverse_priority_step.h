#ifndef REVERSEPRIORITYSTEP_H
#define REVERSEPRIORITYSTEP_H

#include "reverse_priority_task.h"

#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <sstream>
#include <iostream>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <ros/topic.h>
#include <Eigen/Dense>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <eigen_conversions/eigen_kdl.h>



class ReversePriorityStep
  {
  public:
    ReversePriorityStep();
    ~ReversePriorityStep();
    // void init_param(double lambda,double beta_pos,double beta_vel,double K_p,double K_o,double K_j);
    Eigen::VectorXd reverseprioritystep(std::vector<ReversePriorityTask> Tasks, Eigen::VectorXd qd_RP_last, std::vector<state> x_cur, double lambda = 0.9);
  private:
    double K_o;
    double K_p;
    double K_j;
    double lambda;
    double beta_pos;
    double beta_vel;
    void compute_T_matrix(std::vector<ReversePriorityTask> Tasks, std::vector<EigenArray> &T);
    void variable_gain(Eigen::VectorXd err, double &alpha);

  };

#endif