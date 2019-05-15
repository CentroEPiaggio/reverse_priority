#ifndef REVERSEPRIORITYTASK_H
#define REVERSEPRIORITYTASK_H

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
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>


struct Target 
{
       Eigen::VectorXd q_des_;
       Eigen::VectorXd q_dot_des_;
       KDL::Frame x_des_;
       KDL::Twist x_dot_des_;
};


struct EigenArray 
{
    Eigen::MatrixXd matrix;
    Eigen::VectorXd vec;

};

struct state 
{
    Eigen::VectorXd joints;
    double cons;
    KDL::Frame cart;
};

class ReversePriorityTask	
	{
	public:
		ReversePriorityTask() {};
		~ReversePriorityTask() {};
        int task_dimension_ = 1;
        int unil_cons_;
        Eigen::MatrixXd Task_Jacobian_;
        double cons_;
        bool is_cart_;

        Target target_des_;
        std::vector<int> jacobian_row_sel_;
        int priority_;
	private:
	};



#endif