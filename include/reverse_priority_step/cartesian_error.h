// Author: Alessandro Palleschi
// cart_err() computes the cartesian error between two KDL Frames
// returns and Eigen::Vector
#ifndef CARTESIAN_ERROR_H
#define CARTESIAN_ERROR_H
#include <Eigen/Dense>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include <boost/scoped_ptr.hpp>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include "skew_symmetric.h"


  struct quaternion_
  {
    KDL::Vector v;
    double a;
  } quat_curr_, quat_des_;

void cart_err(KDL::Frame x_, KDL::Frame x_des_, Eigen::VectorXd &err_eigen)
{
            KDL::Twist err_ = diff(x_,x_des_);
            Eigen::Matrix<double, 3, 3> skew_;
           
            x_.M.GetQuaternion(quat_curr_.v(0), quat_curr_.v(1), quat_curr_.v(2), quat_curr_.a);

            x_des_.M.GetQuaternion(quat_des_.v(0), quat_des_.v(1), quat_des_.v(2), quat_des_.a);
            double sign_check = quat_des_.a*quat_curr_.a;
            for(int i=0;i<3;i++) sign_check+= quat_curr_.v(i)*quat_des_.v(i);

            if(sign_check < 0) {
            	quat_des_.a *= -1;
            	quat_des_.v = -quat_des_.v;
            }

            skew_symmetric(quat_des_.v, skew_);
            Eigen::Quaterniond quat_des_eig, quat_curr_eig;
            quat_des_eig.w() = quat_des_.a;
            quat_des_eig.vec() << quat_des_.v(0), quat_des_.v(1), quat_des_.v(2);
            quat_curr_eig.w() = quat_curr_.a;
            quat_curr_eig.vec() << quat_curr_.v(0), quat_curr_.v(1), quat_curr_.v(2);

            Eigen::Vector3d e_rot_ = (quat_curr_eig.w() * quat_des_eig.vec()) - (quat_des_eig.w() * quat_curr_eig.vec()) - (skew_ * quat_curr_eig.vec()); 
            
            err_(3) = e_rot_(0);
            err_(4) = e_rot_(1);
            err_(5) = e_rot_(2);

            err_eigen.resize(6);

            err_eigen << err_(0),err_(1),err_(2),err_(3),err_(4),err_(5);

}


#endif