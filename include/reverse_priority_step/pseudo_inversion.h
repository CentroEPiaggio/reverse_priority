// Author: Enrico Corvaglia
// pseudo_inverse() computes the pseudo inverse of matrix M_ using SVD decomposition (can choose between damped and not)
// returns the pseudo inverted matrix M_pinv_

#ifndef PSEUDO_INVERSION_H
#define PSEUDO_INVERSION_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
using namespace Eigen;

inline void pseudo_inverse(const Eigen::MatrixXd &M_, Eigen::MatrixXd &M_pinv_,bool damped = true)
{	

    double epsilon = 0.1;

	double lambda_max_ = damped?1000:0.0;
    double lambda_quad = 0.0;
	JacobiSVD<MatrixXd> svd(M_, ComputeFullU | ComputeFullV);

	JacobiSVD<MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();

	double minimum_sing_vals_ = sing_vals_(sing_vals_.size()-1); // take last element of the vector
	if(minimum_sing_vals_ < epsilon) {
		lambda_quad = ( 1 - (minimum_sing_vals_ / epsilon)* (minimum_sing_vals_ / epsilon) ) * (lambda_max_*lambda_max_);
	}
	


	MatrixXd S_ = M_;	// copying the dimensions of M_, its content is not needed.
	S_.setZero();

    for (int i = 0; i < sing_vals_.size(); i++)
        S_(i,i) = (sing_vals_(i))/(sing_vals_(i)*sing_vals_(i) + lambda_quad);

    M_pinv_ = MatrixXd(svd.matrixV()*S_.transpose()*svd.matrixU().transpose());
}

#endif
