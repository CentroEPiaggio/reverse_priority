// Author: Alessandro Palleschi
// rank() computes the rank of matrix M_ using SVD decomposition (can choose between damped and not)
// the rank of a matrix M_ is computed as the number of singular values that are larger than a tolerance.
// returns the matrix rank

#ifndef MATRIX_RANK_H
#define MATRIX_RANK_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
using namespace Eigen;

inline void matrix_rank(const Eigen::MatrixXd &M_, double &rank,double tol = 1e-15)
{	

	JacobiSVD<MatrixXd> svd(M_, ComputeFullU | ComputeFullV);

	JacobiSVD<MatrixXd>::SingularValuesType sing_vals_ = svd.singularValues();
  
    rank = 0;
    for (int i = 0; i < sing_vals_.size(); i++) 
    {
       if(sing_vals_(i) > tol) rank++;
       else break;
   }

}

#endif
