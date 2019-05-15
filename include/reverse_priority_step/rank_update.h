#ifndef RANK_UPDATE_H
#define RANK_UPDATE_H

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/SVD>
#include "matrix_rank.h"

inline void rank_update(const Eigen::MatrixXd &J_, const Eigen::MatrixXd &J_Aug_pinv_,Eigen::MatrixXd &T)
{ 

  double dim_J = J_.rows();
  double dim_Ja_pinv_ = J_Aug_pinv_.cols();

  T.resize(J_Aug_pinv_.rows(),1);
  T << J_Aug_pinv_.col(0);

  double j=1;
  double i = 1;
  double rank = 0;
  while(i<=dim_J-1)
  {
    while(j<=dim_Ja_pinv_-1)
    {
        T.conservativeResize(T.rows(), T.cols()+1);
          T.col(T.cols()-1) = J_Aug_pinv_.col(i);
          j++;
          matrix_rank(T,rank);
          if(rank==i+1)
          {
            i++;
            break;
          }
          else
          {
          T.conservativeResize(T.rows(), T.cols()-1);
          }
    }
  }
  
}

#endif