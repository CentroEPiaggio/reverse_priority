#include "reverse_priority_step/reverse_priority_step.h"
#include "reverse_priority_step/reverse_priority_task.h"
#include <reverse_priority_step/rank_update.h>
#include <reverse_priority_step/skew_symmetric.h>
#include <reverse_priority_step/smooth_function.h>
#include <reverse_priority_step/unilateral_constraint_activation.h>
#include <reverse_priority_step/pseudo_inversion.h>
#include <reverse_priority_step/cartesian_error.h>


 
ReversePriorityStep::ReversePriorityStep(){

// INIT PARAMETERS
      K_o = 0.5;
      K_p = 1;
      K_j = 1;
      lambda = 0.9;
      beta_pos = 0.01;  
      beta_vel = 0.1;

  };

ReversePriorityStep::~ReversePriorityStep(){};


void ReversePriorityStep::compute_T_matrix(std::vector<ReversePriorityTask> Tasks, std::vector<EigenArray> &T) 
    {
      

      Eigen::MatrixXd Pseudo_Jac;
      Eigen::MatrixXd J_aug;
      std::vector<EigenArray> Jra_p;
      Jra_p.resize(Tasks.size());
      
      int total_dim = Tasks.size()-1;
      
      int cols = (Tasks.at(total_dim).Task_Jacobian_.cols());
      int rows = (Tasks.at(total_dim).Task_Jacobian_.rows());
      
      J_aug.resize(rows,cols);
      J_aug = Tasks.at(total_dim).Task_Jacobian_;
      pseudo_inverse(J_aug,Pseudo_Jac);
      Jra_p.at(total_dim).matrix = Pseudo_Jac;
 

      Eigen::MatrixXd T_aux;
      rank_update(Tasks.at(total_dim).Task_Jacobian_,Pseudo_Jac,T_aux);
      T.at(total_dim).matrix = T_aux;
      
      for(int j=Tasks.size()-2; j>=0; j--) {
        
        //Temp Variables
        Eigen::MatrixXd Temp_Mat;
        Eigen::MatrixXd Aux_Mat = J_aug;

        // Append new Task Jacobian
        J_aug.resize(Aux_Mat.rows()+Tasks.at(j).Task_Jacobian_.rows(), Aux_Mat.cols());
        
        rows = (Tasks.at(j).Task_Jacobian_.rows());
        cols = (Tasks.at(j).Task_Jacobian_.cols());

        J_aug.block(0,0,rows,cols) = Tasks.at(j).Task_Jacobian_;
        J_aug.block(rows,0,Aux_Mat.rows(),Aux_Mat.cols()) = Aux_Mat;

        // Pseudo inverse of Augmented Jacobian;
        pseudo_inverse(J_aug,Temp_Mat);
        
        rank_update(Tasks.at(j).Task_Jacobian_,Temp_Mat,Aux_Mat);

        T.at(j).matrix= Aux_Mat;
    }

  }
    

    Eigen::VectorXd ReversePriorityStep::reverseprioritystep(std::vector<ReversePriorityTask> Tasks, Eigen::VectorXd qd_RP_last, std::vector<state> x_cur, double lambda) 
    {
      
      int alpha = 1;
      std::vector<EigenArray> T(Tasks.size());
      std::vector<EigenArray> x_dot_IK(Tasks.size());
      std::vector<EigenArray> qd_RP_(Tasks.size()+1);

      
      //Activation function
      std::vector<double> h(Tasks.size());
      qd_RP_.at(Tasks.size()).vec.resize(qd_RP_last.size());
      
      // Initial velocity 
      qd_RP_.at(Tasks.size()).vec = lambda*qd_RP_last;
      
      compute_T_matrix(Tasks, T);
      
      std::vector<EigenArray> err_eig_(Tasks.size());
      

      for(int j=Tasks.size()-1; j>=0; j--) 
      {

        int unil_cons =  Tasks.at(j).unil_cons_;
        Eigen::VectorXd q_j = qd_RP_.at(j+1).vec;
        Eigen::MatrixXd J = Tasks.at(j).Task_Jacobian_;
        int task_dim = Tasks.at(j).task_dimension_;
        Eigen::MatrixXd K_ = Eigen::MatrixXd::Identity(task_dim,task_dim);

        if(unil_cons)
        {

      
          
          x_dot_IK.at(j).vec.resize(qd_RP_last.size());
          
          Eigen::MatrixXd xd_lower_mat = J*qd_RP_.at(j+1).vec;
          double xd_lower = xd_lower_mat(0);
          
          h.at(j) = unilateral_constraint_activation(unil_cons, x_cur.at(j).cons, Tasks.at(j).cons_, xd_lower,beta_pos, beta_vel);

          err_eig_.at(j).vec.resize(1);
          err_eig_.at(j).vec << 0;

        }
        else 
        {
          
          h.at(j) = 1;
          
          

          if(Tasks.at(j).is_cart_) 
          {

            KDL::Frame x_ = x_cur.at(j).cart;
            KDL::Frame x_d = Tasks.at(j).target_des_.x_des_;

            Eigen::VectorXd err_;
            cart_err(x_,x_d,err_);
          
            err_eig_.at(j).vec.resize(task_dim);
            x_dot_IK.at(j).vec.resize(task_dim);

            for (int k=0; k<task_dim;k++) 
            {
              double sel = Tasks.at(j).jacobian_row_sel_.at(k);
              err_eig_.at(j).vec(k) = err_(sel);
            }
            
            alpha = (err_eig_.at(j).vec.norm()>0.1)?6:1;

            for (int k=0; k<task_dim;k++) 
            {

              double sel = Tasks.at(j).jacobian_row_sel_.at(k);
              
              if(sel <= 2) K_(k,k) = alpha*K_p;
              else K_(k,k) = alpha*K_o;
              
              x_dot_IK.at(j).vec(k) = Tasks.at(j).target_des_.x_dot_des_(k) +K_(k,k)*err_eig_.at(j).vec(k);

            }
          }

          else 
            {
              
              Eigen::VectorXd err_;
              err_.resize(task_dim);
              err_ = Tasks.at(j).target_des_.q_des_ - x_cur.at(j).joints;
              K_ = K_j*K_; 
              err_eig_.at(j).vec = err_;
             
              for (int k=0; k<task_dim;k++) 
              {

              x_dot_IK.at(j).vec(k) = Tasks.at(j).target_des_.q_dot_des_(k) +K_(k,k)*err_eig_.at(j).vec(k);

              }  
            }

        }

        MatrixXd pinv_aux;
        MatrixXd aux_prod = Tasks.at(j).Task_Jacobian_*T.at(j).matrix;

        pseudo_inverse(aux_prod,pinv_aux,false);

        qd_RP_.at(j).vec = qd_RP_.at(j+1).vec+h.at(j)*(T.at(j).matrix*pinv_aux)*(x_dot_IK.at(j).vec-J*qd_RP_.at(j+1).vec);
        

        /* CHECK FOR ERRORS
        if(qd_RP_.at(j).vec(0)>1000) {
          getchar();
          std::cout << j << std::endl;
          std::cout << h.at(j) << std::endl;
          std::cout << T.at(j).matrix << std::endl;
          std::cout << pinv_aux << std::endl;
          std::cout << J << std::endl;
          std::cout << qd_RP_.at(j+1).vec << std::endl;
          std::cout << x_dot_IK.at(j).vec << std::endl;
          getchar();
        }
        */
      }


      return qd_RP_.at(0).vec;
    
    };