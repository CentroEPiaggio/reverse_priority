#include "reverse_priority_step/reverse_priority_step.h"
#include "reverse_priority_step/pchip.h"
#include "trajectory_generation/TrajectoryGeneration.h"
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <fstream>



void generate_slerp(geometry_msgs::Quaternion q_in,geometry_msgs::Quaternion q_fin,double step, std::vector<geometry_msgs::Quaternion> &q_int) 
{

	tf::Quaternion quat_0(q_in.x,q_in.y,q_in.z,q_in.w);
	tf::Quaternion quat_1(q_fin.x,q_fin.y,q_fin.z,q_fin.w);
	tf::Quaternion inter;


	for(double j=0;j<1;j+=step)
	{
		inter=quat_0.slerp(quat_1,j);
		inter.normalize();
		geometry_msgs::Quaternion q_msg;
		q_msg.w = inter.w();
		q_msg.x = inter.x();
		q_msg.y = inter.y();
		q_msg.z = inter.z();

		q_int.push_back(q_msg);
	}
};

void Kinematics(Eigen::VectorXd q, ReversePriorityTask &task,state &x_)
{
	double q1 = q(0);
	double q2 = q(1);
	double q3 = q(2);
	double q4 = q(3);
	double q5 = q(4);
	double q6 = q(5);
	double q7 = q(6);
    

    Eigen::MatrixXd Jacobian_full;
  	Jacobian_full.resize(6,7);
	
    
    x_.cart.p(0) = (79*cos(q1)*sin(q2))/250 - (33*sin(q1)*sin(q3))/400 - (71*cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))/500 + (13*cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))))/100 + (11*sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))/125 - (11*cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/125 - (71*sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/500 - (13*sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/100 + (33*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)))/400 + (48*sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)))/125 + (33*cos(q1)*cos(q2)*cos(q3))/400 + (48*cos(q1)*cos(q4)*sin(q2))/125 - (33*cos(q1)*sin(q2)*sin(q4))/400;
    x_.cart.p(1) = (33*cos(q1)*sin(q3))/400 + (79*sin(q1)*sin(q2))/250 + (71*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))/500 - (11*sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))/125 - (13*cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))))/100 + (11*cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/125 + (71*sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/500 + (13*sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/100 - (33*cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))/400 - (48*sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))/125 - (33*sin(q1)*sin(q2)*sin(q4))/400 + (33*cos(q2)*cos(q3)*sin(q1))/400 + (48*cos(q4)*sin(q1)*sin(q2))/125;
    x_.cart.p(2) = (79*cos(q2))/250 + (48*cos(q2)*cos(q4))/125 - (33*cos(q3)*sin(q2))/400 - (33*cos(q2)*sin(q4))/400 + (11*cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/125 + (71*sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/500 + (13*sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)))/100 + (13*cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))/100 - (71*cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/500 + (11*sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/125 + (33*cos(q3)*cos(q4)*sin(q2))/400 + (48*cos(q3)*sin(q2)*sin(q4))/125 + 0.3330;
    
    Eigen::Matrix3d R;

    R.row(0) << cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))), - sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))) - cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))), - cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)));
    R.row(1) << sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) - cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))), sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))) + cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))), cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)));
    R.row(2) << sin(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)) + cos(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))), cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)) - sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))), sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) - cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4));

    x_.cart.M = KDL::Rotation(R(0,0),R(1,0),R(2,0),R(0,1),R(1,1),R(2,1),R(0,2),R(1,2),R(2,2));
    Eigen::Quaterniond q_cart;
    q_cart =  R;
    // std::cout << q_cart.x() << std::endl;
    // std::cout << q_cart.y() << std::endl;
    // std::cout << q_cart.z() << std::endl;
    // std::cout << q_cart.w() << std::endl;
    // getchar();
	task.Task_Jacobian_.resize(task.task_dimension_,7);

	Jacobian_full.row(0) <<  (11*sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))/125 - (79*sin(q1)*sin(q2))/250 - (71*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))/500 - (33*cos(q1)*sin(q3))/400 + (13*cos(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))))/100 - (11*cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/125 - (71*sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/500 - (13*sin(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/100 + (33*cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))/400 + (48*sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))/125 + (33*sin(q1)*sin(q2)*sin(q4))/400 - (33*cos(q2)*cos(q3)*sin(q1))/400 - (48*cos(q4)*sin(q1)*sin(q2))/125, (79*cos(q1)*cos(q2))/250 + (13*cos(q7)*(cos(q1)*cos(q6)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q5)*sin(q4) - cos(q3)*cos(q4)*cos(q5)*sin(q2)) + cos(q1)*sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))/100 + (11*cos(q1)*cos(q6)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q5)*sin(q4) - cos(q3)*cos(q4)*cos(q5)*sin(q2)))/125 + (71*cos(q1)*sin(q6)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q5)*sin(q4) - cos(q3)*cos(q4)*cos(q5)*sin(q2)))/500 - (13*cos(q1)*sin(q7)*(cos(q5)*sin(q2)*sin(q3) - cos(q2)*sin(q4)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5)))/100 - (71*cos(q1)*cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/500 + (11*cos(q1)*sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/125 + (48*cos(q1)*cos(q2)*cos(q4))/125 - (33*cos(q1)*cos(q3)*sin(q2))/400 - (33*cos(q1)*cos(q2)*sin(q4))/400 + (33*cos(q1)*cos(q3)*cos(q4)*sin(q2))/400 + (48*cos(q1)*cos(q3)*sin(q2)*sin(q4))/125, (33*cos(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))/400 - (33*cos(q3)*sin(q1))/400 + (48*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))/125 + (11*cos(q6)*(sin(q5)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q4)*cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/125 + (71*sin(q6)*(sin(q5)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q4)*cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/500 - (13*sin(q7)*(cos(q5)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q4)*sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/100 + (13*cos(q7)*(cos(q6)*(sin(q5)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q4)*cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))) + sin(q4)*sin(q6)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/100 - (71*cos(q6)*sin(q4)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))/500 + (11*sin(q4)*sin(q6)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))/125 - (33*cos(q1)*cos(q2)*sin(q3))/400, (11*sin(q6)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)))/125 - (71*cos(q6)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)))/500 + (48*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)))/125 - (33*sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)))/400 + (13*cos(q7)*(sin(q6)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + cos(q5)*cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2))))/100 + (71*cos(q5)*sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))/500 + (13*sin(q5)*sin(q7)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))/100 - (33*cos(q1)*cos(q4)*sin(q2))/400 - (48*cos(q1)*sin(q2)*sin(q4))/125 + (11*cos(q5)*cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))/125, (11*cos(q6)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/125 + (71*sin(q6)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/500 - (13*sin(q7)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/100 + (13*cos(q6)*cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/100, (11*cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))/125 + (13*cos(q7)*(cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))))/100 + (71*sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))/500 - (71*cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/500 + (11*sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/125, - (13*sin(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))))/100 - (13*cos(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/100;
	Jacobian_full.row(1) <<  (79*cos(q1)*sin(q2))/250 - (33*sin(q1)*sin(q3))/400 - (71*cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))/500 + (13*cos(q7)*(sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)))))/100 + (11*sin(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)))/125 - (11*cos(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/125 - (71*sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/500 - (13*sin(q7)*(sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3))))/100 + (33*cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)))/400 + (48*sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)))/125 + (33*cos(q1)*cos(q2)*cos(q3))/400 + (48*cos(q1)*cos(q4)*sin(q2))/125 - (33*cos(q1)*sin(q2)*sin(q4))/400, (79*cos(q2)*sin(q1))/250 + (13*cos(q7)*(cos(q6)*sin(q1)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q5)*sin(q4) - cos(q3)*cos(q4)*cos(q5)*sin(q2)) + sin(q1)*sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))/100 + (11*cos(q6)*sin(q1)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q5)*sin(q4) - cos(q3)*cos(q4)*cos(q5)*sin(q2)))/125 + (71*sin(q1)*sin(q6)*(sin(q2)*sin(q3)*sin(q5) + cos(q2)*cos(q5)*sin(q4) - cos(q3)*cos(q4)*cos(q5)*sin(q2)))/500 - (13*sin(q1)*sin(q7)*(cos(q5)*sin(q2)*sin(q3) - cos(q2)*sin(q4)*sin(q5) + cos(q3)*cos(q4)*sin(q2)*sin(q5)))/100 - (71*cos(q6)*sin(q1)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/500 + (11*sin(q1)*sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/125 + (48*cos(q2)*cos(q4)*sin(q1))/125 - (33*cos(q3)*sin(q1)*sin(q2))/400 - (33*cos(q2)*sin(q1)*sin(q4))/400 + (33*cos(q3)*cos(q4)*sin(q1)*sin(q2))/400 + (48*cos(q3)*sin(q1)*sin(q2)*sin(q4))/125, (33*cos(q1)*cos(q3))/400 - (33*cos(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))/400 - (48*sin(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))/125 - (11*cos(q6)*(sin(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/125 - (71*sin(q6)*(sin(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/500 + (13*sin(q7)*(cos(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + cos(q4)*sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/100 - (13*cos(q7)*(cos(q6)*(sin(q5)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))) + sin(q4)*sin(q6)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/100 + (71*cos(q6)*sin(q4)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))/500 - (11*sin(q4)*sin(q6)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))/125 - (33*cos(q2)*sin(q1)*sin(q3))/400, (71*cos(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)))/500 - (11*sin(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)))/125 - (48*cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))/125 + (33*sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)))/400 - (13*cos(q7)*(sin(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + cos(q5)*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2))))/100 - (48*sin(q1)*sin(q2)*sin(q4))/125 - (11*cos(q5)*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))/125 - (71*cos(q5)*sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))/500 - (13*sin(q5)*sin(q7)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))/100 - (33*cos(q4)*sin(q1)*sin(q2))/400, (13*sin(q7)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/100 - (71*sin(q6)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/500 - (11*cos(q6)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/125 - (13*cos(q6)*cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/100, (71*cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/500 - (71*sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))/500 - (13*cos(q7)*(cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))))/100 - (11*cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)))/125 - (11*sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/125, (13*sin(q7)*(sin(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) - cos(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)))))/100 + (13*cos(q7)*(sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3))))/100;
	Jacobian_full.row(2) <<  0, (33*sin(q2)*sin(q4))/400 - (33*cos(q2)*cos(q3))/400 - (48*cos(q4)*sin(q2))/125 - (79*sin(q2))/250 - (11*cos(q6)*(cos(q5)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) - cos(q2)*sin(q3)*sin(q5)))/125 - (71*sin(q6)*(cos(q5)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) - cos(q2)*sin(q3)*sin(q5)))/500 - (13*sin(q7)*(sin(q5)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) + cos(q2)*cos(q5)*sin(q3)))/100 - (13*cos(q7)*(cos(q6)*(cos(q5)*(sin(q2)*sin(q4) + cos(q2)*cos(q3)*cos(q4)) - cos(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q4)*sin(q2) - cos(q2)*cos(q3)*sin(q4))))/100 + (71*cos(q6)*(cos(q4)*sin(q2) - cos(q2)*cos(q3)*sin(q4)))/500 - (11*sin(q6)*(cos(q4)*sin(q2) - cos(q2)*cos(q3)*sin(q4)))/125 + (33*cos(q2)*cos(q3)*cos(q4))/400 + (48*cos(q2)*cos(q3)*sin(q4))/125, (sin(q2)*(165*sin(q3) - 165*cos(q4)*sin(q3) - 768*sin(q3)*sin(q4) - 176*sin(q3)*sin(q4)*sin(q6) + 176*cos(q3)*cos(q6)*sin(q5) - 260*cos(q3)*cos(q5)*sin(q7) + 284*cos(q6)*sin(q3)*sin(q4) + 284*cos(q3)*sin(q5)*sin(q6) + 176*cos(q4)*cos(q5)*cos(q6)*sin(q3) + 260*cos(q3)*cos(q6)*cos(q7)*sin(q5) + 284*cos(q4)*cos(q5)*sin(q3)*sin(q6) + 260*cos(q4)*sin(q3)*sin(q5)*sin(q7) - 260*cos(q7)*sin(q3)*sin(q4)*sin(q6) + 260*cos(q4)*cos(q5)*cos(q6)*cos(q7)*sin(q3)))/2000, (71*cos(q6)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)))/500 - (48*cos(q2)*sin(q4))/125 - (33*cos(q2)*cos(q4))/400 - (11*sin(q6)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)))/125 - (13*cos(q7)*(sin(q6)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))/100 + (11*cos(q5)*cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/125 + (71*cos(q5)*sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/500 + (13*sin(q5)*sin(q7)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/100 + (48*cos(q3)*cos(q4)*sin(q2))/125 - (33*cos(q3)*sin(q2)*sin(q4))/400, (13*sin(q7)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/100 - (71*sin(q6)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)))/500 - (11*cos(q6)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)))/125 - (13*cos(q6)*cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)))/100, (71*cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/500 - (11*sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)))/125 - (13*cos(q7)*(sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) - cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))/100 + (11*cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/125 + (71*sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4)))/500, (13*cos(q7)*(sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3)))/100 - (13*sin(q7)*(cos(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) + sin(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4))))/100;
	Jacobian_full.row(3) <<  0, -sin(q1), cos(q1)*sin(q2), cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3), sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2), cos(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)) - sin(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)), - cos(q6)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) + cos(q1)*cos(q4)*sin(q2)) - sin(q6)*(cos(q5)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q2)*cos(q3)) - cos(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q3)*sin(q1) + cos(q1)*cos(q2)*sin(q3)));
	Jacobian_full.row(4) <<  0, cos(q1), sin(q1)*sin(q2), cos(q2)*sin(q1)*sin(q3) - cos(q1)*cos(q3), cos(q4)*sin(q1)*sin(q2) - sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)), sin(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) - cos(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)), cos(q6)*(sin(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) - cos(q4)*sin(q1)*sin(q2)) + sin(q6)*(cos(q5)*(cos(q4)*(cos(q1)*sin(q3) + cos(q2)*cos(q3)*sin(q1)) + sin(q1)*sin(q2)*sin(q4)) + sin(q5)*(cos(q1)*cos(q3) - cos(q2)*sin(q1)*sin(q3)));
	Jacobian_full.row(5) <<  1, 0, cos(q2), -sin(q2)*sin(q3), cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4), sin(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) - cos(q5)*sin(q2)*sin(q3), sin(q6)*(cos(q5)*(cos(q2)*sin(q4) - cos(q3)*cos(q4)*sin(q2)) + sin(q2)*sin(q3)*sin(q5)) - cos(q6)*(cos(q2)*cos(q4) + cos(q3)*sin(q2)*sin(q4));

	if(task.task_dimension_ == 6) 
	{
		task.Task_Jacobian_ = Jacobian_full;
		return;
	}
	else
	{
		int i = 0;
		for(auto& it : task.jacobian_row_sel_) {
			task.Task_Jacobian_.row(i) = Jacobian_full.row(it);
			i++;
		}
		return;
	}
}
typedef Eigen::Matrix< double, 7, 1 > 	Vector7d;
int main(int argc, char **argv)
{
	
    ros::init(argc, argv, "Path_Optimization_Client");
    ros::NodeHandle n_;
// Joint Limits Tasks

        Eigen::MatrixXd J7, J6, J5,J4,J3,J2,J1;
	J7.resize(1,7);
	J6.resize(1,7);
	J5.resize(1,7);
	J4.resize(1,7);
	J3.resize(1,7);
	J2.resize(1,7);
	J1.resize(1,7);
	
    J3 << 0,0,0,0,0,0,0;
    J4 << 0,0,0,0,0,0,0;
    J5 << 0,0,0,0,0,0,0;
    J6 << 0,0,0,0,0,0,0;
    J7 << 0,0,0,0,0,0,0;
    J1 << 0,0,0,0,0,0,0;
    J2 << 0,0,0,0,0,0,0;

    J7(6) = 1;
    J6(5) = 1;
    J5(4) = 1;
    J4(3) = 1;
    J3(2) = 1;
    J2(1) = 1;
    J1(0) = 1;

	
  ReversePriorityTask task1;
	task1.unil_cons_ = 1;
	task1.is_cart_ = false;
	task1.Task_Jacobian_.resize(1,7);
	task1.Task_Jacobian_ = J7;
	task1.cons_ = 2.8973;
	

	ReversePriorityTask task2;
	task2.unil_cons_ = 2;
	task2.is_cart_ = false;
	task2.Task_Jacobian_.resize(1,7);
	task2.Task_Jacobian_ = J7;
	task2.cons_ = -2.8973;


//Joint 6
	ReversePriorityTask task3;
	task3.unil_cons_ = 1;
	task3.is_cart_ = false;
	task3.Task_Jacobian_.resize(1,7);
	task3.Task_Jacobian_ = J6;
	task3.cons_ = 3.7525;


	ReversePriorityTask task4;
	task4.unil_cons_ = 2;
	task4.is_cart_ = false;
	task4.Task_Jacobian_.resize(1,7);
	task4.Task_Jacobian_ = J6;
	task4.cons_ =  -0.0175;

//Joint 5

	ReversePriorityTask task5;
	task5.unil_cons_ = 1;
	task5.is_cart_ = false;
	task5.Task_Jacobian_.resize(1,7);
	task5.Task_Jacobian_ = J5;
	task5.cons_ = 2.8973;


	ReversePriorityTask task6;
	task6.unil_cons_ = 2;
	task6.is_cart_ = false;
	task6.Task_Jacobian_.resize(1,7);
	task6.Task_Jacobian_ = J5;
	task6.cons_ = -2.8973;

//Joint 4

	ReversePriorityTask task7;
	task7.unil_cons_ = 1;
	task7.is_cart_ = false;
	task7.Task_Jacobian_.resize(1,7);
	task7.Task_Jacobian_ = J4;
	task7.cons_ = -0.0698;

	ReversePriorityTask task8;
	task8.unil_cons_ = 2;
	task8.is_cart_ = false;
	task8.Task_Jacobian_.resize(1,7);
	task8.Task_Jacobian_ = J4;
	task8.cons_ = -3.0718;


//Joint 3

	ReversePriorityTask task9;
	task9.unil_cons_ = 1;
	task9.is_cart_ = false;
	task9.Task_Jacobian_.resize(1,7);
	task9.Task_Jacobian_ = J3;
	task9.cons_ = 2.8973;

	ReversePriorityTask task10;
	task10.unil_cons_ = 2;
	task10.is_cart_ = false;
	task10.Task_Jacobian_.resize(1,7);
	task10.Task_Jacobian_ = J3;
	task10.cons_ = -2.8973;

//Joint 2

	ReversePriorityTask task11;
	task11.unil_cons_ = 1;
	task11.is_cart_ = false;
	task11.Task_Jacobian_.resize(1,7);
	task11.Task_Jacobian_ = J2;
	task11.cons_ = 1.7628;

	ReversePriorityTask task12;
	task12.unil_cons_ = 2;
	task12.is_cart_ = false;
	task12.Task_Jacobian_.resize(1,7);
	task12.Task_Jacobian_ = J2;
	task12.cons_ = -1.7628;



//Joint 1
	ReversePriorityTask task13;
	task13.unil_cons_ = 1;
	task13.is_cart_ = false;
	task13.Task_Jacobian_.resize(1,7);
	task13.Task_Jacobian_ = J1;
	task13.cons_ = 2.8973;

	ReversePriorityTask task14;
	task14.unil_cons_ = 2;
	task14.is_cart_ = false;
	task14.Task_Jacobian_.resize(1,7);
	task14.Task_Jacobian_ = J1;
	task14.cons_ = -2.8973;


	std::vector<ReversePriorityTask> task;
	task.resize(15);
    task.at(0) = (task1);
    task.at(1) = (task2);
    task.at(2) = (task3);
    task.at(3) = (task4);
    task.at(4) = (task5);
    task.at(5) = (task6);
    task.at(6) = (task7);
    task.at(7) = (task8);
    task.at(8) = (task9);
    task.at(9) = (task10);
    task.at(10) = (task11);
    task.at(11) = (task12);
    task.at(12) = (task13);
    task.at(13) = (task14);

// Generate Trajectory

	double t_tot = 3;
	double Ts = 0.02;
    geometry_msgs::Quaternion or_way_points[3];
    geometry_msgs::Point pos_way_points[8];

    or_way_points[0].x = 1;
    or_way_points[0].y = 0;
    or_way_points[0].z = 0;
    or_way_points[0].w = 0;

    or_way_points[1].x = 1;
    or_way_points[1].y = 0;
    or_way_points[1].z = 0;
    or_way_points[1].w = 0;
                          

    pos_way_points[0].x = 0.6845;
    pos_way_points[0].y = 0;
    pos_way_points[0].z = 0.5895;

    pos_way_points[1].x = 0.6845;
    pos_way_points[1].y = 0;
    pos_way_points[1].z = 0.3895;

    pos_way_points[2].x = 0.6845;
    pos_way_points[2].y = -0.1000;
    pos_way_points[2].z =  0.3895;
             
 
    pos_way_points[3].x =  0.8345;
    pos_way_points[3].y = -0.1000;
    pos_way_points[3].z = 0.3895;


 
    pos_way_points[4].x =  0.8345;
    pos_way_points[4].y = -0.1000+0.2;
    pos_way_points[4].z = 0.3895;

    pos_way_points[5].x =  0.8345-0.15;
    pos_way_points[5].y = -0.1000+0.2;
    pos_way_points[5].z = 0.3895;
    
    pos_way_points[6].x =  0.8345-0.15;
    pos_way_points[6].y = -0.1000+0.2-0.1;
    pos_way_points[6].z = 0.3895;


    pos_way_points[7].x = 0.6845;
    pos_way_points[7].y = 0;
    pos_way_points[7].z = 0.5895;

    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<trajectory_generation::TrajectoryGeneration>("trajectory_generation_server");

    trajectory_generation::TrajectoryGeneration srv;

    srv.request.pos_way_points.push_back(pos_way_points[0]);
    srv.request.pos_way_points.push_back(pos_way_points[1]);
    srv.request.pos_way_points.push_back(pos_way_points[2]);
    srv.request.pos_way_points.push_back(pos_way_points[3]);
    srv.request.pos_way_points.push_back(pos_way_points[4]);
    srv.request.pos_way_points.push_back(pos_way_points[5]);
    srv.request.pos_way_points.push_back(pos_way_points[6]);
    srv.request.pos_way_points.push_back(pos_way_points[7]);


    srv.request.or_way_points.push_back(or_way_points[0]);
    srv.request.or_way_points.push_back(or_way_points[1]);
    
    srv.request.Ts = Ts;
    srv.request.time = t_tot;

    client.call(srv);

    std::vector<Vector7d> q;
    double pi =  3.141592653589793;
    
    Vector7d q_0;
    q_0 << 0, 0, 0, -pi/2, 0, pi/2, 0;
    q.push_back(q_0);

	ReversePriorityTask task15;
	task15.unil_cons_ = 0;
	task15.is_cart_ = true;
	task15.task_dimension_ = 6;

	task15.jacobian_row_sel_.push_back(0);
	task15.jacobian_row_sel_.push_back(1);
	task15.jacobian_row_sel_.push_back(2);
	task15.jacobian_row_sel_.push_back(3);
	task15.jacobian_row_sel_.push_back(4);
	task15.jacobian_row_sel_.push_back(5);
	task15.jacobian_row_sel_.push_back(6);
   

    
    state s1;
    s1.cons = q_0(6);
    state s2;
    s2.cons = q_0(5);
    state s3;
    s3.cons = q_0(4);
    state s4;
    s4.cons = -q_0(3);
    state s5;
    s5.cons = q_0(2);
    state s6;
    s6.cons = q_0(1);
    state s7;
    s7.cons = q_0(0);

    

    std::vector<state> x_cur;
    x_cur.resize(15);
    x_cur.push_back(s1);
    x_cur.push_back(s1);
    x_cur.push_back(s2);
    x_cur.push_back(s2);
    x_cur.push_back(s3);
    x_cur.push_back(s3);
    x_cur.push_back(s4);
    x_cur.push_back(s4);
    x_cur.push_back(s5);
    x_cur.push_back(s5);
    x_cur.push_back(s6);
    x_cur.push_back(s6);
    x_cur.push_back(s7);
    x_cur.push_back(s7);

    // std::ofstream myfile ("//home/alessandro/joint_traj.txt");
    // std::ofstream myfilevel ("//home/alessandro/joint_vel.txt");

    trajectory_msgs::JointTrajectory traj;
    trajectory_msgs::JointTrajectoryPoint point;
    
    point.positions.resize(7);

    int num = 0;
    
    for(auto it=0;it<7;it++)
    {
      point.positions.at(it) = q.at(num)[it];
    }
    traj.points.push_back(point);

    // myfile << q.at(0)[0] << " " <<  q.at(0)[1]  << " " << q.at(0)[2]  << " " <<  q.at(0)[3]  << " " << q.at(0)[4]<< " " << q.at(0)[5]  << " " <<  q.at(0)[6] << "\n";
    state s8;
   	Eigen::VectorXd qd;
   	qd.resize(7);
    auto it_end = srv.response.trajectory.rbegin(); it_end++;
    

 

    for(auto it = srv.response.trajectory.begin(); it != it_end.base(); ++it)
    {
    	geometry_msgs::Pose p_des = *(it+1);
    	geometry_msgs::Pose p_old = *it;
    	Kinematics(q.at(num),task15,s8);
        Vector7d q_old;
        q_old = q.at(num);
    	task15.target_des_.x_des_ = KDL::Frame(
                    KDL::Rotation::Quaternion(p_des.orientation.x,
                    						   p_des.orientation.y,
                    						        p_des.orientation.z,
                    						        p_des.orientation.w),
                    KDL::Vector(p_des.position.x,
   								p_des.position.y,
   								p_des.position.z));
    	KDL::Frame x_old_ = KDL::Frame(
                    KDL::Rotation::Quaternion(p_old.orientation.x,
                    						   p_old.orientation.y,
                    						        p_old.orientation.z,
                    						        p_old.orientation.w),
                    KDL::Vector(p_old.position.x,
   								p_old.position.y,
   								p_old.position.z));
    	task15.target_des_.x_dot_des_ = diff(x_old_, task15.target_des_.x_des_,Ts);
        
	    task.at(14) = task15;
        

        s7.cons = q_old(6);
	    s6.cons = q_old(5);
	    s5.cons = q_old(4);
	    s4.cons = q_old(3);
	    s3.cons = q_old(2);
	    s2.cons = q_old(1);
	    s1.cons = q_old(0);


	    x_cur.at(0) = s7;
	    x_cur.at(1) = s7;
	    x_cur.at(2) = s6;
	    x_cur.at(3) = s6;
	    x_cur.at(4) = s5;
	    x_cur.at(5) = s5;
	    x_cur.at(6) = s4;
	    x_cur.at(7) = s4;
	    x_cur.at(8) = s3;
	    x_cur.at(9) = s3;
	    x_cur.at(10) = s2;
	    x_cur.at(11) = s2;
	    x_cur.at(12) = s1;
	    x_cur.at(13) = s1;
    	x_cur.at(14) = s8;

    	ReversePriorityStep step;
    	Eigen::VectorXd qd_RP_last = qd;
		qd = step.reverseprioritystep(task, qd_RP_last, x_cur);
		Vector7d q_now = q.at(num)+qd*Ts;
		q.push_back(q_now);
     
	    // myfilevel << qd[0] << " " <<  qd[1]  << " " << qd[2]  << " " <<  qd[3]  << " " << qd[4]<< " " << qd[5]  << " " <<  qd[6] << "\n";
		num++; 
	    // myfile << q.at(num)[0] << " " <<  q.at(num)[1]  << " " << q.at(num)[2]  << " " <<  q.at(num)[3]  << " " << q.at(num)[4]<< " " << q.at(num)[5]  << " " <<  q.at(num)[6] << "\n";
    for(int it=0;it<7;it++)
    {
      point.positions.at(it) = q.at(num)[it];
    }
    traj.points.push_back(point);
    
    }
        ROS_INFO("Here");
    ros::Rate r(50);
    ros::Publisher traj_pub = n_.advertise<trajectory_msgs::JointTrajectory>("/panda_command_aux", 1);
    // std::vector<double> x_w,t_w,t,z_w,y_w,z,x,y;
      for(auto it=0;it<traj.points.size();it++){
        trajectory_msgs::JointTrajectory traj_point;
        traj_point.points.push_back(traj.points.at(it));
        traj_pub.publish(traj_point);
        r.sleep();
        ROS_INFO("Here");
      }
       while(ros::ok()) {

    }
    // x_w.push_back(0.6845);
    // x_w.push_back(0.6845);
    // x_w.push_back(0.6845);
    // x_w.push_back(0.8345);
    // x_w.push_back(0.8345);
    // x_w.push_back(0.6845);
    // x_w.push_back(0.6845);
    // x_w.push_back(0.6845);
    
    // double t_tot = 3;
    // for(int kk=0;kk<x_w.size();kk++)
    // {
   	// 	t_w.push_back(kk*t_tot/(x_w.size()-1));
   	// 	std::cout << t_w.at(kk) << std::endl;
   	// 	getchar();
    // }
    
    // int kkk= 0;
    // double Ts = 0.02;
    // for(double i = 0;i<t_tot+Ts;i+=Ts){
    //  	t.push_back(i);
    //  	std::cout << t.at(kkk) << std::endl;
    //  	kkk++;
    //  } 
    // getchar();
    // pchip(t_w,x_w,t,x);

    // getchar();

//     std::vector<geometry_msgs::Quaternion> quat_vec (or_way_points, or_way_points + sizeof(or_way_points) / sizeof(geometry_msgs::Quaternion));

//   	int count = 0;
//     std::vector<geometry_msgs::Quaternion> quat;
// 	double step_or = t_tot/(Ts*(quat_vec.size()-1));
// 	step_or = 1/step_or;
// 	count = 0;
// 	generate_slerp(quat_vec.at(0),quat_vec.at(1),step_or,quat);

//     for (std::vector<geometry_msgs::Quaternion>::iterator it = quat_vec.begin()+1; it != quat_vec.end()-1; ++it) {
    	
//   	    std::vector<geometry_msgs::Quaternion> quat_aux;

//     	generate_slerp(*it,*(it+1),step_or,quat_aux);
//     	int stride= (count>0)?1:0;
//     	quat.insert(
//       quat.end(),
//       std::make_move_iterator(quat_aux.begin()+stride),
//       std::make_move_iterator(quat_aux.end())

//    	);//Joint 7
//     	count ++;
// }



//     for (std::vector<geometry_msgs::Quaternion>::iterator it = quat.begin(); it != quat.end(); ++it) {
//     	  	    geometry_msgs::Quaternion quat_aux_ = *it;
//     	  	    std::cout << quat_aux_.x << " " << std::endl;
//     	  	    std::cout << quat_aux_.y << " " << std::endl;
//     	  	    std::cout << quat_aux_.z << " " << std::endl;
//     	  	    std::cout << quat_aux_.w << " " << std::endl;
// }
//     getchar();


//

	// task15.Task_Jacobian_.resize(6,7);
	// task15.Task_Jacobian_ <<  0  , 0.2565 ,        0 ,   0.0595 ,        0 ,   0.1420 ,        0,
 //    0.6845   ,      0   , 0.6845    ,     0  ,  0.1420    ,     0 ,  -0.1300,
 //         0  , -0.6845     ,    0  ,  0.6020   ,      0  ,  0.2180  ,       0,
 //         0      ,   0    ,     0     ,    0 ,   1.0000     ,    0    ,     0,
 //         0,    1.0000  ,       0 ,  -1.0000  ,       0  , -1.0000 ,        0,
 //    1.0000     ,    0 ,   1.0000  ,       0 ,   0.0000 ,        0 ,  -1.0000;


 //    KDL::Frame x_;

	// x_ = KDL::Frame(
 //                    KDL::Rotation::Quaternion(-0.998750260394966,
 //                    						   -0.049979169270678,
 //                    						        0,
 //                    						        0),
 //                    KDL::Vector(   0.681080351132809,
 //   0.068335973694754,
 //   0.589500000000000));
	// task15.target_des_.x_des_ = x_;



 //     = KDL::Frame(
 //                    KDL::Rotation::Quaternion(1,
 //                    						  0,
 //                    						        0,
 //                    						        0),
 //                    KDL::Vector(0.6845,
 //         						0,
 //    							0.5895));

 //    KDL::Twist x_dot_ = 

 //    task15.target_des_.x_dot_des_ = x_dot_;

    
    
	// std::cout << qd << std::endl;
	return 0;
}