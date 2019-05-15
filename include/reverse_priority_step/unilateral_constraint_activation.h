#ifndef UNIL_CONS_ACT_H
#define UNIL_CONS_ACT_H


#include "smooth_function.h"

double unilateral_constraint_activation(int unil_cons,double x_cur, double x_cons, double xd_lower,double beta_pos, double beta_vel)
{   
    double h_pos =0;
    double h_vel = 0;

    switch(unil_cons)
    {
        case 1:

            if (x_cur < (x_cons - beta_pos)) h_pos = 0;
            
            else if ((x_cur >= x_cons - beta_pos) && x_cur < x_cons) h_pos = smooth_function(((x_cur - x_cons) + beta_pos) / beta_pos);
            
            else h_pos = 1;
        
            if (xd_lower < -beta_vel ) h_vel = 0;
            
            else if (-beta_vel <=xd_lower && xd_lower < 0) h_vel = smooth_function((xd_lower + beta_vel) / beta_vel);
            
            else h_vel = 1;
        
        break;

        case 2:

            if (x_cur > (x_cons + beta_pos))  h_pos = 0;                                             
            
            else if (x_cons < x_cur && x_cur <= (x_cons + beta_pos)) h_pos = smooth_function(((x_cur - x_cons) - beta_pos) / -beta_pos);    
            
            else  h_pos = 1;                                             
        
        
            if (xd_lower > beta_vel) h_vel = 0;
            
            else if (xd_lower <= beta_vel && xd_lower > 0) h_vel = smooth_function((xd_lower - beta_vel) / -beta_vel);
            
            else h_vel = 1;
        break;

}
double h = h_pos*h_vel;

return h;
}

#endif