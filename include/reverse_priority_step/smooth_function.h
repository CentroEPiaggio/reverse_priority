#ifndef SMOOTH_FUNCTION_H
#define SMOOTH_FUNCTION_H


inline double smooth_function(const double a)
{	

double r = 6*pow(a,5) - 15*pow(a,4) + 10*pow(a,3);
return r;
}

#endif
