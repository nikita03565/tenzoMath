// x - array of size 2
// return 2: 2 real roots x[0], x[1]
// return 0: pair of complex roots: x[0]�i*x[1]
int SolveP2(double* x, double a, double b); // solve equation x^2 + a*x + b = 0

// x - array of size 3
// return 3: 3 real roots x[0], x[1], x[2]
// return 1: 1 real root x[0] and pair of complex roots: x[1]�i*x[2]
int SolveP3(double* x, double a, double b, double c); // solve cubic equation x^3 + a*x^2 + b*x + c = 0

// x - array of size 4
// return 4: 4 real roots x[0], x[1], x[2], x[3], possible multiple roots
// return 2: 2 real roots x[0], x[1] and complex x[2]�i*x[3], 
// return 0: two pair of complex roots: x[0]�i*x[1],  x[2]�i*x[3], 
int SolveP4(double* x, double a, double b, double c, double d);
// solve equation x^4 + a*x^3 + b*x^2 + c*x + d = 0 by Dekart-Euler method

//-----------------------------------------------------------------------------
// And some additional functions for internal use.
// Your may remove this definitions from here
int SolveP4Bi(double* x, double b, double d); // solve equation x^4 + b*x^2 + d = 0
int SolveP4De(double* x, double b, double c, double d); // solve equation x^4 + b*x^2 + c*x + d = 0
void CSqrt(double x, double y, double& a, double& b); // returns as a+i*s,  sqrt(x+i*y)
double N4Step(double x, double a, double b, double c, double d);// one Newton step for x^4 + a*x^3 + b*x^2 + c*x + d
