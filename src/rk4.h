// https://people.sc.fsu.edu/~jburkardt/c_src/rk4/rk4.html
void rk4 ( void dydt ( double t, double u[], double f[], void *custom ), double tspan[2], 
  double y0[], int n, int m, double t[], double y[], void *custom );

