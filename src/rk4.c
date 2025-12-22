// https://people.sc.fsu.edu/~jburkardt/c_src/rk4/rk4.html
// Adapted to add a void *custom argument
# include <math.h>
# include <stdio.h>
# include <stdlib.h>
#
# include "rk4.h"

/******************************************************************************/

void rk4 ( void dydt ( double t, double u[], double f[], void *custom ), double tspan[2], 
  double y0[], int n, int m, double t[], double y[], void *custom )

/******************************************************************************/
/*
  Purpose:
 
    rk4() approximates an ODE using a Runge-Kutta fourth order method.

  Licensing:

    This code is distributed under the MIT license. 

  Modified:

    22 April 2020
	Adapted to add a void *custom argument - 13 December 2025

  Author:

    John Burkardt

  Input:

    double DYDT ( double T, double U ), a function which evaluates
    the derivative, or right hand side of the problem.

    double TSPAN[2]: the initial and final times

    double Y0[M]: the initial condition

    int N: the number of steps to take.

    int M: the number of variables.

  Output:

    double t[n+1], y[(n+1)*m]: the times and solution values.
*/
{
  double dt;
  double *f0;
  double *f1;
  double *f2;
  double *f3;
  int i;
  int j;
  double t0;
  double t1;
  double t2;
  double t3;
  double *u0;
  double *u1;
  double *u2;
  double *u3;

  f0 = ( double * ) malloc ( m * sizeof ( double ) );
  f1 = ( double * ) malloc ( m * sizeof ( double ) );
  f2 = ( double * ) malloc ( m * sizeof ( double ) );
  f3 = ( double * ) malloc ( m * sizeof ( double ) );
  u0 = ( double * ) malloc ( m * sizeof ( double ) );
  u1 = ( double * ) malloc ( m * sizeof ( double ) );
  u2 = ( double * ) malloc ( m * sizeof ( double ) );
  u3 = ( double * ) malloc ( m * sizeof ( double ) );

  dt = ( tspan[1] - tspan[0] ) / ( double ) ( n );

  j = 0;
  t[0] = tspan[0];
  for ( i = 0; i < m; i++ )
  {
    y[i+j*m] = y0[i];
  }

  for ( j = 0; j < n; j++ )
  {
    t0 = t[j];
    for ( i = 0; i < m; i++ )
    {
      u0[i] = y[i+j*m];
    }
    dydt ( t0, u0, f0, custom );

    t1 = t0 + dt / 2.0;
    for ( i = 0; i < m; i++ )
    {
      u1[i] = u0[i] + dt * f0[i] / 2.0;
    }
    dydt ( t1, u1, f1, custom );

    t2 = t0 + dt / 2.0;
    for ( i = 0; i < m; i++ )
    {
      u2[i] = u0[i] + dt * f1[i] / 2.0;
    }
    dydt ( t2, u2, f2, custom );

    t3 = t0 + dt;
    for ( i = 0; i < m; i++ )
    {
      u3[i] = u0[i] + dt * f2[i];
    }
    dydt ( t3, u3, f3, custom );

    t[j+1] = t[j] + dt;
    for ( i = 0; i < m; i++ )
    {
       y[i+(j+1)*m] = u0[i] + dt * ( f0[i] + 2.0 * f1[i] + 2.0 * f2[i] + f3[i] ) / 6.0;
    }
  }
/*
  Free memory.
*/
  free ( f0 );
  free ( f1 );
  free ( f2 );
  free ( f3 );
  free ( u0 );
  free ( u1 );
  free ( u2 );
  free ( u3 );

  return;
}

