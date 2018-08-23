/*
 * Automatically Generated from Mathematica.
 * Thu 23 Aug 2018 15:50:23 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "R_rHipYaw.h"

#ifdef _MSC_VER
  #define INLINE __forceinline /* use __forceinline (VC++ specific) */
#else
  #define INLINE inline        /* use standard inline */
#endif

/**
 * Copied from Wolfram Mathematica C Definitions file mdefs.hpp
 * Changed marcos to inline functions (Eric Cousineau)
 */
INLINE double Power(double x, double y) { return pow(x, y); }
INLINE double Sqrt(double x) { return sqrt(x); }

INLINE double Abs(double x) { return fabs(x); }

INLINE double Exp(double x) { return exp(x); }
INLINE double Log(double x) { return log(x); }

INLINE double Sin(double x) { return sin(x); }
INLINE double Cos(double x) { return cos(x); }
INLINE double Tan(double x) { return tan(x); }

INLINE double Csc(double x) { return 1.0/sin(x); }
INLINE double Sec(double x) { return 1.0/cos(x); }

INLINE double ArcSin(double x) { return asin(x); }
INLINE double ArcCos(double x) { return acos(x); }
//INLINE double ArcTan(double x) { return atan(x); }

/* update ArcTan function to use atan2 instead. */
INLINE double ArcTan(double x, double y) { return atan2(y,x); }

INLINE double Sinh(double x) { return sinh(x); }
INLINE double Cosh(double x) { return cosh(x); }
INLINE double Tanh(double x) { return tanh(x); }

#define E 2.71828182845904523536029
#define Pi 3.14159265358979323846264
#define Degree 0.01745329251994329576924

/*
 * Sub functions
 */
static void output1(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t2196;
  double t1307;
  double t2503;
  double t2649;
  double t2591;
  double t2632;
  double t2897;
  double t2937;
  double t2893;
  double t3021;
  double t3042;
  double t3193;
  double t3218;
  double t3259;
  t2196 = Cos(var1[3]);
  t1307 = Cos(var1[11]);
  t2503 = Cos(var1[4]);
  t2649 = Sin(var1[3]);
  t2591 = Sin(var1[11]);
  t2632 = Cos(var1[5]);
  t2897 = Sin(var1[4]);
  t2937 = Sin(var1[5]);
  t2893 = -1.*t2632*t2649;
  t3021 = t2196*t2897*t2937;
  t3042 = t2893 + t3021;
  t3193 = t2196*t2632;
  t3218 = t2649*t2897*t2937;
  t3259 = t3193 + t3218;

  p_output1(0)=t1307*t2196*t2503 + t2591*t3042;
  p_output1(1)=t1307*t2503*t2649 + t2591*t3259;
  p_output1(2)=-1.*t1307*t2897 + t2503*t2591*t2937;
  p_output1(3)=-1.*t2196*t2503*t2591 + t1307*t3042;
  p_output1(4)=-1.*t2503*t2591*t2649 + t1307*t3259;
  p_output1(5)=t2591*t2897 + t1307*t2503*t2937;
  p_output1(6)=t2196*t2632*t2897 + t2649*t2937;
  p_output1(7)=t2632*t2649*t2897 - 1.*t2196*t2937;
  p_output1(8)=t2503*t2632;
}


       
void R_rHipYaw(Eigen::Matrix<double,3,3> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}
