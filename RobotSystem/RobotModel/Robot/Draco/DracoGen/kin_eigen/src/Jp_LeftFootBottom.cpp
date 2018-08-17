/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:54 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_LeftFootBottom.h"

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
static void output1(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  double t307;
  double t673;
  double t698;
  double t699;
  double t786;
  double t50;
  double t125;
  double t228;
  double t394;
  double t399;
  double t539;
  double t594;
  double t908;
  double t1323;
  double t1484;
  double t1531;
  double t1642;
  double t1065;
  double t1195;
  double t1290;
  double t1744;
  double t1821;
  double t1844;
  double t2304;
  double t2331;
  double t2363;
  double t2377;
  double t2180;
  double t2263;
  double t2303;
  double t2747;
  double t2822;
  double t2824;
  double t2962;
  double t3013;
  double t3022;
  double t3107;
  double t3145;
  double t3154;
  double t3159;
  double t3259;
  double t3261;
  double t3262;
  double t3311;
  double t3337;
  double t3371;
  double t3457;
  double t3552;
  double t3555;
  double t3585;
  double t3751;
  double t3765;
  double t3769;
  double t771;
  double t816;
  double t844;
  double t911;
  double t917;
  double t933;
  double t4086;
  double t4133;
  double t4181;
  double t1623;
  double t1657;
  double t1663;
  double t1847;
  double t1888;
  double t1909;
  double t4310;
  double t4316;
  double t4327;
  double t4344;
  double t4348;
  double t4350;
  double t2376;
  double t2512;
  double t2670;
  double t2849;
  double t2913;
  double t2945;
  double t3049;
  double t3129;
  double t3133;
  double t4406;
  double t4411;
  double t4425;
  double t4432;
  double t4490;
  double t4627;
  double t3185;
  double t3193;
  double t3258;
  double t3454;
  double t3463;
  double t3465;
  double t4664;
  double t4698;
  double t4751;
  double t4782;
  double t4839;
  double t4861;
  double t3630;
  double t3631;
  double t3714;
  double t4905;
  double t4934;
  double t4987;
  double t5063;
  double t5077;
  double t5086;
  double t5396;
  double t5401;
  double t5424;
  double t5578;
  double t5587;
  double t5595;
  double t5609;
  double t5613;
  double t5621;
  double t5695;
  double t5707;
  double t5739;
  double t5766;
  double t5773;
  double t5806;
  double t5874;
  double t5877;
  double t5881;
  double t5936;
  double t6008;
  double t6013;
  double t6277;
  double t6284;
  double t6299;
  double t6407;
  double t6414;
  double t6421;
  double t6459;
  double t6472;
  double t6478;
  double t6494;
  double t6560;
  double t6572;
  double t6601;
  double t6606;
  double t6631;
  double t6652;
  double t6661;
  double t6666;
  double t6695;
  double t6700;
  double t6703;
  double t6854;
  double t6896;
  double t6911;
  double t6962;
  double t6976;
  double t6977;
  double t6982;
  double t6983;
  double t6994;
  double t7052;
  double t7078;
  double t7094;
  double t7113;
  double t7115;
  double t7151;
  double t7157;
  double t7159;
  double t7164;
  double t7191;
  double t7220;
  double t7224;
  double t7293;
  double t7297;
  double t7298;
  double t7320;
  double t7321;
  double t7322;
  double t7332;
  double t7336;
  double t7341;
  double t7369;
  double t7374;
  double t7381;
  double t7391;
  double t7399;
  double t7415;
  double t7424;
  double t7438;
  double t7446;
  double t7526;
  double t7528;
  double t7534;
  double t7559;
  double t7560;
  double t7561;
  double t7591;
  double t7596;
  double t7605;
  double t7609;
  double t7615;
  double t7633;
  double t7681;
  double t7691;
  double t7723;
  double t7740;
  double t7744;
  double t7751;
  double t7904;
  double t7905;
  double t7907;
  double t7920;
  double t7927;
  double t7929;
  double t7936;
  double t7949;
  double t7951;
  double t7957;
  double t7978;
  double t7983;
  double t7989;
  double t7992;
  double t7993;
  double t8081;
  double t8086;
  double t8091;
  double t8100;
  double t8103;
  double t8104;
  double t8107;
  double t8115;
  double t8124;
  double t8138;
  double t8142;
  double t8148;
  double t8166;
  double t8181;
  double t8182;
  double t8047;
  double t8048;
  double t8050;
  double t8061;
  double t8070;
  double t8222;
  double t8229;
  double t8231;
  double t8242;
  double t8245;
  double t8276;
  double t8278;
  double t8282;
  double t8287;
  double t8288;
  double t8292;
  double t8300;
  double t8311;
  double t8312;
  double t8322;
  double t8330;
  double t8332;
  double t8334;
  double t8338;
  double t8339;
  double t8399;
  double t8400;
  double t8403;
  double t8420;
  double t8424;
  double t8427;
  double t8438;
  double t8443;
  double t8450;
  double t8456;
  double t8468;
  double t8478;
  double t8490;
  double t8501;
  double t8503;
  double t8506;
  double t8507;
  double t8513;
  double t8613;
  double t8621;
  double t8624;
  double t8648;
  double t8649;
  double t8656;
  double t8664;
  double t8666;
  double t8670;
  double t8590;
  double t8594;
  double t8600;
  double t8603;
  double t8604;
  double t8747;
  double t8757;
  double t8769;
  double t8815;
  double t8823;
  double t8830;
  double t8845;
  double t8849;
  double t8853;
  double t8959;
  double t8962;
  double t8970;
  double t9017;
  double t9019;
  double t9029;
  double t9046;
  double t9050;
  double t9057;
  double t9205;
  double t9206;
  double t9218;
  double t9238;
  double t9244;
  double t9252;
  double t9268;
  double t9280;
  double t8792;
  double t8798;
  double t8802;
  double t9115;
  double t9129;
  double t9145;
  double t9170;
  double t9175;
  double t9367;
  double t9370;
  double t9406;
  double t9415;
  double t9423;
  double t9425;
  double t9433;
  double t9440;
  double t9443;
  double t9457;
  double t9464;
  double t9466;
  double t9470;
  double t9476;
  double t9481;
  double t8993;
  double t8995;
  double t8996;
  double t9581;
  double t9585;
  double t9587;
  double t9595;
  double t9596;
  double t9599;
  double t9601;
  double t9610;
  double t9613;
  double t9615;
  double t9622;
  double t9649;
  double t9654;
  double t9673;
  double t9675;
  double t9767;
  double t9769;
  double t5259;
  double t9737;
  double t9745;
  double t9747;
  double t9755;
  double t9759;
  double t9822;
  double t9827;
  double t9834;
  double t9850;
  double t9851;
  double t9854;
  double t9855;
  double t9893;
  double t9894;
  double t9901;
  double t9908;
  double t9909;
  double t9913;
  double t9922;
  double t9774;
  double t5311;
  double t5332;
  double t9951;
  double t9952;
  double t9958;
  double t9962;
  double t9965;
  double t9858;
  double t9982;
  double t9984;
  double t9985;
  double t9866;
  double t9938;
  double t10001;
  double t10002;
  double t10003;
  double t9944;
  t307 = Sin(var1[3]);
  t673 = Cos(var1[6]);
  t698 = -1.*t673;
  t699 = 1. + t698;
  t786 = Sin(var1[6]);
  t50 = Cos(var1[3]);
  t125 = Cos(var1[5]);
  t228 = -1.*t50*t125;
  t394 = Sin(var1[4]);
  t399 = Sin(var1[5]);
  t539 = -1.*t307*t394*t399;
  t594 = t228 + t539;
  t908 = Cos(var1[4]);
  t1323 = Cos(var1[7]);
  t1484 = -1.*t1323;
  t1531 = 1. + t1484;
  t1642 = Sin(var1[7]);
  t1065 = t673*t594;
  t1195 = t908*t307*t786;
  t1290 = t1065 + t1195;
  t1744 = -1.*t125*t307*t394;
  t1821 = t50*t399;
  t1844 = t1744 + t1821;
  t2304 = Cos(var1[8]);
  t2331 = -1.*t2304;
  t2363 = 1. + t2331;
  t2377 = Sin(var1[8]);
  t2180 = t1323*t1844;
  t2263 = -1.*t1290*t1642;
  t2303 = t2180 + t2263;
  t2747 = -1.*t908*t673*t307;
  t2822 = t594*t786;
  t2824 = t2747 + t2822;
  t2962 = Cos(var1[9]);
  t3013 = -1.*t2962;
  t3022 = 1. + t3013;
  t3107 = Sin(var1[9]);
  t3145 = t2304*t2303;
  t3154 = t2824*t2377;
  t3159 = t3145 + t3154;
  t3259 = t2304*t2824;
  t3261 = -1.*t2303*t2377;
  t3262 = t3259 + t3261;
  t3311 = Cos(var1[10]);
  t3337 = -1.*t3311;
  t3371 = 1. + t3337;
  t3457 = Sin(var1[10]);
  t3552 = -1.*t3107*t3159;
  t3555 = t2962*t3262;
  t3585 = t3552 + t3555;
  t3751 = t2962*t3159;
  t3765 = t3107*t3262;
  t3769 = t3751 + t3765;
  t771 = 0.087004*t699;
  t816 = 0.022225*t786;
  t844 = 0. + t771 + t816;
  t911 = -0.022225*t699;
  t917 = 0.087004*t786;
  t933 = 0. + t911 + t917;
  t4086 = -1.*t125*t307;
  t4133 = t50*t394*t399;
  t4181 = t4086 + t4133;
  t1623 = 0.157004*t1531;
  t1657 = -0.31508*t1642;
  t1663 = 0. + t1623 + t1657;
  t1847 = -0.31508*t1531;
  t1888 = -0.157004*t1642;
  t1909 = 0. + t1847 + t1888;
  t4310 = t673*t4181;
  t4316 = -1.*t50*t908*t786;
  t4327 = t4310 + t4316;
  t4344 = t50*t125*t394;
  t4348 = t307*t399;
  t4350 = t4344 + t4348;
  t2376 = -0.38008*t2363;
  t2512 = -0.022225*t2377;
  t2670 = 0. + t2376 + t2512;
  t2849 = -0.022225*t2363;
  t2913 = 0.38008*t2377;
  t2945 = 0. + t2849 + t2913;
  t3049 = -0.86008*t3022;
  t3129 = -0.022225*t3107;
  t3133 = 0. + t3049 + t3129;
  t4406 = t1323*t4350;
  t4411 = -1.*t4327*t1642;
  t4425 = t4406 + t4411;
  t4432 = t50*t908*t673;
  t4490 = t4181*t786;
  t4627 = t4432 + t4490;
  t3185 = -0.022225*t3022;
  t3193 = 0.86008*t3107;
  t3258 = 0. + t3185 + t3193;
  t3454 = -0.021147*t3371;
  t3463 = 1.34008*t3457;
  t3465 = 0. + t3454 + t3463;
  t4664 = t2304*t4425;
  t4698 = t4627*t2377;
  t4751 = t4664 + t4698;
  t4782 = t2304*t4627;
  t4839 = -1.*t4425*t2377;
  t4861 = t4782 + t4839;
  t3630 = -1.34008*t3371;
  t3631 = -0.021147*t3457;
  t3714 = 0. + t3630 + t3631;
  t4905 = -1.*t3107*t4751;
  t4934 = t2962*t4861;
  t4987 = t4905 + t4934;
  t5063 = t2962*t4751;
  t5077 = t3107*t4861;
  t5086 = t5063 + t5077;
  t5396 = t50*t908*t673*t399;
  t5401 = t50*t394*t786;
  t5424 = t5396 + t5401;
  t5578 = t50*t908*t125*t1323;
  t5587 = -1.*t5424*t1642;
  t5595 = t5578 + t5587;
  t5609 = -1.*t50*t673*t394;
  t5613 = t50*t908*t399*t786;
  t5621 = t5609 + t5613;
  t5695 = t2304*t5595;
  t5707 = t5621*t2377;
  t5739 = t5695 + t5707;
  t5766 = t2304*t5621;
  t5773 = -1.*t5595*t2377;
  t5806 = t5766 + t5773;
  t5874 = -1.*t3107*t5739;
  t5877 = t2962*t5806;
  t5881 = t5874 + t5877;
  t5936 = t2962*t5739;
  t6008 = t3107*t5806;
  t6013 = t5936 + t6008;
  t6277 = t908*t673*t307*t399;
  t6284 = t307*t394*t786;
  t6299 = t6277 + t6284;
  t6407 = t908*t125*t1323*t307;
  t6414 = -1.*t6299*t1642;
  t6421 = t6407 + t6414;
  t6459 = -1.*t673*t307*t394;
  t6472 = t908*t307*t399*t786;
  t6478 = t6459 + t6472;
  t6494 = t2304*t6421;
  t6560 = t6478*t2377;
  t6572 = t6494 + t6560;
  t6601 = t2304*t6478;
  t6606 = -1.*t6421*t2377;
  t6631 = t6601 + t6606;
  t6652 = -1.*t3107*t6572;
  t6661 = t2962*t6631;
  t6666 = t6652 + t6661;
  t6695 = t2962*t6572;
  t6700 = t3107*t6631;
  t6703 = t6695 + t6700;
  t6854 = -1.*t673*t394*t399;
  t6896 = t908*t786;
  t6911 = t6854 + t6896;
  t6962 = -1.*t125*t1323*t394;
  t6976 = -1.*t6911*t1642;
  t6977 = t6962 + t6976;
  t6982 = -1.*t908*t673;
  t6983 = -1.*t394*t399*t786;
  t6994 = t6982 + t6983;
  t7052 = t2304*t6977;
  t7078 = t6994*t2377;
  t7094 = t7052 + t7078;
  t7113 = t2304*t6994;
  t7115 = -1.*t6977*t2377;
  t7151 = t7113 + t7115;
  t7157 = -1.*t3107*t7094;
  t7159 = t2962*t7151;
  t7164 = t7157 + t7159;
  t7191 = t2962*t7094;
  t7220 = t3107*t7151;
  t7224 = t7191 + t7220;
  t7293 = t125*t307;
  t7297 = -1.*t50*t394*t399;
  t7298 = t7293 + t7297;
  t7320 = t1323*t7298;
  t7321 = -1.*t673*t4350*t1642;
  t7322 = t7320 + t7321;
  t7332 = t2304*t7322;
  t7336 = t4350*t786*t2377;
  t7341 = t7332 + t7336;
  t7369 = t2304*t4350*t786;
  t7374 = -1.*t7322*t2377;
  t7381 = t7369 + t7374;
  t7391 = -1.*t3107*t7341;
  t7399 = t2962*t7381;
  t7415 = t7391 + t7399;
  t7424 = t2962*t7341;
  t7438 = t3107*t7381;
  t7446 = t7424 + t7438;
  t7526 = t125*t307*t394;
  t7528 = -1.*t50*t399;
  t7534 = t7526 + t7528;
  t7559 = t1323*t594;
  t7560 = -1.*t673*t7534*t1642;
  t7561 = t7559 + t7560;
  t7591 = t2304*t7561;
  t7596 = t7534*t786*t2377;
  t7605 = t7591 + t7596;
  t7609 = t2304*t7534*t786;
  t7615 = -1.*t7561*t2377;
  t7633 = t7609 + t7615;
  t7681 = -1.*t3107*t7605;
  t7691 = t2962*t7633;
  t7723 = t7681 + t7691;
  t7740 = t2962*t7605;
  t7744 = t3107*t7633;
  t7751 = t7740 + t7744;
  t7904 = -1.*t908*t1323*t399;
  t7905 = -1.*t908*t125*t673*t1642;
  t7907 = t7904 + t7905;
  t7920 = t2304*t7907;
  t7927 = t908*t125*t786*t2377;
  t7929 = t7920 + t7927;
  t7936 = t908*t125*t2304*t786;
  t7949 = -1.*t7907*t2377;
  t7951 = t7936 + t7949;
  t7957 = -1.*t3107*t7929;
  t7978 = t2962*t7951;
  t7983 = t7957 + t7978;
  t7989 = t2962*t7929;
  t7992 = t3107*t7951;
  t7993 = t7989 + t7992;
  t8081 = -1.*t50*t908*t673;
  t8086 = -1.*t4181*t786;
  t8091 = t8081 + t8086;
  t8100 = -1.*t2304*t8091*t1642;
  t8103 = t4327*t2377;
  t8104 = t8100 + t8103;
  t8107 = t2304*t4327;
  t8115 = t8091*t1642*t2377;
  t8124 = t8107 + t8115;
  t8138 = -1.*t3107*t8104;
  t8142 = t2962*t8124;
  t8148 = t8138 + t8142;
  t8166 = t2962*t8104;
  t8181 = t3107*t8124;
  t8182 = t8166 + t8181;
  t8047 = 0.087004*t673;
  t8048 = -0.022225*t786;
  t8050 = t8047 + t8048;
  t8061 = 0.022225*t673;
  t8070 = t8061 + t917;
  t8222 = t50*t125;
  t8229 = t307*t394*t399;
  t8231 = t8222 + t8229;
  t8242 = -1.*t8231*t786;
  t8245 = t2747 + t8242;
  t8276 = t673*t8231;
  t8278 = -1.*t908*t307*t786;
  t8282 = t8276 + t8278;
  t8287 = -1.*t2304*t8245*t1642;
  t8288 = t8282*t2377;
  t8292 = t8287 + t8288;
  t8300 = t2304*t8282;
  t8311 = t8245*t1642*t2377;
  t8312 = t8300 + t8311;
  t8322 = -1.*t3107*t8292;
  t8330 = t2962*t8312;
  t8332 = t8322 + t8330;
  t8334 = t2962*t8292;
  t8338 = t3107*t8312;
  t8339 = t8334 + t8338;
  t8399 = t673*t394;
  t8400 = -1.*t908*t399*t786;
  t8403 = t8399 + t8400;
  t8420 = t908*t673*t399;
  t8424 = t394*t786;
  t8427 = t8420 + t8424;
  t8438 = -1.*t2304*t8403*t1642;
  t8443 = t8427*t2377;
  t8450 = t8438 + t8443;
  t8456 = t2304*t8427;
  t8468 = t8403*t1642*t2377;
  t8478 = t8456 + t8468;
  t8490 = -1.*t3107*t8450;
  t8501 = t2962*t8478;
  t8503 = t8490 + t8501;
  t8506 = t2962*t8450;
  t8507 = t3107*t8478;
  t8513 = t8506 + t8507;
  t8613 = -1.*t1323*t4327;
  t8621 = -1.*t4350*t1642;
  t8624 = t8613 + t8621;
  t8648 = -1.*t2304*t3107*t8624;
  t8649 = -1.*t2962*t8624*t2377;
  t8656 = t8648 + t8649;
  t8664 = t2962*t2304*t8624;
  t8666 = -1.*t3107*t8624*t2377;
  t8670 = t8664 + t8666;
  t8590 = -0.157004*t1323;
  t8594 = t8590 + t1657;
  t8600 = -0.31508*t1323;
  t8603 = 0.157004*t1642;
  t8604 = t8600 + t8603;
  t8747 = -1.*t1323*t8282;
  t8757 = -1.*t7534*t1642;
  t8769 = t8747 + t8757;
  t8815 = -1.*t2304*t3107*t8769;
  t8823 = -1.*t2962*t8769*t2377;
  t8830 = t8815 + t8823;
  t8845 = t2962*t2304*t8769;
  t8849 = -1.*t3107*t8769*t2377;
  t8853 = t8845 + t8849;
  t8959 = -1.*t1323*t8427;
  t8962 = -1.*t908*t125*t1642;
  t8970 = t8959 + t8962;
  t9017 = -1.*t2304*t3107*t8970;
  t9019 = -1.*t2962*t8970*t2377;
  t9029 = t9017 + t9019;
  t9046 = t2962*t2304*t8970;
  t9050 = -1.*t3107*t8970*t2377;
  t9057 = t9046 + t9050;
  t9205 = -1.*t2304*t4425;
  t9206 = -1.*t4627*t2377;
  t9218 = t9205 + t9206;
  t9238 = t3107*t9218;
  t9244 = t9238 + t4934;
  t9252 = t2962*t9218;
  t9268 = -1.*t3107*t4861;
  t9280 = t9252 + t9268;
  t8792 = t1323*t7534;
  t8798 = -1.*t8282*t1642;
  t8802 = t8792 + t8798;
  t9115 = -0.022225*t2304;
  t9129 = -0.38008*t2377;
  t9145 = t9115 + t9129;
  t9170 = 0.38008*t2304;
  t9175 = t9170 + t2512;
  t9367 = t908*t673*t307;
  t9370 = t8231*t786;
  t9406 = t9367 + t9370;
  t9415 = -1.*t2304*t8802;
  t9423 = -1.*t9406*t2377;
  t9425 = t9415 + t9423;
  t9433 = t2304*t9406;
  t9440 = -1.*t8802*t2377;
  t9443 = t9433 + t9440;
  t9457 = t3107*t9425;
  t9464 = t2962*t9443;
  t9466 = t9457 + t9464;
  t9470 = t2962*t9425;
  t9476 = -1.*t3107*t9443;
  t9481 = t9470 + t9476;
  t8993 = t908*t125*t1323;
  t8995 = -1.*t8427*t1642;
  t8996 = t8993 + t8995;
  t9581 = -1.*t673*t394;
  t9585 = t908*t399*t786;
  t9587 = t9581 + t9585;
  t9595 = -1.*t2304*t8996;
  t9596 = -1.*t9587*t2377;
  t9599 = t9595 + t9596;
  t9601 = t2304*t9587;
  t9610 = -1.*t8996*t2377;
  t9613 = t9601 + t9610;
  t9615 = t3107*t9599;
  t9622 = t2962*t9613;
  t9649 = t9615 + t9622;
  t9654 = t2962*t9599;
  t9673 = -1.*t3107*t9613;
  t9675 = t9654 + t9673;
  t9767 = -1.*t2962*t4751;
  t9769 = t9767 + t9268;
  t5259 = t3311*t4987;
  t9737 = -0.022225*t2962;
  t9745 = -0.86008*t3107;
  t9747 = t9737 + t9745;
  t9755 = 0.86008*t2962;
  t9759 = t9755 + t3129;
  t9822 = t2304*t8802;
  t9827 = t9406*t2377;
  t9834 = t9822 + t9827;
  t9850 = -1.*t3107*t9834;
  t9851 = t9850 + t9464;
  t9854 = -1.*t2962*t9834;
  t9855 = t9854 + t9476;
  t9893 = t2304*t8996;
  t9894 = t9587*t2377;
  t9901 = t9893 + t9894;
  t9908 = -1.*t3107*t9901;
  t9909 = t9908 + t9622;
  t9913 = -1.*t2962*t9901;
  t9922 = t9913 + t9673;
  t9774 = -1.*t3457*t4987;
  t5311 = -1.*t3457*t5086;
  t5332 = t5259 + t5311;
  t9951 = 1.34008*t3311;
  t9952 = t9951 + t3631;
  t9958 = -0.021147*t3311;
  t9962 = -1.34008*t3457;
  t9965 = t9958 + t9962;
  t9858 = -1.*t3457*t9851;
  t9982 = t2962*t9834;
  t9984 = t3107*t9443;
  t9985 = t9982 + t9984;
  t9866 = t3311*t9851;
  t9938 = -1.*t3457*t9909;
  t10001 = t2962*t9901;
  t10002 = t3107*t9613;
  t10003 = t10001 + t10002;
  t9944 = t3311*t9909;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1290*t1663 + 0.167004*(t1290*t1323 + t1642*t1844) + t1844*t1909 + t2303*t2670 + t2824*t2945 + t3133*t3159 + t3258*t3262 + t3465*t3585 + t3714*t3769 - 1.325132*(t3457*t3585 + t3311*t3769) + 0.043865*(t3311*t3585 - 1.*t3457*t3769) + t594*t844 - 1.*t307*t908*t933;
  p_output1(10)=t1663*t4327 + t1909*t4350 + 0.167004*(t1323*t4327 + t1642*t4350) + t2670*t4425 + t2945*t4627 + t3133*t4751 + t3258*t4861 + t3465*t4987 + t3714*t5086 - 1.325132*(t3457*t4987 + t3311*t5086) + 0.043865*t5332 + t4181*t844 + t50*t908*t933;
  p_output1(11)=0;
  p_output1(12)=t1663*t5424 + t2670*t5595 + t2945*t5621 + t3133*t5739 + t3258*t5806 + t3465*t5881 + t3714*t6013 - 1.325132*(t3457*t5881 + t3311*t6013) + 0.043865*(t3311*t5881 - 1.*t3457*t6013) + t125*t1909*t50*t908 + t399*t50*t844*t908 + 0.167004*(t1323*t5424 + t125*t1642*t50*t908) - 1.*t394*t50*t933;
  p_output1(13)=t1663*t6299 + t2670*t6421 + t2945*t6478 + t3133*t6572 + t3258*t6631 + t3465*t6666 + t3714*t6703 - 1.325132*(t3457*t6666 + t3311*t6703) + 0.043865*(t3311*t6666 - 1.*t3457*t6703) + t125*t1909*t307*t908 + t307*t399*t844*t908 + 0.167004*(t1323*t6299 + t125*t1642*t307*t908) - 1.*t307*t394*t933;
  p_output1(14)=-1.*t125*t1909*t394 + t1663*t6911 + 0.167004*(-1.*t125*t1642*t394 + t1323*t6911) + t2670*t6977 + t2945*t6994 + t3133*t7094 + t3258*t7151 + t3465*t7164 + t3714*t7224 - 1.325132*(t3457*t7164 + t3311*t7224) + 0.043865*(t3311*t7164 - 1.*t3457*t7224) - 1.*t394*t399*t844 - 1.*t908*t933;
  p_output1(15)=t1663*t4350*t673 + t1909*t7298 + 0.167004*(t1323*t4350*t673 + t1642*t7298) + t2670*t7322 + t3133*t7341 + t3258*t7381 + t3465*t7415 + t3714*t7446 - 1.325132*(t3457*t7415 + t3311*t7446) + 0.043865*(t3311*t7415 - 1.*t3457*t7446) + t2945*t4350*t786 + t4350*t844;
  p_output1(16)=t1909*t594 + t1663*t673*t7534 + 0.167004*(t1642*t594 + t1323*t673*t7534) + t2670*t7561 + t3133*t7605 + t3258*t7633 + t3465*t7723 + t3714*t7751 - 1.325132*(t3457*t7723 + t3311*t7751) + 0.043865*(t3311*t7723 - 1.*t3457*t7751) + t2945*t7534*t786 + t7534*t844;
  p_output1(17)=t2670*t7907 + t3133*t7929 + t3258*t7951 + t3465*t7983 + t3714*t7993 - 1.325132*(t3457*t7983 + t3311*t7993) + 0.043865*(t3311*t7983 - 1.*t3457*t7993) - 1.*t1909*t399*t908 + t125*t1663*t673*t908 + t125*t2945*t786*t908 + t125*t844*t908 + 0.167004*(-1.*t1642*t399*t908 + t125*t1323*t673*t908);
  p_output1(18)=t2945*t4327 + t4181*t8070 + 0.167004*t1323*t8091 + t1663*t8091 - 1.*t1642*t2670*t8091 + t3133*t8104 + t3258*t8124 + t3465*t8148 + t3714*t8182 - 1.325132*(t3457*t8148 + t3311*t8182) + 0.043865*(t3311*t8148 - 1.*t3457*t8182) + t50*t8050*t908;
  p_output1(19)=t8070*t8231 + 0.167004*t1323*t8245 + t1663*t8245 - 1.*t1642*t2670*t8245 + t2945*t8282 + t3133*t8292 + t3258*t8312 + t3465*t8332 + t3714*t8339 - 1.325132*(t3457*t8332 + t3311*t8339) + 0.043865*(t3311*t8332 - 1.*t3457*t8339) + t307*t8050*t908;
  p_output1(20)=-1.*t394*t8050 + 0.167004*t1323*t8403 + t1663*t8403 - 1.*t1642*t2670*t8403 + t2945*t8427 + t3133*t8450 + t3258*t8478 + t3465*t8503 + t3714*t8513 - 1.325132*(t3457*t8503 + t3311*t8513) + 0.043865*(t3311*t8503 - 1.*t3457*t8513) + t399*t8070*t908;
  p_output1(21)=0.167004*t4425 + t4350*t8594 + t4327*t8604 + t2670*t8624 + t2304*t3133*t8624 - 1.*t2377*t3258*t8624 + t3465*t8656 + t3714*t8670 - 1.325132*(t3457*t8656 + t3311*t8670) + 0.043865*(t3311*t8656 - 1.*t3457*t8670);
  p_output1(22)=t7534*t8594 + t8282*t8604 + t2670*t8769 + t2304*t3133*t8769 - 1.*t2377*t3258*t8769 + 0.167004*t8802 + t3465*t8830 + t3714*t8853 - 1.325132*(t3457*t8830 + t3311*t8853) + 0.043865*(t3311*t8830 - 1.*t3457*t8853);
  p_output1(23)=t8427*t8604 + t2670*t8970 + t2304*t3133*t8970 - 1.*t2377*t3258*t8970 + 0.167004*t8996 + t3465*t9029 + t3714*t9057 - 1.325132*(t3457*t9029 + t3311*t9057) + 0.043865*(t3311*t9029 - 1.*t3457*t9057) + t125*t8594*t908;
  p_output1(24)=t3133*t4861 + t4425*t9145 + t4627*t9175 + t3258*t9218 + t3714*t9244 + t3465*t9280 + 0.043865*(-1.*t3457*t9244 + t3311*t9280) - 1.325132*(t3311*t9244 + t3457*t9280);
  p_output1(25)=t8802*t9145 + t9175*t9406 + t3258*t9425 + t3133*t9443 + t3714*t9466 + t3465*t9481 + 0.043865*(-1.*t3457*t9466 + t3311*t9481) - 1.325132*(t3311*t9466 + t3457*t9481);
  p_output1(26)=t8996*t9145 + t9175*t9587 + t3258*t9599 + t3133*t9613 + t3714*t9649 + t3465*t9675 + 0.043865*(-1.*t3457*t9649 + t3311*t9675) - 1.325132*(t3311*t9649 + t3457*t9675);
  p_output1(27)=t3714*t4987 + t4751*t9747 + t4861*t9759 + t3465*t9769 - 1.325132*(t5259 + t3457*t9769) + 0.043865*(t3311*t9769 + t9774);
  p_output1(28)=t9443*t9759 + t9747*t9834 + t3714*t9851 + t3465*t9855 + 0.043865*(t3311*t9855 + t9858) - 1.325132*(t3457*t9855 + t9866);
  p_output1(29)=t9613*t9759 + t9747*t9901 + t3714*t9909 + t3465*t9922 + 0.043865*(t3311*t9922 + t9938) - 1.325132*(t3457*t9922 + t9944);
  p_output1(30)=-1.325132*t5332 + 0.043865*(-1.*t3311*t5086 + t9774) + t4987*t9952 + t5086*t9965;
  p_output1(31)=t9851*t9952 + t9965*t9985 + 0.043865*(t9858 - 1.*t3311*t9985) - 1.325132*(t9866 - 1.*t3457*t9985);
  p_output1(32)=0.043865*(-1.*t10003*t3311 + t9938) - 1.325132*(-1.*t10003*t3457 + t9944) + t9909*t9952 + t10003*t9965;
  p_output1(33)=0;
  p_output1(34)=0;
  p_output1(35)=0;
  p_output1(36)=0;
  p_output1(37)=0;
  p_output1(38)=0;
  p_output1(39)=0;
  p_output1(40)=0;
  p_output1(41)=0;
  p_output1(42)=0;
  p_output1(43)=0;
  p_output1(44)=0;
  p_output1(45)=0;
  p_output1(46)=0;
  p_output1(47)=0;
}


       
void Jp_LeftFootBottom(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}