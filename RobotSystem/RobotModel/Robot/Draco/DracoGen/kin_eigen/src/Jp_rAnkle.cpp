/*
 * Automatically Generated from Mathematica.
 * Thu 16 Aug 2018 23:21:52 GMT-05:00
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "Jp_rAnkle.h"

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
  double t683;
  double t174;
  double t335;
  double t345;
  double t589;
  double t1265;
  double t1067;
  double t1104;
  double t1268;
  double t721;
  double t765;
  double t819;
  double t862;
  double t141;
  double t1576;
  double t1658;
  double t1665;
  double t1108;
  double t1362;
  double t1372;
  double t1813;
  double t1824;
  double t1873;
  double t1955;
  double t1962;
  double t1964;
  double t2080;
  double t2126;
  double t2128;
  double t2156;
  double t2316;
  double t2332;
  double t2359;
  double t2382;
  double t2388;
  double t2394;
  double t2406;
  double t2441;
  double t2503;
  double t2610;
  double t2719;
  double t2759;
  double t2848;
  double t2929;
  double t3030;
  double t3131;
  double t3253;
  double t3358;
  double t3430;
  double t3474;
  double t3721;
  double t3729;
  double t3764;
  double t417;
  double t591;
  double t637;
  double t851;
  double t864;
  double t976;
  double t1469;
  double t1529;
  double t1569;
  double t1768;
  double t1778;
  double t1797;
  double t4364;
  double t4365;
  double t4378;
  double t2058;
  double t2089;
  double t2118;
  double t4305;
  double t4310;
  double t4324;
  double t4392;
  double t4395;
  double t4397;
  double t2309;
  double t2311;
  double t2315;
  double t2401;
  double t2411;
  double t2423;
  double t4417;
  double t4419;
  double t4421;
  double t4434;
  double t4436;
  double t4442;
  double t2649;
  double t2687;
  double t2699;
  double t3151;
  double t3279;
  double t3343;
  double t4458;
  double t4469;
  double t4472;
  double t4696;
  double t4700;
  double t4892;
  double t3555;
  double t3613;
  double t3715;
  double t4921;
  double t4922;
  double t4924;
  double t4935;
  double t5034;
  double t5072;
  double t5654;
  double t5680;
  double t5726;
  double t5783;
  double t5786;
  double t5877;
  double t6176;
  double t6205;
  double t6245;
  double t6277;
  double t6313;
  double t6339;
  double t6391;
  double t6397;
  double t6429;
  double t6467;
  double t6478;
  double t6500;
  double t6529;
  double t6551;
  double t6581;
  double t6786;
  double t6790;
  double t6803;
  double t6873;
  double t6911;
  double t6937;
  double t7009;
  double t7021;
  double t7061;
  double t7078;
  double t7108;
  double t7128;
  double t7155;
  double t7158;
  double t7161;
  double t7165;
  double t7182;
  double t7217;
  double t7229;
  double t7234;
  double t7241;
  double t7325;
  double t7326;
  double t7338;
  double t7343;
  double t7344;
  double t7356;
  double t7397;
  double t7407;
  double t7409;
  double t7424;
  double t7438;
  double t7449;
  double t7474;
  double t7481;
  double t7489;
  double t7504;
  double t7511;
  double t7521;
  double t7534;
  double t7541;
  double t7544;
  double t7664;
  double t7680;
  double t7681;
  double t7713;
  double t7723;
  double t7730;
  double t7755;
  double t7765;
  double t7774;
  double t7810;
  double t7811;
  double t7813;
  double t7820;
  double t7821;
  double t7827;
  double t7833;
  double t7835;
  double t7843;
  double t7936;
  double t7942;
  double t7945;
  double t7963;
  double t7978;
  double t7982;
  double t8003;
  double t8004;
  double t8007;
  double t8026;
  double t8030;
  double t8031;
  double t8057;
  double t8061;
  double t8063;
  double t8067;
  double t8070;
  double t8077;
  double t8116;
  double t8120;
  double t8121;
  double t8148;
  double t8153;
  double t8160;
  double t8182;
  double t8187;
  double t8191;
  double t8197;
  double t8202;
  double t8204;
  double t8207;
  double t8210;
  double t8211;
  double t8302;
  double t8307;
  double t8310;
  double t8326;
  double t8327;
  double t8328;
  double t8332;
  double t8333;
  double t8334;
  double t8340;
  double t8344;
  double t8346;
  double t8349;
  double t8356;
  double t8376;
  double t8278;
  double t8283;
  double t8284;
  double t8291;
  double t8292;
  double t8416;
  double t8417;
  double t8419;
  double t8432;
  double t8434;
  double t8423;
  double t8424;
  double t8425;
  double t8450;
  double t8479;
  double t8480;
  double t8503;
  double t8505;
  double t8506;
  double t8512;
  double t8513;
  double t8515;
  double t8530;
  double t8538;
  double t8546;
  double t8621;
  double t8626;
  double t8631;
  double t8604;
  double t8607;
  double t8617;
  double t8670;
  double t8681;
  double t8682;
  double t8695;
  double t8701;
  double t8714;
  double t8728;
  double t8729;
  double t8730;
  double t8743;
  double t8747;
  double t8765;
  double t8871;
  double t8883;
  double t8902;
  double t8950;
  double t8953;
  double t8954;
  double t8962;
  double t8972;
  double t8977;
  double t8830;
  double t8838;
  double t8846;
  double t8853;
  double t8861;
  double t9035;
  double t9036;
  double t9039;
  double t9065;
  double t9073;
  double t9081;
  double t9087;
  double t9098;
  double t9101;
  double t9154;
  double t9156;
  double t9157;
  double t9187;
  double t9190;
  double t9198;
  double t9206;
  double t9217;
  double t9219;
  double t9333;
  double t9342;
  double t9343;
  double t9351;
  double t9357;
  double t9367;
  double t9373;
  double t9375;
  double t9290;
  double t9291;
  double t9308;
  double t9312;
  double t9313;
  double t9050;
  double t9052;
  double t9053;
  double t9476;
  double t9481;
  double t9505;
  double t9514;
  double t9536;
  double t9550;
  double t9561;
  double t9562;
  double t9569;
  double t9574;
  double t9577;
  double t9579;
  double t9581;
  double t9582;
  double t9583;
  double t9179;
  double t9180;
  double t9183;
  double t9659;
  double t9661;
  double t9664;
  double t9682;
  double t9697;
  double t9709;
  double t9713;
  double t9714;
  double t9715;
  double t9719;
  double t9723;
  double t9724;
  double t9731;
  double t9732;
  double t9736;
  double t9785;
  double t9791;
  double t5288;
  double t9768;
  double t9769;
  double t9770;
  double t9772;
  double t9774;
  double t9822;
  double t9827;
  double t9833;
  double t9839;
  double t9843;
  double t9849;
  double t9850;
  double t9862;
  double t9866;
  double t9867;
  double t9875;
  double t9876;
  double t9878;
  double t9879;
  double t9795;
  double t5295;
  double t5301;
  double t9909;
  double t9910;
  double t9913;
  double t9922;
  double t9923;
  double t9852;
  double t9943;
  double t9944;
  double t9945;
  double t9856;
  double t9882;
  double t9958;
  double t9959;
  double t9960;
  double t9899;
  t683 = Sin(var1[3]);
  t174 = Cos(var1[11]);
  t335 = -1.*t174;
  t345 = 1. + t335;
  t589 = Sin(var1[11]);
  t1265 = Cos(var1[3]);
  t1067 = Cos(var1[5]);
  t1104 = Sin(var1[4]);
  t1268 = Sin(var1[5]);
  t721 = Cos(var1[12]);
  t765 = -1.*t721;
  t819 = 1. + t765;
  t862 = Sin(var1[12]);
  t141 = Cos(var1[4]);
  t1576 = -1.*t1265*t1067;
  t1658 = -1.*t683*t1104*t1268;
  t1665 = t1576 + t1658;
  t1108 = -1.*t1067*t683*t1104;
  t1362 = t1265*t1268;
  t1372 = t1108 + t1362;
  t1813 = t141*t589*t683;
  t1824 = t174*t1665;
  t1873 = t1813 + t1824;
  t1955 = Cos(var1[13]);
  t1962 = -1.*t1955;
  t1964 = 1. + t1962;
  t2080 = Sin(var1[13]);
  t2126 = -1.*t174*t141*t683;
  t2128 = t589*t1665;
  t2156 = t2126 + t2128;
  t2316 = t721*t1372;
  t2332 = -1.*t862*t1873;
  t2359 = t2316 + t2332;
  t2382 = Cos(var1[14]);
  t2388 = -1.*t2382;
  t2394 = 1. + t2388;
  t2406 = Sin(var1[14]);
  t2441 = t2080*t2156;
  t2503 = t1955*t2359;
  t2610 = t2441 + t2503;
  t2719 = t1955*t2156;
  t2759 = -1.*t2080*t2359;
  t2848 = t2719 + t2759;
  t2929 = Cos(var1[15]);
  t3030 = -1.*t2929;
  t3131 = 1. + t3030;
  t3253 = Sin(var1[15]);
  t3358 = -1.*t2406*t2610;
  t3430 = t2382*t2848;
  t3474 = t3358 + t3430;
  t3721 = t2382*t2610;
  t3729 = t2406*t2848;
  t3764 = t3721 + t3729;
  t417 = -0.022225*t345;
  t591 = -0.086996*t589;
  t637 = 0. + t417 + t591;
  t851 = -0.31508*t819;
  t864 = 0.156996*t862;
  t976 = 0. + t851 + t864;
  t1469 = -0.086996*t345;
  t1529 = 0.022225*t589;
  t1569 = 0. + t1469 + t1529;
  t1768 = -0.156996*t819;
  t1778 = -0.31508*t862;
  t1797 = 0. + t1768 + t1778;
  t4364 = -1.*t1067*t683;
  t4365 = t1265*t1104*t1268;
  t4378 = t4364 + t4365;
  t2058 = -0.022225*t1964;
  t2089 = 0.38008*t2080;
  t2118 = 0. + t2058 + t2089;
  t4305 = t1265*t1067*t1104;
  t4310 = t683*t1268;
  t4324 = t4305 + t4310;
  t4392 = -1.*t1265*t141*t589;
  t4395 = t174*t4378;
  t4397 = t4392 + t4395;
  t2309 = -0.38008*t1964;
  t2311 = -0.022225*t2080;
  t2315 = 0. + t2309 + t2311;
  t2401 = -0.86008*t2394;
  t2411 = -0.022225*t2406;
  t2423 = 0. + t2401 + t2411;
  t4417 = t174*t1265*t141;
  t4419 = t589*t4378;
  t4421 = t4417 + t4419;
  t4434 = t721*t4324;
  t4436 = -1.*t862*t4397;
  t4442 = t4434 + t4436;
  t2649 = -0.022225*t2394;
  t2687 = 0.86008*t2406;
  t2699 = 0. + t2649 + t2687;
  t3151 = -0.021147*t3131;
  t3279 = 1.34008*t3253;
  t3343 = 0. + t3151 + t3279;
  t4458 = t2080*t4421;
  t4469 = t1955*t4442;
  t4472 = t4458 + t4469;
  t4696 = t1955*t4421;
  t4700 = -1.*t2080*t4442;
  t4892 = t4696 + t4700;
  t3555 = -1.34008*t3131;
  t3613 = -0.021147*t3253;
  t3715 = 0. + t3555 + t3613;
  t4921 = -1.*t2406*t4472;
  t4922 = t2382*t4892;
  t4924 = t4921 + t4922;
  t4935 = t2382*t4472;
  t5034 = t2406*t4892;
  t5072 = t4935 + t5034;
  t5654 = t1265*t589*t1104;
  t5680 = t174*t1265*t141*t1268;
  t5726 = t5654 + t5680;
  t5783 = -1.*t174*t1265*t1104;
  t5786 = t1265*t141*t589*t1268;
  t5877 = t5783 + t5786;
  t6176 = t721*t1265*t141*t1067;
  t6205 = -1.*t862*t5726;
  t6245 = t6176 + t6205;
  t6277 = t2080*t5877;
  t6313 = t1955*t6245;
  t6339 = t6277 + t6313;
  t6391 = t1955*t5877;
  t6397 = -1.*t2080*t6245;
  t6429 = t6391 + t6397;
  t6467 = -1.*t2406*t6339;
  t6478 = t2382*t6429;
  t6500 = t6467 + t6478;
  t6529 = t2382*t6339;
  t6551 = t2406*t6429;
  t6581 = t6529 + t6551;
  t6786 = t589*t683*t1104;
  t6790 = t174*t141*t683*t1268;
  t6803 = t6786 + t6790;
  t6873 = -1.*t174*t683*t1104;
  t6911 = t141*t589*t683*t1268;
  t6937 = t6873 + t6911;
  t7009 = t721*t141*t1067*t683;
  t7021 = -1.*t862*t6803;
  t7061 = t7009 + t7021;
  t7078 = t2080*t6937;
  t7108 = t1955*t7061;
  t7128 = t7078 + t7108;
  t7155 = t1955*t6937;
  t7158 = -1.*t2080*t7061;
  t7161 = t7155 + t7158;
  t7165 = -1.*t2406*t7128;
  t7182 = t2382*t7161;
  t7217 = t7165 + t7182;
  t7229 = t2382*t7128;
  t7234 = t2406*t7161;
  t7241 = t7229 + t7234;
  t7325 = t141*t589;
  t7326 = -1.*t174*t1104*t1268;
  t7338 = t7325 + t7326;
  t7343 = -1.*t174*t141;
  t7344 = -1.*t589*t1104*t1268;
  t7356 = t7343 + t7344;
  t7397 = -1.*t721*t1067*t1104;
  t7407 = -1.*t862*t7338;
  t7409 = t7397 + t7407;
  t7424 = t2080*t7356;
  t7438 = t1955*t7409;
  t7449 = t7424 + t7438;
  t7474 = t1955*t7356;
  t7481 = -1.*t2080*t7409;
  t7489 = t7474 + t7481;
  t7504 = -1.*t2406*t7449;
  t7511 = t2382*t7489;
  t7521 = t7504 + t7511;
  t7534 = t2382*t7449;
  t7541 = t2406*t7489;
  t7544 = t7534 + t7541;
  t7664 = t1067*t683;
  t7680 = -1.*t1265*t1104*t1268;
  t7681 = t7664 + t7680;
  t7713 = -1.*t174*t862*t4324;
  t7723 = t721*t7681;
  t7730 = t7713 + t7723;
  t7755 = t589*t2080*t4324;
  t7765 = t1955*t7730;
  t7774 = t7755 + t7765;
  t7810 = t1955*t589*t4324;
  t7811 = -1.*t2080*t7730;
  t7813 = t7810 + t7811;
  t7820 = -1.*t2406*t7774;
  t7821 = t2382*t7813;
  t7827 = t7820 + t7821;
  t7833 = t2382*t7774;
  t7835 = t2406*t7813;
  t7843 = t7833 + t7835;
  t7936 = t1067*t683*t1104;
  t7942 = -1.*t1265*t1268;
  t7945 = t7936 + t7942;
  t7963 = -1.*t174*t862*t7945;
  t7978 = t721*t1665;
  t7982 = t7963 + t7978;
  t8003 = t589*t2080*t7945;
  t8004 = t1955*t7982;
  t8007 = t8003 + t8004;
  t8026 = t1955*t589*t7945;
  t8030 = -1.*t2080*t7982;
  t8031 = t8026 + t8030;
  t8057 = -1.*t2406*t8007;
  t8061 = t2382*t8031;
  t8063 = t8057 + t8061;
  t8067 = t2382*t8007;
  t8070 = t2406*t8031;
  t8077 = t8067 + t8070;
  t8116 = -1.*t174*t141*t1067*t862;
  t8120 = -1.*t721*t141*t1268;
  t8121 = t8116 + t8120;
  t8148 = t141*t1067*t589*t2080;
  t8153 = t1955*t8121;
  t8160 = t8148 + t8153;
  t8182 = t1955*t141*t1067*t589;
  t8187 = -1.*t2080*t8121;
  t8191 = t8182 + t8187;
  t8197 = -1.*t2406*t8160;
  t8202 = t2382*t8191;
  t8204 = t8197 + t8202;
  t8207 = t2382*t8160;
  t8210 = t2406*t8191;
  t8211 = t8207 + t8210;
  t8302 = -1.*t174*t1265*t141;
  t8307 = -1.*t589*t4378;
  t8310 = t8302 + t8307;
  t8326 = t2080*t4397;
  t8327 = -1.*t1955*t862*t8310;
  t8328 = t8326 + t8327;
  t8332 = t1955*t4397;
  t8333 = t862*t2080*t8310;
  t8334 = t8332 + t8333;
  t8340 = -1.*t2406*t8328;
  t8344 = t2382*t8334;
  t8346 = t8340 + t8344;
  t8349 = t2382*t8328;
  t8356 = t2406*t8334;
  t8376 = t8349 + t8356;
  t8278 = -0.086996*t174;
  t8283 = -0.022225*t589;
  t8284 = t8278 + t8283;
  t8291 = 0.022225*t174;
  t8292 = t8291 + t591;
  t8416 = t1265*t1067;
  t8417 = t683*t1104*t1268;
  t8419 = t8416 + t8417;
  t8432 = -1.*t589*t8419;
  t8434 = t2126 + t8432;
  t8423 = -1.*t141*t589*t683;
  t8424 = t174*t8419;
  t8425 = t8423 + t8424;
  t8450 = t2080*t8425;
  t8479 = -1.*t1955*t862*t8434;
  t8480 = t8450 + t8479;
  t8503 = t1955*t8425;
  t8505 = t862*t2080*t8434;
  t8506 = t8503 + t8505;
  t8512 = -1.*t2406*t8480;
  t8513 = t2382*t8506;
  t8515 = t8512 + t8513;
  t8530 = t2382*t8480;
  t8538 = t2406*t8506;
  t8546 = t8530 + t8538;
  t8621 = t174*t1104;
  t8626 = -1.*t141*t589*t1268;
  t8631 = t8621 + t8626;
  t8604 = t589*t1104;
  t8607 = t174*t141*t1268;
  t8617 = t8604 + t8607;
  t8670 = t2080*t8617;
  t8681 = -1.*t1955*t862*t8631;
  t8682 = t8670 + t8681;
  t8695 = t1955*t8617;
  t8701 = t862*t2080*t8631;
  t8714 = t8695 + t8701;
  t8728 = -1.*t2406*t8682;
  t8729 = t2382*t8714;
  t8730 = t8728 + t8729;
  t8743 = t2382*t8682;
  t8747 = t2406*t8714;
  t8765 = t8743 + t8747;
  t8871 = -1.*t862*t4324;
  t8883 = -1.*t721*t4397;
  t8902 = t8871 + t8883;
  t8950 = -1.*t2382*t2080*t8902;
  t8953 = -1.*t1955*t2406*t8902;
  t8954 = t8950 + t8953;
  t8962 = t1955*t2382*t8902;
  t8972 = -1.*t2080*t2406*t8902;
  t8977 = t8962 + t8972;
  t8830 = 0.156996*t721;
  t8838 = t8830 + t1778;
  t8846 = -0.31508*t721;
  t8853 = -0.156996*t862;
  t8861 = t8846 + t8853;
  t9035 = -1.*t862*t7945;
  t9036 = -1.*t721*t8425;
  t9039 = t9035 + t9036;
  t9065 = -1.*t2382*t2080*t9039;
  t9073 = -1.*t1955*t2406*t9039;
  t9081 = t9065 + t9073;
  t9087 = t1955*t2382*t9039;
  t9098 = -1.*t2080*t2406*t9039;
  t9101 = t9087 + t9098;
  t9154 = -1.*t141*t1067*t862;
  t9156 = -1.*t721*t8617;
  t9157 = t9154 + t9156;
  t9187 = -1.*t2382*t2080*t9157;
  t9190 = -1.*t1955*t2406*t9157;
  t9198 = t9187 + t9190;
  t9206 = t1955*t2382*t9157;
  t9217 = -1.*t2080*t2406*t9157;
  t9219 = t9206 + t9217;
  t9333 = -1.*t2080*t4421;
  t9342 = -1.*t1955*t4442;
  t9343 = t9333 + t9342;
  t9351 = t2406*t9343;
  t9357 = t9351 + t4922;
  t9367 = t2382*t9343;
  t9373 = -1.*t2406*t4892;
  t9375 = t9367 + t9373;
  t9290 = 0.38008*t1955;
  t9291 = t9290 + t2311;
  t9308 = -0.022225*t1955;
  t9312 = -0.38008*t2080;
  t9313 = t9308 + t9312;
  t9050 = t721*t7945;
  t9052 = -1.*t862*t8425;
  t9053 = t9050 + t9052;
  t9476 = t174*t141*t683;
  t9481 = t589*t8419;
  t9505 = t9476 + t9481;
  t9514 = -1.*t2080*t9505;
  t9536 = -1.*t1955*t9053;
  t9550 = t9514 + t9536;
  t9561 = t1955*t9505;
  t9562 = -1.*t2080*t9053;
  t9569 = t9561 + t9562;
  t9574 = t2406*t9550;
  t9577 = t2382*t9569;
  t9579 = t9574 + t9577;
  t9581 = t2382*t9550;
  t9582 = -1.*t2406*t9569;
  t9583 = t9581 + t9582;
  t9179 = t721*t141*t1067;
  t9180 = -1.*t862*t8617;
  t9183 = t9179 + t9180;
  t9659 = -1.*t174*t1104;
  t9661 = t141*t589*t1268;
  t9664 = t9659 + t9661;
  t9682 = -1.*t2080*t9664;
  t9697 = -1.*t1955*t9183;
  t9709 = t9682 + t9697;
  t9713 = t1955*t9664;
  t9714 = -1.*t2080*t9183;
  t9715 = t9713 + t9714;
  t9719 = t2406*t9709;
  t9723 = t2382*t9715;
  t9724 = t9719 + t9723;
  t9731 = t2382*t9709;
  t9732 = -1.*t2406*t9715;
  t9736 = t9731 + t9732;
  t9785 = -1.*t2382*t4472;
  t9791 = t9785 + t9373;
  t5288 = t2929*t4924;
  t9768 = -0.022225*t2382;
  t9769 = -0.86008*t2406;
  t9770 = t9768 + t9769;
  t9772 = 0.86008*t2382;
  t9774 = t9772 + t2411;
  t9822 = t2080*t9505;
  t9827 = t1955*t9053;
  t9833 = t9822 + t9827;
  t9839 = -1.*t2406*t9833;
  t9843 = t9839 + t9577;
  t9849 = -1.*t2382*t9833;
  t9850 = t9849 + t9582;
  t9862 = t2080*t9664;
  t9866 = t1955*t9183;
  t9867 = t9862 + t9866;
  t9875 = -1.*t2406*t9867;
  t9876 = t9875 + t9723;
  t9878 = -1.*t2382*t9867;
  t9879 = t9878 + t9732;
  t9795 = -1.*t3253*t4924;
  t5295 = -1.*t3253*t5072;
  t5301 = t5288 + t5295;
  t9909 = 1.34008*t2929;
  t9910 = t9909 + t3613;
  t9913 = -0.021147*t2929;
  t9922 = -1.34008*t3253;
  t9923 = t9913 + t9922;
  t9852 = -1.*t3253*t9843;
  t9943 = t2382*t9833;
  t9944 = t2406*t9569;
  t9945 = t9943 + t9944;
  t9856 = t2929*t9843;
  t9882 = -1.*t3253*t9876;
  t9958 = t2382*t9867;
  t9959 = t2406*t9715;
  t9960 = t9958 + t9959;
  t9899 = t2929*t9876;

  p_output1(0)=1.;
  p_output1(1)=0;
  p_output1(2)=0;
  p_output1(3)=0;
  p_output1(4)=1.;
  p_output1(5)=0;
  p_output1(6)=0;
  p_output1(7)=0;
  p_output1(8)=1.;
  p_output1(9)=t1569*t1665 + t1797*t1873 + t2118*t2156 + t2315*t2359 + t2423*t2610 + t2699*t2848 + t3343*t3474 + t3715*t3764 - 1.34008*(t3253*t3474 + t2929*t3764) - 0.021147*(t2929*t3474 - 1.*t3253*t3764) - 1.*t141*t637*t683 - 0.166996*(t1873*t721 + t1372*t862) + t1372*t976;
  p_output1(10)=t1569*t4378 + t1797*t4397 + t2118*t4421 + t2315*t4442 + t2423*t4472 + t2699*t4892 + t3343*t4924 + t3715*t5072 - 1.34008*(t3253*t4924 + t2929*t5072) - 0.021147*t5301 + t1265*t141*t637 - 0.166996*(t4397*t721 + t4324*t862) + t4324*t976;
  p_output1(11)=0;
  p_output1(12)=t1265*t1268*t141*t1569 + t1797*t5726 + t2118*t5877 + t2315*t6245 + t2423*t6339 - 1.*t1104*t1265*t637 + t2699*t6429 + t3343*t6500 + t3715*t6581 - 1.34008*(t3253*t6500 + t2929*t6581) - 0.021147*(t2929*t6500 - 1.*t3253*t6581) - 0.166996*(t5726*t721 + t1067*t1265*t141*t862) + t1067*t1265*t141*t976;
  p_output1(13)=t1797*t6803 + t1268*t141*t1569*t683 - 1.*t1104*t637*t683 + t2118*t6937 + t2315*t7061 + t2423*t7128 + t2699*t7161 + t3343*t7217 + t3715*t7241 - 1.34008*(t3253*t7217 + t2929*t7241) - 0.021147*(t2929*t7217 - 1.*t3253*t7241) - 0.166996*(t6803*t721 + t1067*t141*t683*t862) + t1067*t141*t683*t976;
  p_output1(14)=-1.*t1104*t1268*t1569 - 1.*t141*t637 + t1797*t7338 + t2118*t7356 + t2315*t7409 + t2423*t7449 + t2699*t7489 + t3343*t7521 + t3715*t7544 - 1.34008*(t3253*t7521 + t2929*t7544) - 0.021147*(t2929*t7521 - 1.*t3253*t7544) - 0.166996*(t721*t7338 - 1.*t1067*t1104*t862) - 1.*t1067*t1104*t976;
  p_output1(15)=t1569*t4324 + t174*t1797*t4324 + t2118*t4324*t589 + t2315*t7730 + t2423*t7774 + t2699*t7813 + t3343*t7827 + t3715*t7843 - 1.34008*(t3253*t7827 + t2929*t7843) - 0.021147*(t2929*t7827 - 1.*t3253*t7843) - 0.166996*(t174*t4324*t721 + t7681*t862) + t7681*t976;
  p_output1(16)=t1569*t7945 + t174*t1797*t7945 + t2118*t589*t7945 + t2315*t7982 + t2423*t8007 + t2699*t8031 + t3343*t8063 + t3715*t8077 - 1.34008*(t3253*t8063 + t2929*t8077) - 0.021147*(t2929*t8063 - 1.*t3253*t8077) - 0.166996*(t174*t721*t7945 + t1665*t862) + t1665*t976;
  p_output1(17)=t1067*t141*t1569 + t1067*t141*t174*t1797 + t1067*t141*t2118*t589 + t2315*t8121 + t2423*t8160 + t2699*t8191 + t3343*t8204 + t3715*t8211 - 1.34008*(t3253*t8204 + t2929*t8211) - 0.021147*(t2929*t8204 - 1.*t3253*t8211) - 0.166996*(t1067*t141*t174*t721 - 1.*t1268*t141*t862) - 1.*t1268*t141*t976;
  p_output1(18)=0;
  p_output1(19)=0;
  p_output1(20)=0;
  p_output1(21)=0;
  p_output1(22)=0;
  p_output1(23)=0;
  p_output1(24)=0;
  p_output1(25)=0;
  p_output1(26)=0;
  p_output1(27)=0;
  p_output1(28)=0;
  p_output1(29)=0;
  p_output1(30)=0;
  p_output1(31)=0;
  p_output1(32)=0;
  p_output1(33)=t2118*t4397 + t1265*t141*t8284 + t4378*t8292 + t1797*t8310 - 0.166996*t721*t8310 + t2423*t8328 + t2699*t8334 + t3343*t8346 + t3715*t8376 - 1.34008*(t3253*t8346 + t2929*t8376) - 0.021147*(t2929*t8346 - 1.*t3253*t8376) - 1.*t2315*t8310*t862;
  p_output1(34)=t141*t683*t8284 + t8292*t8419 + t2118*t8425 + t1797*t8434 - 0.166996*t721*t8434 + t2423*t8480 + t2699*t8506 + t3343*t8515 + t3715*t8546 - 1.34008*(t3253*t8515 + t2929*t8546) - 0.021147*(t2929*t8515 - 1.*t3253*t8546) - 1.*t2315*t8434*t862;
  p_output1(35)=-1.*t1104*t8284 + t1268*t141*t8292 + t2118*t8617 + t1797*t8631 - 0.166996*t721*t8631 - 1.*t2315*t862*t8631 + t2423*t8682 + t2699*t8714 + t3343*t8730 + t3715*t8765 - 1.34008*(t3253*t8730 + t2929*t8765) - 0.021147*(t2929*t8730 - 1.*t3253*t8765);
  p_output1(36)=-0.166996*t4442 + t4324*t8838 + t4397*t8861 + t2315*t8902 + t1955*t2423*t8902 - 1.*t2080*t2699*t8902 + t3343*t8954 + t3715*t8977 - 1.34008*(t3253*t8954 + t2929*t8977) - 0.021147*(t2929*t8954 - 1.*t3253*t8977);
  p_output1(37)=t7945*t8838 + t8425*t8861 + t2315*t9039 + t1955*t2423*t9039 - 1.*t2080*t2699*t9039 - 0.166996*t9053 + t3343*t9081 + t3715*t9101 - 1.34008*(t3253*t9081 + t2929*t9101) - 0.021147*(t2929*t9081 - 1.*t3253*t9101);
  p_output1(38)=t1067*t141*t8838 + t8617*t8861 + t2315*t9157 + t1955*t2423*t9157 - 1.*t2080*t2699*t9157 - 0.166996*t9183 + t3343*t9198 + t3715*t9219 - 1.34008*(t3253*t9198 + t2929*t9219) - 0.021147*(t2929*t9198 - 1.*t3253*t9219);
  p_output1(39)=t2423*t4892 + t4421*t9291 + t4442*t9313 + t2699*t9343 + t3715*t9357 + t3343*t9375 - 0.021147*(-1.*t3253*t9357 + t2929*t9375) - 1.34008*(t2929*t9357 + t3253*t9375);
  p_output1(40)=t9053*t9313 + t9291*t9505 + t2699*t9550 + t2423*t9569 + t3715*t9579 + t3343*t9583 - 0.021147*(-1.*t3253*t9579 + t2929*t9583) - 1.34008*(t2929*t9579 + t3253*t9583);
  p_output1(41)=t9183*t9313 + t9291*t9664 + t2699*t9709 + t2423*t9715 + t3715*t9724 + t3343*t9736 - 0.021147*(-1.*t3253*t9724 + t2929*t9736) - 1.34008*(t2929*t9724 + t3253*t9736);
  p_output1(42)=t3715*t4924 + t4472*t9770 + t4892*t9774 + t3343*t9791 - 1.34008*(t5288 + t3253*t9791) - 0.021147*(t2929*t9791 + t9795);
  p_output1(43)=t9569*t9774 + t9770*t9833 + t3715*t9843 + t3343*t9850 - 0.021147*(t2929*t9850 + t9852) - 1.34008*(t3253*t9850 + t9856);
  p_output1(44)=t9715*t9774 + t9770*t9867 + t3715*t9876 + t3343*t9879 - 0.021147*(t2929*t9879 + t9882) - 1.34008*(t3253*t9879 + t9899);
  p_output1(45)=-1.34008*t5301 - 0.021147*(-1.*t2929*t5072 + t9795) + t4924*t9910 + t5072*t9923;
  p_output1(46)=t9843*t9910 + t9923*t9945 - 0.021147*(t9852 - 1.*t2929*t9945) - 1.34008*(t9856 - 1.*t3253*t9945);
  p_output1(47)=t9876*t9910 + t9923*t9960 - 0.021147*(t9882 - 1.*t2929*t9960) - 1.34008*(t9899 - 1.*t3253*t9960);
}


       
void Jp_rAnkle(Eigen::Matrix<double,3,16> &p_output1, const Eigen::Matrix<double,16,1> &var1)
{
  // Call Subroutines
  output1(p_output1, var1);

}