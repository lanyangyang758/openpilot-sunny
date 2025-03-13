#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1476640846964750484) {
   out_1476640846964750484[0] = delta_x[0] + nom_x[0];
   out_1476640846964750484[1] = delta_x[1] + nom_x[1];
   out_1476640846964750484[2] = delta_x[2] + nom_x[2];
   out_1476640846964750484[3] = delta_x[3] + nom_x[3];
   out_1476640846964750484[4] = delta_x[4] + nom_x[4];
   out_1476640846964750484[5] = delta_x[5] + nom_x[5];
   out_1476640846964750484[6] = delta_x[6] + nom_x[6];
   out_1476640846964750484[7] = delta_x[7] + nom_x[7];
   out_1476640846964750484[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1337065726167235347) {
   out_1337065726167235347[0] = -nom_x[0] + true_x[0];
   out_1337065726167235347[1] = -nom_x[1] + true_x[1];
   out_1337065726167235347[2] = -nom_x[2] + true_x[2];
   out_1337065726167235347[3] = -nom_x[3] + true_x[3];
   out_1337065726167235347[4] = -nom_x[4] + true_x[4];
   out_1337065726167235347[5] = -nom_x[5] + true_x[5];
   out_1337065726167235347[6] = -nom_x[6] + true_x[6];
   out_1337065726167235347[7] = -nom_x[7] + true_x[7];
   out_1337065726167235347[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_6413824007684546309) {
   out_6413824007684546309[0] = 1.0;
   out_6413824007684546309[1] = 0.0;
   out_6413824007684546309[2] = 0.0;
   out_6413824007684546309[3] = 0.0;
   out_6413824007684546309[4] = 0.0;
   out_6413824007684546309[5] = 0.0;
   out_6413824007684546309[6] = 0.0;
   out_6413824007684546309[7] = 0.0;
   out_6413824007684546309[8] = 0.0;
   out_6413824007684546309[9] = 0.0;
   out_6413824007684546309[10] = 1.0;
   out_6413824007684546309[11] = 0.0;
   out_6413824007684546309[12] = 0.0;
   out_6413824007684546309[13] = 0.0;
   out_6413824007684546309[14] = 0.0;
   out_6413824007684546309[15] = 0.0;
   out_6413824007684546309[16] = 0.0;
   out_6413824007684546309[17] = 0.0;
   out_6413824007684546309[18] = 0.0;
   out_6413824007684546309[19] = 0.0;
   out_6413824007684546309[20] = 1.0;
   out_6413824007684546309[21] = 0.0;
   out_6413824007684546309[22] = 0.0;
   out_6413824007684546309[23] = 0.0;
   out_6413824007684546309[24] = 0.0;
   out_6413824007684546309[25] = 0.0;
   out_6413824007684546309[26] = 0.0;
   out_6413824007684546309[27] = 0.0;
   out_6413824007684546309[28] = 0.0;
   out_6413824007684546309[29] = 0.0;
   out_6413824007684546309[30] = 1.0;
   out_6413824007684546309[31] = 0.0;
   out_6413824007684546309[32] = 0.0;
   out_6413824007684546309[33] = 0.0;
   out_6413824007684546309[34] = 0.0;
   out_6413824007684546309[35] = 0.0;
   out_6413824007684546309[36] = 0.0;
   out_6413824007684546309[37] = 0.0;
   out_6413824007684546309[38] = 0.0;
   out_6413824007684546309[39] = 0.0;
   out_6413824007684546309[40] = 1.0;
   out_6413824007684546309[41] = 0.0;
   out_6413824007684546309[42] = 0.0;
   out_6413824007684546309[43] = 0.0;
   out_6413824007684546309[44] = 0.0;
   out_6413824007684546309[45] = 0.0;
   out_6413824007684546309[46] = 0.0;
   out_6413824007684546309[47] = 0.0;
   out_6413824007684546309[48] = 0.0;
   out_6413824007684546309[49] = 0.0;
   out_6413824007684546309[50] = 1.0;
   out_6413824007684546309[51] = 0.0;
   out_6413824007684546309[52] = 0.0;
   out_6413824007684546309[53] = 0.0;
   out_6413824007684546309[54] = 0.0;
   out_6413824007684546309[55] = 0.0;
   out_6413824007684546309[56] = 0.0;
   out_6413824007684546309[57] = 0.0;
   out_6413824007684546309[58] = 0.0;
   out_6413824007684546309[59] = 0.0;
   out_6413824007684546309[60] = 1.0;
   out_6413824007684546309[61] = 0.0;
   out_6413824007684546309[62] = 0.0;
   out_6413824007684546309[63] = 0.0;
   out_6413824007684546309[64] = 0.0;
   out_6413824007684546309[65] = 0.0;
   out_6413824007684546309[66] = 0.0;
   out_6413824007684546309[67] = 0.0;
   out_6413824007684546309[68] = 0.0;
   out_6413824007684546309[69] = 0.0;
   out_6413824007684546309[70] = 1.0;
   out_6413824007684546309[71] = 0.0;
   out_6413824007684546309[72] = 0.0;
   out_6413824007684546309[73] = 0.0;
   out_6413824007684546309[74] = 0.0;
   out_6413824007684546309[75] = 0.0;
   out_6413824007684546309[76] = 0.0;
   out_6413824007684546309[77] = 0.0;
   out_6413824007684546309[78] = 0.0;
   out_6413824007684546309[79] = 0.0;
   out_6413824007684546309[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_6030544084839343108) {
   out_6030544084839343108[0] = state[0];
   out_6030544084839343108[1] = state[1];
   out_6030544084839343108[2] = state[2];
   out_6030544084839343108[3] = state[3];
   out_6030544084839343108[4] = state[4];
   out_6030544084839343108[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_6030544084839343108[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_6030544084839343108[7] = state[7];
   out_6030544084839343108[8] = state[8];
}
void F_fun(double *state, double dt, double *out_4730643025875685796) {
   out_4730643025875685796[0] = 1;
   out_4730643025875685796[1] = 0;
   out_4730643025875685796[2] = 0;
   out_4730643025875685796[3] = 0;
   out_4730643025875685796[4] = 0;
   out_4730643025875685796[5] = 0;
   out_4730643025875685796[6] = 0;
   out_4730643025875685796[7] = 0;
   out_4730643025875685796[8] = 0;
   out_4730643025875685796[9] = 0;
   out_4730643025875685796[10] = 1;
   out_4730643025875685796[11] = 0;
   out_4730643025875685796[12] = 0;
   out_4730643025875685796[13] = 0;
   out_4730643025875685796[14] = 0;
   out_4730643025875685796[15] = 0;
   out_4730643025875685796[16] = 0;
   out_4730643025875685796[17] = 0;
   out_4730643025875685796[18] = 0;
   out_4730643025875685796[19] = 0;
   out_4730643025875685796[20] = 1;
   out_4730643025875685796[21] = 0;
   out_4730643025875685796[22] = 0;
   out_4730643025875685796[23] = 0;
   out_4730643025875685796[24] = 0;
   out_4730643025875685796[25] = 0;
   out_4730643025875685796[26] = 0;
   out_4730643025875685796[27] = 0;
   out_4730643025875685796[28] = 0;
   out_4730643025875685796[29] = 0;
   out_4730643025875685796[30] = 1;
   out_4730643025875685796[31] = 0;
   out_4730643025875685796[32] = 0;
   out_4730643025875685796[33] = 0;
   out_4730643025875685796[34] = 0;
   out_4730643025875685796[35] = 0;
   out_4730643025875685796[36] = 0;
   out_4730643025875685796[37] = 0;
   out_4730643025875685796[38] = 0;
   out_4730643025875685796[39] = 0;
   out_4730643025875685796[40] = 1;
   out_4730643025875685796[41] = 0;
   out_4730643025875685796[42] = 0;
   out_4730643025875685796[43] = 0;
   out_4730643025875685796[44] = 0;
   out_4730643025875685796[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_4730643025875685796[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_4730643025875685796[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4730643025875685796[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_4730643025875685796[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_4730643025875685796[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_4730643025875685796[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_4730643025875685796[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_4730643025875685796[53] = -9.8000000000000007*dt;
   out_4730643025875685796[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_4730643025875685796[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_4730643025875685796[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4730643025875685796[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4730643025875685796[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_4730643025875685796[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_4730643025875685796[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_4730643025875685796[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_4730643025875685796[62] = 0;
   out_4730643025875685796[63] = 0;
   out_4730643025875685796[64] = 0;
   out_4730643025875685796[65] = 0;
   out_4730643025875685796[66] = 0;
   out_4730643025875685796[67] = 0;
   out_4730643025875685796[68] = 0;
   out_4730643025875685796[69] = 0;
   out_4730643025875685796[70] = 1;
   out_4730643025875685796[71] = 0;
   out_4730643025875685796[72] = 0;
   out_4730643025875685796[73] = 0;
   out_4730643025875685796[74] = 0;
   out_4730643025875685796[75] = 0;
   out_4730643025875685796[76] = 0;
   out_4730643025875685796[77] = 0;
   out_4730643025875685796[78] = 0;
   out_4730643025875685796[79] = 0;
   out_4730643025875685796[80] = 1;
}
void h_25(double *state, double *unused, double *out_7238773340579693769) {
   out_7238773340579693769[0] = state[6];
}
void H_25(double *state, double *unused, double *out_1618993164756677200) {
   out_1618993164756677200[0] = 0;
   out_1618993164756677200[1] = 0;
   out_1618993164756677200[2] = 0;
   out_1618993164756677200[3] = 0;
   out_1618993164756677200[4] = 0;
   out_1618993164756677200[5] = 0;
   out_1618993164756677200[6] = 1;
   out_1618993164756677200[7] = 0;
   out_1618993164756677200[8] = 0;
}
void h_24(double *state, double *unused, double *out_5222807389593457554) {
   out_5222807389593457554[0] = state[4];
   out_5222807389593457554[1] = state[5];
}
void H_24(double *state, double *unused, double *out_7525873545194199962) {
   out_7525873545194199962[0] = 0;
   out_7525873545194199962[1] = 0;
   out_7525873545194199962[2] = 0;
   out_7525873545194199962[3] = 0;
   out_7525873545194199962[4] = 1;
   out_7525873545194199962[5] = 0;
   out_7525873545194199962[6] = 0;
   out_7525873545194199962[7] = 0;
   out_7525873545194199962[8] = 0;
   out_7525873545194199962[9] = 0;
   out_7525873545194199962[10] = 0;
   out_7525873545194199962[11] = 0;
   out_7525873545194199962[12] = 0;
   out_7525873545194199962[13] = 0;
   out_7525873545194199962[14] = 1;
   out_7525873545194199962[15] = 0;
   out_7525873545194199962[16] = 0;
   out_7525873545194199962[17] = 0;
}
void h_30(double *state, double *unused, double *out_5052523537289197718) {
   out_5052523537289197718[0] = state[4];
}
void H_30(double *state, double *unused, double *out_4137326123263925827) {
   out_4137326123263925827[0] = 0;
   out_4137326123263925827[1] = 0;
   out_4137326123263925827[2] = 0;
   out_4137326123263925827[3] = 0;
   out_4137326123263925827[4] = 1;
   out_4137326123263925827[5] = 0;
   out_4137326123263925827[6] = 0;
   out_4137326123263925827[7] = 0;
   out_4137326123263925827[8] = 0;
}
void h_26(double *state, double *unused, double *out_2239971434019344335) {
   out_2239971434019344335[0] = state[7];
}
void H_26(double *state, double *unused, double *out_2122510154117379024) {
   out_2122510154117379024[0] = 0;
   out_2122510154117379024[1] = 0;
   out_2122510154117379024[2] = 0;
   out_2122510154117379024[3] = 0;
   out_2122510154117379024[4] = 0;
   out_2122510154117379024[5] = 0;
   out_2122510154117379024[6] = 0;
   out_2122510154117379024[7] = 1;
   out_2122510154117379024[8] = 0;
}
void h_27(double *state, double *unused, double *out_8589027207090189811) {
   out_8589027207090189811[0] = state[3];
}
void H_27(double *state, double *unused, double *out_1962562811463500916) {
   out_1962562811463500916[0] = 0;
   out_1962562811463500916[1] = 0;
   out_1962562811463500916[2] = 0;
   out_1962562811463500916[3] = 1;
   out_1962562811463500916[4] = 0;
   out_1962562811463500916[5] = 0;
   out_1962562811463500916[6] = 0;
   out_1962562811463500916[7] = 0;
   out_1962562811463500916[8] = 0;
}
void h_29(double *state, double *unused, double *out_7656928858097895369) {
   out_7656928858097895369[0] = state[1];
}
void H_29(double *state, double *unused, double *out_4647557467578318011) {
   out_4647557467578318011[0] = 0;
   out_4647557467578318011[1] = 1;
   out_4647557467578318011[2] = 0;
   out_4647557467578318011[3] = 0;
   out_4647557467578318011[4] = 0;
   out_4647557467578318011[5] = 0;
   out_4647557467578318011[6] = 0;
   out_4647557467578318011[7] = 0;
   out_4647557467578318011[8] = 0;
}
void h_28(double *state, double *unused, double *out_1139879946750669479) {
   out_1139879946750669479[0] = state[0];
}
void H_28(double *state, double *unused, double *out_434841549491212563) {
   out_434841549491212563[0] = 1;
   out_434841549491212563[1] = 0;
   out_434841549491212563[2] = 0;
   out_434841549491212563[3] = 0;
   out_434841549491212563[4] = 0;
   out_434841549491212563[5] = 0;
   out_434841549491212563[6] = 0;
   out_434841549491212563[7] = 0;
   out_434841549491212563[8] = 0;
}
void h_31(double *state, double *unused, double *out_6963579278295187880) {
   out_6963579278295187880[0] = state[8];
}
void H_31(double *state, double *unused, double *out_2748718256350730500) {
   out_2748718256350730500[0] = 0;
   out_2748718256350730500[1] = 0;
   out_2748718256350730500[2] = 0;
   out_2748718256350730500[3] = 0;
   out_2748718256350730500[4] = 0;
   out_2748718256350730500[5] = 0;
   out_2748718256350730500[6] = 0;
   out_2748718256350730500[7] = 0;
   out_2748718256350730500[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_1476640846964750484) {
  err_fun(nom_x, delta_x, out_1476640846964750484);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1337065726167235347) {
  inv_err_fun(nom_x, true_x, out_1337065726167235347);
}
void car_H_mod_fun(double *state, double *out_6413824007684546309) {
  H_mod_fun(state, out_6413824007684546309);
}
void car_f_fun(double *state, double dt, double *out_6030544084839343108) {
  f_fun(state,  dt, out_6030544084839343108);
}
void car_F_fun(double *state, double dt, double *out_4730643025875685796) {
  F_fun(state,  dt, out_4730643025875685796);
}
void car_h_25(double *state, double *unused, double *out_7238773340579693769) {
  h_25(state, unused, out_7238773340579693769);
}
void car_H_25(double *state, double *unused, double *out_1618993164756677200) {
  H_25(state, unused, out_1618993164756677200);
}
void car_h_24(double *state, double *unused, double *out_5222807389593457554) {
  h_24(state, unused, out_5222807389593457554);
}
void car_H_24(double *state, double *unused, double *out_7525873545194199962) {
  H_24(state, unused, out_7525873545194199962);
}
void car_h_30(double *state, double *unused, double *out_5052523537289197718) {
  h_30(state, unused, out_5052523537289197718);
}
void car_H_30(double *state, double *unused, double *out_4137326123263925827) {
  H_30(state, unused, out_4137326123263925827);
}
void car_h_26(double *state, double *unused, double *out_2239971434019344335) {
  h_26(state, unused, out_2239971434019344335);
}
void car_H_26(double *state, double *unused, double *out_2122510154117379024) {
  H_26(state, unused, out_2122510154117379024);
}
void car_h_27(double *state, double *unused, double *out_8589027207090189811) {
  h_27(state, unused, out_8589027207090189811);
}
void car_H_27(double *state, double *unused, double *out_1962562811463500916) {
  H_27(state, unused, out_1962562811463500916);
}
void car_h_29(double *state, double *unused, double *out_7656928858097895369) {
  h_29(state, unused, out_7656928858097895369);
}
void car_H_29(double *state, double *unused, double *out_4647557467578318011) {
  H_29(state, unused, out_4647557467578318011);
}
void car_h_28(double *state, double *unused, double *out_1139879946750669479) {
  h_28(state, unused, out_1139879946750669479);
}
void car_H_28(double *state, double *unused, double *out_434841549491212563) {
  H_28(state, unused, out_434841549491212563);
}
void car_h_31(double *state, double *unused, double *out_6963579278295187880) {
  h_31(state, unused, out_6963579278295187880);
}
void car_H_31(double *state, double *unused, double *out_2748718256350730500) {
  H_31(state, unused, out_2748718256350730500);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
