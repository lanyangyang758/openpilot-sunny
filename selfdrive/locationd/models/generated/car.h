#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_1476640846964750484);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1337065726167235347);
void car_H_mod_fun(double *state, double *out_6413824007684546309);
void car_f_fun(double *state, double dt, double *out_6030544084839343108);
void car_F_fun(double *state, double dt, double *out_4730643025875685796);
void car_h_25(double *state, double *unused, double *out_7238773340579693769);
void car_H_25(double *state, double *unused, double *out_1618993164756677200);
void car_h_24(double *state, double *unused, double *out_5222807389593457554);
void car_H_24(double *state, double *unused, double *out_7525873545194199962);
void car_h_30(double *state, double *unused, double *out_5052523537289197718);
void car_H_30(double *state, double *unused, double *out_4137326123263925827);
void car_h_26(double *state, double *unused, double *out_2239971434019344335);
void car_H_26(double *state, double *unused, double *out_2122510154117379024);
void car_h_27(double *state, double *unused, double *out_8589027207090189811);
void car_H_27(double *state, double *unused, double *out_1962562811463500916);
void car_h_29(double *state, double *unused, double *out_7656928858097895369);
void car_H_29(double *state, double *unused, double *out_4647557467578318011);
void car_h_28(double *state, double *unused, double *out_1139879946750669479);
void car_H_28(double *state, double *unused, double *out_434841549491212563);
void car_h_31(double *state, double *unused, double *out_6963579278295187880);
void car_H_31(double *state, double *unused, double *out_2748718256350730500);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}