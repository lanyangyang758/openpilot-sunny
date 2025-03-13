#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_5969624291114388899);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_118426523294258694);
void pose_H_mod_fun(double *state, double *out_4844240443325636947);
void pose_f_fun(double *state, double dt, double *out_7218946590605097822);
void pose_F_fun(double *state, double dt, double *out_6776632317341105523);
void pose_h_4(double *state, double *unused, double *out_4775284966991639497);
void pose_H_4(double *state, double *unused, double *out_2314075146619284269);
void pose_h_10(double *state, double *unused, double *out_8636542913891770906);
void pose_H_10(double *state, double *unused, double *out_5365847866423626156);
void pose_h_13(double *state, double *unused, double *out_5134342860672958820);
void pose_H_13(double *state, double *unused, double *out_5296556061697416660);
void pose_h_14(double *state, double *unused, double *out_2499754816787507650);
void pose_H_14(double *state, double *unused, double *out_5396863578914656565);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}