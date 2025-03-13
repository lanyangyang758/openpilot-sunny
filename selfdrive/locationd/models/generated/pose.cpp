#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.13.2                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5969624291114388899) {
   out_5969624291114388899[0] = delta_x[0] + nom_x[0];
   out_5969624291114388899[1] = delta_x[1] + nom_x[1];
   out_5969624291114388899[2] = delta_x[2] + nom_x[2];
   out_5969624291114388899[3] = delta_x[3] + nom_x[3];
   out_5969624291114388899[4] = delta_x[4] + nom_x[4];
   out_5969624291114388899[5] = delta_x[5] + nom_x[5];
   out_5969624291114388899[6] = delta_x[6] + nom_x[6];
   out_5969624291114388899[7] = delta_x[7] + nom_x[7];
   out_5969624291114388899[8] = delta_x[8] + nom_x[8];
   out_5969624291114388899[9] = delta_x[9] + nom_x[9];
   out_5969624291114388899[10] = delta_x[10] + nom_x[10];
   out_5969624291114388899[11] = delta_x[11] + nom_x[11];
   out_5969624291114388899[12] = delta_x[12] + nom_x[12];
   out_5969624291114388899[13] = delta_x[13] + nom_x[13];
   out_5969624291114388899[14] = delta_x[14] + nom_x[14];
   out_5969624291114388899[15] = delta_x[15] + nom_x[15];
   out_5969624291114388899[16] = delta_x[16] + nom_x[16];
   out_5969624291114388899[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_118426523294258694) {
   out_118426523294258694[0] = -nom_x[0] + true_x[0];
   out_118426523294258694[1] = -nom_x[1] + true_x[1];
   out_118426523294258694[2] = -nom_x[2] + true_x[2];
   out_118426523294258694[3] = -nom_x[3] + true_x[3];
   out_118426523294258694[4] = -nom_x[4] + true_x[4];
   out_118426523294258694[5] = -nom_x[5] + true_x[5];
   out_118426523294258694[6] = -nom_x[6] + true_x[6];
   out_118426523294258694[7] = -nom_x[7] + true_x[7];
   out_118426523294258694[8] = -nom_x[8] + true_x[8];
   out_118426523294258694[9] = -nom_x[9] + true_x[9];
   out_118426523294258694[10] = -nom_x[10] + true_x[10];
   out_118426523294258694[11] = -nom_x[11] + true_x[11];
   out_118426523294258694[12] = -nom_x[12] + true_x[12];
   out_118426523294258694[13] = -nom_x[13] + true_x[13];
   out_118426523294258694[14] = -nom_x[14] + true_x[14];
   out_118426523294258694[15] = -nom_x[15] + true_x[15];
   out_118426523294258694[16] = -nom_x[16] + true_x[16];
   out_118426523294258694[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4844240443325636947) {
   out_4844240443325636947[0] = 1.0;
   out_4844240443325636947[1] = 0.0;
   out_4844240443325636947[2] = 0.0;
   out_4844240443325636947[3] = 0.0;
   out_4844240443325636947[4] = 0.0;
   out_4844240443325636947[5] = 0.0;
   out_4844240443325636947[6] = 0.0;
   out_4844240443325636947[7] = 0.0;
   out_4844240443325636947[8] = 0.0;
   out_4844240443325636947[9] = 0.0;
   out_4844240443325636947[10] = 0.0;
   out_4844240443325636947[11] = 0.0;
   out_4844240443325636947[12] = 0.0;
   out_4844240443325636947[13] = 0.0;
   out_4844240443325636947[14] = 0.0;
   out_4844240443325636947[15] = 0.0;
   out_4844240443325636947[16] = 0.0;
   out_4844240443325636947[17] = 0.0;
   out_4844240443325636947[18] = 0.0;
   out_4844240443325636947[19] = 1.0;
   out_4844240443325636947[20] = 0.0;
   out_4844240443325636947[21] = 0.0;
   out_4844240443325636947[22] = 0.0;
   out_4844240443325636947[23] = 0.0;
   out_4844240443325636947[24] = 0.0;
   out_4844240443325636947[25] = 0.0;
   out_4844240443325636947[26] = 0.0;
   out_4844240443325636947[27] = 0.0;
   out_4844240443325636947[28] = 0.0;
   out_4844240443325636947[29] = 0.0;
   out_4844240443325636947[30] = 0.0;
   out_4844240443325636947[31] = 0.0;
   out_4844240443325636947[32] = 0.0;
   out_4844240443325636947[33] = 0.0;
   out_4844240443325636947[34] = 0.0;
   out_4844240443325636947[35] = 0.0;
   out_4844240443325636947[36] = 0.0;
   out_4844240443325636947[37] = 0.0;
   out_4844240443325636947[38] = 1.0;
   out_4844240443325636947[39] = 0.0;
   out_4844240443325636947[40] = 0.0;
   out_4844240443325636947[41] = 0.0;
   out_4844240443325636947[42] = 0.0;
   out_4844240443325636947[43] = 0.0;
   out_4844240443325636947[44] = 0.0;
   out_4844240443325636947[45] = 0.0;
   out_4844240443325636947[46] = 0.0;
   out_4844240443325636947[47] = 0.0;
   out_4844240443325636947[48] = 0.0;
   out_4844240443325636947[49] = 0.0;
   out_4844240443325636947[50] = 0.0;
   out_4844240443325636947[51] = 0.0;
   out_4844240443325636947[52] = 0.0;
   out_4844240443325636947[53] = 0.0;
   out_4844240443325636947[54] = 0.0;
   out_4844240443325636947[55] = 0.0;
   out_4844240443325636947[56] = 0.0;
   out_4844240443325636947[57] = 1.0;
   out_4844240443325636947[58] = 0.0;
   out_4844240443325636947[59] = 0.0;
   out_4844240443325636947[60] = 0.0;
   out_4844240443325636947[61] = 0.0;
   out_4844240443325636947[62] = 0.0;
   out_4844240443325636947[63] = 0.0;
   out_4844240443325636947[64] = 0.0;
   out_4844240443325636947[65] = 0.0;
   out_4844240443325636947[66] = 0.0;
   out_4844240443325636947[67] = 0.0;
   out_4844240443325636947[68] = 0.0;
   out_4844240443325636947[69] = 0.0;
   out_4844240443325636947[70] = 0.0;
   out_4844240443325636947[71] = 0.0;
   out_4844240443325636947[72] = 0.0;
   out_4844240443325636947[73] = 0.0;
   out_4844240443325636947[74] = 0.0;
   out_4844240443325636947[75] = 0.0;
   out_4844240443325636947[76] = 1.0;
   out_4844240443325636947[77] = 0.0;
   out_4844240443325636947[78] = 0.0;
   out_4844240443325636947[79] = 0.0;
   out_4844240443325636947[80] = 0.0;
   out_4844240443325636947[81] = 0.0;
   out_4844240443325636947[82] = 0.0;
   out_4844240443325636947[83] = 0.0;
   out_4844240443325636947[84] = 0.0;
   out_4844240443325636947[85] = 0.0;
   out_4844240443325636947[86] = 0.0;
   out_4844240443325636947[87] = 0.0;
   out_4844240443325636947[88] = 0.0;
   out_4844240443325636947[89] = 0.0;
   out_4844240443325636947[90] = 0.0;
   out_4844240443325636947[91] = 0.0;
   out_4844240443325636947[92] = 0.0;
   out_4844240443325636947[93] = 0.0;
   out_4844240443325636947[94] = 0.0;
   out_4844240443325636947[95] = 1.0;
   out_4844240443325636947[96] = 0.0;
   out_4844240443325636947[97] = 0.0;
   out_4844240443325636947[98] = 0.0;
   out_4844240443325636947[99] = 0.0;
   out_4844240443325636947[100] = 0.0;
   out_4844240443325636947[101] = 0.0;
   out_4844240443325636947[102] = 0.0;
   out_4844240443325636947[103] = 0.0;
   out_4844240443325636947[104] = 0.0;
   out_4844240443325636947[105] = 0.0;
   out_4844240443325636947[106] = 0.0;
   out_4844240443325636947[107] = 0.0;
   out_4844240443325636947[108] = 0.0;
   out_4844240443325636947[109] = 0.0;
   out_4844240443325636947[110] = 0.0;
   out_4844240443325636947[111] = 0.0;
   out_4844240443325636947[112] = 0.0;
   out_4844240443325636947[113] = 0.0;
   out_4844240443325636947[114] = 1.0;
   out_4844240443325636947[115] = 0.0;
   out_4844240443325636947[116] = 0.0;
   out_4844240443325636947[117] = 0.0;
   out_4844240443325636947[118] = 0.0;
   out_4844240443325636947[119] = 0.0;
   out_4844240443325636947[120] = 0.0;
   out_4844240443325636947[121] = 0.0;
   out_4844240443325636947[122] = 0.0;
   out_4844240443325636947[123] = 0.0;
   out_4844240443325636947[124] = 0.0;
   out_4844240443325636947[125] = 0.0;
   out_4844240443325636947[126] = 0.0;
   out_4844240443325636947[127] = 0.0;
   out_4844240443325636947[128] = 0.0;
   out_4844240443325636947[129] = 0.0;
   out_4844240443325636947[130] = 0.0;
   out_4844240443325636947[131] = 0.0;
   out_4844240443325636947[132] = 0.0;
   out_4844240443325636947[133] = 1.0;
   out_4844240443325636947[134] = 0.0;
   out_4844240443325636947[135] = 0.0;
   out_4844240443325636947[136] = 0.0;
   out_4844240443325636947[137] = 0.0;
   out_4844240443325636947[138] = 0.0;
   out_4844240443325636947[139] = 0.0;
   out_4844240443325636947[140] = 0.0;
   out_4844240443325636947[141] = 0.0;
   out_4844240443325636947[142] = 0.0;
   out_4844240443325636947[143] = 0.0;
   out_4844240443325636947[144] = 0.0;
   out_4844240443325636947[145] = 0.0;
   out_4844240443325636947[146] = 0.0;
   out_4844240443325636947[147] = 0.0;
   out_4844240443325636947[148] = 0.0;
   out_4844240443325636947[149] = 0.0;
   out_4844240443325636947[150] = 0.0;
   out_4844240443325636947[151] = 0.0;
   out_4844240443325636947[152] = 1.0;
   out_4844240443325636947[153] = 0.0;
   out_4844240443325636947[154] = 0.0;
   out_4844240443325636947[155] = 0.0;
   out_4844240443325636947[156] = 0.0;
   out_4844240443325636947[157] = 0.0;
   out_4844240443325636947[158] = 0.0;
   out_4844240443325636947[159] = 0.0;
   out_4844240443325636947[160] = 0.0;
   out_4844240443325636947[161] = 0.0;
   out_4844240443325636947[162] = 0.0;
   out_4844240443325636947[163] = 0.0;
   out_4844240443325636947[164] = 0.0;
   out_4844240443325636947[165] = 0.0;
   out_4844240443325636947[166] = 0.0;
   out_4844240443325636947[167] = 0.0;
   out_4844240443325636947[168] = 0.0;
   out_4844240443325636947[169] = 0.0;
   out_4844240443325636947[170] = 0.0;
   out_4844240443325636947[171] = 1.0;
   out_4844240443325636947[172] = 0.0;
   out_4844240443325636947[173] = 0.0;
   out_4844240443325636947[174] = 0.0;
   out_4844240443325636947[175] = 0.0;
   out_4844240443325636947[176] = 0.0;
   out_4844240443325636947[177] = 0.0;
   out_4844240443325636947[178] = 0.0;
   out_4844240443325636947[179] = 0.0;
   out_4844240443325636947[180] = 0.0;
   out_4844240443325636947[181] = 0.0;
   out_4844240443325636947[182] = 0.0;
   out_4844240443325636947[183] = 0.0;
   out_4844240443325636947[184] = 0.0;
   out_4844240443325636947[185] = 0.0;
   out_4844240443325636947[186] = 0.0;
   out_4844240443325636947[187] = 0.0;
   out_4844240443325636947[188] = 0.0;
   out_4844240443325636947[189] = 0.0;
   out_4844240443325636947[190] = 1.0;
   out_4844240443325636947[191] = 0.0;
   out_4844240443325636947[192] = 0.0;
   out_4844240443325636947[193] = 0.0;
   out_4844240443325636947[194] = 0.0;
   out_4844240443325636947[195] = 0.0;
   out_4844240443325636947[196] = 0.0;
   out_4844240443325636947[197] = 0.0;
   out_4844240443325636947[198] = 0.0;
   out_4844240443325636947[199] = 0.0;
   out_4844240443325636947[200] = 0.0;
   out_4844240443325636947[201] = 0.0;
   out_4844240443325636947[202] = 0.0;
   out_4844240443325636947[203] = 0.0;
   out_4844240443325636947[204] = 0.0;
   out_4844240443325636947[205] = 0.0;
   out_4844240443325636947[206] = 0.0;
   out_4844240443325636947[207] = 0.0;
   out_4844240443325636947[208] = 0.0;
   out_4844240443325636947[209] = 1.0;
   out_4844240443325636947[210] = 0.0;
   out_4844240443325636947[211] = 0.0;
   out_4844240443325636947[212] = 0.0;
   out_4844240443325636947[213] = 0.0;
   out_4844240443325636947[214] = 0.0;
   out_4844240443325636947[215] = 0.0;
   out_4844240443325636947[216] = 0.0;
   out_4844240443325636947[217] = 0.0;
   out_4844240443325636947[218] = 0.0;
   out_4844240443325636947[219] = 0.0;
   out_4844240443325636947[220] = 0.0;
   out_4844240443325636947[221] = 0.0;
   out_4844240443325636947[222] = 0.0;
   out_4844240443325636947[223] = 0.0;
   out_4844240443325636947[224] = 0.0;
   out_4844240443325636947[225] = 0.0;
   out_4844240443325636947[226] = 0.0;
   out_4844240443325636947[227] = 0.0;
   out_4844240443325636947[228] = 1.0;
   out_4844240443325636947[229] = 0.0;
   out_4844240443325636947[230] = 0.0;
   out_4844240443325636947[231] = 0.0;
   out_4844240443325636947[232] = 0.0;
   out_4844240443325636947[233] = 0.0;
   out_4844240443325636947[234] = 0.0;
   out_4844240443325636947[235] = 0.0;
   out_4844240443325636947[236] = 0.0;
   out_4844240443325636947[237] = 0.0;
   out_4844240443325636947[238] = 0.0;
   out_4844240443325636947[239] = 0.0;
   out_4844240443325636947[240] = 0.0;
   out_4844240443325636947[241] = 0.0;
   out_4844240443325636947[242] = 0.0;
   out_4844240443325636947[243] = 0.0;
   out_4844240443325636947[244] = 0.0;
   out_4844240443325636947[245] = 0.0;
   out_4844240443325636947[246] = 0.0;
   out_4844240443325636947[247] = 1.0;
   out_4844240443325636947[248] = 0.0;
   out_4844240443325636947[249] = 0.0;
   out_4844240443325636947[250] = 0.0;
   out_4844240443325636947[251] = 0.0;
   out_4844240443325636947[252] = 0.0;
   out_4844240443325636947[253] = 0.0;
   out_4844240443325636947[254] = 0.0;
   out_4844240443325636947[255] = 0.0;
   out_4844240443325636947[256] = 0.0;
   out_4844240443325636947[257] = 0.0;
   out_4844240443325636947[258] = 0.0;
   out_4844240443325636947[259] = 0.0;
   out_4844240443325636947[260] = 0.0;
   out_4844240443325636947[261] = 0.0;
   out_4844240443325636947[262] = 0.0;
   out_4844240443325636947[263] = 0.0;
   out_4844240443325636947[264] = 0.0;
   out_4844240443325636947[265] = 0.0;
   out_4844240443325636947[266] = 1.0;
   out_4844240443325636947[267] = 0.0;
   out_4844240443325636947[268] = 0.0;
   out_4844240443325636947[269] = 0.0;
   out_4844240443325636947[270] = 0.0;
   out_4844240443325636947[271] = 0.0;
   out_4844240443325636947[272] = 0.0;
   out_4844240443325636947[273] = 0.0;
   out_4844240443325636947[274] = 0.0;
   out_4844240443325636947[275] = 0.0;
   out_4844240443325636947[276] = 0.0;
   out_4844240443325636947[277] = 0.0;
   out_4844240443325636947[278] = 0.0;
   out_4844240443325636947[279] = 0.0;
   out_4844240443325636947[280] = 0.0;
   out_4844240443325636947[281] = 0.0;
   out_4844240443325636947[282] = 0.0;
   out_4844240443325636947[283] = 0.0;
   out_4844240443325636947[284] = 0.0;
   out_4844240443325636947[285] = 1.0;
   out_4844240443325636947[286] = 0.0;
   out_4844240443325636947[287] = 0.0;
   out_4844240443325636947[288] = 0.0;
   out_4844240443325636947[289] = 0.0;
   out_4844240443325636947[290] = 0.0;
   out_4844240443325636947[291] = 0.0;
   out_4844240443325636947[292] = 0.0;
   out_4844240443325636947[293] = 0.0;
   out_4844240443325636947[294] = 0.0;
   out_4844240443325636947[295] = 0.0;
   out_4844240443325636947[296] = 0.0;
   out_4844240443325636947[297] = 0.0;
   out_4844240443325636947[298] = 0.0;
   out_4844240443325636947[299] = 0.0;
   out_4844240443325636947[300] = 0.0;
   out_4844240443325636947[301] = 0.0;
   out_4844240443325636947[302] = 0.0;
   out_4844240443325636947[303] = 0.0;
   out_4844240443325636947[304] = 1.0;
   out_4844240443325636947[305] = 0.0;
   out_4844240443325636947[306] = 0.0;
   out_4844240443325636947[307] = 0.0;
   out_4844240443325636947[308] = 0.0;
   out_4844240443325636947[309] = 0.0;
   out_4844240443325636947[310] = 0.0;
   out_4844240443325636947[311] = 0.0;
   out_4844240443325636947[312] = 0.0;
   out_4844240443325636947[313] = 0.0;
   out_4844240443325636947[314] = 0.0;
   out_4844240443325636947[315] = 0.0;
   out_4844240443325636947[316] = 0.0;
   out_4844240443325636947[317] = 0.0;
   out_4844240443325636947[318] = 0.0;
   out_4844240443325636947[319] = 0.0;
   out_4844240443325636947[320] = 0.0;
   out_4844240443325636947[321] = 0.0;
   out_4844240443325636947[322] = 0.0;
   out_4844240443325636947[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_7218946590605097822) {
   out_7218946590605097822[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_7218946590605097822[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_7218946590605097822[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_7218946590605097822[3] = dt*state[12] + state[3];
   out_7218946590605097822[4] = dt*state[13] + state[4];
   out_7218946590605097822[5] = dt*state[14] + state[5];
   out_7218946590605097822[6] = state[6];
   out_7218946590605097822[7] = state[7];
   out_7218946590605097822[8] = state[8];
   out_7218946590605097822[9] = state[9];
   out_7218946590605097822[10] = state[10];
   out_7218946590605097822[11] = state[11];
   out_7218946590605097822[12] = state[12];
   out_7218946590605097822[13] = state[13];
   out_7218946590605097822[14] = state[14];
   out_7218946590605097822[15] = state[15];
   out_7218946590605097822[16] = state[16];
   out_7218946590605097822[17] = state[17];
}
void F_fun(double *state, double dt, double *out_6776632317341105523) {
   out_6776632317341105523[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6776632317341105523[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6776632317341105523[2] = 0;
   out_6776632317341105523[3] = 0;
   out_6776632317341105523[4] = 0;
   out_6776632317341105523[5] = 0;
   out_6776632317341105523[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6776632317341105523[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6776632317341105523[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_6776632317341105523[9] = 0;
   out_6776632317341105523[10] = 0;
   out_6776632317341105523[11] = 0;
   out_6776632317341105523[12] = 0;
   out_6776632317341105523[13] = 0;
   out_6776632317341105523[14] = 0;
   out_6776632317341105523[15] = 0;
   out_6776632317341105523[16] = 0;
   out_6776632317341105523[17] = 0;
   out_6776632317341105523[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6776632317341105523[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6776632317341105523[20] = 0;
   out_6776632317341105523[21] = 0;
   out_6776632317341105523[22] = 0;
   out_6776632317341105523[23] = 0;
   out_6776632317341105523[24] = 0;
   out_6776632317341105523[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6776632317341105523[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_6776632317341105523[27] = 0;
   out_6776632317341105523[28] = 0;
   out_6776632317341105523[29] = 0;
   out_6776632317341105523[30] = 0;
   out_6776632317341105523[31] = 0;
   out_6776632317341105523[32] = 0;
   out_6776632317341105523[33] = 0;
   out_6776632317341105523[34] = 0;
   out_6776632317341105523[35] = 0;
   out_6776632317341105523[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6776632317341105523[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6776632317341105523[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6776632317341105523[39] = 0;
   out_6776632317341105523[40] = 0;
   out_6776632317341105523[41] = 0;
   out_6776632317341105523[42] = 0;
   out_6776632317341105523[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6776632317341105523[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_6776632317341105523[45] = 0;
   out_6776632317341105523[46] = 0;
   out_6776632317341105523[47] = 0;
   out_6776632317341105523[48] = 0;
   out_6776632317341105523[49] = 0;
   out_6776632317341105523[50] = 0;
   out_6776632317341105523[51] = 0;
   out_6776632317341105523[52] = 0;
   out_6776632317341105523[53] = 0;
   out_6776632317341105523[54] = 0;
   out_6776632317341105523[55] = 0;
   out_6776632317341105523[56] = 0;
   out_6776632317341105523[57] = 1;
   out_6776632317341105523[58] = 0;
   out_6776632317341105523[59] = 0;
   out_6776632317341105523[60] = 0;
   out_6776632317341105523[61] = 0;
   out_6776632317341105523[62] = 0;
   out_6776632317341105523[63] = 0;
   out_6776632317341105523[64] = 0;
   out_6776632317341105523[65] = 0;
   out_6776632317341105523[66] = dt;
   out_6776632317341105523[67] = 0;
   out_6776632317341105523[68] = 0;
   out_6776632317341105523[69] = 0;
   out_6776632317341105523[70] = 0;
   out_6776632317341105523[71] = 0;
   out_6776632317341105523[72] = 0;
   out_6776632317341105523[73] = 0;
   out_6776632317341105523[74] = 0;
   out_6776632317341105523[75] = 0;
   out_6776632317341105523[76] = 1;
   out_6776632317341105523[77] = 0;
   out_6776632317341105523[78] = 0;
   out_6776632317341105523[79] = 0;
   out_6776632317341105523[80] = 0;
   out_6776632317341105523[81] = 0;
   out_6776632317341105523[82] = 0;
   out_6776632317341105523[83] = 0;
   out_6776632317341105523[84] = 0;
   out_6776632317341105523[85] = dt;
   out_6776632317341105523[86] = 0;
   out_6776632317341105523[87] = 0;
   out_6776632317341105523[88] = 0;
   out_6776632317341105523[89] = 0;
   out_6776632317341105523[90] = 0;
   out_6776632317341105523[91] = 0;
   out_6776632317341105523[92] = 0;
   out_6776632317341105523[93] = 0;
   out_6776632317341105523[94] = 0;
   out_6776632317341105523[95] = 1;
   out_6776632317341105523[96] = 0;
   out_6776632317341105523[97] = 0;
   out_6776632317341105523[98] = 0;
   out_6776632317341105523[99] = 0;
   out_6776632317341105523[100] = 0;
   out_6776632317341105523[101] = 0;
   out_6776632317341105523[102] = 0;
   out_6776632317341105523[103] = 0;
   out_6776632317341105523[104] = dt;
   out_6776632317341105523[105] = 0;
   out_6776632317341105523[106] = 0;
   out_6776632317341105523[107] = 0;
   out_6776632317341105523[108] = 0;
   out_6776632317341105523[109] = 0;
   out_6776632317341105523[110] = 0;
   out_6776632317341105523[111] = 0;
   out_6776632317341105523[112] = 0;
   out_6776632317341105523[113] = 0;
   out_6776632317341105523[114] = 1;
   out_6776632317341105523[115] = 0;
   out_6776632317341105523[116] = 0;
   out_6776632317341105523[117] = 0;
   out_6776632317341105523[118] = 0;
   out_6776632317341105523[119] = 0;
   out_6776632317341105523[120] = 0;
   out_6776632317341105523[121] = 0;
   out_6776632317341105523[122] = 0;
   out_6776632317341105523[123] = 0;
   out_6776632317341105523[124] = 0;
   out_6776632317341105523[125] = 0;
   out_6776632317341105523[126] = 0;
   out_6776632317341105523[127] = 0;
   out_6776632317341105523[128] = 0;
   out_6776632317341105523[129] = 0;
   out_6776632317341105523[130] = 0;
   out_6776632317341105523[131] = 0;
   out_6776632317341105523[132] = 0;
   out_6776632317341105523[133] = 1;
   out_6776632317341105523[134] = 0;
   out_6776632317341105523[135] = 0;
   out_6776632317341105523[136] = 0;
   out_6776632317341105523[137] = 0;
   out_6776632317341105523[138] = 0;
   out_6776632317341105523[139] = 0;
   out_6776632317341105523[140] = 0;
   out_6776632317341105523[141] = 0;
   out_6776632317341105523[142] = 0;
   out_6776632317341105523[143] = 0;
   out_6776632317341105523[144] = 0;
   out_6776632317341105523[145] = 0;
   out_6776632317341105523[146] = 0;
   out_6776632317341105523[147] = 0;
   out_6776632317341105523[148] = 0;
   out_6776632317341105523[149] = 0;
   out_6776632317341105523[150] = 0;
   out_6776632317341105523[151] = 0;
   out_6776632317341105523[152] = 1;
   out_6776632317341105523[153] = 0;
   out_6776632317341105523[154] = 0;
   out_6776632317341105523[155] = 0;
   out_6776632317341105523[156] = 0;
   out_6776632317341105523[157] = 0;
   out_6776632317341105523[158] = 0;
   out_6776632317341105523[159] = 0;
   out_6776632317341105523[160] = 0;
   out_6776632317341105523[161] = 0;
   out_6776632317341105523[162] = 0;
   out_6776632317341105523[163] = 0;
   out_6776632317341105523[164] = 0;
   out_6776632317341105523[165] = 0;
   out_6776632317341105523[166] = 0;
   out_6776632317341105523[167] = 0;
   out_6776632317341105523[168] = 0;
   out_6776632317341105523[169] = 0;
   out_6776632317341105523[170] = 0;
   out_6776632317341105523[171] = 1;
   out_6776632317341105523[172] = 0;
   out_6776632317341105523[173] = 0;
   out_6776632317341105523[174] = 0;
   out_6776632317341105523[175] = 0;
   out_6776632317341105523[176] = 0;
   out_6776632317341105523[177] = 0;
   out_6776632317341105523[178] = 0;
   out_6776632317341105523[179] = 0;
   out_6776632317341105523[180] = 0;
   out_6776632317341105523[181] = 0;
   out_6776632317341105523[182] = 0;
   out_6776632317341105523[183] = 0;
   out_6776632317341105523[184] = 0;
   out_6776632317341105523[185] = 0;
   out_6776632317341105523[186] = 0;
   out_6776632317341105523[187] = 0;
   out_6776632317341105523[188] = 0;
   out_6776632317341105523[189] = 0;
   out_6776632317341105523[190] = 1;
   out_6776632317341105523[191] = 0;
   out_6776632317341105523[192] = 0;
   out_6776632317341105523[193] = 0;
   out_6776632317341105523[194] = 0;
   out_6776632317341105523[195] = 0;
   out_6776632317341105523[196] = 0;
   out_6776632317341105523[197] = 0;
   out_6776632317341105523[198] = 0;
   out_6776632317341105523[199] = 0;
   out_6776632317341105523[200] = 0;
   out_6776632317341105523[201] = 0;
   out_6776632317341105523[202] = 0;
   out_6776632317341105523[203] = 0;
   out_6776632317341105523[204] = 0;
   out_6776632317341105523[205] = 0;
   out_6776632317341105523[206] = 0;
   out_6776632317341105523[207] = 0;
   out_6776632317341105523[208] = 0;
   out_6776632317341105523[209] = 1;
   out_6776632317341105523[210] = 0;
   out_6776632317341105523[211] = 0;
   out_6776632317341105523[212] = 0;
   out_6776632317341105523[213] = 0;
   out_6776632317341105523[214] = 0;
   out_6776632317341105523[215] = 0;
   out_6776632317341105523[216] = 0;
   out_6776632317341105523[217] = 0;
   out_6776632317341105523[218] = 0;
   out_6776632317341105523[219] = 0;
   out_6776632317341105523[220] = 0;
   out_6776632317341105523[221] = 0;
   out_6776632317341105523[222] = 0;
   out_6776632317341105523[223] = 0;
   out_6776632317341105523[224] = 0;
   out_6776632317341105523[225] = 0;
   out_6776632317341105523[226] = 0;
   out_6776632317341105523[227] = 0;
   out_6776632317341105523[228] = 1;
   out_6776632317341105523[229] = 0;
   out_6776632317341105523[230] = 0;
   out_6776632317341105523[231] = 0;
   out_6776632317341105523[232] = 0;
   out_6776632317341105523[233] = 0;
   out_6776632317341105523[234] = 0;
   out_6776632317341105523[235] = 0;
   out_6776632317341105523[236] = 0;
   out_6776632317341105523[237] = 0;
   out_6776632317341105523[238] = 0;
   out_6776632317341105523[239] = 0;
   out_6776632317341105523[240] = 0;
   out_6776632317341105523[241] = 0;
   out_6776632317341105523[242] = 0;
   out_6776632317341105523[243] = 0;
   out_6776632317341105523[244] = 0;
   out_6776632317341105523[245] = 0;
   out_6776632317341105523[246] = 0;
   out_6776632317341105523[247] = 1;
   out_6776632317341105523[248] = 0;
   out_6776632317341105523[249] = 0;
   out_6776632317341105523[250] = 0;
   out_6776632317341105523[251] = 0;
   out_6776632317341105523[252] = 0;
   out_6776632317341105523[253] = 0;
   out_6776632317341105523[254] = 0;
   out_6776632317341105523[255] = 0;
   out_6776632317341105523[256] = 0;
   out_6776632317341105523[257] = 0;
   out_6776632317341105523[258] = 0;
   out_6776632317341105523[259] = 0;
   out_6776632317341105523[260] = 0;
   out_6776632317341105523[261] = 0;
   out_6776632317341105523[262] = 0;
   out_6776632317341105523[263] = 0;
   out_6776632317341105523[264] = 0;
   out_6776632317341105523[265] = 0;
   out_6776632317341105523[266] = 1;
   out_6776632317341105523[267] = 0;
   out_6776632317341105523[268] = 0;
   out_6776632317341105523[269] = 0;
   out_6776632317341105523[270] = 0;
   out_6776632317341105523[271] = 0;
   out_6776632317341105523[272] = 0;
   out_6776632317341105523[273] = 0;
   out_6776632317341105523[274] = 0;
   out_6776632317341105523[275] = 0;
   out_6776632317341105523[276] = 0;
   out_6776632317341105523[277] = 0;
   out_6776632317341105523[278] = 0;
   out_6776632317341105523[279] = 0;
   out_6776632317341105523[280] = 0;
   out_6776632317341105523[281] = 0;
   out_6776632317341105523[282] = 0;
   out_6776632317341105523[283] = 0;
   out_6776632317341105523[284] = 0;
   out_6776632317341105523[285] = 1;
   out_6776632317341105523[286] = 0;
   out_6776632317341105523[287] = 0;
   out_6776632317341105523[288] = 0;
   out_6776632317341105523[289] = 0;
   out_6776632317341105523[290] = 0;
   out_6776632317341105523[291] = 0;
   out_6776632317341105523[292] = 0;
   out_6776632317341105523[293] = 0;
   out_6776632317341105523[294] = 0;
   out_6776632317341105523[295] = 0;
   out_6776632317341105523[296] = 0;
   out_6776632317341105523[297] = 0;
   out_6776632317341105523[298] = 0;
   out_6776632317341105523[299] = 0;
   out_6776632317341105523[300] = 0;
   out_6776632317341105523[301] = 0;
   out_6776632317341105523[302] = 0;
   out_6776632317341105523[303] = 0;
   out_6776632317341105523[304] = 1;
   out_6776632317341105523[305] = 0;
   out_6776632317341105523[306] = 0;
   out_6776632317341105523[307] = 0;
   out_6776632317341105523[308] = 0;
   out_6776632317341105523[309] = 0;
   out_6776632317341105523[310] = 0;
   out_6776632317341105523[311] = 0;
   out_6776632317341105523[312] = 0;
   out_6776632317341105523[313] = 0;
   out_6776632317341105523[314] = 0;
   out_6776632317341105523[315] = 0;
   out_6776632317341105523[316] = 0;
   out_6776632317341105523[317] = 0;
   out_6776632317341105523[318] = 0;
   out_6776632317341105523[319] = 0;
   out_6776632317341105523[320] = 0;
   out_6776632317341105523[321] = 0;
   out_6776632317341105523[322] = 0;
   out_6776632317341105523[323] = 1;
}
void h_4(double *state, double *unused, double *out_4775284966991639497) {
   out_4775284966991639497[0] = state[6] + state[9];
   out_4775284966991639497[1] = state[7] + state[10];
   out_4775284966991639497[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_2314075146619284269) {
   out_2314075146619284269[0] = 0;
   out_2314075146619284269[1] = 0;
   out_2314075146619284269[2] = 0;
   out_2314075146619284269[3] = 0;
   out_2314075146619284269[4] = 0;
   out_2314075146619284269[5] = 0;
   out_2314075146619284269[6] = 1;
   out_2314075146619284269[7] = 0;
   out_2314075146619284269[8] = 0;
   out_2314075146619284269[9] = 1;
   out_2314075146619284269[10] = 0;
   out_2314075146619284269[11] = 0;
   out_2314075146619284269[12] = 0;
   out_2314075146619284269[13] = 0;
   out_2314075146619284269[14] = 0;
   out_2314075146619284269[15] = 0;
   out_2314075146619284269[16] = 0;
   out_2314075146619284269[17] = 0;
   out_2314075146619284269[18] = 0;
   out_2314075146619284269[19] = 0;
   out_2314075146619284269[20] = 0;
   out_2314075146619284269[21] = 0;
   out_2314075146619284269[22] = 0;
   out_2314075146619284269[23] = 0;
   out_2314075146619284269[24] = 0;
   out_2314075146619284269[25] = 1;
   out_2314075146619284269[26] = 0;
   out_2314075146619284269[27] = 0;
   out_2314075146619284269[28] = 1;
   out_2314075146619284269[29] = 0;
   out_2314075146619284269[30] = 0;
   out_2314075146619284269[31] = 0;
   out_2314075146619284269[32] = 0;
   out_2314075146619284269[33] = 0;
   out_2314075146619284269[34] = 0;
   out_2314075146619284269[35] = 0;
   out_2314075146619284269[36] = 0;
   out_2314075146619284269[37] = 0;
   out_2314075146619284269[38] = 0;
   out_2314075146619284269[39] = 0;
   out_2314075146619284269[40] = 0;
   out_2314075146619284269[41] = 0;
   out_2314075146619284269[42] = 0;
   out_2314075146619284269[43] = 0;
   out_2314075146619284269[44] = 1;
   out_2314075146619284269[45] = 0;
   out_2314075146619284269[46] = 0;
   out_2314075146619284269[47] = 1;
   out_2314075146619284269[48] = 0;
   out_2314075146619284269[49] = 0;
   out_2314075146619284269[50] = 0;
   out_2314075146619284269[51] = 0;
   out_2314075146619284269[52] = 0;
   out_2314075146619284269[53] = 0;
}
void h_10(double *state, double *unused, double *out_8636542913891770906) {
   out_8636542913891770906[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_8636542913891770906[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_8636542913891770906[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5365847866423626156) {
   out_5365847866423626156[0] = 0;
   out_5365847866423626156[1] = 9.8100000000000005*cos(state[1]);
   out_5365847866423626156[2] = 0;
   out_5365847866423626156[3] = 0;
   out_5365847866423626156[4] = -state[8];
   out_5365847866423626156[5] = state[7];
   out_5365847866423626156[6] = 0;
   out_5365847866423626156[7] = state[5];
   out_5365847866423626156[8] = -state[4];
   out_5365847866423626156[9] = 0;
   out_5365847866423626156[10] = 0;
   out_5365847866423626156[11] = 0;
   out_5365847866423626156[12] = 1;
   out_5365847866423626156[13] = 0;
   out_5365847866423626156[14] = 0;
   out_5365847866423626156[15] = 1;
   out_5365847866423626156[16] = 0;
   out_5365847866423626156[17] = 0;
   out_5365847866423626156[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5365847866423626156[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5365847866423626156[20] = 0;
   out_5365847866423626156[21] = state[8];
   out_5365847866423626156[22] = 0;
   out_5365847866423626156[23] = -state[6];
   out_5365847866423626156[24] = -state[5];
   out_5365847866423626156[25] = 0;
   out_5365847866423626156[26] = state[3];
   out_5365847866423626156[27] = 0;
   out_5365847866423626156[28] = 0;
   out_5365847866423626156[29] = 0;
   out_5365847866423626156[30] = 0;
   out_5365847866423626156[31] = 1;
   out_5365847866423626156[32] = 0;
   out_5365847866423626156[33] = 0;
   out_5365847866423626156[34] = 1;
   out_5365847866423626156[35] = 0;
   out_5365847866423626156[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5365847866423626156[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5365847866423626156[38] = 0;
   out_5365847866423626156[39] = -state[7];
   out_5365847866423626156[40] = state[6];
   out_5365847866423626156[41] = 0;
   out_5365847866423626156[42] = state[4];
   out_5365847866423626156[43] = -state[3];
   out_5365847866423626156[44] = 0;
   out_5365847866423626156[45] = 0;
   out_5365847866423626156[46] = 0;
   out_5365847866423626156[47] = 0;
   out_5365847866423626156[48] = 0;
   out_5365847866423626156[49] = 0;
   out_5365847866423626156[50] = 1;
   out_5365847866423626156[51] = 0;
   out_5365847866423626156[52] = 0;
   out_5365847866423626156[53] = 1;
}
void h_13(double *state, double *unused, double *out_5134342860672958820) {
   out_5134342860672958820[0] = state[3];
   out_5134342860672958820[1] = state[4];
   out_5134342860672958820[2] = state[5];
}
void H_13(double *state, double *unused, double *out_5296556061697416660) {
   out_5296556061697416660[0] = 0;
   out_5296556061697416660[1] = 0;
   out_5296556061697416660[2] = 0;
   out_5296556061697416660[3] = 1;
   out_5296556061697416660[4] = 0;
   out_5296556061697416660[5] = 0;
   out_5296556061697416660[6] = 0;
   out_5296556061697416660[7] = 0;
   out_5296556061697416660[8] = 0;
   out_5296556061697416660[9] = 0;
   out_5296556061697416660[10] = 0;
   out_5296556061697416660[11] = 0;
   out_5296556061697416660[12] = 0;
   out_5296556061697416660[13] = 0;
   out_5296556061697416660[14] = 0;
   out_5296556061697416660[15] = 0;
   out_5296556061697416660[16] = 0;
   out_5296556061697416660[17] = 0;
   out_5296556061697416660[18] = 0;
   out_5296556061697416660[19] = 0;
   out_5296556061697416660[20] = 0;
   out_5296556061697416660[21] = 0;
   out_5296556061697416660[22] = 1;
   out_5296556061697416660[23] = 0;
   out_5296556061697416660[24] = 0;
   out_5296556061697416660[25] = 0;
   out_5296556061697416660[26] = 0;
   out_5296556061697416660[27] = 0;
   out_5296556061697416660[28] = 0;
   out_5296556061697416660[29] = 0;
   out_5296556061697416660[30] = 0;
   out_5296556061697416660[31] = 0;
   out_5296556061697416660[32] = 0;
   out_5296556061697416660[33] = 0;
   out_5296556061697416660[34] = 0;
   out_5296556061697416660[35] = 0;
   out_5296556061697416660[36] = 0;
   out_5296556061697416660[37] = 0;
   out_5296556061697416660[38] = 0;
   out_5296556061697416660[39] = 0;
   out_5296556061697416660[40] = 0;
   out_5296556061697416660[41] = 1;
   out_5296556061697416660[42] = 0;
   out_5296556061697416660[43] = 0;
   out_5296556061697416660[44] = 0;
   out_5296556061697416660[45] = 0;
   out_5296556061697416660[46] = 0;
   out_5296556061697416660[47] = 0;
   out_5296556061697416660[48] = 0;
   out_5296556061697416660[49] = 0;
   out_5296556061697416660[50] = 0;
   out_5296556061697416660[51] = 0;
   out_5296556061697416660[52] = 0;
   out_5296556061697416660[53] = 0;
}
void h_14(double *state, double *unused, double *out_2499754816787507650) {
   out_2499754816787507650[0] = state[6];
   out_2499754816787507650[1] = state[7];
   out_2499754816787507650[2] = state[8];
}
void H_14(double *state, double *unused, double *out_5396863578914656565) {
   out_5396863578914656565[0] = 0;
   out_5396863578914656565[1] = 0;
   out_5396863578914656565[2] = 0;
   out_5396863578914656565[3] = 0;
   out_5396863578914656565[4] = 0;
   out_5396863578914656565[5] = 0;
   out_5396863578914656565[6] = 1;
   out_5396863578914656565[7] = 0;
   out_5396863578914656565[8] = 0;
   out_5396863578914656565[9] = 0;
   out_5396863578914656565[10] = 0;
   out_5396863578914656565[11] = 0;
   out_5396863578914656565[12] = 0;
   out_5396863578914656565[13] = 0;
   out_5396863578914656565[14] = 0;
   out_5396863578914656565[15] = 0;
   out_5396863578914656565[16] = 0;
   out_5396863578914656565[17] = 0;
   out_5396863578914656565[18] = 0;
   out_5396863578914656565[19] = 0;
   out_5396863578914656565[20] = 0;
   out_5396863578914656565[21] = 0;
   out_5396863578914656565[22] = 0;
   out_5396863578914656565[23] = 0;
   out_5396863578914656565[24] = 0;
   out_5396863578914656565[25] = 1;
   out_5396863578914656565[26] = 0;
   out_5396863578914656565[27] = 0;
   out_5396863578914656565[28] = 0;
   out_5396863578914656565[29] = 0;
   out_5396863578914656565[30] = 0;
   out_5396863578914656565[31] = 0;
   out_5396863578914656565[32] = 0;
   out_5396863578914656565[33] = 0;
   out_5396863578914656565[34] = 0;
   out_5396863578914656565[35] = 0;
   out_5396863578914656565[36] = 0;
   out_5396863578914656565[37] = 0;
   out_5396863578914656565[38] = 0;
   out_5396863578914656565[39] = 0;
   out_5396863578914656565[40] = 0;
   out_5396863578914656565[41] = 0;
   out_5396863578914656565[42] = 0;
   out_5396863578914656565[43] = 0;
   out_5396863578914656565[44] = 1;
   out_5396863578914656565[45] = 0;
   out_5396863578914656565[46] = 0;
   out_5396863578914656565[47] = 0;
   out_5396863578914656565[48] = 0;
   out_5396863578914656565[49] = 0;
   out_5396863578914656565[50] = 0;
   out_5396863578914656565[51] = 0;
   out_5396863578914656565[52] = 0;
   out_5396863578914656565[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_5969624291114388899) {
  err_fun(nom_x, delta_x, out_5969624291114388899);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_118426523294258694) {
  inv_err_fun(nom_x, true_x, out_118426523294258694);
}
void pose_H_mod_fun(double *state, double *out_4844240443325636947) {
  H_mod_fun(state, out_4844240443325636947);
}
void pose_f_fun(double *state, double dt, double *out_7218946590605097822) {
  f_fun(state,  dt, out_7218946590605097822);
}
void pose_F_fun(double *state, double dt, double *out_6776632317341105523) {
  F_fun(state,  dt, out_6776632317341105523);
}
void pose_h_4(double *state, double *unused, double *out_4775284966991639497) {
  h_4(state, unused, out_4775284966991639497);
}
void pose_H_4(double *state, double *unused, double *out_2314075146619284269) {
  H_4(state, unused, out_2314075146619284269);
}
void pose_h_10(double *state, double *unused, double *out_8636542913891770906) {
  h_10(state, unused, out_8636542913891770906);
}
void pose_H_10(double *state, double *unused, double *out_5365847866423626156) {
  H_10(state, unused, out_5365847866423626156);
}
void pose_h_13(double *state, double *unused, double *out_5134342860672958820) {
  h_13(state, unused, out_5134342860672958820);
}
void pose_H_13(double *state, double *unused, double *out_5296556061697416660) {
  H_13(state, unused, out_5296556061697416660);
}
void pose_h_14(double *state, double *unused, double *out_2499754816787507650) {
  h_14(state, unused, out_2499754816787507650);
}
void pose_H_14(double *state, double *unused, double *out_5396863578914656565) {
  H_14(state, unused, out_5396863578914656565);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
