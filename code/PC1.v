
module PC1
(
 input [64:0] key,
 output [28:0] cbits,
 output [28:0] dbits
);

assign cbits[1]  = key[57];
assign cbits[2]  = key[49];
assign cbits[3]  = key[41];
assign cbits[4]  = key[33];
assign cbits[5]  = key[25];
assign cbits[6]  = key[17];
assign cbits[7]  = key[9];
assign cbits[8]  = key[1];
assign cbits[9] = key[58];
assign cbits[10] = key[50];
assign cbits[11] = key[42];
assign cbits[12] = key[34];
assign cbits[13] = key[26];
assign cbits[14] = key[18];
assign cbits[15] = key[10];
assign cbits[16] = key[2];
assign cbits[17] = key[59];
assign cbits[18] = key[51];
assign cbits[19] = key[43];
assign cbits[20] = key[35];
assign cbits[21] = key[27];
assign cbits[22] = key[19];
assign cbits[23] = key[11];
assign cbits[24] = key[3];
assign cbits[25] = key[60];
assign cbits[26] = key[52];
assign cbits[27] = key[44];
assign cbits[28] = key[36];
assign dbits[1] = key[63];
assign dbits[2] = key[55];
assign dbits[3] = key[47];
assign dbits[4] = key[39];
assign dbits[5] = key[31];
assign dbits[6] = key[23];
assign dbits[7] = key[15];
assign dbits[8] = key[7];
assign dbits[9] = key[62];
assign dbits[10] = key[54];
assign dbits[11] = key[46];
assign dbits[12] = key[38];
assign dbits[13] = key[30];
assign dbits[14] = key[22];
assign dbits[15] = key[14];
assign dbits[16] = key[6];
assign dbits[17] = key[61];
assign dbits[18] = key[53];
assign dbits[19] = key[45];
assign dbits[20] = key[37];
assign dbits[21] = key[29];
assign dbits[22] = key[21];
assign dbits[23] = key[13];
assign dbits[24] = key[5];
assign dbits[25] = key[28];
assign dbits[26] = key[20];
assign dbits[27] = key[12];
assign dbits[28] = key[4];


endmodule
