module E
(
 input [31:0] data_in,
 output [47:0] data_out
);

assign data_out[0]  = data_in[31];
assign data_out[1]  = data_in[0];
assign data_out[2]  = data_in[1];
assign data_out[3]  = data_in[2];
assign data_out[4]  = data_in[3];
assign data_out[5]  = data_in[4];
assign data_out[6]  = data_in[3];
assign data_out[7]  = data_in[4];
assign data_out[8]  = data_in[5];
assign data_out[9]  = data_in[6];
assign data_out[10] = data_in[7];
assign data_out[11] = data_in[8];
assign data_out[12] = data_in[7];
assign data_out[13] = data_in[8];
assign data_out[14] = data_in[9];
assign data_out[15] = data_in[10];
assign data_out[16] = data_in[11];
assign data_out[17] = data_in[12];
assign data_out[18] = data_in[11];
assign data_out[19] = data_in[12];
assign data_out[20] = data_in[13];
assign data_out[21] = data_in[14];
assign data_out[22] = data_in[15];
assign data_out[23] = data_in[16];
assign data_out[24] = data_in[15];
assign data_out[25] = data_in[16];
assign data_out[26] = data_in[17];
assign data_out[27] = data_in[18];
assign data_out[28] = data_in[19];
assign data_out[29] = data_in[20];
assign data_out[30] = data_in[19];
assign data_out[31] = data_in[20];
assign data_out[32] = data_in[21];
assign data_out[33] = data_in[22];
assign data_out[34] = data_in[23];
assign data_out[35] = data_in[24];
assign data_out[36] = data_in[23];
assign data_out[37] = data_in[24];
assign data_out[38] = data_in[25];
assign data_out[39] = data_in[26];
assign data_out[40] = data_in[27];
assign data_out[41] = data_in[28];
assign data_out[42] = data_in[27];
assign data_out[43] = data_in[28];
assign data_out[44] = data_in[29];
assign data_out[45] = data_in[30];
assign data_out[46] = data_in[31];
assign data_out[47] = data_in[0];

endmodule



module IP
(
 input [63:0] data_in,
 output [63:0] data_out
);

assign data_out[0] = data_in[57];
assign data_out[1] = data_in[49];
assign data_out[2] = data_in[41];
assign data_out[3] = data_in[33];
assign data_out[4] = data_in[25];
assign data_out[5] = data_in[17];
assign data_out[6] = data_in[9];
assign data_out[7] = data_in[1];
assign data_out[8] = data_in[59];
assign data_out[9] = data_in[51];
assign data_out[10] = data_in[43];
assign data_out[11] = data_in[35];
assign data_out[12] = data_in[27];
assign data_out[13] = data_in[19];
assign data_out[14] = data_in[11];
assign data_out[15] = data_in[3];
assign data_out[16] = data_in[61];
assign data_out[17] = data_in[53];
assign data_out[18] = data_in[45];
assign data_out[19] = data_in[37];
assign data_out[20] = data_in[29];
assign data_out[21] = data_in[21];
assign data_out[22] = data_in[13];
assign data_out[23] = data_in[5];
assign data_out[24] = data_in[63];
assign data_out[25] = data_in[55];
assign data_out[26] = data_in[47];
assign data_out[27] = data_in[39];
assign data_out[28] = data_in[31];
assign data_out[29] = data_in[23];
assign data_out[30] = data_in[15];
assign data_out[31] = data_in[7];
assign data_out[32] = data_in[56];
assign data_out[33] = data_in[48];
assign data_out[34] = data_in[40];
assign data_out[35] = data_in[32];
assign data_out[36] = data_in[24];
assign data_out[37] = data_in[16];
assign data_out[38] = data_in[8];
assign data_out[39] = data_in[0];
assign data_out[40] = data_in[58];
assign data_out[41] = data_in[50];
assign data_out[42] = data_in[42];
assign data_out[43] = data_in[34];
assign data_out[44] = data_in[26];
assign data_out[45] = data_in[18];
assign data_out[46] = data_in[10];
assign data_out[47] = data_in[2];
assign data_out[48] = data_in[60];
assign data_out[49] = data_in[52];
assign data_out[50] = data_in[44];
assign data_out[51] = data_in[36];
assign data_out[52] = data_in[28];
assign data_out[53] = data_in[20];
assign data_out[54] = data_in[12];
assign data_out[55] = data_in[4];
assign data_out[56] = data_in[62];
assign data_out[57] = data_in[54];
assign data_out[58] = data_in[46];
assign data_out[59] = data_in[38];
assign data_out[60] = data_in[30];
assign data_out[61] = data_in[22];
assign data_out[62] = data_in[14];
assign data_out[63] = data_in[6];

endmodule



module P
(
 input [31:0] data_in,
 output [31:0] data_out
);

assign data_out[0]  = data_in[15];
assign data_out[1]  = data_in[6];
assign data_out[2]  = data_in[19];
assign data_out[3]  = data_in[20];
assign data_out[4]  = data_in[28];
assign data_out[5]  = data_in[11];
assign data_out[6]  = data_in[27];
assign data_out[7]  = data_in[16];
assign data_out[8]  = data_in[0];
assign data_out[9]  = data_in[14];
assign data_out[10] = data_in[22];
assign data_out[11] = data_in[25];
assign data_out[12] = data_in[4];
assign data_out[13] = data_in[17];
assign data_out[14] = data_in[30];
assign data_out[15] = data_in[9];
assign data_out[16] = data_in[1];
assign data_out[17] = data_in[7];
assign data_out[18] = data_in[23];
assign data_out[19] = data_in[13];
assign data_out[20] = data_in[31];
assign data_out[21] = data_in[26];
assign data_out[22] = data_in[2];
assign data_out[23] = data_in[8];
assign data_out[24] = data_in[18];
assign data_out[25] = data_in[12];
assign data_out[26] = data_in[29];
assign data_out[27] = data_in[5];
assign data_out[28] = data_in[21];
assign data_out[29] = data_in[10];
assign data_out[30] = data_in[3];
assign data_out[31] = data_in[24];

endmodule




module PC1
(
 input [63:0] key,
 output [27:0] cbits,
 output [27:0] dbits
);

assign cbits[0]  = key[56];
assign cbits[1]  = key[48];
assign cbits[2]  = key[40];
assign cbits[3]  = key[32];
assign cbits[4]  = key[24];
assign cbits[5]  = key[16];
assign cbits[6]  = key[8];
assign cbits[7]  = key[0];
assign cbits[8]  = key[57];
assign cbits[9]  = key[49];
assign cbits[10] = key[41];
assign cbits[11] = key[33];
assign cbits[12] = key[25];
assign cbits[13] = key[17];
assign cbits[14] = key[9];
assign cbits[15] = key[1];
assign cbits[16] = key[58];
assign cbits[17] = key[50];
assign cbits[18] = key[42];
assign cbits[19] = key[34];
assign cbits[20] = key[26];
assign cbits[21] = key[18];
assign cbits[22] = key[10];
assign cbits[23] = key[2];
assign cbits[24] = key[59];
assign cbits[25] = key[51];
assign cbits[26] = key[43];
assign cbits[27] = key[35];
assign dbits[0]  = key[62];
assign dbits[1]  = key[54];
assign dbits[2]  = key[46];
assign dbits[3]  = key[38];
assign dbits[4]  = key[30];
assign dbits[5]  = key[22];
assign dbits[6]  = key[14];
assign dbits[7]  = key[6];
assign dbits[8]  = key[61];
assign dbits[9]  = key[53];
assign dbits[10] = key[45];
assign dbits[11] = key[37];
assign dbits[12] = key[29];
assign dbits[13] = key[21];
assign dbits[14] = key[13];
assign dbits[15] = key[5];
assign dbits[16] = key[27];
assign dbits[17] = key[19];
assign dbits[18] = key[11];
assign dbits[19] = key[3];
assign dbits[20] = key[26];
assign dbits[21] = key[18];
assign dbits[22] = key[10];
assign dbits[23] = key[2];
assign dbits[24] = key[27];
assign dbits[25] = key[19];
assign dbits[26] = key[11];
assign dbits[27] = key[3];



endmodule



module PC2
(
  input      [55:0] data_in,
  output     [47:0] data_out
);

assign data_out[0]  = data_in[13];
assign data_out[1]  = data_in[16];
assign data_out[2]  = data_in[10];
assign data_out[3]  = data_in[23];
assign data_out[4]  = data_in[0];
assign data_out[5]  = data_in[4];
assign data_out[6]  = data_in[2];
assign data_out[7]  = data_in[27];
assign data_out[8]  = data_in[14];
assign data_out[9]  = data_in[5];
assign data_out[10] = data_in[20];
assign data_out[11] = data_in[9];
assign data_out[12] = data_in[22];
assign data_out[13] = data_in[18];
assign data_out[14] = data_in[11];
assign data_out[15] = data_in[3];
assign data_out[16] = data_in[25];
assign data_out[17] = data_in[7];
assign data_out[18] = data_in[15];
assign data_out[19] = data_in[6];
assign data_out[20] = data_in[26];
assign data_out[21] = data_in[19];
assign data_out[22] = data_in[12];
assign data_out[23] = data_in[1];
assign data_out[24] = data_in[40];
assign data_out[25] = data_in[51];
assign data_out[26] = data_in[30];
assign data_out[27] = data_in[36];
assign data_out[28] = data_in[46];
assign data_out[29] = data_in[54];
assign data_out[30] = data_in[29];
assign data_out[31] = data_in[39];
assign data_out[32] = data_in[50];
assign data_out[33] = data_in[44];
assign data_out[34] = data_in[32];
assign data_out[35] = data_in[47];
assign data_out[36] = data_in[43];
assign data_out[37] = data_in[48];
assign data_out[38] = data_in[38];
assign data_out[39] = data_in[55];
assign data_out[40] = data_in[33];
assign data_out[41] = data_in[52];
assign data_out[42] = data_in[45];
assign data_out[43] = data_in[41];
assign data_out[44] = data_in[49];
assign data_out[45] = data_in[35];
assign data_out[46] = data_in[28];
assign data_out[47] = data_in[31];



endmodule



module feistel_network 
  (
    input  wire [63:0] plaintext,
    
    input  wire [47:0] K1,
    input  wire [47:0] K2,
    input  wire [47:0] K3,
    input  wire [47:0] K4,
    input  wire [47:0] K5,
    input  wire [47:0] K6,
    input  wire [47:0] K7,
    input  wire [47:0] K8,
    input  wire [47:0] K9,
    input  wire [47:0] K10,
    input  wire [47:0] K11,
    input  wire [47:0] K12,
    input  wire [47:0] K13,
    input  wire [47:0] K14,
    input  wire [47:0] K15,
    input  wire [47:0] K16,    
    
    output wire [63:0] ciphertext
  );

    function [31:0] bitwise_plus(input [31:0] a, b);
      begin
        bitwise_plus[0] = a[0] + b[0];
        bitwise_plus[1] = a[1] + b[1];
        bitwise_plus[2] = a[2] + b[2];
        bitwise_plus[3] = a[3] + b[3];
        bitwise_plus[4] = a[4] + b[4];
        bitwise_plus[5] = a[5] + b[5];
        bitwise_plus[6] = a[6] + b[6];
        bitwise_plus[7] = a[7] + b[7];
        bitwise_plus[8] = a[8] + b[8];
        bitwise_plus[9] = a[9] + b[9];
        bitwise_plus[10] = a[10] + b[10];
        bitwise_plus[11] = a[11] + b[11];
        bitwise_plus[12] = a[12] + b[12];
        bitwise_plus[13] = a[13] + b[13];
        bitwise_plus[14] = a[14] + b[14];
        bitwise_plus[15] = a[15] + b[15];
        bitwise_plus[16] = a[16] + b[16];
        bitwise_plus[17] = a[17] + b[17];
        bitwise_plus[18] = a[18] + b[18];
        bitwise_plus[19] = a[19] + b[19];
        bitwise_plus[20] = a[20] + b[20];
        bitwise_plus[21] = a[21] + b[21];
        bitwise_plus[22] = a[22] + b[22];
        bitwise_plus[23] = a[23] + b[23];
        bitwise_plus[24] = a[24] + b[24];
        bitwise_plus[25] = a[25] + b[25];
        bitwise_plus[26] = a[26] + b[26];
        bitwise_plus[27] = a[27] + b[27];
        bitwise_plus[28] = a[28] + b[28];
        bitwise_plus[29] = a[29] + b[29];
        bitwise_plus[30] = a[30] + b[30];
        bitwise_plus[31] = a[31] + b[31];
      end
    endfunction


    wire [31:0] L0;  wire [31:0] R0;
    reg  [31:0] L1;  reg  [31:0] R1;
    reg  [31:0] L2;  reg  [31:0] R2;
    reg  [31:0] L3;  reg  [31:0] R3;
    reg  [31:0] L4;  reg  [31:0] R4;
    reg  [31:0] L5;  reg  [31:0] R5;
    reg  [31:0] L6;  reg  [31:0] R6;
    reg  [31:0] L7;  reg  [31:0] R7;
    reg  [31:0] L8;  reg  [31:0] R8;
    reg  [31:0] L9;  reg  [31:0] R9;
    reg  [31:0] L10; reg  [31:0] R10;
    reg  [31:0] L11; reg  [31:0] R11;
    reg  [31:0] L12; reg  [31:0] R12;
    reg  [31:0] L13; reg  [31:0] R13;
    reg  [31:0] L14; reg  [31:0] R14;
    reg  [31:0] L15; reg  [31:0] R15;
    reg  [31:0] L16; reg  [31:0] R16;


    IP Initial_Permutation
      (
        .data_in(plaintext),
        .data_out({L0, R0})
      );

    wire [31:0] f1_out;
    wire [31:0] f2_out;
    wire [31:0] f3_out;
    wire [31:0] f4_out;
    wire [31:0] f5_out;
    wire [31:0] f6_out;
    wire [31:0] f7_out;
    wire [31:0] f8_out;
    wire [31:0] f9_out;
    wire [31:0] f10_out;
    wire [31:0] f11_out;
    wire [31:0] f12_out;
    wire [31:0] f13_out;
    wire [31:0] f14_out;
    wire [31:0] f15_out;
    wire [31:0] f16_out;

    f_function f1 (.R(R0), .Key(K1), .f_out(f1_out));
    f_function f2 (.R(R1), .Key(K2), .f_out(f2_out));
    f_function f3 (.R(R2), .Key(K3), .f_out(f3_out));
    f_function f4 (.R(R3), .Key(K4), .f_out(f4_out));
    f_function f5 (.R(R4), .Key(K5), .f_out(f5_out));
    f_function f6 (.R(R5), .Key(K6), .f_out(f6_out));
    f_function f7 (.R(R6), .Key(K7), .f_out(f7_out));
    f_function f8 (.R(R7), .Key(K8), .f_out(f8_out));
    f_function f9 (.R(R8), .Key(K9), .f_out(f9_out));
    f_function f10 (.R(R9), .Key(K10), .f_out(f10_out));
    f_function f11 (.R(R10), .Key(K11), .f_out(f11_out));
    f_function f12 (.R(R11), .Key(K12), .f_out(f12_out));
    f_function f13 (.R(R12), .Key(K13), .f_out(f13_out));
    f_function f14 (.R(R13), .Key(K14), .f_out(f14_out));
    f_function f15 (.R(R14), .Key(K15), .f_out(f15_out));
    f_function f16 (.R(R15), .Key(K16), .f_out(f16_out));


    always @(*) 
	begin
        R1 = bitwise_plus(L0, f1_out); L1 = R0;
        R2 = bitwise_plus(L1, f2_out); L2 = R1;
        R3 = bitwise_plus(L2, f3_out); L3 = R2;
        R4 = bitwise_plus(L3, f4_out); L4 = R3;
        R5 = bitwise_plus(L4, f5_out); L5 = R4;
        R6 = bitwise_plus(L5, f6_out); L6 = R5;
        R7 = bitwise_plus(L6, f7_out); L7 = R6;
        R8 = bitwise_plus(L7, f8_out); L8 = R7;
        R9 = bitwise_plus(L8, f9_out); L9 = R8;
        R10 = bitwise_plus(L9, f10_out); L10 = R9;
        R11 = bitwise_plus(L10, f11_out); L11 = R10;
        R12 = bitwise_plus(L11, f12_out); L12 = R11;
        R13 = bitwise_plus(L12, f13_out); L13 = R12;
        R14 = bitwise_plus(L13, f14_out); L14 = R13;
        R15 = bitwise_plus(L14, f15_out); L15 = R14;
        R16 = bitwise_plus(L15, f16_out); L16 = R15;
    end

    i_IP Inverse_Initial_Permutation (.data_in({R16, L16}), .data_out(ciphertext));

endmodule



module f_function

(
 input   wire  [31:0] R,
 input   wire  [47:0] Key,
 output  wire  [31:0] f_out
);

wire [47:0] E_out;
wire [47:0] S_in;
wire [31:0] P_in;
wire [31:0] P_out;
wire [3:0]  p1;
wire [3:0]  p2;
wire [3:0]  p3;
wire [3:0]  p4;
wire [3:0]  p5;
wire [3:0]  p6;
wire [3:0]  p7;
wire [3:0]  p8;

assign S_in = Key ^ E_out;
assign P_in = { p1 , p2 , p3 , p4 , p5 , p6 , p7 , p8 };
assign f_out = P_out;

E    instan_of_E  (.data_in(R)         , .data_out(E_out));
s1   instan_of_s1 (.s1_in(S_in[47:42]) , .s1_out(p1));   
s2   instan_of_s2 (.s2_in(S_in[41:36]) , .s2_out(p2));
s3   instan_of_s3 (.s3_in(S_in[35:30]) , .s3_out(p3));
s4   instan_of_s4 (.s4_in(S_in[29:24]) , .s4_out(p4));
s5   instan_of_s5 (.s5_in(S_in[23:18]) , .s5_out(p5));
s6   instan_of_s6 (.s6_in(S_in[17:12]) , .s6_out(p6));
s7   instan_of_s7 (.s7_in(S_in[11:6])  , .s7_out(p7));
s8   instan_of_s8 (.s8_in(S_in[5:0])   , .s8_out(p8));
P    instan_of_P  (.data_in(P_in)      , .data_out(P_out));                

endmodule



module i_IP
(
 input [63:0] data_in,
 output [63:0] data_out
);

assign data_out[0] = data_in[39];
assign data_out[1] = data_in[7];
assign data_out[2] = data_in[47];
assign data_out[3] = data_in[15];
assign data_out[4] = data_in[55];
assign data_out[5] = data_in[23];
assign data_out[6] = data_in[63];
assign data_out[7] = data_in[31];
assign data_out[8] = data_in[38];
assign data_out[9] = data_in[6];
assign data_out[10] = data_in[46];
assign data_out[11] = data_in[14];
assign data_out[12] = data_in[54];
assign data_out[13] = data_in[22];
assign data_out[14] = data_in[62];
assign data_out[15] = data_in[30];
assign data_out[16] = data_in[37];
assign data_out[17] = data_in[5];
assign data_out[18] = data_in[45];
assign data_out[19] = data_in[13];
assign data_out[20] = data_in[53];
assign data_out[21] = data_in[21];
assign data_out[22] = data_in[61];
assign data_out[23] = data_in[29];
assign data_out[24] = data_in[36];
assign data_out[25] = data_in[4];
assign data_out[26] = data_in[44];
assign data_out[27] = data_in[12];
assign data_out[28] = data_in[52];
assign data_out[29] = data_in[20];
assign data_out[30] = data_in[60];
assign data_out[31] = data_in[28];
assign data_out[32] = data_in[35];
assign data_out[33] = data_in[3];
assign data_out[34] = data_in[43];
assign data_out[35] = data_in[11];
assign data_out[36] = data_in[51];
assign data_out[37] = data_in[19];
assign data_out[38] = data_in[59];
assign data_out[39] = data_in[27];
assign data_out[40] = data_in[34];
assign data_out[41] = data_in[2];
assign data_out[42] = data_in[42];
assign data_out[43] = data_in[10];
assign data_out[44] = data_in[50];
assign data_out[45] = data_in[18];
assign data_out[46] = data_in[58];
assign data_out[47] = data_in[26];
assign data_out[48] = data_in[33];
assign data_out[49] = data_in[1];
assign data_out[50] = data_in[41];
assign data_out[51] = data_in[9];
assign data_out[52] = data_in[49];
assign data_out[53] = data_in[17];
assign data_out[54] = data_in[57];
assign data_out[55] = data_in[25];
assign data_out[56] = data_in[32];
assign data_out[57] = data_in[0];
assign data_out[58] = data_in[40];
assign data_out[59] = data_in[8];
assign data_out[60] = data_in[48];
assign data_out[61] = data_in[16];
assign data_out[62] = data_in[56];
assign data_out[63] = data_in[24];

endmodule



module input_register
  (
    input  wire        clk,
    input  wire        reset,
    input  wire [63:0] data_in,
    input  wire        load,
    output reg  [63:0] data_out
  );

reg [63:0] data;

always @(posedge clk)
  if (reset)
    data <= 64'b0;
  else
    if (load == 1'b1)
      data <= data_in;

always @(data)
  data_out <= data;

endmodule



module key_schedule
(
   input wire  [63:0] key_in,
   output wire [47:0] K1,
   output wire [47:0] K2,
   output wire [47:0] K3,
   output wire [47:0] K4,
   output wire [47:0] K5,
   output wire [47:0] K6,
   output wire [47:0] K7,
   output wire [47:0] K8,
   output wire [47:0] K9,
   output wire [47:0] K10,
   output wire [47:0] K11,
   output wire [47:0] K12,
   output wire [47:0] K13,
   output wire [47:0] K14,
   output wire [47:0] K15,
   output wire [47:0] K16
);

wire [27:0] c0;
wire [27:0] d0;
reg  [27:0] c1;
reg  [27:0] d1;
reg  [27:0] c2;
reg  [27:0] d2;
reg  [27:0] c3;
reg  [27:0] d3;
reg  [27:0] c4;
reg  [27:0] d4;
reg  [27:0] c5;
reg  [27:0] d5;
reg  [27:0] c6;
reg  [27:0] d6;
reg  [27:0] c7;
reg  [27:0] d7;
reg  [27:0] c8;
reg  [27:0] d8;
reg  [27:0] c9;
reg  [27:0] d9;
reg  [27:0] c10;
reg  [27:0] d10;
reg  [27:0] c11;
reg  [27:0] d11;
reg  [27:0] c12;
reg  [27:0] d12;
reg  [27:0] c13;
reg  [27:0] d13;
reg  [27:0] c14;
reg  [27:0] d14;
reg  [27:0] c15;
reg  [27:0] d15;
reg  [27:0] c16;
reg  [27:0] d16;


always @(*)
    begin  
        // 0->1 one shift
        d1 = {d0[26:0], 1'b0};
        c1 = {c0[26:0], 1'b0};
        // 1->2 one shift
        d2 = {d1[26:0], 1'b0};
        c2 = {c1[26:0], 1'b0};     
        // 2->3 two shift
        d3 = {d2[25:0], 2'b0};
        c3 = {c2[25:0], 2'b0};     
        // 3->4 two shift
        d4 = {d3[25:0], 2'b0};
        c4 = {c3[25:0], 2'b0};      
        // 4->5 two shift
        d5 = {d4[25:0], 2'b0};
        c5 = {c4[25:0], 2'b0};      
        // 5->6 two shift
        d6 = {d5[25:0], 2'b0};
        c6 = {c5[25:0], 2'b0};     
        // 6->7 two shift 
        d7 = {d6[25:0], 2'b0};
        c7 = {c6[25:0], 2'b0};     
        // 7->8 two shift 
        d8 = {d7[25:0], 2'b0};
        c8 = {c7[25:0], 2'b0};    
        // 8->9 one shift 
        d9 = {d8[25:0], 2'b0};
        c9 = {c8[25:0], 2'b0};     
        // 9->10 two shift
        d10 = {d9[25:0], 2'b0};
        c10 = {c9[25:0], 2'b0};     
        // 10->11 two shift
        d11 = {d10[25:0], 2'b0};
        c11 = {c10[25:0], 2'b0};   
        // 11->12 two shift 
        d12 = {d11[25:0], 2'b0};
        c12 = {c11[25:0], 2'b0};   
        // 12->13 two shift
        d13 = {d12[25:0], 2'b0};
        c13 = {c12[25:0], 2'b0};    
        // 13->14 two shift
        d14 = {d13[25:0], 2'b0};
        c14 = {c13[25:0], 2'b0};   
        // 14->15 two shift
        d15 = {d14[25:0], 2'b0};
        c15 = {c14[25:0], 2'b0};  
        // 15->16 one shift
        d16 = {d15[26:0], 1'b0};
        c16 = {c15[26:0], 1'b0};
    end


 PC1 PC1_0  (.key(key_in), .cbits(c0), .dbits(d0));
 
PC2 PC2_1  (.data_in({c1, d1}  ), .data_out(K1) );
PC2 PC2_2  (.data_in({c2, d2}  ), .data_out(K2) );
PC2 PC2_3  (.data_in({c3, d3}  ), .data_out(K3) );
PC2 PC2_4  (.data_in({c4, d4}  ), .data_out(K4) );
PC2 PC2_5  (.data_in({c5, d5}  ), .data_out(K5) );
PC2 PC2_6  (.data_in({c6, d6}  ), .data_out(K6) );
PC2 PC2_7  (.data_in({c7, d7}  ), .data_out(K7) );
PC2 PC2_8  (.data_in({c8, d8}  ), .data_out(K8) );
PC2 PC2_9  (.data_in({c9, d9}  ), .data_out(K9) );
PC2 PC2_10 (.data_in({c10, d10}), .data_out(K10));
PC2 PC2_11 (.data_in({c11, d11}), .data_out(K11));
PC2 PC2_12 (.data_in({c12, d12}), .data_out(K12));
PC2 PC2_13 (.data_in({c13, d13}), .data_out(K13));
PC2 PC2_14 (.data_in({c14, d14}), .data_out(K14));
PC2 PC2_15 (.data_in({c15, d15}), .data_out(K15));
PC2 PC2_16 (.data_in({c16, d16}), .data_out(K16));

 
endmodule



module lab_7_top (
input	   	  clk,
input	 	  rst,
input  [63:0] key_in,
input  [63:0] data_in,
input	 	  load,
output [63:0] data_out		 
);

wire [63:0] plaintext;
wire [63:0] ciphertext;
wire [63:0] key_samp;
wire [47:0] K1;
wire [47:0] K2;
wire [47:0] K3;
wire [47:0] K4;
wire [47:0] K5;
wire [47:0] K6;
wire [47:0] K7;
wire [47:0] K8;
wire [47:0] K9;
wire [47:0] K10;
wire [47:0] K11;
wire [47:0] K12;
wire [47:0] K13;
wire [47:0] K14;
wire [47:0] K15;
wire [47:0] K16;

input_register i_data_samp(.clk(clk), .reset(rst), .data_in(data_in), .load(load), .data_out(plaintext));
input_register i_key_samp(.clk(clk), .reset(rst), .data_in(key_in), .load(load), .data_out(key_samp));

key_schedule i_key_schedule( //TODO !!! real names to be given
.key_in(key_samp),
.K1(K1),
.K2(K2),
.K3(K3),
.K4(K4),
.K5(K5),
.K6(K6),
.K7(K7),
.K8(K8),
.K9(K9),
.K10(K10),
.K11(K11),
.K12(K12),
.K13(K13),
.K14(K14),
.K15(K15),
.K16(K16)
);

feistel_network i_feistel_network( //TODO !!! real names to be given
.plaintext(plaintext),
.K1(K1),
.K2(K2),
.K3(K3),
.K4(K4),
.K5(K5),
.K6(K6),
.K7(K7),
.K8(K8),
.K9(K9),
.K10(K10),
.K11(K11),
.K12(K12),
.K13(K13),
.K14(K14),
.K15(K15),
.K16(K16),
.ciphertext(ciphertext)
);

output_register i_output_tegister (.clk(clk), .reset(rst), .ciphertext(ciphertext), .data_out(data_out));

endmodule










module output_register
  (
    input  wire        clk,
    input  wire        reset,
    input  wire [63:0] ciphertext,
    output reg  [63:0] data_out
  );


always @(posedge clk)
  if (reset)
    data_out <= 64'b0;
  else
    data_out <= ciphertext;

endmodule



module s1
(
 input  wire  [5:0]  s1_in,
 output reg   [3:0]  s1_out
);


always @ (*)
   case(s1_in)
      6'b000000 :  s1_out = 4'he;
      6'b000001 :  s1_out = 4'h0;
	  6'b000010 :  s1_out = 4'h4;
	  6'b000011 :  s1_out = 4'hf;
	  6'b000100 :  s1_out = 4'hd;
	  6'b000101 :  s1_out = 4'h7;
	  6'b000110 :  s1_out = 4'h1;
	  6'b000111 :  s1_out = 4'h4;
	  6'b001000 :  s1_out = 4'h2;
	  6'b001001 :  s1_out = 4'he;
	  6'b001010 :  s1_out = 4'hf;
	  6'b001011 :  s1_out = 4'h2;
	  6'b001100 :  s1_out = 4'hb;
	  6'b001101 :  s1_out = 4'hd;
	  6'b001110 :  s1_out = 4'h8;
	  6'b001111 :  s1_out = 4'h1;
	  6'b010000 :  s1_out = 4'h3;
	  6'b010001 :  s1_out = 4'ha;
	  6'b010010 :  s1_out = 4'ha;
	  6'b010011 :  s1_out = 4'h6;
	  6'b010100 :  s1_out = 4'h6;
	  6'b010101 :  s1_out = 4'hc;
	  6'b010110 :  s1_out = 4'hc;
	  6'b010111 :  s1_out = 4'hb;
	  6'b011000 :  s1_out = 4'h5;
	  6'b011001 :  s1_out = 4'h9;
      6'b011010 :  s1_out = 4'h9;
      6'b011011 :  s1_out = 4'h5;	  
	  6'b011100 :  s1_out = 4'h0;
      6'b011101 :  s1_out = 4'h3;
      6'b011110 :  s1_out = 4'h7;
      6'b011111 :  s1_out = 4'h8;
      6'b100000 :  s1_out = 4'h4;
      6'b100001 :  s1_out = 4'hf;
      6'b100010 :  s1_out = 4'h1;
      6'b100011 :  s1_out = 4'hc;
      6'b100100 :  s1_out = 4'he;
      6'b100101 :  s1_out = 4'h8;
      6'b100110 :  s1_out = 4'h8;
      6'b100111 :  s1_out = 4'h2;
      6'b101000 :  s1_out = 4'hd;
      6'b101001 :  s1_out = 4'h4;
      6'b101010 :  s1_out = 4'h6;
      6'b101011 :  s1_out = 4'h9;	 
      6'b101100 :  s1_out = 4'h2;	
      6'b101101 :  s1_out = 4'h1;
      6'b101110 :  s1_out = 4'hb;
      6'b101111 :  s1_out = 4'h7;
      6'b110000 :  s1_out = 4'hf;
      6'b110001 :  s1_out = 4'h5;
      6'b110010 :  s1_out = 4'hc;
      6'b110011 :  s1_out = 4'hb;
	  6'b110100 :  s1_out = 4'h9;
      6'b110101 :  s1_out = 4'h3;
      6'b110110 :  s1_out = 4'h7;
      6'b110111 :  s1_out = 4'he;	  
      6'b111000 :  s1_out = 4'h3;	 
      6'b111001 :  s1_out = 4'ha;
      6'b111010 :  s1_out = 4'ha;	
      6'b111011 :  s1_out = 4'h0;
      6'b111100 :  s1_out = 4'h5;	 
      6'b111101 :  s1_out = 4'h6;
      6'b111110 :  s1_out = 4'h0;
      6'b111111 :  s1_out = 4'hd;
  
	 endcase

endmodule	 
	  
	  
	  
	  
	  
	  
module s2
(
input wire [5:0] s2_in,
output reg [3:0] s2_out
);

always @ (*)
	case(s2_in)
		6'b000000 : s2_out = 4'd15;
		6'b000001 : s2_out = 4'd3;
		6'b000010 : s2_out = 4'd1;
		6'b000011 : s2_out = 4'd13;
		6'b000100 : s2_out = 4'd8;
		6'b000101 : s2_out = 4'd4;
		6'b000110 : s2_out = 4'd14;
		6'b000111 : s2_out = 4'd7;
		6'b001000 : s2_out = 4'd6;
		6'b001001 : s2_out = 4'd15;
		6'b001010 : s2_out = 4'd11;
		6'b001011 : s2_out = 4'd2;
		6'b001100 : s2_out = 4'd3;
		6'b001101 : s2_out = 4'd8;
		6'b001110 : s2_out = 4'd4;
		6'b001111 : s2_out = 4'd14;
		6'b010000 : s2_out = 4'd9;
		6'b010001 : s2_out = 4'd12;
		6'b010010 : s2_out = 4'd7;
		6'b010011 : s2_out = 4'd0;
		6'b010100 : s2_out = 4'd2;
		6'b010101 : s2_out = 4'd1;
		6'b010110 : s2_out = 4'd13;
		6'b010111 : s2_out = 4'd10;
		6'b011000 : s2_out = 4'd12;
		6'b011001 : s2_out = 4'd6;
		6'b011010 : s2_out = 4'd0;
		6'b011011 : s2_out = 4'd9;
		6'b011100 : s2_out = 4'd5;
		6'b011101 : s2_out = 4'd11;
		6'b011110 : s2_out = 4'd10;
		6'b011111 : s2_out = 4'd5;
		6'b100000 : s2_out = 4'd0;
		6'b100001 : s2_out = 4'd13;
		6'b100010 : s2_out = 4'd14;
		6'b100011 : s2_out = 4'd8;
		6'b100100 : s2_out = 4'd7;
		6'b100101 : s2_out = 4'd10;
		6'b100110 : s2_out = 4'd11;
		6'b100111 : s2_out = 4'd1;
		6'b101000 : s2_out = 4'd10;
		6'b101001 : s2_out = 4'd3;
		6'b101010 : s2_out = 4'd4;
		6'b101011 : s2_out = 4'd15;
		6'b101100 : s2_out = 4'd13;
		6'b101101 : s2_out = 4'd4;
		6'b101110 : s2_out = 4'd1;
		6'b101111 : s2_out = 4'd2;
		6'b110000 : s2_out = 4'd5;
		6'b110001 : s2_out = 4'd11;
		6'b110010 : s2_out = 4'd8;
		6'b110011 : s2_out = 4'd6;
		6'b110100 : s2_out = 4'd12;
		6'b110101 : s2_out = 4'd7;
		6'b110110 : s2_out = 4'd6;
		6'b110111 : s2_out = 4'd12;
		6'b111000 : s2_out = 4'd9;
		6'b111001 : s2_out = 4'd0;
		6'b111010 : s2_out = 4'd3;
		6'b111011 : s2_out = 4'd5;
		6'b111100 : s2_out = 4'd2;
		6'b111101 : s2_out = 4'd14;
		6'b111110 : s2_out = 4'd15;
		6'b111111 : s2_out = 4'd9;
		
		endcase
		
endmodule



module s3
(
input wire [5:0] s3_in,
output reg [3:0] s3_out
);

always @ (*)
    case(s3_in)
            6'b000000 : s3_out = 4'd10;
            6'b000001 : s3_out = 4'd13;
            6'b000010 : s3_out = 4'd0;
            6'b000011 : s3_out = 4'd7;
            6'b000100 : s3_out = 4'd9;
            6'b000101 : s3_out = 4'd0;
            6'b000110 : s3_out = 4'd14;
            6'b000111 : s3_out = 4'd9;
            6'b001000 : s3_out = 4'd6;
            6'b001001 : s3_out = 4'd3;
            6'b001010 : s3_out = 4'd3;
            6'b001011 : s3_out = 4'd4;
            6'b001100 : s3_out = 4'd15;
            6'b001101 : s3_out = 4'd6;
            6'b001110 : s3_out = 4'd5;
            6'b001111 : s3_out = 4'd10;
            6'b010000 : s3_out = 4'd1;
            6'b010001 : s3_out = 4'd2;
            6'b010010 : s3_out = 4'd13;
            6'b010011 : s3_out = 4'd8;
            6'b010100 : s3_out = 4'd12;
            6'b010101 : s3_out = 4'd5;
            6'b010110 : s3_out = 4'd7;
            6'b010111 : s3_out = 4'd14;
            6'b011000 : s3_out = 4'd11;
            6'b011001 : s3_out = 4'd12;
            6'b011010 : s3_out = 4'd4;
            6'b011011 : s3_out = 4'd11;
            6'b011100 : s3_out = 4'd2;
            6'b011101 : s3_out = 4'd15;
            6'b011110 : s3_out = 4'd8;
            6'b011111 : s3_out = 4'd1;
            6'b100000 : s3_out = 4'd13;
            6'b100001 : s3_out = 4'd1;
            6'b100010 : s3_out = 4'd6;
            6'b100011 : s3_out = 4'd10;
            6'b100100 : s3_out = 4'd4;
            6'b100101 : s3_out = 4'd13;
            6'b100110 : s3_out = 4'd9;
            6'b100111 : s3_out = 4'd0;
            6'b101000 : s3_out = 4'd8;
            6'b101001 : s3_out = 4'd6;
            6'b101010 : s3_out = 4'd15;
            6'b101011 : s3_out = 4'd9;
            6'b101100 : s3_out = 4'd3;
            6'b101101 : s3_out = 4'd8;
            6'b101110 : s3_out = 4'd0;
            6'b101111 : s3_out = 4'd7;
            6'b110000 : s3_out = 4'd11;
            6'b110001 : s3_out = 4'd4;
            6'b110010 : s3_out = 4'd1;
            6'b110011 : s3_out = 4'd15;
            6'b110100 : s3_out = 4'd2;
            6'b110101 : s3_out = 4'd14;
            6'b110110 : s3_out = 4'd12;
            6'b110111 : s3_out = 4'd3;
            6'b111000 : s3_out = 4'd5;
            6'b111001 : s3_out = 4'd11;
            6'b111010 : s3_out = 4'd10;
            6'b111011 : s3_out = 4'd5;
            6'b111100 : s3_out = 4'd14;
            6'b111101 : s3_out = 4'd2;
            6'b111110 : s3_out = 4'd7;
            6'b111111 : s3_out = 4'd12;
    endcase
endmodule




module s4
(
input wire [5:0] s4_in,
output reg [3:0] s4_out
);

always @ (*)
    case(s4_in)
            6'b000000 : s4_out = 4'd7;
            6'b000001 : s4_out = 4'd13;
            6'b000010 : s4_out = 4'd13;
            6'b000011 : s4_out = 4'd8;
            6'b000100 : s4_out = 4'd14;
            6'b000101 : s4_out = 4'd11;
            6'b000110 : s4_out = 4'd3;
            6'b000111 : s4_out = 4'd5;
            6'b001000 : s4_out = 4'd0;
            6'b001001 : s4_out = 4'd6;
            6'b001010 : s4_out = 4'd6;
            6'b001011 : s4_out = 4'd15;
            6'b001100 : s4_out = 4'd9;
            6'b001101 : s4_out = 4'd0;
            6'b001110 : s4_out = 4'd10;
            6'b001111 : s4_out = 4'd3;
            6'b010000 : s4_out = 4'd1;
            6'b010001 : s4_out = 4'd4;
            6'b010010 : s4_out = 4'd2;
            6'b010011 : s4_out = 4'd7;
            6'b010100 : s4_out = 4'd8;
            6'b010101 : s4_out = 4'd2;
            6'b010110 : s4_out = 4'd5;
            6'b010111 : s4_out = 4'd12;
            6'b011000 : s4_out = 4'd11;
            6'b011001 : s4_out = 4'd1;
            6'b011010 : s4_out = 4'd12;
            6'b011011 : s4_out = 4'd10;
            6'b011100 : s4_out = 4'd4;
            6'b011101 : s4_out = 4'd14;
            6'b011110 : s4_out = 4'd15;
            6'b011111 : s4_out = 4'd9;
            6'b100000 : s4_out = 4'd10;
            6'b100001 : s4_out = 4'd3;
            6'b100010 : s4_out = 4'd6;
            6'b100011 : s4_out = 4'd15;
            6'b100100 : s4_out = 4'd9;
            6'b100101 : s4_out = 4'd0;
            6'b100110 : s4_out = 4'd0;
            6'b100111 : s4_out = 4'd6;
            6'b101000 : s4_out = 4'd12;
            6'b101001 : s4_out = 4'd10;
            6'b101010 : s4_out = 4'd11;
            6'b101011 : s4_out = 4'd1;
            6'b101100 : s4_out = 4'd7;
            6'b101101 : s4_out = 4'd13;
            6'b101110 : s4_out = 4'd13;
            6'b101111 : s4_out = 4'd8;
            6'b110000 : s4_out = 4'd15;
            6'b110001 : s4_out = 4'd9;
            6'b110010 : s4_out = 4'd1;
            6'b110011 : s4_out = 4'd4;
            6'b110100 : s4_out = 4'd3;
            6'b110101 : s4_out = 4'd5;
            6'b110110 : s4_out = 4'd14;
            6'b110111 : s4_out = 4'd11;
            6'b111000 : s4_out = 4'd5;
            6'b111001 : s4_out = 4'd12;
            6'b111010 : s4_out = 4'd2;
            6'b111011 : s4_out = 4'd7;
            6'b111100 : s4_out = 4'd8;
            6'b111101 : s4_out = 4'd2;
            6'b111110 : s4_out = 4'd4;
            6'b111111 : s4_out = 4'd14;
    endcase
endmodule

            
    
    
    
	
	
	
	module s5
(
input wire [5:0] s5_in,
output reg [3:0] s5_out
);

always @ (*)
    case(s5_in)
		6'b000000 : s5_out = 4'd2;
		6'b000001 : s5_out = 4'd14;
		6'b000010 : s5_out = 4'd12;
		6'b000011 : s5_out = 4'd11;
		6'b000100 : s5_out = 4'd4;
		6'b000101 : s5_out = 4'd2;
		6'b000110 : s5_out = 4'd1;
		6'b000111 : s5_out = 4'd12;
		6'b001000 : s5_out = 4'd7;
		6'b001001 : s5_out = 4'd4;
		6'b001010 : s5_out = 4'd10;
		6'b001011 : s5_out = 4'd7;
		6'b001100 : s5_out = 4'd11;
		6'b001101 : s5_out = 4'd13;
		6'b001110 : s5_out = 4'd6;
		6'b001111 : s5_out = 4'd1;
		6'b010000 : s5_out = 4'd8;
		6'b010001 : s5_out = 4'd5;
		6'b010010 : s5_out = 4'd5;
		6'b010011 : s5_out = 4'd0;
		6'b010100 : s5_out = 4'd3;
		6'b010101 : s5_out = 4'd15;
		6'b010110 : s5_out = 4'd15;
		6'b010111 : s5_out = 4'd10;
		6'b011000 : s5_out = 4'd13;
		6'b011001 : s5_out = 4'd3;
		6'b011010 : s5_out = 4'd0;
		6'b011011 : s5_out = 4'd9;
		6'b011100 : s5_out = 4'd14;
		6'b011101 : s5_out = 4'd8;
		6'b011110 : s5_out = 4'd9;
		6'b011111 : s5_out = 4'd6;
		6'b100000 : s5_out = 4'd4;
		6'b100001 : s5_out = 4'd11;
		6'b100010 : s5_out = 4'd2;
		6'b100011 : s5_out = 4'd8;
		6'b100100 : s5_out = 4'd1;
		6'b100101 : s5_out = 4'd12;
		6'b100110 : s5_out = 4'd11;
		6'b100111 : s5_out = 4'd7;
		6'b101000 : s5_out = 4'd10;
		6'b101001 : s5_out = 4'd1;
		6'b101010 : s5_out = 4'd13;
		6'b101011 : s5_out = 4'd14;
		6'b101100 : s5_out = 4'd7;
		6'b101101 : s5_out = 4'd2;
		6'b101110 : s5_out = 4'd8;
		6'b101111 : s5_out = 4'd13;
		6'b110000 : s5_out = 4'd15;
		6'b110001 : s5_out = 4'd6;
		6'b110010 : s5_out = 4'd9;
		6'b110011 : s5_out = 4'd15;
		6'b110100 : s5_out = 4'd12;
		6'b110101 : s5_out = 4'd0;
		6'b110110 : s5_out = 4'd5;
		6'b110111 : s5_out = 4'd9;
		6'b111000 : s5_out = 4'd6;
		6'b111001 : s5_out = 4'd10;
		6'b111010 : s5_out = 4'd3;
		6'b111011 : s5_out = 4'd4;
		6'b111100 : s5_out = 4'd0;
		6'b111101 : s5_out = 4'd5;
		6'b111110 : s5_out = 4'd14;
		6'b111111 : s5_out = 4'd3;
    endcase
endmodule





module s6
(
input wire [5:0] s6_in,
output reg [3:0] s6_out
);

always @ (*)
    case(s6_in)
		6'b000000 : s6_out = 4'd12;
		6'b000001 : s6_out = 4'd10;
		6'b000010 : s6_out = 4'd1;
		6'b000011 : s6_out = 4'd15;
		6'b000100 : s6_out = 4'd10;
		6'b000101 : s6_out = 4'd4;
		6'b000110 : s6_out = 4'd15;
		6'b000111 : s6_out = 4'd2;
		6'b001000 : s6_out = 4'd9;
		6'b001001 : s6_out = 4'd7;
		6'b001010 : s6_out = 4'd2;
		6'b001011 : s6_out = 4'd12;
		6'b001100 : s6_out = 4'd6;
		6'b001101 : s6_out = 4'd9;
		6'b001110 : s6_out = 4'd8;
		6'b001111 : s6_out = 4'd5;
		6'b010000 : s6_out = 4'd0;
		6'b010001 : s6_out = 4'd6;
		6'b010010 : s6_out = 4'd13;
		6'b010011 : s6_out = 4'd1;
		6'b010100 : s6_out = 4'd3;
		6'b010101 : s6_out = 4'd13;
		6'b010110 : s6_out = 4'd4;
		6'b010111 : s6_out = 4'd14;
		6'b011000 : s6_out = 4'd14;
		6'b011001 : s6_out = 4'd0;
		6'b011010 : s6_out = 4'd7;
		6'b011011 : s6_out = 4'd11;
		6'b011100 : s6_out = 4'd5;
		6'b011101 : s6_out = 4'd3;
		6'b011110 : s6_out = 4'd11;
		6'b011111 : s6_out = 4'd8;
		6'b100000 : s6_out = 4'd9;
		6'b100001 : s6_out = 4'd4;
		6'b100010 : s6_out = 4'd14;
		6'b100011 : s6_out = 4'd3;
		6'b100100 : s6_out = 4'd15;
		6'b100101 : s6_out = 4'd2;
		6'b100110 : s6_out = 4'd5;
		6'b100111 : s6_out = 4'd12;
		6'b101000 : s6_out = 4'd2;
		6'b101001 : s6_out = 4'd9;
		6'b101010 : s6_out = 4'd8;
		6'b101011 : s6_out = 4'd5;
		6'b101100 : s6_out = 4'd12;
		6'b101101 : s6_out = 4'd15;
		6'b101110 : s6_out = 4'd3;
		6'b101111 : s6_out = 4'd10;
		6'b110000 : s6_out = 4'd7;
		6'b110001 : s6_out = 4'd11;
		6'b110010 : s6_out = 4'd0;
		6'b110011 : s6_out = 4'd14;
		6'b110100 : s6_out = 4'd4;
		6'b110101 : s6_out = 4'd1;
		6'b110110 : s6_out = 4'd10;
		6'b110111 : s6_out = 4'd7;
		6'b111000 : s6_out = 4'd1;
		6'b111001 : s6_out = 4'd6;
		6'b111010 : s6_out = 4'd13;
		6'b111011 : s6_out = 4'd0;
		6'b111100 : s6_out = 4'd11;
		6'b111101 : s6_out = 4'd8;
		6'b111110 : s6_out = 4'd6;
		6'b111111 : s6_out = 4'd13;
    endcase
endmodule





module s7
(
input wire [5:0] s7_in,
output reg [3:0] s7_out
);

always @ (*)
    case(s7_in)
		6'b000000 : s7_out = 4'd4;
		6'b000001 : s7_out = 4'd13;
		6'b000010 : s7_out = 4'd11;
		6'b000011 : s7_out = 4'd0;
		6'b000100 : s7_out = 4'd2;
		6'b000101 : s7_out = 4'd11;
		6'b000110 : s7_out = 4'd14;
		6'b000111 : s7_out = 4'd7;
		6'b001000 : s7_out = 4'd15;
		6'b001001 : s7_out = 4'd4;
		6'b001010 : s7_out = 4'd0;
		6'b001011 : s7_out = 4'd9;
		6'b001100 : s7_out = 4'd8;
		6'b001101 : s7_out = 4'd1;
		6'b001110 : s7_out = 4'd13;
		6'b001111 : s7_out = 4'd10;
		6'b010000 : s7_out = 4'd3;
		6'b010001 : s7_out = 4'd14;
		6'b010010 : s7_out = 4'd12;
		6'b010011 : s7_out = 4'd3;
		6'b010100 : s7_out = 4'd9;
		6'b010101 : s7_out = 4'd5;
		6'b010110 : s7_out = 4'd7;
		6'b010111 : s7_out = 4'd12;
		6'b011000 : s7_out = 4'd5;
		6'b011001 : s7_out = 4'd2;
		6'b011010 : s7_out = 4'd10;
		6'b011011 : s7_out = 4'd15;
		6'b011100 : s7_out = 4'd6;
		6'b011101 : s7_out = 4'd8;
		6'b011110 : s7_out = 4'd1;
		6'b011111 : s7_out = 4'd6;
		6'b100000 : s7_out = 4'd1;
		6'b100001 : s7_out = 4'd6;
		6'b100010 : s7_out = 4'd4;
		6'b100011 : s7_out = 4'd11;
		6'b100100 : s7_out = 4'd11;
		6'b100101 : s7_out = 4'd13;
		6'b100110 : s7_out = 4'd13;
		6'b100111 : s7_out = 4'd8;
		6'b101000 : s7_out = 4'd12;
		6'b101001 : s7_out = 4'd1;
		6'b101010 : s7_out = 4'd3;
		6'b101011 : s7_out = 4'd4;
		6'b101100 : s7_out = 4'd7;
		6'b101101 : s7_out = 4'd10;
		6'b101110 : s7_out = 4'd14;
		6'b101111 : s7_out = 4'd7;
		6'b110000 : s7_out = 4'd10;
		6'b110001 : s7_out = 4'd9;
		6'b110010 : s7_out = 4'd15;
		6'b110011 : s7_out = 4'd5;
		6'b110100 : s7_out = 4'd6;
		6'b110101 : s7_out = 4'd0;
		6'b110110 : s7_out = 4'd8;
		6'b110111 : s7_out = 4'd15;
		6'b111000 : s7_out = 4'd0;
		6'b111001 : s7_out = 4'd14;
		6'b111010 : s7_out = 4'd5;
		6'b111011 : s7_out = 4'd2;
		6'b111100 : s7_out = 4'd9;
		6'b111101 : s7_out = 4'd3;
		6'b111110 : s7_out = 4'd2;
		6'b111111 : s7_out = 4'd12;
    endcase
endmodule





module s8
(
 input  wire  [5:0]  s8_in,
 output reg   [3:0]  s8_out
);


always @ (*)
   case(s8_in)
		6'b000000 : s8_out = 4'd13;
		6'b000001 : s8_out = 4'd1;
		6'b000010 : s8_out = 4'd2;
		6'b000011 : s8_out = 4'd15;
		6'b000100 : s8_out = 4'd8;
		6'b000101 : s8_out = 4'd13;
		6'b000110 : s8_out = 4'd4;
		6'b000111 : s8_out = 4'd8;
		6'b001000 : s8_out = 4'd6;
		6'b001001 : s8_out = 4'd10;
		6'b001010 : s8_out = 4'd15;
		6'b001011 : s8_out = 4'd3;
		6'b001100 : s8_out = 4'd11;
		6'b001101 : s8_out = 4'd7;
		6'b001110 : s8_out = 4'd1;
		6'b001111 : s8_out = 4'd4;
		6'b010000 : s8_out = 4'd10;
		6'b010001 : s8_out = 4'd12;
		6'b010010 : s8_out = 4'd9;
		6'b010011 : s8_out = 4'd5;
		6'b010100 : s8_out = 4'd3;
		6'b010101 : s8_out = 4'd6;
		6'b010110 : s8_out = 4'd14;
		6'b010111 : s8_out = 4'd11;
		6'b011000 : s8_out = 4'd5;
		6'b011001 : s8_out = 4'd0;
		6'b011010 : s8_out = 4'd0;
		6'b011011 : s8_out = 4'd14;
		6'b011100 : s8_out = 4'd12;
		6'b011101 : s8_out = 4'd9;
		6'b011110 : s8_out = 4'd7;
		6'b011111 : s8_out = 4'd2;
		6'b100000 : s8_out = 4'd7;
		6'b100001 : s8_out = 4'd2;
		6'b100010 : s8_out = 4'd11;
		6'b100011 : s8_out = 4'd1;
		6'b100100 : s8_out = 4'd4;
		6'b100101 : s8_out = 4'd14;
		6'b100110 : s8_out = 4'd1;
		6'b100111 : s8_out = 4'd7;
		6'b101000 : s8_out = 4'd9;
		6'b101001 : s8_out = 4'd4;
		6'b101010 : s8_out = 4'd12;
		6'b101011 : s8_out = 4'd10;
		6'b101100 : s8_out = 4'd14;
		6'b101101 : s8_out = 4'd8;
		6'b101110 : s8_out = 4'd2;
		6'b101111 : s8_out = 4'd13;
		6'b110000 : s8_out = 4'd0;
		6'b110001 : s8_out = 4'd15;
		6'b110010 : s8_out = 4'd6;
		6'b110011 : s8_out = 4'd12;
		6'b110100 : s8_out = 4'd10;
		6'b110101 : s8_out = 4'd9;
		6'b110110 : s8_out = 4'd13;
		6'b110111 : s8_out = 4'd0;
		6'b111000 : s8_out = 4'd15;
		6'b111001 : s8_out = 4'd3;
		6'b111010 : s8_out = 4'd3;
		6'b111011 : s8_out = 4'd5;
		6'b111100 : s8_out = 4'd5;
		6'b111101 : s8_out = 4'd6;
		6'b111110 : s8_out = 4'd8;
		6'b111111 : s8_out = 4'd11;
		
	endcase

endmodule



//reset, clk

`timescale 1ns/1ns
`default_nettype none 




module tb_7_top();

wire [47:0] tb_E_data_out;
reg  [31:0] tb_E_data_in;
E E_in_tb (.data_in(tb_E_data_in), .data_out(tb_E_data_out));


wire [63:0] tb_IP_data_out;
reg  [63:0] tb_IP_data_in;
IP IP_in_tb(.data_in(tb_IP_data_in), .data_out(tb_IP_data_out));


wire [63:0] tb_i_IP_data_out;
reg  [63:0] tb_i_IP_data_in;
i_IP i_IP_in_tb(.data_in(tb_i_IP_data_in), .data_out(tb_i_IP_data_out));


wire [31:0] tb_P_data_out;
reg  [31:0] tb_P_data_in;
P P_in_tb(.data_in(tb_P_data_in), .data_out(tb_P_data_out));


wire [27:0] tb_PC1_cbits;
wire [27:0] tb_PC1_dbits;
reg  [63:0] tb_PC1_key;
PC1 PC1_in_tb(.key(tb_PC1_key), .cbits(tb_PC1_cbits), .dbits(tb_PC1_dbits));


wire [47:0] tb_PC2_data_out;
reg  [55:0] tb_PC2_data_in;
PC2 PC2_in_tb(.data_in(tb_PC2_data_in), .data_out(tb_PC2_data_out));


reg  [31:0] tb_f_function_R;
reg  [47:0] tb_f_function_Key;
wire [31:0] tb_f_function_f_out;
f_function f_function_in_tb(.R(tb_f_function_R), .Key(tb_f_function_Key), .f_out(tb_f_function_f_out));



initial begin
    tb_E_data_in = 32'h00000000;
	#10 $display("E-function"); $display("data_in = %8h, data_out = %12h", tb_E_data_in, tb_E_data_out);
    tb_E_data_in = 32'h00000001;
	#10 $display("data_in = %8h, data_out = %12h", tb_E_data_in, tb_E_data_out);
    tb_E_data_in = 32'h00000100;
	#10 $display("data_in = %8h, data_out = %12h", tb_E_data_in, tb_E_data_out);
    tb_E_data_in = 32'h01234567;
	#10 $display("data_in = %8h, data_out = %12h", tb_E_data_in, tb_E_data_out);
    tb_E_data_in = 32'h98765432;
	#10 $display("data_in = %8h, data_out = %12h", tb_E_data_in, tb_E_data_out);
    tb_E_data_in = 32'h1b1b1b1b;
	#10 $display("data_in = %8h, data_out = %12h", tb_E_data_in, tb_E_data_out);
    tb_E_data_in = 32'hfedcba98;
	#10 $display("data_in = %8h, data_out = %12h", tb_E_data_in, tb_E_data_out);
    tb_E_data_in = 32'h44444444;
	#10 $display("data_in = %8h, data_out = %12h", tb_E_data_in, tb_E_data_out);
    tb_E_data_in = 32'h29292929;
	#10 $display("data_in = %8h, data_out = %12h", tb_E_data_in, tb_E_data_out);
    tb_E_data_in = 32'h35636363;
	#10 $display("data_in = %8h, data_out = %12h", tb_E_data_in, tb_E_data_out);
    tb_E_data_in = 32'h26535636;
	#10 $display("data_in = %8h, data_out = %12h", tb_E_data_in, tb_E_data_out);
    tb_E_data_in = 32'h19b97caa;
	#10 $display("data_in = %8h, data_out = %12h", tb_E_data_in, tb_E_data_out);
    tb_E_data_in = 32'hf0f0f0f0;
	#10 $display("data_in = %8h, data_out = %12h", tb_E_data_in, tb_E_data_out);
    tb_E_data_in = 32'hffffffff;
	#10 $display("data_in = %8h, data_out = %12h", tb_E_data_in, tb_E_data_out);
	#10 $display();
	
    tb_IP_data_in = 64'h0000000000000000;
	#10 $display("IP"); $display("data_in = %16h, data_out = %16h", tb_IP_data_in, tb_IP_data_out);
    tb_IP_data_in = 64'h0000000100000001;
	#10 $display("data_in = %16h, data_out = %16h", tb_IP_data_in, tb_IP_data_out);
    tb_IP_data_in = 64'h0000010000000100;
	#10 $display("data_in = %16h, data_out = %16h", tb_IP_data_in, tb_IP_data_out);
    tb_IP_data_in = 64'h0123456701234567;
	#10 $display("data_in = %16h, data_out = %16h", tb_IP_data_in, tb_IP_data_out);
    tb_IP_data_in = 64'h9876543298765432;
	#10 $display("data_in = %16h, data_out = %16h", tb_IP_data_in, tb_IP_data_out);
    tb_IP_data_in = 64'h1b1b1b1b1b1b1b1b;
	#10 $display("data_in = %16h, data_out = %16h", tb_IP_data_in, tb_IP_data_out);
    tb_IP_data_in = 64'hfedcba98fedcba98;
	#10 $display("data_in = %16h, data_out = %16h", tb_IP_data_in, tb_IP_data_out);
    tb_IP_data_in = 64'h4444444444444444;
	#10 $display("data_in = %16h, data_out = %16h", tb_IP_data_in, tb_IP_data_out);
    tb_IP_data_in = 64'h2929292929292929;
	#10 $display("data_in = %16h, data_out = %16h", tb_IP_data_in, tb_IP_data_out);
    tb_IP_data_in = 64'h3563636335636363;
	#10 $display("data_in = %16h, data_out = %16h", tb_IP_data_in, tb_IP_data_out);
    tb_IP_data_in = 64'h2653563626535636;
	#10 $display("data_in = %16h, data_out = %16h", tb_IP_data_in, tb_IP_data_out);
    tb_IP_data_in = 64'h19b97caa19b97caa;
	#10 $display("data_in = %16h, data_out = %16h", tb_IP_data_in, tb_IP_data_out);
    tb_IP_data_in = 64'hf0f0f0f0f0f0f0f0;
	#10 $display("data_in = %16h, data_out = %16h", tb_IP_data_in, tb_IP_data_out);
    tb_IP_data_in = 64'hffffffffffffffff;
	#10 $display("data_in = %16h, data_out = %16h", tb_IP_data_in, tb_IP_data_out);
	#10 $display();
	
    tb_i_IP_data_in = 64'h0000000000000000;
	#10 $display("Inverse IP"); $display("data_in = %16h, data_out = %16h", tb_i_IP_data_in, tb_i_IP_data_out);
    tb_i_IP_data_in = 64'h0000000100000001;
	#10 $display("data_in = %16h, data_out = %16h", tb_i_IP_data_in, tb_i_IP_data_out);
    tb_i_IP_data_in = 64'h0000010000000100;
	#10 $display("data_in = %16h, data_out = %16h", tb_i_IP_data_in, tb_i_IP_data_out);
    tb_i_IP_data_in = 64'h0123456701234567;
	#10 $display("data_in = %16h, data_out = %16h", tb_i_IP_data_in, tb_i_IP_data_out);
    tb_i_IP_data_in = 64'h9876543298765432;
	#10 $display("data_in = %16h, data_out = %16h", tb_i_IP_data_in, tb_i_IP_data_out);
    tb_i_IP_data_in = 64'h1b1b1b1b1b1b1b1b;
	#10 $display("data_in = %16h, data_out = %16h", tb_i_IP_data_in, tb_i_IP_data_out);
    tb_i_IP_data_in = 64'hfedcba98fedcba98;
	#10 $display("data_in = %16h, data_out = %16h", tb_i_IP_data_in, tb_i_IP_data_out);
    tb_i_IP_data_in = 64'h4444444444444444;
	#10 $display("data_in = %16h, data_out = %16h", tb_i_IP_data_in, tb_i_IP_data_out);
    tb_i_IP_data_in = 64'h2929292929292929;
	#10 $display("data_in = %16h, data_out = %16h", tb_i_IP_data_in, tb_i_IP_data_out);
    tb_i_IP_data_in = 64'h3563636335636363;
	#10 $display("data_in = %16h, data_out = %16h", tb_i_IP_data_in, tb_i_IP_data_out);
    tb_i_IP_data_in = 64'h2653563626535636;
	#10 $display("data_in = %16h, data_out = %16h", tb_i_IP_data_in, tb_i_IP_data_out);
    tb_i_IP_data_in = 64'h19b97caa19b97caa;
	#10 $display("data_in = %16h, data_out = %16h", tb_i_IP_data_in, tb_i_IP_data_out);
    tb_i_IP_data_in = 64'hf0f0f0f0f0f0f0f0;
	#10 $display("data_in = %16h, data_out = %16h", tb_i_IP_data_in, tb_i_IP_data_out);
    tb_i_IP_data_in = 64'hffffffffffffffff;
	#10 $display("data_in = %16h, data_out = %16h", tb_i_IP_data_in, tb_i_IP_data_out);
	#10 $display();
	
	#10 $display("Inverse IP(IP)");
    tb_IP_data_in = 64'ha0cfecf1fbbb29f9; #1 tb_i_IP_data_in = tb_IP_data_out;
    #10 $display("data_in = %16h, IP(data_in) = %16h, Inverse IP(IP(data_in)) = %16h", tb_IP_data_in, tb_IP_data_out, tb_i_IP_data_out); if (tb_IP_data_in != tb_i_IP_data_out) $display("      FAIL!!!");
	tb_IP_data_in = 64'h6b504997d4f7636d; #1 tb_i_IP_data_in = tb_IP_data_out;
    #10 $display("data_in = %16h, IP(data_in) = %16h, Inverse IP(IP(data_in)) = %16h", tb_IP_data_in, tb_IP_data_out, tb_i_IP_data_out); if (tb_IP_data_in != tb_i_IP_data_out) $display("      FAIL!!!");
	tb_IP_data_in = 64'h5550caef42e98731; #1 tb_i_IP_data_in = tb_IP_data_out;
    #10 $display("data_in = %16h, IP(data_in) = %16h, Inverse IP(IP(data_in)) = %16h", tb_IP_data_in, tb_IP_data_out, tb_i_IP_data_out); if (tb_IP_data_in != tb_i_IP_data_out) $display("      FAIL!!!");
	tb_IP_data_in = 64'h5a6973197f3b8c2c; #1 tb_i_IP_data_in = tb_IP_data_out;
    #10 $display("data_in = %16h, IP(data_in) = %16h, Inverse IP(IP(data_in)) = %16h", tb_IP_data_in, tb_IP_data_out, tb_i_IP_data_out); if (tb_IP_data_in != tb_i_IP_data_out) $display("      FAIL!!!");
	tb_IP_data_in = 64'ha5b255fee4ccbecb; #1 tb_i_IP_data_in = tb_IP_data_out;
    #10 $display("data_in = %16h, IP(data_in) = %16h, Inverse IP(IP(data_in)) = %16h", tb_IP_data_in, tb_IP_data_out, tb_i_IP_data_out); if (tb_IP_data_in != tb_i_IP_data_out) $display("      FAIL!!!");
	tb_IP_data_in = 64'h310dee27c7ee3936; #1 tb_i_IP_data_in = tb_IP_data_out;
    #10 $display("data_in = %16h, IP(data_in) = %16h, Inverse IP(IP(data_in)) = %16h", tb_IP_data_in, tb_IP_data_out, tb_i_IP_data_out); if (tb_IP_data_in != tb_i_IP_data_out) $display("      FAIL!!!");
	tb_IP_data_in = 64'ha39d63b8f6e4a7a5; #1 tb_i_IP_data_in = tb_IP_data_out;
    #10 $display("data_in = %16h, IP(data_in) = %16h, Inverse IP(IP(data_in)) = %16h", tb_IP_data_in, tb_IP_data_out, tb_i_IP_data_out); if (tb_IP_data_in != tb_i_IP_data_out) $display("      FAIL!!!");
	tb_IP_data_in = 64'heb1eae6b75cd0a6f; #1 tb_i_IP_data_in = tb_IP_data_out;
    #10 $display("data_in = %16h, IP(data_in) = %16h, Inverse IP(IP(data_in)) = %16h", tb_IP_data_in, tb_IP_data_out, tb_i_IP_data_out); if (tb_IP_data_in != tb_i_IP_data_out) $display("      FAIL!!!");
	tb_IP_data_in = 64'h1bdc7d6df8a7d213; #1 tb_i_IP_data_in = tb_IP_data_out;
    #10 $display("data_in = %16h, IP(data_in) = %16h, Inverse IP(IP(data_in)) = %16h", tb_IP_data_in, tb_IP_data_out, tb_i_IP_data_out); if (tb_IP_data_in != tb_i_IP_data_out) $display("      FAIL!!!");
	tb_IP_data_in = 64'h9fe92e3676a47ed8; #1 tb_i_IP_data_in = tb_IP_data_out;
	#10 $display();
	
	#10 $display("P-function"); 
    tb_P_data_in = 32'h97c3ce78;
    #10 $display("data_in = %8h, data_out = %8h", tb_P_data_in, tb_P_data_out);
    tb_P_data_in = 32'h6682f256;
    #10 $display("data_in = %8h, data_out = %8h", tb_P_data_in, tb_P_data_out);
    tb_P_data_in = 32'ha2fe2efe;
    #10 $display("data_in = %8h, data_out = %8h", tb_P_data_in, tb_P_data_out);
    tb_P_data_in = 32'hd67f6061;
    #10 $display("data_in = %8h, data_out = %8h", tb_P_data_in, tb_P_data_out);
    tb_P_data_in = 32'h839dbef;
    #10 $display("data_in = %8h, data_out = %8h", tb_P_data_in, tb_P_data_out);
    tb_P_data_in = 32'h43a1b2f6;
    #10 $display("data_in = %8h, data_out = %8h", tb_P_data_in, tb_P_data_out);
    tb_P_data_in = 32'h8a0d43fc;
    #10 $display("data_in = %8h, data_out = %8h", tb_P_data_in, tb_P_data_out);
    tb_P_data_in = 32'hfa5c55c7;
    #10 $display("data_in = %8h, data_out = %8h", tb_P_data_in, tb_P_data_out);
    tb_P_data_in = 32'hc52d2ec2;
    #10 $display("data_in = %8h, data_out = %8h", tb_P_data_in, tb_P_data_out);
    tb_P_data_in = 32'h15f5e4cd;
    #10 $display("data_in = %8h, data_out = %8h", tb_P_data_in, tb_P_data_out);
	#10 $display();
	
	#10 $display("PC1");
    tb_PC1_key = 64'h7a1129438283b7ad;
    #10 $display("key = %16h, cbits = %7h, dbits = %7h", tb_PC1_key, tb_PC1_cbits, tb_PC1_dbits);
    tb_PC1_key = 64'h556167a6280e5c48;
    #10 $display("key = %16h, cbits = %7h, dbits = %7h", tb_PC1_key, tb_PC1_cbits, tb_PC1_dbits);
    tb_PC1_key = 64'h6a4d0ed1bc67bfb1;
    #10 $display("key = %16h, cbits = %7h, dbits = %7h", tb_PC1_key, tb_PC1_cbits, tb_PC1_dbits);
    tb_PC1_key = 64'he304691548fbc451;
    #10 $display("key = %16h, cbits = %7h, dbits = %7h", tb_PC1_key, tb_PC1_cbits, tb_PC1_dbits);
    tb_PC1_key = 64'h79d6420b31260afe;
    #10 $display("key = %16h, cbits = %7h, dbits = %7h", tb_PC1_key, tb_PC1_cbits, tb_PC1_dbits);
    tb_PC1_key = 64'hc7aa9b680259f363;
    #10 $display("key = %16h, cbits = %7h, dbits = %7h", tb_PC1_key, tb_PC1_cbits, tb_PC1_dbits);
    tb_PC1_key = 64'h1d6655312e92892f;
    #10 $display("key = %16h, cbits = %7h, dbits = %7h", tb_PC1_key, tb_PC1_cbits, tb_PC1_dbits);
    tb_PC1_key = 64'h8ef2e602ca6ba761;
    #10 $display("key = %16h, cbits = %7h, dbits = %7h", tb_PC1_key, tb_PC1_cbits, tb_PC1_dbits);
    tb_PC1_key = 64'hba242369f0647668;
    #10 $display("key = %16h, cbits = %7h, dbits = %7h", tb_PC1_key, tb_PC1_cbits, tb_PC1_dbits);
    tb_PC1_key = 64'h1f33c959de72731e;
    #10 $display("key = %16h, cbits = %7h, dbits = %7h", tb_PC1_key, tb_PC1_cbits, tb_PC1_dbits);	
	#10 $display();

	#10 $display("PC2"); 
    tb_PC2_data_in = 56'ha49335c64c50ac;
    #10 $display("data_in = %14h, data_out = %12h", tb_PC2_data_in, tb_PC2_data_out);
    tb_PC2_data_in = 56'h6526b718a92d27;
    #10 $display("data_in = %14h, data_out = %12h", tb_PC2_data_in, tb_PC2_data_out);
    tb_PC2_data_in = 56'h5b4ea81161870e;
    #10 $display("data_in = %14h, data_out = %12h", tb_PC2_data_in, tb_PC2_data_out);
    tb_PC2_data_in = 56'h37796949e7b047;
    #10 $display("data_in = %14h, data_out = %12h", tb_PC2_data_in, tb_PC2_data_out);
    tb_PC2_data_in = 56'ha6a89f5599ce0a;
    #10 $display("data_in = %14h, data_out = %12h", tb_PC2_data_in, tb_PC2_data_out);
    tb_PC2_data_in = 56'hb5e40bda072006;
    #10 $display("data_in = %14h, data_out = %12h", tb_PC2_data_in, tb_PC2_data_out);
    tb_PC2_data_in = 56'h1ee278d628d18;
    #10 $display("data_in = %14h, data_out = %12h", tb_PC2_data_in, tb_PC2_data_out);
    tb_PC2_data_in = 56'h3e7f814bef4861;
    #10 $display("data_in = %14h, data_out = %12h", tb_PC2_data_in, tb_PC2_data_out);
    tb_PC2_data_in = 56'hfefed14eff7201;
    #10 $display("data_in = %14h, data_out = %12h", tb_PC2_data_in, tb_PC2_data_out);
    tb_PC2_data_in = 56'hb50a053ccddd07;
    #10 $display("data_in = %14h, data_out = %12h", tb_PC2_data_in, tb_PC2_data_out);
	#10 $display();

	#10 $display("F-function"); 
    tb_f_function_R = 32'hc2505924; tb_f_function_Key = 48'hfbf438939dc6;
    #10 $display("R = %8h, Key = %12h, f_out = %8h", tb_f_function_R, tb_f_function_Key, tb_f_function_f_out);
    tb_f_function_R = 32'h858e3878; tb_f_function_Key = 48'h1df4d98b9def;
    #10 $display("R = %8h, Key = %12h, f_out = %8h", tb_f_function_R, tb_f_function_Key, tb_f_function_f_out);
    tb_f_function_R = 32'h650a257f; tb_f_function_Key = 48'h93445453c966;
    #10 $display("R = %8h, Key = %12h, f_out = %8h", tb_f_function_R, tb_f_function_Key, tb_f_function_f_out);
    tb_f_function_R = 32'hff849764; tb_f_function_Key = 48'h7445a761d28c;
    #10 $display("R = %8h, Key = %12h, f_out = %8h", tb_f_function_R, tb_f_function_Key, tb_f_function_f_out);
    tb_f_function_R = 32'h63ff07b6; tb_f_function_Key = 48'hc108ec2d076e;
    #10 $display("R = %8h, Key = %12h, f_out = %8h", tb_f_function_R, tb_f_function_Key, tb_f_function_f_out);
    tb_f_function_R = 32'h2c105471; tb_f_function_Key = 48'h6c7a8951f453;
    #10 $display("R = %8h, Key = %12h, f_out = %8h", tb_f_function_R, tb_f_function_Key, tb_f_function_f_out);
    tb_f_function_R = 32'h92f8411f; tb_f_function_Key = 48'hf2075d1eae1;
    #10 $display("R = %8h, Key = %12h, f_out = %8h", tb_f_function_R, tb_f_function_Key, tb_f_function_f_out);
    tb_f_function_R = 32'hae5734af; tb_f_function_Key = 48'hf6c0bcc66c8b;
    #10 $display("R = %8h, Key = %12h, f_out = %8h", tb_f_function_R, tb_f_function_Key, tb_f_function_f_out);
    tb_f_function_R = 32'h5eec3806; tb_f_function_Key = 48'h2bad551ac24e;
    #10 $display("R = %8h, Key = %12h, f_out = %8h", tb_f_function_R, tb_f_function_Key, tb_f_function_f_out);
    tb_f_function_R = 32'h340dc6ac; tb_f_function_Key = 48'hd6c204b89ab7;
    #10 $display("R = %8h, Key = %12h, f_out = %8h", tb_f_function_R, tb_f_function_Key, tb_f_function_f_out);
	#10 $display();

end


endmodule


