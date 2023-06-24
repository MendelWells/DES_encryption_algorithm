module Feistel_network 
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


wire [31:0] L0; wire [31:0] R0;
wire [31:0] L1; wire [31:0] R1;
wire [31:0] L2; wire [31:0] R2;
wire [31:0] L3; wire [31:0] R3;
wire [31:0] L4; wire [31:0] R4;
wire [31:0] L5; wire [31:0] R5;
wire [31:0] L6; wire [31:0] R6;
wire [31:0] L7; wire [31:0] R7;
wire [31:0] L8; wire [31:0] R8;
wire [31:0] L9; wire [31:0] R9;
wire [31:0] L10; wire [31:0] R10;
wire [31:0] L11; wire [31:0] R11;
wire [31:0] L12; wire [31:0] R12;
wire [31:0] L13; wire [31:0] R13;
wire [31:0] L14; wire [31:0] R14;
wire [31:0] L15; wire [31:0] R15;
wire [31:0] L16; wire [31:0] R16;


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


assign R1 = bitwise_plus(L0, f1_out); assign L1 = R0;
assign R2 = bitwise_plus(L1, f2_out); assign L2 = R1;
assign R3 = bitwise_plus(L2, f3_out); assign L3 = R2;
assign R4 = bitwise_plus(L3, f4_out); assign L4 = R3;
assign R5 = bitwise_plus(L4, f5_out); assign L5 = R4;
assign R6 = bitwise_plus(L5, f6_out); assign L6 = R5;
assign R7 = bitwise_plus(L6, f7_out); assign L7 = R6;
assign R8 = bitwise_plus(L7, f8_out); assign L8 = R7;
assign R9 = bitwise_plus(L8, f9_out); assign L9 = R8;
assign R10 = bitwise_plus(L9, f10_out); assign L10 = R9;
assign R11 = bitwise_plus(L10, f11_out); assign L11 = R10;
assign R12 = bitwise_plus(L11, f12_out); assign L12 = R11;
assign R13 = bitwise_plus(L12, f13_out); assign L13 = R12;
assign R14 = bitwise_plus(L13, f14_out); assign L14 = R13;
assign R15 = bitwise_plus(L14, f15_out); assign L15 = R14;
assign R16 = bitwise_plus(L15, f16_out); assign L16 = R15;

i_IP Inverse_Initial_Permutation (.data_in({R16, L16}), .data_out(ciphertext));

endmodule
