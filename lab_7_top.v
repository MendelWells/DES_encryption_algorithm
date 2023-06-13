

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
wire [47:0] key_1;
wire [47:0] key_2;
wire [47:0] key_3;
wire [47:0] key_4;
wire [47:0] key_5;
wire [47:0] key_6;
wire [47:0] key_7;
wire [47:0] key_8;
wire [47:0] key_9;
wire [47:0] key_10;
wire [47:0] key_11;
wire [47:0] key_12;
wire [47:0] key_13;
wire [47:0] key_14;
wire [47:0] key_15;
wire [47:0] key_16;

input_register i_data_samp(.clk(clk), .reset(rst), .data_in(data_in), .load(load), .data_out(plaintext));
input_register i_key_samp(.clk(clk), .reset(rst), .data_in(key_in), .load(load), .data_out(key_samp));

key_schedule i_key_schedule( //TODO !!! real names to be given
.key_in(key_samp),
.key_1(key_1),
.key_2(key_2),
.key_3(key_3),
.key_4(key_4),
.key_5(key_5),
.key_6(key_6),
.key_7(key_7),
.key_8(key_8),
.key_9(key_9),
.key_10(key_10),
.key_11(key_11),
.key_12(key_12),
.key_13(key_13),
.key_14(key_14),
.key_15(key_15),
.key_16(key_16)
);

feistel_network i_feistel_network( //TODO !!! real names to be given
.plaintext(plaintext),
.key_1(key_1),
.key_2(key_2),
.key_3(key_3),
.key_4(key_4),
.key_5(key_5),
.key_6(key_6),
.key_7(key_7),
.key_8(key_8),
.key_9(key_9),
.key_10(key_10),
.key_11(key_11),
.key_12(key_12),
.key_13(key_13),
.key_14(key_14),
.key_15(key_15),
.key_16(key_16),
.ciphertext(ciphertext)
);

output_tegister i_output_tegister (.clk(clk), .reset(rst), .ciphertext(ciphertext), .data_out(data_out));

endmodule







