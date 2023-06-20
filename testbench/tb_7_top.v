//reset, clk

`timescale 1n/1n
`default_nettype none 
module tb_7_top();

reg  	  	   tb_clk = 0;
reg  	  	   tb_reset = 1;
reg 	[63:0] tb_key_in;
reg 	[63:0] tb_data_in;
reg 	  	   tb_load = 0;
wire	[63:0] tb_data_out;
reg		[63:0] key_test, plaintext_test, ciphertext_test;
integer   	   data_file_in;
integer    	   statusD;
reg		[31:0] comper;		// string


always begin				//10MHz
	#50 tb_clk = ~tb_clk;
end

initial begin
	#100 reset = 0; load = 1;
	#100 load = 0;
	#100 load = 1;
end

initial begin
	tb_data_in 	   = 64'h0000000000000000;	// reset = 1
	tb_key_in 	   = 64'h8000000000000000;	// reset = 1,  expected false
	#100 tb_key_in = 64'h4000000000000000;	// load  = 1,  expected true
	#100 tb_key_in = 64'h2000000000000000;	// load  = 0,  expected false
	#100 tb_key_in = 64'h1000000000000000;	// load  = 1,  expected true
	#100 tb_key_in = 64'h0800000000000000;
	#100 tb_key_in = 64'h0400000000000000;
	#100 tb_key_in = 64'h0200000000000000;
	#100 tb_key_in = 64'h0100000000000000;
	#100 tb_key_in = 64'h0080000000000000;
	#100 tb_key_in = 64'h0040000000000000;
	#100 tb_key_in = 64'h0020000000000000;
	#100 tb_key_in = 64'h0010000000000000;
	#100 tb_key_in = 64'h0008000000000000;
	#100 tb_key_in = 64'h0004000000000000;
	#100 tb_key_in = 64'h0002000000000000;
	#100 tb_key_in = 64'h0001000000000000;
end


initial 
  data_file_in = $fopen("test_vectors.txt", "r");
  
  
initial 
begin
  while ( ! $feof(data_file_in)) 
  begin
    @ (posedge clk);
	statusD = $fscanf(data_file_in,"%h %h %h\n" , key_test, plaintext_test, ciphertext_test);
	repeat (1) @ (posedge clk);
	if (ciphertext_test == tb_data_out)
		comper = "PASS";
	else
		comper = "FAIL";
	$display("time = %8t | K = %016h | P = %016h | C = %016h | result = %016h | %s",
			  $time, key_test, plaintext_test, ciphertext_test, tb_data_out, comper);
  end
end



lab_7_top DUT
(
.clk (tb_clk),
.reset (tb_reset),
.key_in (tb_key_in),
.data_in (tb_data_in),
.load (tb_load),
.data_out (tb_data_out)
);

endmodule
