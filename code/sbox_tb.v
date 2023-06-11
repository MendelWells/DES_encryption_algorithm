`timescale 1 ps / 1 ps
module sbox_tb;

  // Declare inputs and outputs
  reg [5:0] x;
  wire [3:0] S_x;

  // Instantiate the unit under test
  s2 dut (
    .s2_in(x),
    .s2_out(S_x)
  );

  // Generate clock signal
  reg clk = 0;
  always #5 clk = ~clk;

  // Stimulus generation
  initial begin
    // Write test cases
		 x = 6'd0; 
		 #10;
		 x = 6'd1; 
		 #10;
		 x = 6'd2; 
		 #10;
		 x = 6'd3; 
		 #10;
		 x = 6'd4; 
		 #10;
		 x = 6'd5; 
		 #10;
		 x = 6'd6; 
		 #10;
		 x = 6'd7; 
		 #10;
		 x = 6'd8; 
		 #10;
		 x = 6'd9; 
		 #10;
		 x = 6'd10; 
		 #10;
		 x = 6'd11; 
		 #10;
		 x = 6'd12; 
		 #10;
		 x = 6'd13; 
		 #10;
		 x = 6'd14; 
		 #10;
		 x = 6'd15; 
		 #10;
		 x = 6'd16; 
		 #10;
		 x = 6'd17; 
		 #10;
		 x = 6'd18; 
		 #10;
		 x = 6'd19; 
		 #10;
		 x = 6'd20; 
		 #10;
		 x = 6'd21; 
		 #10;
		 x = 6'd22; 
		 #10;
		 x = 6'd23; 
		 #10;
		 x = 6'd24; 
		 #10;
		 x = 6'd25; 
		 #10;
		 x = 6'd26; 
		 #10;
		 x = 6'd27; 
		 #10;
		 x = 6'd28; 
		 #10;
		 x = 6'd29; 
		 #10;
		 x = 6'd30; 
		 #10;
		 x = 6'd31; 
		 #10;
		 x = 6'd32; 
		 #10;
		 x = 6'd33; 
		 #10;
		 x = 6'd34; 
		 #10;
		 x = 6'd35; 
		 #10;
		 x = 6'd36; 
		 #10;
		 x = 6'd37; 
		 #10;
		 x = 6'd38; 
		 #10;
		 x = 6'd39; 
		 #10;
		 x = 6'd40; 
		 #10;
		 x = 6'd41; 
		 #10;
		 x = 6'd42; 
		 #10;
		 x = 6'd43; 
		 #10;
		 x = 6'd44; 
		 #10;
		 x = 6'd45; 
		 #10;
		 x = 6'd46; 
		 #10;
		 x = 6'd47; 
		 #10;
		 x = 6'd48; 
		 #10;
		 x = 6'd49; 
		 #10;
		 x = 6'd50; 
		 #10;
		 x = 6'd51; 
		 #10;
		 x = 6'd52; 
		 #10;
		 x = 6'd53; 
		 #10;
		 x = 6'd54; 
		 #10;
		 x = 6'd55; 
		 #10;
		 x = 6'd56; 
		 #10;
		 x = 6'd57; 
		 #10;
		 x = 6'd58; 
		 #10;
		 x = 6'd59; 
		 #10;
		 x = 6'd60; 
		 #10;
		 x = 6'd61; 
		 #10;
		 x = 6'd62; 
		 #10;
		 x = 6'd63; 
		 #10;
    // End simulation
    $finish;
  end

  // Monitor output
  always @(S_x) 
	$display("time = %8t | x=%d  | S_x = %d | s2_out = %d",$time,x,S_x, dut.s2_out);


endmodule
