module input_register
  (
    input  wire        clk,
    input  wire        reset,
    input  wire [63:0] data_in,
    input  wire        load,
    output reg  [63:0] plaintext
  );

reg [63:0] data;

always @(posedge clk)
  if (reset)
    data <= 64'b0;
  else
    if (load == 1'b1)
      data <= data_in;

always @(posedge clk)
  assign plaintext <= data;

endmodule
