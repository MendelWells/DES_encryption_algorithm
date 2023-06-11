
vlib work
vlog s2.v
vlog sbox_tb.v
vsim -voptargs=+acc=lprn sbox_tb

add wave -position end  sim:/sbox_tb/dut/s2_in
add wave -position end  sim:/sbox_tb/dut/s2_out
add wave -position end  sim:/sbox_tb/x
add wave -position end  sim:/sbox_tb/S_x
add wave -position end  sim:/sbox_tb/clk