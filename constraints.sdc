# Two MCLK domains, one image (ratio logic, MCLK-agnostic):
# 45.1584 MHz -> 176.4 kHz out, 49.152 MHz -> 192 kHz out.
# Constrain the TIGHTER (49.152 MHz / 20.345 ns): passing there passes
# at 45.1584 MHz too (same logic, slower clock).
create_clock -name mclk_in -period 20.345 [get_ports {mclk_in}]
# Worst-case input BCLK: 384 kHz x 64 = 24.576 MHz (MCLK/2).
create_clock -name i2s_bclk_in -period 40.690 [get_ports {i2s_bclk_in}]
create_generated_clock -name i2s_bclk_out -source [get_ports {mclk_in}] -divide_by 8 [get_ports {i2s_bclk_out}]
create_generated_clock -name i2s_lrck_out -source [get_ports {mclk_in}] -divide_by 256 [get_ports {i2s_lrck_out}]
set_clock_groups -asynchronous -group [get_clocks {mclk_in}] -group [get_clocks {i2s_bclk_in}]
