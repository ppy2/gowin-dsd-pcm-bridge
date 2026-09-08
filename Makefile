SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c

BUILD ?= build
# Sim RTL: shipped path (top + receiver + rate detect + dup/drop SRC +
# TPDF dither + TX16). The retired FIR (lpf_8k_4th_seq + lpf_8k_4th +
# biquad_df1) stays in the tree as reference only, OUT of build and sim.
# Gowin IDE project: top.v, i2s_receiver.v, rate_detect.v,
# src_dupdrop.v, dither_24_16.v, i2s_transmitter.v
# (REMOVE lpf_8k_4th_seq.v, lpf_8k_4th.v, biquad_df1.v if present).
RTL := top.v i2s_receiver.v rate_detect.v src_dupdrop.v dither_24_16.v i2s_transmitter.v

.PHONY: all verify sim synth clean

all: verify

verify: sim synth

sim: $(BUILD)/sim/tb_top.pass $(BUILD)/sim/tb_src.pass $(BUILD)/sim/tb_dither.pass

$(BUILD)/sim/tb_top.pass: tb/tb_top.v $(RTL)
	mkdir -p $(BUILD)/sim
	iverilog -g2012 -Wall -s tb_top -o $(BUILD)/sim/top.vvp tb/tb_top.v $(RTL)
	vvp $(BUILD)/sim/top.vvp | tee $(BUILD)/sim/top.log
	grep -q "PASS tb_top" $(BUILD)/sim/top.log
	touch $@

$(BUILD)/sim/tb_src.pass: tb/tb_src.v rate_detect.v src_dupdrop.v
	mkdir -p $(BUILD)/sim
	iverilog -g2012 -Wall -s tb_src -o $(BUILD)/sim/src.vvp tb/tb_src.v rate_detect.v src_dupdrop.v
	vvp $(BUILD)/sim/src.vvp | tee $(BUILD)/sim/src.log
	grep -q "PASS tb_src" $(BUILD)/sim/src.log
	touch $@

$(BUILD)/sim/tb_dither.pass: tb/tb_dither_24_16.v dither_24_16.v
	mkdir -p $(BUILD)/sim
	iverilog -g2012 -Wall -s tb_dither_24_16 -o $(BUILD)/sim/dither.vvp tb/tb_dither_24_16.v dither_24_16.v
	vvp $(BUILD)/sim/dither.vvp | tee $(BUILD)/sim/dither.log
	grep -q "PASS tb_dither_24_16" $(BUILD)/sim/dither.log
	touch $@

synth:
# NOTE: local yosys is 0.23 (synth_gowin there hangs in ABC on wide
# multipliers); the real synthesis is the Gowin IDE on Windows.
# This gate proves clean elaboration + the expected datapath shape:
# ZERO multipliers (FIR removed: pure dup/drop + dither = adds only),
# zero inferred memories.
	yosys -p "read_verilog $(RTL); hierarchy -check -top top; prep -top top; stat" | tee $(BUILD)/synth.log
	! grep -iE "error" $(BUILD)/synth.log
	! grep -E '[$$]mul +[1-9]' $(BUILD)/synth.log
	! grep -E 'Number of memories: +[1-9]' $(BUILD)/synth.log

clean:
	rm -rf $(BUILD)
