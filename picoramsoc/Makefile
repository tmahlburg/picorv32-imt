CROSS=riscv32-unknown-elf-
FMT_HEX_CMD=./fmt_hex.py
CFLAGS=

# ---- BASYS 3 Board ----
#TODO: automate synthesis and simulation via Makefile
#hx8ksim: hx8kdemo_tb.vvp hx8kdemo_fw.hex
#	vvp -N $< +firmware=hx8kdemo_fw.hex
#
#hx8ksynsim: hx8kdemo_syn_tb.vvp hx8kdemo_fw.hex
#	vvp -N $< +firmware=hx8kdemo_fw.hex#
#
#hx8kdemo.blif: hx8kdemo.v spimemio.v simpleuart.v picosoc.v ../picorv32.v
#	yosys -ql hx8kdemo.log -p 'synth_ice40 -top hx8kdemo -blif hx8kdemo.blif' $^
#
#hx8kdemo_tb.vvp: hx8kdemo_tb.v hx8kdemo.v spimemio.v simpleuart.v picosoc.v ../picorv32.v spiflash.v
#	iverilog -s testbench -o $@ $^ `yosys-config --datdir/ice40/cells_sim.v`
#
#hx8kdemo_syn_tb.vvp: hx8kdemo_tb.v hx8kdemo_syn.v spiflash.v
#	iverilog -s testbench -o $@ $^ `yosys-config --datdir/ice40/cells_sim.v`
#
#hx8kdemo_syn.v: hx8kdemo.blif
#	yosys -p 'read_blif -wideports hx8kdemo.blif; write_verilog hx8kdemo_syn.v'
#
#hx8kdemo.asc: hx8kdemo.pcf hx8kdemo.blif
#	arachne-pnr -d 8k -o hx8kdemo.asc -p hx8kdemo.pcf hx8kdemo.blif
#
#hx8kdemo.bin: hx8kdemo.asc
#	icetime -d hx8k -c 12 -mtr hx8kdemo.rpt hx8kdemo.asc
#	icepack hx8kdemo.asc hx8kdemo.bin
#
#hx8kprog: hx8kdemo.bin hx8kdemo_fw.bin
#	iceprog hx8kdemo.bin
#	iceprog -o 1M hx8kdemo_fw.bin

basys3_sections.lds: sections.lds
	$(CROSS)cpp -P -DBASYS3 -o $@ $^

basys3_fw.elf: basys3_sections.lds start.s firmware.c
	$(CROSS)gcc $(CFLAGS) -DBASYS3 -march=rv32i -Wl,-Bstatic,-T,basys3_sections.lds,--strip-debug -ffreestanding -nostdlib -o basys3_fw.elf start.s firmware.c

basys3_fw.hex: basys3_fw.elf
	$(CROSS)objcopy -O verilog basys3_fw.elf basys3_fw.hex
	$(FMT_HEX_CMD) basys3_fw.hex

basys3_fw.bin: basys3_fw.elf
	$(CROSS)objcopy -O binary basys3_fw.elf basys3_fw.bin

basys3_fw.s: basys3_sections.lds start.s firmware.c
	$(CROSS)gcc $(CFLAGS) -DBASYS3 -S -march=rv32i -Wl,-Bstatic,-T,basys3_sections.lds,--strip-debug -ffreestanding -nostdlib start.s firmware.c

# ---- ASIC Synthesis Tests ----

#cmos.log: spimemio.v simpleuart.v picosoc.v ../picorv32.v
#	yosys -l cmos.log -p 'synth -top picosoc; abc -g cmos2; opt -fast; stat' $^

# ---- Clean ----

clean:
	rm -f testbench.vvp testbench.vcd
	rm -f basys3_fw.elf basys3_fw.hex basys3_fw.bin basys3_fw.s cmos.log
	rm -f hx8kdemo.blif hx8kdemo.log hx8kdemo.asc hx8kdemo.rpt hx8kdemo.bin
	rm -f hx8kdemo_syn.v hx8kdemo_syn_tb.vvp hx8kdemo_tb.vvp

.PHONY: spiflash_tb clean
.PHONY: hx8kprog hx8kprog_fw hx8ksim hx8ksynsim
.PHONY: icebprog icebprog_fw icebsim icebsynsim
