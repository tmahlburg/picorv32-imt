
VIVADO_BASE = /opt/Xilinx/Vivado/2020.1
VIVADO = $(VIVADO_BASE)/bin/vivado
XVLOG = $(VIVADO_BASE)/bin/xvlog
XELAB = $(VIVADO_BASE)/bin/xelab
GLBL = $(VIVADO_BASE)/data/verilog/src/glbl.v

export VIVADO

help:
	@echo ""
	@echo "Check utilization"
	@echo "	make area"
	@echo ""

synth_%:
	rm -f $@.log
	$(VIVADO) -nojournal -log $@.log -mode batch -source $@.tcl
	rm -rf .Xil fsm_encoding.os synth_*.backup.log usage_statistics_webtalk.*
	-grep -B4 -A10 'Slice LUTs' $@.log
	-grep -B1 -A9 ^Slack $@.log && echo
	
area: synth_area
	-grep -B4 -A10 'Slice LUTs' synth_area.log

