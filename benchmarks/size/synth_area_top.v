`ifndef THREAD_NUM
	`define THREAD_NUM 5
`endif

module top (
	input clk, resetn,
	output trap,

	output        mem_valid,
	output        mem_instr,
	input         mem_ready,

	output [31:0] mem_addr,
	output [31:0] mem_wdata,
	output [ 3:0] mem_wstrb,
	input  [31:0] mem_rdata,

	output        instr_valid,
	input         instr_ready,

	output [31:0] instr_addr,
	input  [31:0] instr_rdata,

	// Look-Ahead Interface
	output        mem_la_read,
	output        mem_la_write,
	output [31:0] mem_la_addr,
	output [31:0] mem_la_wdata,
	output [ 3:0] mem_la_wstrb
);
	picorv32 #(
		.THREADS(`THREAD_NUM))
	picorv32 (
		.clk         (clk         ),
		.resetn      (resetn      ),
		.trap        (trap        ),
		.mem_valid   (mem_valid   ),
		.mem_instr   (mem_instr   ),
		.mem_ready   (mem_ready   ),
		.mem_addr    (mem_addr    ),
		.mem_wdata   (mem_wdata   ),
		.mem_wstrb   (mem_wstrb   ),
		.mem_rdata   (mem_rdata   ),
		.mem_la_read (mem_la_read ),
		.mem_la_write(mem_la_write),
		.mem_la_addr (mem_la_addr ),
		.mem_la_wdata(mem_la_wdata),
		.mem_la_wstrb(mem_la_wstrb),
		.instr_valid (instr_valid ),
		.instr_ready (instr_ready ),
		.instr_addr  (instr_addr  ),
		.instr_rdata (instr_rdata )
	);
endmodule
