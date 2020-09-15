/*
 *  PicoRV32-imt -- A Small RISC-V (RV32I) Processor Core extended with Interleaved Multithreading
 *
 *  Copyright (C) 2020  Till Mahlburg
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

/* verilator lint_off WIDTH */
/* verilator lint_off PINMISSING */
/* verilator lint_off CASEOVERLAP */
/* verilator lint_off CASEINCOMPLETE */

`timescale 1 ns / 1 ps
// `default_nettype none
// `define DEBUGNETS
// `define DEBUGREGS
// `define DEBUGASM
// `define DEBUG

`ifdef DEBUG
  `define debug(debug_command) debug_command
`else
  `define debug(debug_command)
`endif

`ifdef FORMAL
  `define FORMAL_KEEP (* keep *)
  `define assert(assert_expr) assert(assert_expr)
`else
  `ifdef DEBUGNETS
    `define FORMAL_KEEP (* keep *)
  `else
    `define FORMAL_KEEP
  `endif
  `define assert(assert_expr) empty_statement
`endif

// uncomment this for register file in extra module
`define PICORV32_REGS picorv32_regs

// this macro can be used to check if the verilog files in your
// design are read in the correct order.
`define PICORV32_V


/***************************************************************
 * picorv32
 ***************************************************************/

module picorv32 #(
	parameter [ 0:0] ENABLE_COUNTERS = 1,
	parameter [ 0:0] ENABLE_COUNTERS64 = 1,
	parameter [ 0:0] ENABLE_REGS_DUALPORT = 1,
	parameter [ 0:0] LATCHED_MEM_RDATA = 0,
	parameter [ 0:0] TWO_STAGE_SHIFT = 0,
	parameter [ 0:0] BARREL_SHIFTER = 0,
	parameter [ 0:0] TWO_CYCLE_COMPARE = 0,
	parameter [ 0:0] TWO_CYCLE_ALU = 0,
	parameter [ 0:0] CATCH_MISALIGN = 1,
	parameter [ 0:0] CATCH_ILLINSN = 1,
	parameter [ 0:0] ENABLE_IRQ = 0,
	parameter [ 0:0] ENABLE_IRQ_QREGS = 1,
	parameter [ 0:0] ENABLE_IRQ_TIMER = 1,
	parameter [ 0:0] ENABLE_TRACE = 0,
	parameter [ 0:0] REGS_INIT_ZERO = 0,
	parameter [31:0] MASKED_IRQ = 32'h 0000_0000,
	parameter [31:0] LATCHED_IRQ = 32'h ffff_ffff,
	parameter [31:0] PROGADDR_RESET = 32'h 0000_0000,
	parameter [31:0] PROGADDR_IRQ = 32'h 0000_0010,
	parameter [31:0] STACKADDR = 32'h ffff_ffff,
	parameter [ 2:0] THREADS = 2
) (
	input clk, resetn,
	output reg trap,

	output reg        mem_valid,
	output reg        mem_instr,
	input             mem_ready,

	output reg [31:0] mem_addr,
	output reg [31:0] mem_wdata,
	output reg [ 3:0] mem_wstrb,
	input      [31:0] mem_rdata,

	// Look-Ahead Interface
	output            mem_la_read,
	output            mem_la_write,
	output     [31:0] mem_la_addr,
	output reg [31:0] mem_la_wdata,
	output reg [ 3:0] mem_la_wstrb,

	// Second read-only RAM port for instructions
	output			  instr_valid,
	input 	      	  instr_ready,
	output	   [31:0] instr_addr,
	input      [31:0] instr_rdata,

	// IRQ Interface
	input      [31:0] irq,
	output reg [31:0] eoi,

`ifdef RISCV_FORMAL
	output reg        rvfi_valid,
	output reg [63:0] rvfi_order,
	output reg [31:0] rvfi_insn,
	output reg        rvfi_trap,
	output reg        rvfi_halt,
	output reg        rvfi_intr,
	output reg [ 1:0] rvfi_mode,
	output reg [ 1:0] rvfi_ixl,
	output reg [ 4:0] rvfi_rs1_addr,
	output reg [ 4:0] rvfi_rs2_addr,
	output reg [31:0] rvfi_rs1_rdata,
	output reg [31:0] rvfi_rs2_rdata,
	output reg [ 4:0] rvfi_rd_addr,
	output reg [31:0] rvfi_rd_wdata,
	output reg [31:0] rvfi_pc_rdata,
	output reg [31:0] rvfi_pc_wdata,
	output reg [31:0] rvfi_mem_addr,
	output reg [ 3:0] rvfi_mem_rmask,
	output reg [ 3:0] rvfi_mem_wmask,
	output reg [31:0] rvfi_mem_rdata,
	output reg [31:0] rvfi_mem_wdata,

	output reg [63:0] rvfi_csr_mcycle_rmask,
	output reg [63:0] rvfi_csr_mcycle_wmask,
	output reg [63:0] rvfi_csr_mcycle_rdata,
	output reg [63:0] rvfi_csr_mcycle_wdata,

	output reg [63:0] rvfi_csr_minstret_rmask,
	output reg [63:0] rvfi_csr_minstret_wmask,
	output reg [63:0] rvfi_csr_minstret_rdata,
	output reg [63:0] rvfi_csr_minstret_wdata,
`endif

	// Trace Interface
	output reg        trace_valid,
	output reg [35:0] trace_data
);
	localparam integer irq_timer = 0;
	localparam integer irq_ebreak = 1;
	localparam integer irq_buserror = 2;

	localparam integer irqregs_offset = 32;
	localparam integer regfile_size = 32 + 4*ENABLE_IRQ*ENABLE_IRQ_QREGS;
	localparam integer regindex_bits = 5 + ENABLE_IRQ*ENABLE_IRQ_QREGS;

	localparam [35:0] TRACE_BRANCH = {4'b 0001, 32'b 0};
	localparam [35:0] TRACE_ADDR   = {4'b 0010, 32'b 0};
	localparam [35:0] TRACE_IRQ    = {4'b 1000, 32'b 0};

	reg [63:0] count_cycle, count_instr;
	reg [31:0] reg_pc [0:THREADS-1];
	reg [31:0] reg_next_pc [0:THREADS-1];
	reg [31:0] reg_op1 [0:THREADS-1];
	reg [31:0] reg_op2 [0:THREADS-1];
	reg [31:0] reg_out [0:THREADS-1];
	reg [4:0] reg_sh [0:THREADS-1];

	reg [31:0] dbg_insn_opcode;
	reg [31:0] dbg_insn_addr;

	wire dbg_mem_valid = mem_valid;
	wire dbg_mem_instr = mem_instr;
	wire dbg_mem_ready = mem_ready;
	wire [31:0] dbg_mem_addr  = mem_addr;
	wire [31:0] dbg_mem_wdata = mem_wdata;
	wire [ 3:0] dbg_mem_wstrb = mem_wstrb;
	wire [31:0] dbg_mem_rdata = mem_rdata;

	wire [31:0] next_pc [0:THREADS-1];

	reg irq_delay;
	reg irq_active;
	reg [31:0] irq_mask;
	reg [31:0] irq_pending;
	reg [31:0] timer;

	/* THREAD CONTROL */
	localparam [2:0] no_hart = 3'b111;
	reg [2:0] trap_hart = no_hart;
	reg [2:0] fetch_hart = no_hart;
	reg [2:0] ld_rs1_hart = no_hart;
	reg [2:0] ld_rs2_hart = no_hart;
	reg [2:0] exec_hart = no_hart;
	reg [2:0] shift_hart = no_hart;
	reg [2:0] stmem_hart = no_hart;
	reg [2:0] ldmem_hart = no_hart;

	// hart_ready[hart_id] == cpu_state_*, if hart_id is ready to transition to the next cpu state
	reg [7:0] hart_ready [0:THREADS-1];

	reg [7:0] hart_counter;

	task empty_statement;
		// This task is used by the `assert directive in non-formal mode to
		// avoid empty statement (which are unsupported by plain Verilog syntax).
		begin end
	endtask

	// TODO DEBUG
`ifdef DEBUGREGS
	wire [31:0] dbg_reg_x0  = 0;
	wire [31:0] dbg_reg_x1  = cpuregs[1];
	wire [31:0] dbg_reg_x2  = cpuregs[2];
	wire [31:0] dbg_reg_x3  = cpuregs[3];
	wire [31:0] dbg_reg_x4  = cpuregs[4];
	wire [31:0] dbg_reg_x5  = cpuregs[5];
	wire [31:0] dbg_reg_x6  = cpuregs[6];
	wire [31:0] dbg_reg_x7  = cpuregs[7];
	wire [31:0] dbg_reg_x8  = cpuregs[8];
	wire [31:0] dbg_reg_x9  = cpuregs[9];
	wire [31:0] dbg_reg_x10 = cpuregs[10];
	wire [31:0] dbg_reg_x11 = cpuregs[11];
	wire [31:0] dbg_reg_x12 = cpuregs[12];
	wire [31:0] dbg_reg_x13 = cpuregs[13];
	wire [31:0] dbg_reg_x14 = cpuregs[14];
	wire [31:0] dbg_reg_x15 = cpuregs[15];
	wire [31:0] dbg_reg_x16 = cpuregs[16];
	wire [31:0] dbg_reg_x17 = cpuregs[17];
	wire [31:0] dbg_reg_x18 = cpuregs[18];
	wire [31:0] dbg_reg_x19 = cpuregs[19];
	wire [31:0] dbg_reg_x20 = cpuregs[20];
	wire [31:0] dbg_reg_x21 = cpuregs[21];
	wire [31:0] dbg_reg_x22 = cpuregs[22];
	wire [31:0] dbg_reg_x23 = cpuregs[23];
	wire [31:0] dbg_reg_x24 = cpuregs[24];
	wire [31:0] dbg_reg_x25 = cpuregs[25];
	wire [31:0] dbg_reg_x26 = cpuregs[26];
	wire [31:0] dbg_reg_x27 = cpuregs[27];
	wire [31:0] dbg_reg_x28 = cpuregs[28];
	wire [31:0] dbg_reg_x29 = cpuregs[29];
	wire [31:0] dbg_reg_x30 = cpuregs[30];
	wire [31:0] dbg_reg_x31 = cpuregs[31];
`endif

	// Memory Interface

	reg [1:0] mem_state;
	reg [1:0] mem_wordsize;
	reg [31:0] mem_rdata_word;
	reg mem_do_rdata;
	reg mem_do_wdata;

	wire mem_xfer;

	assign mem_xfer = mem_valid && mem_ready;

	wire mem_done = resetn && (mem_xfer && |mem_state && (mem_do_rdata || mem_do_wdata));

	assign mem_la_write = resetn && !mem_state && mem_do_wdata;
	assign mem_la_read = resetn && (!mem_state && mem_do_rdata);
	assign mem_la_addr = mem_do_rdata ? {reg_op1[ldmem_hart][31:2], 2'b00} :
		mem_do_wdata ? {reg_op1[stmem_hart][31:2], 2'b00} : mem_addr;

	always @* begin
		(* full_case *)
		case (mem_wordsize)
			0: begin
				mem_la_wdata = reg_op2[stmem_hart];
				mem_la_wstrb = 4'b1111;
				mem_rdata_word = mem_rdata;
			end
			1: begin
				mem_la_wdata = {2{reg_op2[stmem_hart][15:0]}};
				mem_la_wstrb = reg_op1[stmem_hart][1] ? 4'b1100 : 4'b0011;
				case (reg_op1[ldmem_hart][1])
					1'b0: mem_rdata_word = {16'b0, mem_rdata[15: 0]};
					1'b1: mem_rdata_word = {16'b0, mem_rdata[31:16]};
				endcase
			end
			2: begin
				mem_la_wdata = {4{reg_op2[stmem_hart][7:0]}};
				mem_la_wstrb = 4'b0001 << reg_op1[stmem_hart][1:0];
				case (reg_op1[ldmem_hart][1:0])
					2'b00: mem_rdata_word = {24'b0, mem_rdata[ 7: 0]};
					2'b01: mem_rdata_word = {24'b0, mem_rdata[15: 8]};
					2'b10: mem_rdata_word = {24'b0, mem_rdata[23:16]};
					2'b11: mem_rdata_word = {24'b0, mem_rdata[31:24]};
				endcase
			end
		endcase
	end

	always @(posedge clk) begin
		if (resetn && !trap) begin
			if (mem_do_rdata)
				`assert(!mem_do_wdata);
			if (mem_do_wdata)
				`assert(!mem_do_rdata);
			if (mem_state == 2)
				`assert(mem_valid);
		end
	end

	always @(posedge clk) begin
		if (!resetn || trap) begin
			if (!resetn)
				mem_state <= 0;
			if (!resetn || mem_ready)
				mem_valid <= 0;
		end else begin
			if (mem_la_read || mem_la_write) begin
				mem_addr <= mem_la_addr;
				mem_wstrb <= mem_la_wstrb & {4{mem_la_write}};
			end
			if (mem_la_write) begin
				mem_wdata <= mem_la_wdata;
			end
			case (mem_state)
				0: begin
					if (mem_do_rdata) begin
						mem_valid <= 1;
						mem_wstrb <= 0;
						mem_state <= 1;
					end
					if (mem_do_wdata) begin
						mem_valid <= 1;
						mem_state <= 2;
					end
				end
				1: begin
					`assert(mem_wstrb == 0);
					`assert(mem_do_rdata);
					`assert(mem_valid);
					if (mem_xfer) begin
						mem_valid <= 0;
						mem_state <= 0;
					end
				end
				2: begin
					`assert(mem_wstrb != 0);
					`assert(mem_do_wdata);
					if (mem_xfer) begin
						mem_valid <= 0;
						mem_state <= 0;
					end
				end
			endcase
		end
	end

	// instruction memory interface

	reg instr_state;
	reg [31:0] instr_rdata_q;
	reg instr_do_rinst;

	reg [31:0] instr_addr_q;

	wire instr_xfer = instr_valid && instr_ready;

	wire instr_done = resetn && instr_xfer && instr_state && instr_do_rinst;

	assign instr_valid = instr_do_rinst;

	wire [31:0] instr_rdata_latched = (instr_xfer || LATCHED_MEM_RDATA) ? instr_rdata : instr_rdata_q;

	assign instr_addr = fetch_hart != no_hart ? {next_pc[fetch_hart][31:2], 2'b00} : instr_addr_q;

	always @(posedge clk) begin
		if (instr_xfer)
			instr_rdata_q <= instr_rdata;
	end

	always @(posedge clk) begin
		if (!resetn || trap) begin
			if (!resetn)
                    instr_state <= 0;
		end else begin
			instr_addr_q <= instr_addr;
			case (instr_state)
				0: begin
					if (instr_do_rinst) begin
						mem_instr <= instr_do_rinst;
						instr_state <= 1;
					end
				end
				1: begin
					`assert(instr_do_rinst);
					`assert(instr_valid == 1);
					`assert(mem_instr == (instr_do_rinst));
					if (instr_xfer) begin
						instr_state <= 0;
					end
				end
			endcase
		end
	end


	// Instruction Decoder

	reg [THREADS-1:0] instr_lui, instr_auipc, instr_jal, instr_jalr;
	reg [THREADS-1:0] instr_beq, instr_bne, instr_blt, instr_bge, instr_bltu, instr_bgeu;
	reg [THREADS-1:0] instr_lb, instr_lh, instr_lw, instr_lbu, instr_lhu, instr_sb, instr_sh, instr_sw;
	reg [THREADS-1:0] instr_addi, instr_slti, instr_sltiu, instr_xori, instr_ori, instr_andi, instr_slli, instr_srli, instr_srai;
	reg [THREADS-1:0] instr_add, instr_sub, instr_sll, instr_slt, instr_sltu, instr_xor, instr_srl, instr_sra, instr_or, instr_and;
	reg [THREADS-1:0] instr_rdcycle, instr_rdcycleh, instr_rdinstr, instr_rdinstrh, instr_ecall_ebreak;
	reg [THREADS-1:0] instr_getq, instr_setq, instr_retirq, instr_maskirq, instr_waitirq, instr_timer;
	// USED FOR RETURNING THE CURRENT HART_ID
	reg [THREADS-1:0] instr_csrrs;
	wire [THREADS-1:0] instr_trap;

	reg [regindex_bits-1:0] decoded_rd [0:THREADS-1];
	reg [regindex_bits-1:0] decoded_rs1 [0:THREADS-1];
	reg [regindex_bits-1:0] decoded_rs2 [0:THREADS-1];
	reg [31:0] decoded_imm [0:THREADS-1];
	reg [31:0] decoded_imm_j [0:THREADS-1];
	reg decoder_trigger;
	reg decoder_trigger_q;

	reg [THREADS-1:0] is_lui_auipc_jal;
	reg [THREADS-1:0] is_lb_lh_lw_lbu_lhu;
	reg [THREADS-1:0] is_slli_srli_srai;
	reg [THREADS-1:0] is_jalr_addi_slti_sltiu_xori_ori_andi;
	reg [THREADS-1:0] is_sb_sh_sw;
	reg [THREADS-1:0] is_sll_srl_sra;
	reg [THREADS-1:0] is_lui_auipc_jal_jalr_addi_add_sub;
	reg [THREADS-1:0] is_slti_blt_slt;
	reg [THREADS-1:0] is_sltiu_bltu_sltu;
	reg [THREADS-1:0] is_beq_bne_blt_bge_bltu_bgeu;
	reg [THREADS-1:0] is_lbu_lhu_lw;
	reg [THREADS-1:0] is_alu_reg_imm;
	reg [THREADS-1:0] is_alu_reg_reg;
	reg [THREADS-1:0] is_compare;

	reg [THREADS-1:0] is_csrrs = 0;

	wire [THREADS-1:0] is_rdcycle_rdcycleh_rdinstr_rdinstrh;

	genvar a;
	generate
	for (a = 0; a < THREADS; a = a + 1) begin
		assign instr_trap[a] = (CATCH_ILLINSN) && !{instr_lui[a], instr_auipc[a], instr_jal[a], instr_jalr[a],
			   instr_beq[a], instr_bne[a], instr_blt[a], instr_bge[a], instr_bltu[a], instr_bgeu[a],
			   instr_lb[a], instr_lh[a], instr_lw[a], instr_lbu[a], instr_lhu[a], instr_sb[a], instr_sh[a], instr_sw[a],
			   instr_addi[a], instr_slti[a], instr_sltiu[a], instr_xori[a], instr_ori[a], instr_andi[a], instr_slli[a], instr_srli[a], instr_srai[a],
			   instr_add[a], instr_sub[a], instr_sll[a], instr_slt[a], instr_sltu[a], instr_xor[a], instr_srl[a], instr_sra[a], instr_or[a], instr_and[a],
			   instr_rdcycle[a], instr_rdcycleh[a], instr_rdinstr[a], instr_rdinstrh[a],
			   instr_getq[a], instr_setq[a], instr_retirq[a], instr_maskirq[a], instr_waitirq[a], instr_timer[a], instr_csrrs[a]};

		assign is_rdcycle_rdcycleh_rdinstr_rdinstrh[a] = |{instr_rdcycle[a], instr_rdcycleh[a], instr_rdinstr[a], instr_rdinstrh[a]};
	end
	endgenerate

	/* TODO: DEBUG
	reg [63:0] new_ascii_instr;
	`FORMAL_KEEP reg [63:0] dbg_ascii_instr;
	`FORMAL_KEEP reg [31:0] dbg_insn_imm;
	`FORMAL_KEEP reg [4:0] dbg_insn_rs1;
	`FORMAL_KEEP reg [4:0] dbg_insn_rs2;
	`FORMAL_KEEP reg [4:0] dbg_insn_rd;
	`FORMAL_KEEP reg [31:0] dbg_rs1val;
	`FORMAL_KEEP reg [31:0] dbg_rs2val;
	`FORMAL_KEEP reg dbg_rs1val_valid;
	`FORMAL_KEEP reg dbg_rs2val_valid;

	always @* begin
		new_ascii_instr = "";

		if (instr_lui)      new_ascii_instr = "lui";
		if (instr_auipc)    new_ascii_instr = "auipc";
		if (instr_jal)      new_ascii_instr = "jal";
		if (instr_jalr)     new_ascii_instr = "jalr";

		if (instr_beq)      new_ascii_instr = "beq";
		if (instr_bne)      new_ascii_instr = "bne";
		if (instr_blt)      new_ascii_instr = "blt";
		if (instr_bge)      new_ascii_instr = "bge";
		if (instr_bltu)     new_ascii_instr = "bltu";
		if (instr_bgeu)     new_ascii_instr = "bgeu";

		if (instr_lb)       new_ascii_instr = "lb";
		if (instr_lh)       new_ascii_instr = "lh";
		if (instr_lw)       new_ascii_instr = "lw";
		if (instr_lbu)      new_ascii_instr = "lbu";
		if (instr_lhu)      new_ascii_instr = "lhu";
		if (instr_sb)       new_ascii_instr = "sb";
		if (instr_sh)       new_ascii_instr = "sh";
		if (instr_sw)       new_ascii_instr = "sw";

		if (instr_addi)     new_ascii_instr = "addi";
		if (instr_slti)     new_ascii_instr = "slti";
		if (instr_sltiu)    new_ascii_instr = "sltiu";
		if (instr_xori)     new_ascii_instr = "xori";
		if (instr_ori)      new_ascii_instr = "ori";
		if (instr_andi)     new_ascii_instr = "andi";
		if (instr_slli)     new_ascii_instr = "slli";
		if (instr_srli)     new_ascii_instr = "srli";
		if (instr_srai)     new_ascii_instr = "srai";

		if (instr_add)      new_ascii_instr = "add";
		if (instr_sub)      new_ascii_instr = "sub";
		if (instr_sll)      new_ascii_instr = "sll";
		if (instr_slt)      new_ascii_instr = "slt";
		if (instr_sltu)     new_ascii_instr = "sltu";
		if (instr_xor)      new_ascii_instr = "xor";
		if (instr_srl)      new_ascii_instr = "srl";
		if (instr_sra)      new_ascii_instr = "sra";
		if (instr_or)       new_ascii_instr = "or";
		if (instr_and)      new_ascii_instr = "and";

		if (instr_rdcycle)  new_ascii_instr = "rdcycle";
		if (instr_rdcycleh) new_ascii_instr = "rdcycleh";
		if (instr_rdinstr)  new_ascii_instr = "rdinstr";
		if (instr_rdinstrh) new_ascii_instr = "rdinstrh";

		if (instr_getq)     new_ascii_instr = "getq";
		if (instr_setq)     new_ascii_instr = "setq";
		if (instr_retirq)   new_ascii_instr = "retirq";
		if (instr_maskirq)  new_ascii_instr = "maskirq";
		if (instr_waitirq)  new_ascii_instr = "waitirq";
		if (instr_timer)    new_ascii_instr = "timer";

		if (instr_csrrs)	new_ascii_instr = "csrrs";
	end

	*/
	// TODO: DEBUG
`ifdef DEBUGASM
	always @(posedge clk) begin
		if (dbg_next) begin
			$display("debugasm %x %x %s", dbg_insn_addr, dbg_insn_opcode, dbg_ascii_instr ? dbg_ascii_instr : "*");
		end
	end
`endif

`ifdef DEBUG
	always @(posedge clk) begin
		if (dbg_next) begin
			if (&dbg_insn_opcode[1:0])
				$display("DECODE: 0x%08x 0x%08x %-0s", dbg_insn_addr, dbg_insn_opcode, dbg_ascii_instr ? dbg_ascii_instr : "UNKNOWN");
			else
				$display("DECODE: 0x%08x     0x%04x %-0s", dbg_insn_addr, dbg_insn_opcode[15:0], dbg_ascii_instr ? dbg_ascii_instr : "UNKNOWN");
		end
	end
`endif

	reg [THREADS-1:0] csr_unknown;

	integer b;
	always @(posedge clk) begin
		for (b = 0; b < THREADS; b = b + 1)
			is_lbu_lhu_lw[b] <= |{instr_lbu[b], instr_lhu[b], instr_lw[b]};

		if (instr_do_rinst && instr_done) begin
			instr_lui[fetch_hart]     <= instr_rdata_latched[6:0] == 7'b0110111;
			instr_auipc[fetch_hart]   <= instr_rdata_latched[6:0] == 7'b0010111;
			instr_jal[fetch_hart]     <= instr_rdata_latched[6:0] == 7'b1101111;
			instr_jalr[fetch_hart]    <= instr_rdata_latched[6:0] == 7'b1100111 && instr_rdata_latched[14:12] == 3'b000;
			// TODO: IRQ
			instr_retirq[fetch_hart]  <= instr_rdata_latched[6:0] == 7'b0001011 && instr_rdata_latched[31:25] == 7'b0000010 && ENABLE_IRQ;
			instr_waitirq[fetch_hart] <= instr_rdata_latched[6:0] == 7'b0001011 && instr_rdata_latched[31:25] == 7'b0000100 && ENABLE_IRQ;

			instr_csrrs[fetch_hart]   <= instr_rdata_latched[6:0] == 7'b1110011 && instr_rdata_latched[14:12] == 3'b010;

			is_beq_bne_blt_bge_bltu_bgeu[fetch_hart] <= instr_rdata_latched[6:0] == 7'b1100011;
			is_lb_lh_lw_lbu_lhu[fetch_hart]          <= instr_rdata_latched[6:0] == 7'b0000011;
			is_sb_sh_sw[fetch_hart]                  <= instr_rdata_latched[6:0] == 7'b0100011;
			is_alu_reg_imm[fetch_hart]               <= instr_rdata_latched[6:0] == 7'b0010011;
			is_alu_reg_reg[fetch_hart]               <= instr_rdata_latched[6:0] == 7'b0110011;

			{ decoded_imm_j[fetch_hart][31:20], decoded_imm_j[fetch_hart][10:1], decoded_imm_j[fetch_hart][11], decoded_imm_j[fetch_hart][19:12], decoded_imm_j[fetch_hart][0] } <= $signed({instr_rdata_latched[31:12], 1'b0});

			decoded_rd[fetch_hart] <= instr_rdata_latched[11:7];
			decoded_rs1[fetch_hart] <= instr_rdata_latched[19:15];
			decoded_rs2[fetch_hart] <= instr_rdata_latched[24:20];

			// TODO: IRQ
			if (instr_rdata_latched[6:0] == 7'b0001011 && instr_rdata_latched[31:25] == 7'b0000000 && ENABLE_IRQ && ENABLE_IRQ_QREGS)
				decoded_rs1[fetch_hart][regindex_bits-1] <= 1; // instr_getq

			if (instr_rdata_latched[6:0] == 7'b0001011 && instr_rdata_latched[31:25] == 7'b0000010 && ENABLE_IRQ)
				decoded_rs1[fetch_hart] <= ENABLE_IRQ_QREGS ? irqregs_offset : 3; // instr_retirq
		end

		if (decoder_trigger) begin
			instr_beq[fetch_hart]   <= is_beq_bne_blt_bge_bltu_bgeu[fetch_hart] && instr_rdata_q[14:12] == 3'b000;
			instr_bne[fetch_hart]   <= is_beq_bne_blt_bge_bltu_bgeu[fetch_hart] && instr_rdata_q[14:12] == 3'b001;
			instr_blt[fetch_hart]   <= is_beq_bne_blt_bge_bltu_bgeu[fetch_hart] && instr_rdata_q[14:12] == 3'b100;
			instr_bge[fetch_hart]   <= is_beq_bne_blt_bge_bltu_bgeu[fetch_hart] && instr_rdata_q[14:12] == 3'b101;
			instr_bltu[fetch_hart]  <= is_beq_bne_blt_bge_bltu_bgeu[fetch_hart] && instr_rdata_q[14:12] == 3'b110;
			instr_bgeu[fetch_hart]  <= is_beq_bne_blt_bge_bltu_bgeu[fetch_hart] && instr_rdata_q[14:12] == 3'b111;

			instr_lb[fetch_hart]    <= is_lb_lh_lw_lbu_lhu[fetch_hart] && instr_rdata_q[14:12] == 3'b000;
			instr_lh[fetch_hart]    <= is_lb_lh_lw_lbu_lhu[fetch_hart] && instr_rdata_q[14:12] == 3'b001;
			instr_lw[fetch_hart]    <= is_lb_lh_lw_lbu_lhu[fetch_hart] && instr_rdata_q[14:12] == 3'b010;
			instr_lbu[fetch_hart]   <= is_lb_lh_lw_lbu_lhu[fetch_hart] && instr_rdata_q[14:12] == 3'b100;
			instr_lhu[fetch_hart]   <= is_lb_lh_lw_lbu_lhu[fetch_hart] && instr_rdata_q[14:12] == 3'b101;

			instr_sb[fetch_hart]    <= is_sb_sh_sw[fetch_hart] && instr_rdata_q[14:12] == 3'b000;
			instr_sh[fetch_hart]    <= is_sb_sh_sw[fetch_hart] && instr_rdata_q[14:12] == 3'b001;
			instr_sw[fetch_hart]    <= is_sb_sh_sw[fetch_hart] && instr_rdata_q[14:12] == 3'b010;

			instr_addi[fetch_hart]  <= is_alu_reg_imm[fetch_hart] && instr_rdata_q[14:12] == 3'b000;
			instr_slti[fetch_hart]  <= is_alu_reg_imm[fetch_hart] && instr_rdata_q[14:12] == 3'b010;
			instr_sltiu[fetch_hart] <= is_alu_reg_imm[fetch_hart] && instr_rdata_q[14:12] == 3'b011;
			instr_xori[fetch_hart]  <= is_alu_reg_imm[fetch_hart] && instr_rdata_q[14:12] == 3'b100;
			instr_ori[fetch_hart]   <= is_alu_reg_imm[fetch_hart] && instr_rdata_q[14:12] == 3'b110;
			instr_andi[fetch_hart]  <= is_alu_reg_imm[fetch_hart] && instr_rdata_q[14:12] == 3'b111;

			instr_slli[fetch_hart]  <= is_alu_reg_imm[fetch_hart] && instr_rdata_q[14:12] == 3'b001 && instr_rdata_q[31:25] == 7'b0000000;
			instr_srli[fetch_hart]  <= is_alu_reg_imm[fetch_hart] && instr_rdata_q[14:12] == 3'b101 && instr_rdata_q[31:25] == 7'b0000000;
			instr_srai[fetch_hart]  <= is_alu_reg_imm[fetch_hart] && instr_rdata_q[14:12] == 3'b101 && instr_rdata_q[31:25] == 7'b0100000;

			instr_add[fetch_hart] <= is_alu_reg_reg[fetch_hart] && instr_rdata_q[14:12] == 3'b000 && instr_rdata_q[31:25] == 7'b0000000;
			instr_sub[fetch_hart] <= is_alu_reg_reg[fetch_hart] && instr_rdata_q[14:12] == 3'b000 && instr_rdata_q[31:25] == 7'b0100000;
			instr_sll[fetch_hart] <= is_alu_reg_reg[fetch_hart] && instr_rdata_q[14:12] == 3'b001 && instr_rdata_q[31:25] == 7'b0000000;
			instr_slt[fetch_hart] <= is_alu_reg_reg[fetch_hart] && instr_rdata_q[14:12] == 3'b010 && instr_rdata_q[31:25] == 7'b0000000;
			instr_sltu[fetch_hart] <= is_alu_reg_reg[fetch_hart] && instr_rdata_q[14:12] == 3'b011 && instr_rdata_q[31:25] == 7'b0000000;
			instr_xor[fetch_hart] <= is_alu_reg_reg[fetch_hart] && instr_rdata_q[14:12] == 3'b100 && instr_rdata_q[31:25] == 7'b0000000;
			instr_srl[fetch_hart] <= is_alu_reg_reg[fetch_hart] && instr_rdata_q[14:12] == 3'b101 && instr_rdata_q[31:25] == 7'b0000000;
			instr_sra[fetch_hart] <= is_alu_reg_reg[fetch_hart] && instr_rdata_q[14:12] == 3'b101 && instr_rdata_q[31:25] == 7'b0100000;
			instr_or[fetch_hart] <= is_alu_reg_reg[fetch_hart] && instr_rdata_q[14:12] == 3'b110 && instr_rdata_q[31:25] == 7'b0000000;
			instr_and[fetch_hart] <= is_alu_reg_reg[fetch_hart] && instr_rdata_q[14:12] == 3'b111 && instr_rdata_q[31:25] == 7'b0000000;

			instr_rdcycle[fetch_hart]  <= ((instr_rdata_q[6:0] == 7'b1110011 && instr_rdata_q[31:12] == 'b11000000000000000010) ||
			                    		  (instr_rdata_q[6:0] == 7'b1110011 && instr_rdata_q[31:12] == 'b11000000000100000010)) && ENABLE_COUNTERS;
			instr_rdcycleh[fetch_hart] <= ((instr_rdata_q[6:0] == 7'b1110011 && instr_rdata_q[31:12] == 'b11001000000000000010) ||
			                              (instr_rdata_q[6:0] == 7'b1110011 && instr_rdata_q[31:12] == 'b11001000000100000010)) && ENABLE_COUNTERS && ENABLE_COUNTERS64;
			instr_rdinstr[fetch_hart]  <=  (instr_rdata_q[6:0] == 7'b1110011 && instr_rdata_q[31:12] == 'b11000000001000000010) && ENABLE_COUNTERS;
			instr_rdinstrh[fetch_hart] <=  (instr_rdata_q[6:0] == 7'b1110011 && instr_rdata_q[31:12] == 'b11001000001000000010) && ENABLE_COUNTERS && ENABLE_COUNTERS64;

			instr_ecall_ebreak[fetch_hart] <= instr_rdata_q[6:0] == 7'b1110011 && !instr_rdata_q[31:21] && !instr_rdata_q[19:7];

			// TODO: IRQ
			instr_getq[fetch_hart]    <= instr_rdata_q[6:0] == 7'b0001011 && instr_rdata_q[31:25] == 7'b0000000 && ENABLE_IRQ && ENABLE_IRQ_QREGS;
			instr_setq[fetch_hart]    <= instr_rdata_q[6:0] == 7'b0001011 && instr_rdata_q[31:25] == 7'b0000001 && ENABLE_IRQ && ENABLE_IRQ_QREGS;
			instr_maskirq[fetch_hart] <= instr_rdata_q[6:0] == 7'b0001011 && instr_rdata_q[31:25] == 7'b0000011 && ENABLE_IRQ;
			instr_timer[fetch_hart]   <= instr_rdata_q[6:0] == 7'b0001011 && instr_rdata_q[31:25] == 7'b0000101 && ENABLE_IRQ && ENABLE_IRQ_TIMER;

			is_slli_srli_srai[fetch_hart] <= is_alu_reg_imm[fetch_hart] && |{
				instr_rdata_q[14:12] == 3'b001 && instr_rdata_q[31:25] == 7'b0000000,
				instr_rdata_q[14:12] == 3'b101 && instr_rdata_q[31:25] == 7'b0000000,
				instr_rdata_q[14:12] == 3'b101 && instr_rdata_q[31:25] == 7'b0100000
			};

			is_jalr_addi_slti_sltiu_xori_ori_andi[fetch_hart] <= instr_jalr[fetch_hart] || is_alu_reg_imm[fetch_hart] && |{
				instr_rdata_q[14:12] == 3'b000,
				instr_rdata_q[14:12] == 3'b010,
				instr_rdata_q[14:12] == 3'b011,
				instr_rdata_q[14:12] == 3'b100,
				instr_rdata_q[14:12] == 3'b110,
				instr_rdata_q[14:12] == 3'b111
			};

			is_sll_srl_sra[fetch_hart] <= is_alu_reg_reg[fetch_hart] && |{
				instr_rdata_q[14:12] == 3'b001 && instr_rdata_q[31:25] == 7'b0000000,
				instr_rdata_q[14:12] == 3'b101 && instr_rdata_q[31:25] == 7'b0000000,
				instr_rdata_q[14:12] == 3'b101 && instr_rdata_q[31:25] == 7'b0100000
			};

			is_lui_auipc_jal_jalr_addi_add_sub[fetch_hart] <= 0;
			is_compare[fetch_hart] <= 0;

			(* parallel_case *)
			case (1'b1)
				instr_jal[fetch_hart]:
					decoded_imm[fetch_hart] <= decoded_imm_j[fetch_hart];
				|{instr_lui[fetch_hart], instr_auipc[fetch_hart]}:
					decoded_imm[fetch_hart] <= instr_rdata_q[31:12] << 12;
				|{instr_jalr[fetch_hart], is_lb_lh_lw_lbu_lhu[fetch_hart], is_alu_reg_imm[fetch_hart]}:
					decoded_imm[fetch_hart] <= $signed(instr_rdata_q[31:20]);
				is_beq_bne_blt_bge_bltu_bgeu[fetch_hart]:
					decoded_imm[fetch_hart] <= $signed({instr_rdata_q[31], instr_rdata_q[7], instr_rdata_q[30:25], instr_rdata_q[11:8], 1'b0});
				is_sb_sh_sw[fetch_hart]:
					decoded_imm[fetch_hart] <= $signed({instr_rdata_q[31:25], instr_rdata_q[11:7]});
				is_csrrs[fetch_hart]:
					// only handle requests for the mhartid csr
					if (instr_rdata_q[31:20] != 'hf14)
						csr_unknown[fetch_hart] <= 1;
				default:
					decoded_imm[fetch_hart] <= 1'bx;
			endcase
		end

		if (!resetn) begin
			is_beq_bne_blt_bge_bltu_bgeu <= 0;
			is_compare <= 0;

			instr_beq   <= 0;
			instr_bne   <= 0;
			instr_blt   <= 0;
			instr_bge   <= 0;
			instr_bltu  <= 0;
			instr_bgeu  <= 0;

			instr_addi  <= 0;
			instr_slti  <= 0;
			instr_sltiu <= 0;
			instr_xori  <= 0;
			instr_ori   <= 0;
			instr_andi  <= 0;

			instr_add   <= 0;
			instr_sub   <= 0;
			instr_sll   <= 0;
			instr_slt   <= 0;
			instr_sltu  <= 0;
			instr_xor   <= 0;
			instr_srl   <= 0;
			instr_sra   <= 0;
			instr_or    <= 0;
			instr_and   <= 0;

			instr_csrrs <= 0;
		end

		for (b = 0; b < THREADS; b = b + 1) begin
			is_csrrs[b] <= instr_csrrs[b];

			is_lui_auipc_jal[b] <= |{instr_lui[b], instr_auipc[b], instr_jal[b]};
			is_lui_auipc_jal_jalr_addi_add_sub[b] <= |{instr_lui[b], instr_auipc[b], instr_jal[b], instr_jalr[b], instr_addi[b], instr_add[b], instr_sub[b]};
			is_slti_blt_slt[b] <= |{instr_slti[b], instr_blt[b], instr_slt[b]};
			is_sltiu_bltu_sltu[b] <= |{instr_sltiu[b], instr_bltu[b], instr_sltu[b]};
			is_compare[b] <= |{is_beq_bne_blt_bge_bltu_bgeu[b], instr_slti[b], instr_slt[b], instr_sltiu[b], instr_sltu[b]};
		end
	end


	// Main State Machine

	localparam cpu_state_trap   = 8'b10000000;
	localparam cpu_state_fetch  = 8'b01000000;
	localparam cpu_state_ld_rs1 = 8'b00100000;
	localparam cpu_state_ld_rs2 = 8'b00010000;
	localparam cpu_state_exec   = 8'b00001000;
	localparam cpu_state_shift  = 8'b00000100;
	localparam cpu_state_stmem  = 8'b00000010;
	localparam cpu_state_ldmem  = 8'b00000001;
	localparam cpu_state_busy   = 8'b11111111;

	reg [1:0] irq_state;

	`FORMAL_KEEP reg [127:0] dbg_ascii_state [0:THREADS-1];

	integer i;
	always @* begin
		for (i = 0; i < THREADS; i = i + 1) begin
			dbg_ascii_state[i] = "";
			if (hart_ready[i] == cpu_state_trap)   dbg_ascii_state[i] = "trap";
			if (hart_ready[i] == cpu_state_fetch)  dbg_ascii_state[i] = "fetch";
			if (hart_ready[i] == cpu_state_ld_rs1) dbg_ascii_state[i] = "ld_rs1";
			if (hart_ready[i] == cpu_state_ld_rs2) dbg_ascii_state[i] = "ld_rs2";
			if (hart_ready[i] == cpu_state_exec)   dbg_ascii_state[i] = "exec";
			if (hart_ready[i] == cpu_state_shift)  dbg_ascii_state[i] = "shift";
			if (hart_ready[i] == cpu_state_stmem)  dbg_ascii_state[i] = "stmem";
			if (hart_ready[i] == cpu_state_ldmem)  dbg_ascii_state[i] = "ldmem";
			if (hart_ready[i] == cpu_state_busy)   dbg_ascii_state[i] = "busy";
		end
	end

	reg set_instr_do_rinst;
	reg set_mem_do_rdata;
	reg set_mem_do_wdata;

	reg latched_store [0:THREADS-1];
	reg latched_stalu [0:THREADS-1];
	reg latched_branch [0:THREADS-1];
	reg latched_trace;
	reg latched_is_lu [0:THREADS-1];
	reg latched_is_lh [0:THREADS-1];
	reg latched_is_lb [0:THREADS-1];
	reg [regindex_bits-1:0] latched_rd [0:THREADS-1];

	reg [31:0] current_pc;
	genvar h;
	generate
		for (h = 0; h < THREADS; h = h + 1)
			assign next_pc[h] = latched_store[h] && latched_branch[h] ? reg_out[h] & ~1 : reg_next_pc[h];
	endgenerate

	// TODO: IRQ
	reg [31:0] next_irq_pending;
	reg do_waitirq;

	reg [31:0] alu_out [0:THREADS-1];
	reg [31:0] alu_out_q [0:THREADS-1];
	reg alu_out_0 [0:THREADS-1];
	reg alu_out_0_q [0:THREADS-1];
	reg alu_wait, alu_wait_2;

	reg [31:0] alu_add_sub;
	reg [31:0] alu_shl, alu_shr;
	reg alu_eq, alu_ltu, alu_lts;

	/* ALU */
	generate if (TWO_CYCLE_ALU) begin
		always @(posedge clk) begin
			alu_add_sub <= instr_sub[exec_hart] ? reg_op1[exec_hart] - reg_op2[exec_hart] : reg_op1[exec_hart] + reg_op2[exec_hart];
			alu_eq <= reg_op1[exec_hart] == reg_op2[exec_hart];
			alu_lts <= $signed(reg_op1[exec_hart]) < $signed(reg_op2[exec_hart]);
			alu_ltu <= reg_op1[exec_hart] < reg_op2[exec_hart];
			alu_shl <= reg_op1[exec_hart] << reg_op2[exec_hart][4:0];
			alu_shr <= $signed({instr_sra[exec_hart] || instr_srai[exec_hart] ? reg_op1[exec_hart][31] : 1'b0, reg_op1[exec_hart]}) >>> reg_op2[exec_hart][4:0];
		end
	end else begin
		always @* begin
			alu_add_sub = instr_sub[exec_hart] ? reg_op1[exec_hart] - reg_op2[exec_hart] : reg_op1[exec_hart] + reg_op2[exec_hart];
			alu_eq = reg_op1[exec_hart] == reg_op2[exec_hart];
			alu_lts = $signed(reg_op1[exec_hart]) < $signed(reg_op2[exec_hart]);
			alu_ltu = reg_op1[exec_hart] < reg_op2[exec_hart];
			alu_shl = reg_op1[exec_hart] << reg_op2[exec_hart][4:0];
			alu_shr = $signed({instr_sra[exec_hart] || instr_srai[exec_hart] ? reg_op1[exec_hart][31] : 1'b0, reg_op1[exec_hart]}) >>> reg_op2[exec_hart][4:0];
		end
	end endgenerate

	always @* begin
		if (exec_hart != no_hart) begin
			alu_out_0[exec_hart] = 'bx;
			(* parallel_case, full_case *)
			case (1'b1)
				instr_beq[exec_hart]:
					alu_out_0[exec_hart] = alu_eq;
				instr_bne[exec_hart]:
					alu_out_0[exec_hart] = !alu_eq;
				instr_bge[exec_hart]:
					alu_out_0[exec_hart] = !alu_lts;
				instr_bgeu[exec_hart]:
					alu_out_0[exec_hart] = !alu_ltu;
				is_slti_blt_slt[exec_hart] && (!TWO_CYCLE_COMPARE || !{instr_beq[exec_hart],instr_bne[exec_hart],instr_bge[exec_hart],instr_bgeu[exec_hart]}):
					alu_out_0[exec_hart] = alu_lts;
				is_sltiu_bltu_sltu[exec_hart] && (!TWO_CYCLE_COMPARE || !{instr_beq[exec_hart],instr_bne[exec_hart],instr_bge[exec_hart],instr_bgeu[exec_hart]}):
					alu_out_0[exec_hart] = alu_ltu;
			endcase

			alu_out[exec_hart] = 'bx;
			(* parallel_case, full_case *)
			case (1'b1)
				is_lui_auipc_jal_jalr_addi_add_sub[exec_hart]:
					alu_out[exec_hart] = alu_add_sub;
				is_compare[exec_hart]:
					alu_out[exec_hart] = alu_out_0[exec_hart];
				instr_xori[exec_hart] || instr_xor[exec_hart]:
					alu_out[exec_hart] = reg_op1[exec_hart] ^ reg_op2[exec_hart];
				instr_ori[exec_hart] || instr_or[exec_hart]:
					alu_out[exec_hart] = reg_op1[exec_hart] | reg_op2[exec_hart];
				instr_andi[exec_hart] || instr_and[exec_hart]:
					alu_out[exec_hart] = reg_op1[exec_hart] & reg_op2[exec_hart];
				BARREL_SHIFTER && (instr_sll[exec_hart] || instr_slli[exec_hart]):
					alu_out[exec_hart] = alu_shl;
				BARREL_SHIFTER && (instr_srl[exec_hart] || instr_srli[exec_hart]|| instr_sra[exec_hart]|| instr_srai[exec_hart]):
					alu_out[exec_hart] = alu_shr;
			endcase
		end

`ifdef RISCV_FORMAL_BLACKBOX_ALU
		alu_out_0 = $anyseq;
		alu_out = $anyseq;
`endif
	end
	/* END ALU */

	/* WRITE TO REG */
	reg cpuregs_write [0:THREADS-1];
	reg [31:0] cpuregs_wrdata [0:THREADS-1];

	reg [31:0] cpuregs_rs1 [0:THREADS-1];
	reg [31:0] cpuregs_rs2 [0:THREADS-1];
	reg [regindex_bits-1:0] decoded_rs [0:THREADS-1];

	always @* begin
		if (fetch_hart != no_hart) begin
			cpuregs_write[fetch_hart] = 0;
			cpuregs_wrdata[fetch_hart] = 'bx;

			(* parallel_case *)
			case (1'b1)
				latched_branch[fetch_hart]: begin
					cpuregs_wrdata[fetch_hart] = reg_pc[fetch_hart] + 4;
					cpuregs_write[fetch_hart] = 1;
				end
				latched_store[fetch_hart] && !latched_branch[fetch_hart]: begin
					cpuregs_wrdata[fetch_hart] = latched_stalu[fetch_hart] ? alu_out_q[fetch_hart] : reg_out[fetch_hart];
					cpuregs_write[fetch_hart] = 1;
				end
				ENABLE_IRQ && irq_state[0]: begin
					cpuregs_wrdata[fetch_hart] = reg_next_pc[fetch_hart];
					cpuregs_write[fetch_hart] = 1;
				end
				ENABLE_IRQ && irq_state[1]: begin
					cpuregs_wrdata[fetch_hart] = irq_pending & ~irq_mask;
					cpuregs_write[fetch_hart] = 1;
				end
			endcase
		end
	end
	/* END WRITE TO REG */

	/* INSTANTIATE EXTERNAL REG */
	wire [31:0] cpuregs_rdata1 [0:THREADS-1];
	wire [31:0] cpuregs_rdata2 [0:THREADS-1];

	wire [5:0] cpuregs_waddr [0:THREADS-1];
	wire [5:0] cpuregs_raddr1 [0:THREADS-1];
	wire [5:0] cpuregs_raddr2 [0:THREADS-1];


	genvar j;
	generate
		for (j = 0; j < THREADS; j = j + 1) begin : cpuregs
			`PICORV32_REGS cpuregs (
				.clk(clk),
				.wen(cpuregs_write[j] && latched_rd[j]),
				.waddr(cpuregs_waddr[j]),
				.raddr1(cpuregs_raddr1[j]),
				.raddr2(cpuregs_raddr2[j]),
				.wdata(cpuregs_wrdata[j]),
				.rdata1(cpuregs_rdata1[j]),
				.rdata2(cpuregs_rdata2[j])
			);
			assign cpuregs_waddr[j] = latched_rd[j];
			assign cpuregs_raddr1[j] = ENABLE_REGS_DUALPORT ? decoded_rs1[j] : decoded_rs[j];
			assign cpuregs_raddr2[j] = ENABLE_REGS_DUALPORT ? decoded_rs2[j] : 0;
		end
	endgenerate

	generate
		for (j = 0; j < THREADS; j = j + 1) begin
			always @* begin
				decoded_rs[j] = 'bx;
				if (ENABLE_REGS_DUALPORT) begin
					cpuregs_rs1[j] = decoded_rs1[j] ? cpuregs_rdata1[j] : 0;
					cpuregs_rs2[j] = decoded_rs2[j] ? cpuregs_rdata2[j] : 0;
				end else begin
					decoded_rs[j] = (ld_rs2_hart == j) ? decoded_rs2[j] : decoded_rs1[j];
					cpuregs_rs1[j] = decoded_rs[j] ? cpuregs_rdata1[j] : 0;
					cpuregs_rs2[j] = cpuregs_rs1[j];
				end
			end
		end
	endgenerate

	integer k;
	/* MAIN CPU LOOP */
	always @(posedge clk) begin
		trap <= 0;
		set_instr_do_rinst = 0;
		set_mem_do_rdata = 0;
		set_mem_do_wdata = 0;

		for (k = 0; k < THREADS; k = k + 1) begin
			alu_out_0_q[k] <= alu_out_0[k];
			alu_out_q[k] <= alu_out[k];
		end

		alu_wait <= 0;
		alu_wait_2 <= 0;

		if (ENABLE_COUNTERS) begin
			count_cycle <= resetn ? count_cycle + 1 : 0;
			if (!ENABLE_COUNTERS64) count_cycle[63:32] <= 0;
		end else begin
			count_cycle <= 'bx;
			count_instr <= 'bx;
		end

		// TODO: IRQ
		next_irq_pending = ENABLE_IRQ ? irq_pending & LATCHED_IRQ : 'bx;

		if (ENABLE_IRQ && ENABLE_IRQ_TIMER && timer) begin
			timer <= timer - 1;
		end

		decoder_trigger <= instr_do_rinst && instr_done;
		decoder_trigger_q <= decoder_trigger;
		// TODO: IRQ
		do_waitirq <= 0;

		trace_valid <= 0;

		if (!ENABLE_TRACE)
			trace_data <= 'bx;

		/* CPU SETUP */
		if (!resetn) begin
			for (k = 0; k < THREADS; k = k + 1) begin
				reg_pc[k] <= PROGADDR_RESET;
				reg_next_pc[k] <= PROGADDR_RESET;
				latched_store[k] <= 0;
				latched_stalu[k] <= 0;
				latched_branch[k] <= 0;
				latched_is_lu[k] <= 0;
				latched_is_lh[k] <= 0;
				latched_is_lb[k] <= 0;
			end
			if (ENABLE_COUNTERS)
				count_instr <= 0;
			latched_trace <= 0;
			//TODO IRQ
			irq_active <= 0;
			irq_delay <= 0;
			irq_mask <= ~0;
			next_irq_pending <= 0;
			irq_state <= 0;
			eoi <= 0;
			timer <= 0;
			if (~STACKADDR) begin
				for (k = 0; k < THREADS; k = k + 1) begin
					latched_store[k] <= 1;
					latched_rd[k] <= 2;
					reg_out[k] <= STACKADDR;
				end
			end
			fetch_hart = 0;
			hart_counter = 1;
			hart_ready[0] = cpu_state_busy;
			for (k = 1; k < THREADS; k = k + 1)
				hart_ready[k] = cpu_state_fetch;
		end else begin
			/* MAIN CPU BARREL */
			if (trap_hart != no_hart)
				trap <= 1;

			if (fetch_hart != no_hart) begin
				instr_do_rinst <= !do_waitirq;

				current_pc = reg_next_pc[fetch_hart];

				(* parallel_case *)
				case (1'b1)
					latched_branch[fetch_hart]: begin
						current_pc = latched_store[fetch_hart] ? (latched_stalu[fetch_hart] ? alu_out_q[fetch_hart] : reg_out[fetch_hart]) & ~1 : reg_next_pc[fetch_hart];
						`debug($display("ST_RD:  %2d 0x%08x, BRANCH 0x%08x", latched_rd[fetch_hart], reg_pc + 4, current_pc);)
					end
					latched_store[fetch_hart] && !latched_branch[fetch_hart]: begin
						`debug($display("ST_RD:  %2d 0x%08x", latched_rd[fetch_hart], latched_stalu[fetch_hart] ? alu_out_q[fetch_hart] : reg_out[fetch_hart]);)
					end
					// TODO: IRQ
					ENABLE_IRQ && irq_state[0]: begin
						current_pc[fetch_hart] = PROGADDR_IRQ;
						irq_active <= 1;
						instr_do_rinst <= 1;
					end
					ENABLE_IRQ && irq_state[1]: begin
						eoi <= irq_pending & ~irq_mask;
						next_irq_pending = next_irq_pending & irq_mask;
					end
				endcase

				if (ENABLE_TRACE && latched_trace) begin
					latched_trace <= 0;
					trace_valid <= 1;
					if (latched_branch[fetch_hart])
						trace_data <= (irq_active ? TRACE_IRQ : 0) | TRACE_BRANCH | (current_pc & 32'hfffffffe);
					else
						trace_data <= (irq_active ? TRACE_IRQ : 0) | (latched_stalu[fetch_hart] ? alu_out_q[fetch_hart] : reg_out[fetch_hart]);
				end

				reg_pc[fetch_hart] <= current_pc;
				reg_next_pc[fetch_hart] <= current_pc;

				latched_store[fetch_hart] <= 0;
				latched_stalu[fetch_hart] <= 0;
				latched_branch[fetch_hart] <= 0;
				latched_is_lu[fetch_hart] <= 0;
				latched_is_lh[fetch_hart] <= 0;
				latched_is_lb[fetch_hart] <= 0;
				latched_rd[fetch_hart] <= decoded_rd[fetch_hart];

				// TODO: IRQ
				if (ENABLE_IRQ && ((decoder_trigger && !irq_active && !irq_delay && |(irq_pending & ~irq_mask)) || irq_state)) begin
					irq_state <=
						irq_state == 2'b00 ? 2'b01 :
						irq_state == 2'b01 ? 2'b10 : 2'b00;
					if (ENABLE_IRQ_QREGS)
						latched_rd[fetch_hart] <= irqregs_offset | irq_state[0];
					else
						latched_rd[fetch_hart] <= irq_state[0] ? 4 : 3;
				end else
				if (ENABLE_IRQ && (decoder_trigger || do_waitirq) && instr_waitirq) begin
					if (irq_pending) begin
						latched_store[fetch_hart] <= 1;
						reg_out[fetch_hart] <= irq_pending;
						reg_next_pc[fetch_hart] <= current_pc + 4;
						instr_do_rinst <= 1;
					end else
						do_waitirq <= 1;
				end else
				if (decoder_trigger) begin
					`debug($display("-- %-0t", $time);)
					irq_delay <= irq_active;
					reg_next_pc[fetch_hart] <= current_pc + 4;
					if (ENABLE_TRACE)
						latched_trace <= 1;
					if (ENABLE_COUNTERS) begin
						count_instr <= count_instr + 1;
						if (!ENABLE_COUNTERS64) count_instr[63:32] <= 0;
					end
					if (instr_jal[fetch_hart]) begin
						reg_next_pc[fetch_hart] <= current_pc + decoded_imm_j[fetch_hart];
						latched_branch[fetch_hart] <= 1;
					end else begin
						instr_do_rinst <= 0;
						hart_ready[fetch_hart] = cpu_state_ld_rs1;
						fetch_hart = no_hart;
					end
				end
			end

			if (ld_rs1_hart != no_hart) begin
				reg_op1[ld_rs1_hart] <= 'bx;
				reg_op2[ld_rs1_hart] <= 'bx;

				(* parallel_case *)
				case (1'b1)
					CATCH_ILLINSN && instr_trap[ld_rs1_hart]: begin
						// TODO: IRQ
						`debug($display("EBREAK OR UNSUPPORTED INSN AT 0x%08x", reg_pc);)
						if (ENABLE_IRQ && !irq_mask[irq_ebreak] && !irq_active) begin
							next_irq_pending[irq_ebreak] = 1;
							hart_ready[ld_rs1_hart] = cpu_state_fetch;
							ld_rs1_hart = no_hart;
						end else begin
							hart_ready[ld_rs1_hart] = cpu_state_trap;
							ld_rs1_hart = no_hart;
						end
					end
					ENABLE_COUNTERS && is_rdcycle_rdcycleh_rdinstr_rdinstrh[ld_rs1_hart]: begin
						(* parallel_case, full_case *)
						case (1'b1)
							instr_rdcycle[ld_rs1_hart]:
								reg_out[ld_rs1_hart] <= count_cycle[31:0];
							instr_rdcycleh[ld_rs1_hart] && ENABLE_COUNTERS64:
								reg_out[ld_rs1_hart] <= count_cycle[63:32];
							instr_rdinstr[ld_rs1_hart]:
								reg_out[ld_rs1_hart] <= count_instr[31:0];
							instr_rdinstrh[ld_rs1_hart] && ENABLE_COUNTERS64:
								reg_out[ld_rs1_hart] <= count_instr[63:32];
						endcase
						latched_store[ld_rs1_hart] <= 1;
						hart_ready[ld_rs1_hart] = cpu_state_fetch;
						ld_rs1_hart = no_hart;
					end
					is_lui_auipc_jal[ld_rs1_hart]: begin
						reg_op1[ld_rs1_hart] <= instr_lui[ld_rs1_hart] ? 0 : reg_pc[ld_rs1_hart];
						reg_op2[ld_rs1_hart] <= decoded_imm[ld_rs1_hart];
						if (TWO_CYCLE_ALU)
							alu_wait <= 1;
						hart_ready[ld_rs1_hart] = cpu_state_exec;
						ld_rs1_hart = no_hart;
					end
					// TODO: IRQ
					ENABLE_IRQ && ENABLE_IRQ_QREGS && instr_getq[ld_rs1_hart]: begin
						`debug($display("LD_RS1: %2d 0x%08x", decoded_rs1[ld_rs1_hart], cpuregs_rs1[ld_rs1_hart]);)
						reg_out[ld_rs1_hart] <= cpuregs_rs1[ld_rs1_hart];
						//TODO: DEBUG
						//dbg_rs1val <= cpuregs_rs1[ld_rs1_hart];
						//dbg_rs1val_valid <= 1;
						latched_store[ld_rs1_hart] <= 1;
						hart_ready[ld_rs1_hart] = cpu_state_fetch;
						ld_rs1_hart = no_hart;
					end
					ENABLE_IRQ && ENABLE_IRQ_QREGS && instr_setq[ld_rs1_hart]: begin
						`debug($display("LD_RS1: %2d 0x%08x", decoded_rs1[ld_rs1_hart], cpuregs_rs1[ld_rs1_hart]);)
						reg_out[ld_rs1_hart] <= cpuregs_rs1[ld_rs1_hart];
						//TODO: DEBUG
						//dbg_rs1val <= cpuregs_rs1[ld_rs1_hart];
						//dbg_rs1val_valid <= 1;
						latched_rd[ld_rs1_hart] <= latched_rd[ld_rs1_hart] | irqregs_offset;
						latched_store[ld_rs1_hart] <= 1;
						hart_ready[ld_rs1_hart] = cpu_state_fetch;
						ld_rs1_hart = no_hart;
					end
					ENABLE_IRQ && instr_retirq[ld_rs1_hart]: begin
						eoi <= 0;
						irq_active <= 0;
						latched_branch[fetch_hart] <= 1;
						latched_store[ld_rs1_hart] <= 1;
						`debug($display("LD_RS1: %2d 0x%08x", decoded_rs1[ld_rs1_hart], cpuregs_rs1[ld_rs1_hart]);)
						reg_out[ld_rs1_hart] <= CATCH_MISALIGN ? (cpuregs_rs1[ld_rs1_hart] & 32'h fffffffe) : cpuregs_rs1[ld_rs1_hart];
						//TODO: DEBUG
						//dbg_rs1val <= cpuregs_rs1[ld_rs1_hart];
						//dbg_rs1val_valid <= 1;
						hart_ready[ld_rs1_hart] = cpu_state_fetch;
						ld_rs1_hart = no_hart;
					end
					ENABLE_IRQ && instr_maskirq[ld_rs1_hart]: begin
						latched_store[ld_rs1_hart] <= 1;
						reg_out[ld_rs1_hart] <= irq_mask;
						`debug($display("LD_RS1: %2d 0x%08x", decoded_rs1[ld_rs1_hart], cpuregs_rs1[ld_rs1_hart]);)
						irq_mask <= cpuregs_rs1[ld_rs1_hart] | MASKED_IRQ;
						//TODO: DEBUG
						//dbg_rs1val <= cpuregs_rs1[ld_rs1_hart];
						//dbg_rs1val_valid <= 1;
						hart_ready[ld_rs1_hart] = cpu_state_fetch;
						ld_rs1_hart = no_hart;
					end
					ENABLE_IRQ && ENABLE_IRQ_TIMER && instr_timer[ld_rs1_hart]: begin
						latched_store[ld_rs1_hart] <= 1;
						reg_out[ld_rs1_hart] <= timer;
						`debug($display("LD_RS1: %2d 0x%08x", decoded_rs1[ld_rs1_hart], cpuregs_rs1[ld_rs1_hart]);)
						timer <= cpuregs_rs1[ld_rs1_hart];
						//TODO: DEBUG
						//dbg_rs1val <= cpuregs_rs1[ld_rs1_hart];
						//dbg_rs1val_valid <= 1;
						hart_ready[ld_rs1_hart] = cpu_state_fetch;
						ld_rs1_hart = no_hart;
					end
					is_lb_lh_lw_lbu_lhu[ld_rs1_hart] && !instr_trap[ld_rs1_hart]: begin
						`debug($display("LD_RS1: %2d 0x%08x", decoded_rs1[ld_rs1_hart], cpuregs_rs1[ld_rs1_hart]);)
						reg_op1[ld_rs1_hart] <= cpuregs_rs1[ld_rs1_hart];
						//TODO: DEBUG
						//dbg_rs1val <= cpuregs_rs1[ld_rs1_hart];
						//dbg_rs1val_valid <= 1;
						hart_ready[ld_rs1_hart] = cpu_state_ldmem;
						ld_rs1_hart = no_hart;
					end
					is_slli_srli_srai[ld_rs1_hart] && !BARREL_SHIFTER: begin
						`debug($display("LD_RS1: %2d 0x%08x", decoded_rs1[ld_rs1_hart], cpuregs_rs1[ld_rs1_hart]);)
						reg_op1[ld_rs1_hart] <= cpuregs_rs1[ld_rs1_hart];
						//TODO: DEBUG
						//dbg_rs1val <= cpuregs_rs1[ld_rs1_hart];
						//dbg_rs1val_valid <= 1;
						reg_sh[ld_rs1_hart] <= decoded_rs2[ld_rs1_hart];
						hart_ready[ld_rs1_hart] = cpu_state_shift;
						ld_rs1_hart = no_hart;
					end
					is_jalr_addi_slti_sltiu_xori_ori_andi[ld_rs1_hart], is_slli_srli_srai[ld_rs1_hart] && BARREL_SHIFTER: begin
						`debug($display("LD_RS1: %2d 0x%08x", decoded_rs1[ld_rs1_hart], cpuregs_rs1[ld_rs1_hart]);)
						reg_op1[ld_rs1_hart] <= cpuregs_rs1[ld_rs1_hart];
						//TODO: DEBUG
						//dbg_rs1val <= cpuregs_rs1[ld_rs1_hart];
						//dbg_rs1val_valid <= 1;
						reg_op2[ld_rs1_hart] <= is_slli_srli_srai[ld_rs1_hart] && BARREL_SHIFTER ? decoded_rs2[ld_rs1_hart] : decoded_imm[ld_rs1_hart];
						if (TWO_CYCLE_ALU)
							alu_wait <= 1;
						hart_ready[ld_rs1_hart] = cpu_state_exec;
						ld_rs1_hart = no_hart;
					end
					default: begin
						`debug($display("LD_RS1: %2d 0x%08x", decoded_rs1[ld_rs1_hart], cpuregs_rs1[ld_rs1_hart]);)
						reg_op1[ld_rs1_hart] <= cpuregs_rs1[ld_rs1_hart];
						//TODO: DEBUG
						//dbg_rs1val <= cpuregs_rs1[ld_rs1_hart];
						//dbg_rs1val_valid <= 1;
						if (ENABLE_REGS_DUALPORT) begin
							`debug($display("LD_RS2: %2d 0x%08x", decoded_rs2[ld_rs1_hart], cpuregs_rs2[ld_rs1_hart]);)
							reg_sh[ld_rs1_hart] <= cpuregs_rs2[ld_rs1_hart];
							reg_op2[ld_rs1_hart] <= cpuregs_rs2[ld_rs1_hart];
							//TODO: DEBUG
							//dbg_rs2val <= cpuregs_rs2[ld_rs1_hart];
							//dbg_rs2val_valid <= 1;
							(* parallel_case *)
							case (1'b1)
								is_sb_sh_sw[ld_rs1_hart]: begin
									hart_ready[ld_rs1_hart] = cpu_state_stmem;
									ld_rs1_hart = no_hart;
								end
								is_sll_srl_sra[ld_rs1_hart] && !BARREL_SHIFTER: begin
									hart_ready[ld_rs1_hart] = cpu_state_shift;
									ld_rs1_hart = no_hart;
								end
								default: begin
									if (TWO_CYCLE_ALU || (TWO_CYCLE_COMPARE && is_beq_bne_blt_bge_bltu_bgeu[ld_rs1_hart])) begin
										alu_wait_2 <= TWO_CYCLE_ALU && (TWO_CYCLE_COMPARE && is_beq_bne_blt_bge_bltu_bgeu[ld_rs1_hart]);
										alu_wait <= 1;
									end else
									hart_ready[ld_rs1_hart] = cpu_state_exec;
									ld_rs1_hart = no_hart;
								end
							endcase
						end else begin
							// TODO: ld_rs2
							hart_ready[ld_rs1_hart] = cpu_state_ld_rs2;
							ld_rs1_hart = no_hart;
						end
					end
				endcase
			end

			// TODO: ld_rs2
			if (ld_rs2_hart != no_hart) begin
				`debug($display("LD_RS2: %2d 0x%08x", decoded_rs2[ld_rs2_hart], cpuregs_rs2[ld_rs2_hart]);)
				reg_sh[ld_rs2_hart] <= cpuregs_rs2[ld_rs2_hart];
				reg_op2[ld_rs2_hart] <= cpuregs_rs2[ld_rs2_hart];
				//TODO: DEBUG
				//dbg_rs2val <= cpuregs_rs2[ld_rs2_hart];
				//dbg_rs2val_valid <= 1;

				(* parallel_case *)
				case (1'b1)
					is_sb_sh_sw[ld_rs2_hart]: begin
						hart_ready[ld_rs2_hart] = cpu_state_stmem;
						ld_rs2_hart = no_hart;
						instr_do_rinst <= 1;
					end
					is_sll_srl_sra[ld_rs2_hart] && !BARREL_SHIFTER: begin
						hart_ready[ld_rs2_hart] = cpu_state_shift;
						ld_rs2_hart = no_hart;
					end
					default: begin
						if (TWO_CYCLE_ALU || (TWO_CYCLE_COMPARE && is_beq_bne_blt_bge_bltu_bgeu[ld_rs2_hart])) begin
							alu_wait_2 <= TWO_CYCLE_ALU && (TWO_CYCLE_COMPARE && is_beq_bne_blt_bge_bltu_bgeu[ld_rs2_hart]);
							alu_wait <= 1;
						end
						hart_ready[ld_rs2_hart] = cpu_state_exec;
						ld_rs2_hart = no_hart;
					end
				endcase
			end

			if (exec_hart != no_hart) begin
				reg_out[exec_hart] <= reg_pc[exec_hart] + decoded_imm[exec_hart];
				if ((TWO_CYCLE_ALU || TWO_CYCLE_COMPARE) && (alu_wait || alu_wait_2)) begin
					alu_wait <= alu_wait_2;
				end else
				if (is_beq_bne_blt_bge_bltu_bgeu[exec_hart]) begin
					latched_rd[exec_hart] <= 0;
					latched_store[exec_hart] <= TWO_CYCLE_COMPARE ? alu_out_0_q[exec_hart] : alu_out_0[exec_hart];
					latched_branch[exec_hart] <= TWO_CYCLE_COMPARE ? alu_out_0_q[exec_hart] : alu_out_0[exec_hart];
					hart_ready[exec_hart] = cpu_state_fetch;
					exec_hart = no_hart;
				end else if (is_csrrs[exec_hart]) begin
					reg_out[exec_hart] <= exec_hart;
					latched_store[exec_hart] <= 1;
					hart_ready[exec_hart] = cpu_state_fetch;
					exec_hart = no_hart;
				end else begin
					latched_branch[exec_hart] <= instr_jalr[exec_hart];
					latched_store[exec_hart] <= 1;
					latched_stalu[exec_hart] <= 1;
					hart_ready[exec_hart] = cpu_state_fetch;
					exec_hart = no_hart;
				end
			end

			if (shift_hart != no_hart) begin
				latched_store[shift_hart] <= 1;
				if (reg_sh[shift_hart] == 0) begin
					reg_out[shift_hart] <= reg_op1[shift_hart];
					hart_ready[shift_hart] = cpu_state_fetch;
					shift_hart = no_hart;
				end else if (TWO_STAGE_SHIFT && reg_sh[shift_hart] >= 4) begin
					(* parallel_case, full_case *)
					case (1'b1)
						instr_slli[shift_hart] || instr_sll[shift_hart]: reg_op1[shift_hart] <= reg_op1[shift_hart] << 4;
						instr_srli[shift_hart] || instr_srl[shift_hart]: reg_op1[shift_hart] <= reg_op1[shift_hart] >> 4;
						instr_srai[shift_hart] || instr_sra[shift_hart]: reg_op1[shift_hart] <= $signed(reg_op1[shift_hart]) >>> 4;
					endcase
					reg_sh[shift_hart] <= reg_sh[shift_hart] - 4;
				end else begin
					(* parallel_case, full_case *)
					case (1'b1)
						instr_slli[shift_hart] || instr_sll[shift_hart]: reg_op1[shift_hart] <= reg_op1[shift_hart] << 1;
						instr_srli[shift_hart] || instr_srl[shift_hart]: reg_op1[shift_hart] <= reg_op1[shift_hart] >> 1;
						instr_srai[shift_hart] || instr_sra[shift_hart]: reg_op1[shift_hart] <= $signed(reg_op1[shift_hart]) >>> 1;
					endcase
					reg_sh[shift_hart] <= reg_sh[shift_hart] - 1;
				end
			end

			if (stmem_hart != no_hart) begin
				if (ENABLE_TRACE)
					reg_out[stmem_hart] <= reg_op2[stmem_hart];
				if (!mem_do_wdata) begin
					(* parallel_case, full_case *)
					case (1'b1)
						instr_sb[stmem_hart]: mem_wordsize <= 2;
						instr_sh[stmem_hart]: mem_wordsize <= 1;
						instr_sw[stmem_hart]: mem_wordsize <= 0;
					endcase
					if (ENABLE_TRACE) begin
						trace_valid <= 1;
						trace_data <= (irq_active ? TRACE_IRQ : 0) | TRACE_ADDR | ((reg_op1[stmem_hart] + decoded_imm[stmem_hart]) & 32'hffffffff);
					end
					reg_op1[stmem_hart] <= reg_op1[stmem_hart] + decoded_imm[stmem_hart];
					set_mem_do_wdata = 1;
				end
				if (mem_done) begin
					hart_ready[stmem_hart] = cpu_state_fetch;
					stmem_hart = no_hart;
				end
			end

			if (ldmem_hart != no_hart) begin
				latched_store[ldmem_hart] <= 1;
				if (!mem_do_rdata) begin
					(* parallel_case, full_case *)
					case (1'b1)
						instr_lb[ldmem_hart] || instr_lbu[ldmem_hart]: mem_wordsize <= 2;
						instr_lh[ldmem_hart] || instr_lhu[ldmem_hart]: mem_wordsize <= 1;
						instr_lw[ldmem_hart]: mem_wordsize <= 0;
					endcase
					latched_is_lu[ldmem_hart] <= is_lbu_lhu_lw[ldmem_hart];
					latched_is_lh[ldmem_hart] <= instr_lh[ldmem_hart];
					latched_is_lb[ldmem_hart] <= instr_lb[ldmem_hart];
					if (ENABLE_TRACE) begin
						trace_valid <= 1;
						trace_data <= (irq_active ? TRACE_IRQ : 0) | TRACE_ADDR | ((reg_op1[ldmem_hart] + decoded_imm[ldmem_hart]) & 32'hffffffff);
					end
					reg_op1[ldmem_hart] <= reg_op1[ldmem_hart] + decoded_imm[ldmem_hart];
					set_mem_do_rdata = 1;
				end
				(* parallel_case, full_case *)
				case (1'b1)
					latched_is_lu[ldmem_hart]: reg_out[ldmem_hart] <= mem_rdata_word;
					latched_is_lh[ldmem_hart]: reg_out[ldmem_hart] <= $signed(mem_rdata_word[15:0]);
					latched_is_lb[ldmem_hart]: reg_out[ldmem_hart] <= $signed(mem_rdata_word[7:0]);
				endcase
				if (mem_done) begin
					hart_ready[ldmem_hart] = cpu_state_fetch;
					ldmem_hart = no_hart;
				end
			end
		end

		/* THREAD SCHEDULING */

		/* This part is making the scheduling fair, by actually alternating the thread allowed to fetch using the hart_counter */
		if (hart_ready[hart_counter] == cpu_state_fetch && fetch_hart == no_hart) begin
			hart_ready[hart_counter] = cpu_state_busy;
			fetch_hart = hart_counter;
			hart_counter = hart_counter + 1;
			instr_do_rinst <= !do_waitirq;
		end

		/* blocking assignments are intentional */
		for (k = 0; k < THREADS; k = k + 1) begin
			case (hart_ready[k])
				cpu_state_trap: begin
					hart_ready[k] = cpu_state_busy;
					trap_hart = k;
				end
				cpu_state_ld_rs1: begin
					// TODO: ld_rs2_hart
					if (ld_rs1_hart == no_hart/* && ld_rs2_hart == no_hart*/) begin
						hart_ready[k] = cpu_state_busy;
						ld_rs1_hart = k;
					end
				end
				// TODO: ld_rs2_hart
				/*cpu_state_ld_rs2: begin
					if (ld_rs2_hart && ld_rs2_hart == no_hart) begin
						hart_ready[k] = cpu_state_busy;
						ld_rs2_hart = k;
					end
				end*/
				cpu_state_exec: begin
					if (exec_hart == no_hart && shift_hart == no_hart && stmem_hart == no_hart && ldmem_hart == no_hart) begin
						hart_ready[k] = cpu_state_busy;
						exec_hart = k;
					end
				end
				cpu_state_shift: begin
					if (exec_hart == no_hart && shift_hart == no_hart && stmem_hart == no_hart && ldmem_hart == no_hart) begin
						hart_ready[k] = cpu_state_busy;
						shift_hart = k;
					end
				end
				cpu_state_stmem: begin
					if (exec_hart == no_hart && shift_hart == no_hart && stmem_hart == no_hart && ldmem_hart == no_hart) begin
						hart_ready[k] = cpu_state_busy;
						stmem_hart = k;
					end
				end
				cpu_state_ldmem: begin
					if (exec_hart == no_hart && shift_hart == no_hart && stmem_hart == no_hart && ldmem_hart == no_hart) begin
						hart_ready[k] = cpu_state_busy;
						ldmem_hart = k;
					end
				end
			endcase
		end

		if (hart_counter >= THREADS) begin
			hart_counter = 0;
		end

		// TODO: IRQ
		if (ENABLE_IRQ) begin
			next_irq_pending = next_irq_pending | irq;
			if(ENABLE_IRQ_TIMER && timer)
				if (timer - 1 == 0)
					next_irq_pending[irq_timer] = 1;
		end


		// TODO: IRQ CATCH_MISALIGN
		/*
		if (CATCH_MISALIGN && resetn && (mem_do_rdata || mem_do_wdata)) begin
			for (k = 0; k < THREADS; k = k + 1) begin
				if (mem_wordsize == 0 && reg_op1[k][1:0] != 0) begin
					`debug($display("MISALIGNED WORD: 0x%08x", reg_op1[k]);)
					if (ENABLE_IRQ && !irq_mask[irq_buserror] && !irq_active) begin
						next_irq_pending[irq_buserror] = 1;
					end else begin
						hart_ready[k] = cpu_state_trap;
					end
				end
				if (mem_wordsize == 1 && reg_op1[k][0] != 0) begin
					`debug($display("MISALIGNED HALFWORD: 0x%08x", reg_op1[k]);)
					if (ENABLE_IRQ && !irq_mask[irq_buserror] && !irq_active) begin
						next_irq_pending[irq_buserror] = 1;
					end else begin
						hart_ready[k] = cpu_state_trap;
					end
				end
			end
		end

		for (k = 0; k < THREADS; k = k + 1) begin
			if (CATCH_MISALIGN && resetn && instr_do_rinst && (COMPRESSED_ISA ? reg_pc[k][0] : |reg_pc[k][1:0])) begin
				`debug($display("MISALIGNED INSTRUCTION: 0x%08x", reg_pc[k]);)
				if (ENABLE_IRQ && !irq_mask[irq_buserror] && !irq_active) begin
					next_irq_pending[irq_buserror] = 1;
				end else begin
					hart_ready[k] = cpu_state_trap;
				end
			end
		end
		*/

		if (!CATCH_ILLINSN && decoder_trigger_q && instr_ecall_ebreak) begin
			for (k = 0; k < THREADS; k = k + 1) begin
				hart_ready[k] = cpu_state_trap;
			end
		end

		if (csr_unknown) begin
			for (k = 0; k < THREADS; k = k + 1) begin
				hart_ready[k] = cpu_state_trap;
			end
		end

		if (!resetn || mem_done) begin
			mem_do_rdata <= 0;
			mem_do_wdata <= 0;
		end

		if (!resetn || instr_done) begin
			instr_do_rinst <= 0;
		end

		if (set_instr_do_rinst)
			instr_do_rinst <= 1;
		if (set_mem_do_rdata)
			mem_do_rdata <= 1;
		if (set_mem_do_wdata)
			mem_do_wdata <= 1;

		// TODO: IRQ
		irq_pending <= next_irq_pending & ~MASKED_IRQ;

		if (!CATCH_MISALIGN) begin
			for (k = 0; k < THREADS; k = k + 1) begin
				reg_pc[k][1:0] <= 0;
				reg_next_pc[k][1:0] <= 0;
			end
		end

		current_pc = 'bx;
	end

`ifdef RISCV_FORMAL
	reg dbg_irq_call;
	reg dbg_irq_enter;
	reg [31:0] dbg_irq_ret;
	always @(posedge clk) begin
		rvfi_valid <= resetn && (launch_next_insn || trap) && dbg_valid_insn;
		rvfi_order <= resetn ? rvfi_order + rvfi_valid : 0;

		rvfi_insn <= dbg_insn_opcode;
		//rvfi_rs1_addr <= dbg_rs1val_valid ? dbg_insn_rs1 : 0;
		//rvfi_rs2_addr <= dbg_rs2val_valid ? dbg_insn_rs2 : 0;
		rvfi_pc_rdata <= dbg_insn_addr;
		//rvfi_rs1_rdata <= dbg_rs1val_valid ? dbg_rs1val : 0;
		//rvfi_rs2_rdata <= dbg_rs2val_valid ? dbg_rs2val : 0;
		rvfi_trap <= trap;
		rvfi_halt <= trap;
		rvfi_intr <= dbg_irq_enter;
		rvfi_mode <= 3;
		rvfi_ixl <= 1;

		if (!resetn) begin
			dbg_irq_call <= 0;
			dbg_irq_enter <= 0;
		end else
		if (rvfi_valid) begin
			dbg_irq_call <= 0;
			dbg_irq_enter <= dbg_irq_call;
		end else
		if (irq_state == 1) begin
			dbg_irq_call <= 1;
			dbg_irq_ret <= next_pc;
		end

		if (!resetn) begin
			rvfi_rd_addr <= 0;
			rvfi_rd_wdata <= 0;
		end else
		if (cpuregs_write && !irq_state) begin
`ifdef PICORV32_TESTBUG_003
			rvfi_rd_addr <= latched_rd ^ 1;
`else
			rvfi_rd_addr <= latched_rd;
`endif
`ifdef PICORV32_TESTBUG_004
			rvfi_rd_wdata <= latched_rd ? cpuregs_wrdata[active_hart] ^ 1 : 0;
`else
			rvfi_rd_wdata <= latched_rd ? cpuregs_wrdata[active_hart] : 0;
`endif
		end else
		if (rvfi_valid) begin
			rvfi_rd_addr <= 0;
			rvfi_rd_wdata <= 0;
		end

		casez (dbg_insn_opcode)
			32'b 0000000_?????_000??_???_?????_0001011: begin // getq
				rvfi_rs1_addr <= 0;
				rvfi_rs1_rdata <= 0;
			end
			32'b 0000001_?????_?????_???_000??_0001011: begin // setq
				rvfi_rd_addr <= 0;
				rvfi_rd_wdata <= 0;
			end
			32'b 0000010_?????_00000_???_00000_0001011: begin // retirq
				rvfi_rs1_addr <= 0;
				rvfi_rs1_rdata <= 0;
			end
		endcase

		if (!dbg_irq_call) begin
			if (dbg_mem_instr) begin
				rvfi_mem_addr <= 0;
				rvfi_mem_rmask <= 0;
				rvfi_mem_wmask <= 0;
				rvfi_mem_rdata <= 0;
				rvfi_mem_wdata <= 0;
			end else
			if (dbg_mem_valid && dbg_mem_ready) begin
				rvfi_mem_addr <= dbg_mem_addr;
				rvfi_mem_rmask <= dbg_mem_wstrb ? 0 : ~0;
				rvfi_mem_wmask <= dbg_mem_wstrb;
				rvfi_mem_rdata <= dbg_mem_rdata;
				rvfi_mem_wdata <= dbg_mem_wdata;
			end
		end
	end

	always @* begin
`ifdef PICORV32_TESTBUG_005
		rvfi_pc_wdata = (dbg_irq_call ? dbg_irq_ret : dbg_insn_addr) ^ 4;
`else
		rvfi_pc_wdata = dbg_irq_call ? dbg_irq_ret : dbg_insn_addr;
`endif

		rvfi_csr_mcycle_rmask = 0;
		rvfi_csr_mcycle_wmask = 0;
		rvfi_csr_mcycle_rdata = 0;
		rvfi_csr_mcycle_wdata = 0;

		rvfi_csr_minstret_rmask = 0;
		rvfi_csr_minstret_wmask = 0;
		rvfi_csr_minstret_rdata = 0;
		rvfi_csr_minstret_wdata = 0;

		if (rvfi_valid && rvfi_insn[6:0] == 7'b 1110011 && rvfi_insn[13:12] == 3'b010) begin
			if (rvfi_insn[31:20] == 12'h C00) begin
				rvfi_csr_mcycle_rmask = 64'h 0000_0000_FFFF_FFFF;
				rvfi_csr_mcycle_rdata = {32'h 0000_0000, rvfi_rd_wdata};
			end
			if (rvfi_insn[31:20] == 12'h C80) begin
				rvfi_csr_mcycle_rmask = 64'h FFFF_FFFF_0000_0000;
				rvfi_csr_mcycle_rdata = {rvfi_rd_wdata, 32'h 0000_0000};
			end
			if (rvfi_insn[31:20] == 12'h C02) begin
				rvfi_csr_minstret_rmask = 64'h 0000_0000_FFFF_FFFF;
				rvfi_csr_minstret_rdata = {32'h 0000_0000, rvfi_rd_wdata};
			end
			if (rvfi_insn[31:20] == 12'h C82) begin
				rvfi_csr_minstret_rmask = 64'h FFFF_FFFF_0000_0000;
				rvfi_csr_minstret_rdata = {rvfi_rd_wdata, 32'h 0000_0000};
			end
		end
	end
`endif

	// Formal Verification | TODO
`ifdef FORMAL
	reg [3:0] last_mem_nowait;
	always @(posedge clk)
		last_mem_nowait <= {last_mem_nowait, mem_ready || !mem_valid};

	// stall the memory interface for max 4 cycles
	restrict property (|last_mem_nowait || mem_ready || !mem_valid);

	// resetn low in first cycle, after that resetn high
	restrict property (resetn != $initstate);

	// this just makes it much easier to read traces. uncomment as needed.
	// assume property (mem_valid || !mem_ready);

	reg ok;
	always @* begin
		if (resetn) begin
			// instruction fetches are read-only
			if (mem_valid && mem_instr)
				assert (mem_wstrb == 0);

			// cpu_state must be valid
			ok = 0;
			if (cpu_state == cpu_state_trap)   ok = 1;
			if (cpu_state == cpu_state_fetch)  ok = 1;
			if (cpu_state == cpu_state_ld_rs1) ok = 1;
			if (cpu_state == cpu_state_ld_rs2) ok = !ENABLE_REGS_DUALPORT;
			if (cpu_state == cpu_state_exec)   ok = 1;
			if (cpu_state == cpu_state_shift)  ok = 1;
			if (cpu_state == cpu_state_stmem)  ok = 1;
			if (cpu_state == cpu_state_ldmem)  ok = 1;
			assert (ok);
		end
	end

	reg last_mem_la_read = 0;
	reg last_mem_la_write = 0;
	reg [31:0] last_mem_la_addr;
	reg [31:0] last_mem_la_wdata;
	reg [3:0] last_mem_la_wstrb = 0;

	always @(posedge clk) begin
		last_mem_la_read <= mem_la_read;
		last_mem_la_write <= mem_la_write;
		last_mem_la_addr <= mem_la_addr;
		last_mem_la_wdata <= mem_la_wdata;
		last_mem_la_wstrb <= mem_la_wstrb;

		if (last_mem_la_read) begin
			assert(mem_valid);
			assert(mem_addr == last_mem_la_addr);
			assert(mem_wstrb == 0);
		end
		if (last_mem_la_write) begin
			assert(mem_valid);
			assert(mem_addr == last_mem_la_addr);
			assert(mem_wdata == last_mem_la_wdata);
			assert(mem_wstrb == last_mem_la_wstrb);
		end
		if (mem_la_read || mem_la_write) begin
			assert(!mem_valid || mem_ready);
		end
	end
`endif
endmodule

// This is a simple example implementation of PICORV32_REGS.
// Use the PICORV32_REGS mechanism if you want to use custom
// memory resources to implement the processor register file.
// Note that your implementation must match the requirements of
// the PicoRV32 configuration. (e.g. QREGS, etc)
module picorv32_regs (
	input clk, wen,
	input [5:0] waddr,
	input [5:0] raddr1,
	input [5:0] raddr2,
	input [31:0] wdata,
	output [31:0] rdata1,
	output [31:0] rdata2
);
	reg [31:0] regs [0:30];

	always @(posedge clk)
		if (wen) regs[waddr[4:0]] <= wdata;

	assign rdata1 = regs[raddr1[4:0]];
	assign rdata2 = regs[raddr2[4:0]];
endmodule
