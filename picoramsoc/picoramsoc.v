/*
 *  PicoRAMSoC - A simple example SoC using PicoRV32-imt
 *
 *  Copyright (C) 2017  Clifford Wolf <clifford@clifford.at>
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

`timescale 1ns / 1ps

`ifndef PICORAMSOC_MEM
`define PICORAMSOC_MEM picoramsoc_mem
`endif

// this macro can be used to check if the verilog files in your
// design are read in the correct order.
`define PICOSOC_V


module picoramsoc(
    input clk,
    input resetn,

    output        iomem_valid,
    input         iomem_ready,
    output [ 3:0] iomem_wstrb,
    output [31:0] iomem_addr,
    output [31:0] iomem_wdata,
    input  [31:0] iomem_rdata,

    input  irq_5,
    input  irq_6,
    input  irq_7,

    output ser_tx,
    input  ser_rx
);
	parameter [0:0] BARREL_SHIFTER = 1;
    parameter [0:0] ENABLE_COUNTERS = 1;
    parameter [0:0] ENABLE_IRQ_QREGS = 0;

	parameter integer MEM_WORDS = 4096;
	parameter [31:0] STACKADDR = (4*MEM_WORDS);       // end of memory
	parameter [31:0] PROGADDR_RESET = 32'h 0000_0100; // address 0x100 on internal RAM
	parameter [31:0] PROGADDR_IRQ = 32'h 0000_0000;

	reg [31:0] irq;
	wire irq_stall = 0;
	wire irq_uart = 0;

	always @* begin
	    irq = 0;
	    irq[3] = irq_stall;
	    irq[4] = irq_uart;
	    irq[5] = irq_5;
	    irq[6] = irq_6;
	    irq[7] = irq_7;
	end

	wire mem_valid;
	wire mem_instr;
	wire mem_ready;
	wire [31:0] mem_addr;
	wire [31:0] mem_wdata;
	wire [3:0] mem_wstrb;
	wire [31:0] mem_rdata;

	wire instr_valid;
	wire instr_ready;
	wire [31:0] instr_addr;
	wire [31:0] instr_rdata;

	reg ram_ready;
	wire [31:0] ram_rdata;

	reg instr_ram_ready;

	assign iomem_valid = mem_valid && (mem_addr[31:24] > 8'h 01);
	assign iomem_wstrb = mem_wstrb;
	assign iomem_addr = mem_addr;
	assign iomem_wdata = mem_wdata;

	wire        simpleuart_reg_div_sel = mem_valid && (mem_addr == 32'h 0200_0004);
	wire [31:0] simpleuart_reg_div_do;

	wire        simpleuart_reg_dat_sel = mem_valid && (mem_addr == 32'h 0200_0008);
	wire [31:0] simpleuart_reg_dat_do;
	wire        simpleuart_reg_dat_wait;

	assign instr_ready = instr_ram_ready;

	assign mem_ready = (iomem_valid && iomem_ready) || ram_ready ||
	        simpleuart_reg_div_sel || (simpleuart_reg_dat_sel && !simpleuart_reg_dat_wait);

	assign mem_rdata = (iomem_valid && iomem_ready) ? iomem_rdata : ram_ready ? ram_rdata :
	        simpleuart_reg_div_sel ? simpleuart_reg_div_do :
	        simpleuart_reg_dat_sel ? simpleuart_reg_dat_do : 32'h 0000_0000;

	picorv32-imt #(
	    .STACKADDR(STACKADDR),
	    .PROGADDR_RESET(PROGADDR_RESET),
	    .PROGADDR_IRQ(PROGADDR_IRQ),
	    .BARREL_SHIFTER(BARREL_SHIFTER),
	    .ENABLE_COUNTERS(ENABLE_COUNTERS),
	    .ENABLE_IRQ(1),
	    .ENABLE_IRQ_QREGS(ENABLE_IRQ_QREGS)
	) cpu (
	    .clk         (clk),
	    .resetn      (resetn),
	    .mem_valid   (mem_valid),
	    .instr_valid (instr_valid),
	    .mem_instr   (mem_instr),
	    .mem_ready   (mem_ready),
	    .instr_ready (instr_ready),
	    .mem_addr    (mem_addr),
	    .instr_addr  (instr_addr),
	    .mem_wdata   (mem_wdata),
	    .mem_wstrb   (mem_wstrb),
	    .mem_rdata   (mem_rdata),
	    .instr_rdata (instr_rdata),
	    .irq         (irq)
	);

	simpleuart simpleuart (
	    .clk         (clk         ),
	    .resetn      (resetn      ),

	    .ser_tx      (ser_tx      ),
	    .ser_rx      (ser_rx      ),

	    .reg_div_we  (simpleuart_reg_div_sel ? mem_wstrb : 4'b 0000),
	    .reg_div_di  (mem_wdata),
	    .reg_div_do  (simpleuart_reg_div_do),

	    .reg_dat_we  (simpleuart_reg_dat_sel ? mem_wstrb[0] : 1'b 0),
	    .reg_dat_re  (simpleuart_reg_dat_sel && !mem_wstrb),
	    .reg_dat_di  (mem_wdata),
	    .reg_dat_do  (simpleuart_reg_dat_do),
	    .reg_dat_wait(simpleuart_reg_dat_wait)
	);

	always @(posedge clk) begin
	    ram_ready <= mem_valid && !mem_ready && mem_addr < 4*MEM_WORDS;
	    instr_ram_ready <= instr_valid && !instr_ready && instr_addr < 4*MEM_WORDS;
	end

	`PICORAMSOC_MEM #(
	    .WORDS(MEM_WORDS)
	) memory (
	    .clk(clk),
	    .wen((mem_valid && !mem_ready && mem_addr < 4*MEM_WORDS) ? mem_wstrb : 4'b0),
	    .addr1(mem_addr[23:2]),
	    .addr2(instr_addr[23:2]),
	    .wdata(mem_wdata),
	    .rdata1(ram_rdata),
	    .rdata2(instr_rdata)
	);
endmodule

module picoramsoc_mem #(
	parameter integer WORDS = 4096
	) (
	input clk,
	input [3:0] wen,
	input [21:0] addr1,
	input [21:0] addr2,
	input [31:0] wdata,
	output reg [31:0] rdata1,
	output reg [31:0] rdata2
	);
	reg [31:0] mem [0:WORDS-1];

	initial
		$readmemh("firmware.hex", mem);

	always @(posedge clk) begin
	    rdata1 <= mem[addr1];
	    if (wen[0]) mem[addr1][ 7: 0] <= wdata[ 7: 0];
	    if (wen[1]) mem[addr1][15: 8] <= wdata[15: 8];
	    if (wen[2]) mem[addr1][23:16] <= wdata[23:16];
	    if (wen[3]) mem[addr1][31:24] <= wdata[31:24];
		rdata2 <= mem[addr2];
	end
endmodule
