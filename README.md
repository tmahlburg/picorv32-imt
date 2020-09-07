
PicoRV32-imt - A Size-Optimized RISC-V CPU extended with IMT
============================================================

PicoRV32-imt is a CPU core based on Claire Wolf's [PicoRV32](https://github.com/cliffordwolf/picorv32). To simplify the implementation of interleaved multithreading in this core, some features of the source have been dropped (for now). This core just implements the [RISC-V RV32I Instruction Set](http://riscv.org/). The interrupt controller of the original is still there but yet to be tested. This CPU has not yet successfully run on real hardware, but it works well in simulation.

Tools (gcc, binutils, etc..) can be obtained via the [RISC-V Website](https://riscv.org/software-status/).
The examples bundled with PicoRV32-imt expect various RV32 toolchains to be installed in `/opt/riscv32i`. See
the [build instructions below](#building-a-pure-rv32i-toolchain) for details.

PicoRV32-imt is free and open hardware licensed under the [ISC license](http://en.wikipedia.org/wiki/ISC_license)
(a license that is similar in terms to the MIT license or the 2-clause BSD license).

#### Table of Contents

- [Features and Typical Applications](#features-and-typical-applications)
- [Files in this Repository](#files-in-this-repository)
- [Verilog Module Parameters](#verilog-module-parameters)
- [Cycles per Instruction Performance](#cycles-per-instruction-performance)
- [PicoRV32 Native Memory Interface](#picorv32-native-memory-interface)
- [Custom Instructions for IRQ Handling](#custom-instructions-for-irq-handling)
- [Building a pure RV32I Toolchain](#building-a-pure-rv32i-toolchain)
- [Linking binaries with newlib for PicoRV32](#linking-binaries-with-newlib-for-picorv32)

Features and Typical Applications
---------------------------------

- Small (750-2000 LUTs in 7-Series Xilinx Architecture)
- High f<sub>max</sub> (250-450 MHz on 7-Series Xilinx FPGAs)
- Selectable native memory interface or AXI4-Lite master
- Optional IRQ support (using a simple custom ISA)
- Optional Co-Processor Interface

This CPU is meant to be used as auxiliary processor in FPGA designs and ASICs. Due
to its high f<sub>max</sub> it can be integrated in most existing designs without crossing
clock domains. When operated on a lower frequency, it will have a lot of timing
slack and thus can be added to a design without compromising timing closure.

For even smaller size it is possible disable support for registers `x16`..`x31` as
well as `RDCYCLE[H]`, `RDTIME[H]`, and `RDINSTRET[H]` instructions, turning the
processor into an RV32E core.

Furthermore it is possible to choose between a dual-port and a single-port
register file implementation. The former provides better performance while
the latter results in a smaller core.

*Note: In architectures that implement the register file in dedicated memory
resources, such as many FPGAs, disabling the 16 upper registers and/or
disabling the dual-port register file may not further reduce the core size.*

The core exists in three variations: `picorv32`, `picorv32_axi` and `picorv32_wb`.
The first provides a simple native memory interface, that is easy to use in simple
environments. `picorv32_axi` provides an AXI-4 Lite Master interface that can
easily be integrated with existing systems that are already using the AXI
standard. `picorv32_wb` provides a Wishbone master interface.

A separate core `picorv32_axi_adapter` is provided to bridge between the native
memory interface and AXI4. This core can be used to create custom cores that
include one or more PicoRV32 cores together with local RAM, ROM, and
memory-mapped peripherals, communicating with each other using the native
interface, and communicating with the outside world via AXI4.

The optional IRQ feature can be used to react to events from the outside, implement
fault handlers, or catch instructions from a larger ISA and emulate them in
software.

The optional Pico Co-Processor Interface (PCPI) can be used to implement
non-branching instructions in an external coprocessor. Implementations
of PCPI cores that implement the M Standard Extension instructions
`MUL[H[SU|U]]` and `DIV[U]/REM[U]` are included in this package.


Files in this Repository
------------------------

#### README.md

You are reading it right now.

#### picorv32.v

This Verilog file contains the following Verilog modules:

| Module                   | Description                                                           |
| ------------------------ | --------------------------------------------------------------------- |
| `picorv32`               | The PicoRV32-imt CPU                                                  |

Simply copy this file into your project.

#### Makefile and testbenches

A basic test environment. Run `make test` to run the standard test bench (`testbench.v`)
in the standard configurations. There are other test benches and configurations. See
the `test_*` make target in the Makefile for details.

Run `make test_ez` to run `testbench_ez.v`, a very simple test bench that does
not require an external firmware .hex file. This can be useful in environments
where the RISC-V compiler toolchain is not available.

*Note: The test bench is using Icarus Verilog. However, Icarus Verilog 0.9.7
(the latest release at the time of writing) has a few bugs that prevent the
test bench from running. Upgrade to the latest github master of Icarus Verilog
to run the test bench.*

#### firmware/

A simple test firmware. This runs the basic tests from `tests/`, some C code, tests IRQ
handling and the multiply PCPI core.

All the code in `firmware/` is in the public domain. Simply copy whatever you can use.

#### tests/

Simple instruction-level tests from [riscv-tests](https://github.com/riscv/riscv-tests).

#### dhrystone/

Another simple test firmware that runs the Dhrystone benchmark.

#### picosoc/

A simple example SoC using PicoRV32-imt that can execute code directly from a
memory mapped SPI flash.

### picoramsoc/

A simple example SoC using PicoRV32-imt that can execute code from RAM via a memory file.

#### scripts/

Various scripts and examples for different (synthesis) tools and hardware architectures.


Verilog Module Parameters
-------------------------

The following Verilog module parameters can be used to configure the PicoRV32
core.

#### ENABLE_COUNTERS (default = 1)

This parameter enables support for the `RDCYCLE[H]`, `RDTIME[H]`, and
`RDINSTRET[H]` instructions. This instructions will cause a hardware
trap (like any other unsupported instruction) if `ENABLE_COUNTERS` is set to zero.

*Note: Strictly speaking the `RDCYCLE[H]`, `RDTIME[H]`, and `RDINSTRET[H]`
instructions are not optional for an RV32I core. But chances are they are not
going to be missed after the application code has been debugged and profiled.
This instructions are optional for an RV32E core.*

#### ENABLE_COUNTERS64 (default = 1)

This parameter enables support for the `RDCYCLEH`, `RDTIMEH`, and `RDINSTRETH`
instructions. If this parameter is set to 0, and `ENABLE_COUNTERS` is set to 1,
then only the `RDCYCLE`, `RDTIME`, and `RDINSTRET` instructions are available.

#### ENABLE_REGS_DUALPORT (default = 1)

The register file can be implemented with two or one read ports. A dual ported
register file improves performance a bit, but can also increase the size of
the core.

#### LATCHED_MEM_RDATA (default = 0)

Set this to 1 if the `mem_rdata` is kept stable by the external circuit after a
transaction. In the default configuration the PicoRV32 core only expects the
`mem_rdata` input to be valid in the cycle with `mem_valid && mem_ready` and
latches the value internally.

#### TWO_STAGE_SHIFT (default = 1)

By default shift operations are performed in two stages: first shifts in units
of 4 bits and then shifts in units of 1 bit. This speeds up shift operations,
but adds additional hardware. Set this parameter to 0 to disable the two-stage
shift to further reduce the size of the core.

#### BARREL_SHIFTER (default = 0)

By default shift operations are performed by successively shifting by a
small amount (see `TWO_STAGE_SHIFT` above). With this option set, a barrel
shifter is used instead.

#### TWO_CYCLE_COMPARE (default = 0)

This relaxes the longest data path a bit by adding an additional FF stage
at the cost of adding an additional clock cycle delay to the conditional
branch instructions.

*Note: Enabling this parameter will be most effective when retiming (aka
"register balancing") is enabled in the synthesis flow.*

#### TWO_CYCLE_ALU (default = 0)

This adds an additional FF stage in the ALU data path, improving timing
at the cost of an additional clock cycle for all instructions that use
the ALU.

*Note: Enabling this parameter will be most effective when retiming (aka
"register balancing") is enabled in the synthesis flow.*

#### CATCH_MISALIGN (default = 1)

Set this to 0 to disable the circuitry for catching misaligned memory
accesses.

#### CATCH_ILLINSN (default = 1)

Set this to 0 to disable the circuitry for catching illegal instructions.

The core will still trap on `EBREAK` instructions with this option
set to 0. With IRQs enabled, an `EBREAK` normally triggers an IRQ 1. With
this option set to 0, an `EBREAK` will trap the processor without
triggering an interrupt.

#### ENABLE_IRQ (default = 0)

Set this to 1 to enable IRQs. (see "Custom Instructions for IRQ Handling" below
for a discussion of IRQs)

#### ENABLE_IRQ_QREGS (default = 1)

Set this to 0 to disable support for the `getq` and `setq` instructions. Without
the q-registers, the irq return address will be stored in x3 (gp) and the IRQ
bitmask in x4 (tp), the global pointer and thread pointer registers according
to the RISC-V ABI.  Code generated from ordinary C code will not interact with
those registers.

Support for q-registers is always disabled when ENABLE_IRQ is set to 0.

#### ENABLE_IRQ_TIMER (default = 1)

Set this to 0 to disable support for the `timer` instruction.

Support for the timer is always disabled when ENABLE_IRQ is set to 0.

#### ENABLE_TRACE (default = 0)

Produce an execution trace using the `trace_valid` and `trace_data` output ports.
For a demontration of this feature run `make test_vcd` to create a trace file
and then run `python3 showtrace.py testbench.trace firmware/firmware.elf` to decode
it.

#### REGS_INIT_ZERO (default = 0)

Set this to 1 to initialize all registers to zero (using a Verilog `initial` block).
This can be useful for simulation or formal verification.

#### MASKED_IRQ (default = 32'h 0000_0000)

A 1 bit in this bitmask corresponds to a permanently disabled IRQ.

#### LATCHED_IRQ (default = 32'h ffff_ffff)

A 1 bit in this bitmask indicates that the corresponding IRQ is "latched", i.e.
when the IRQ line is high for only one cycle, the interrupt will be marked as
pending and stay pending until the interrupt handler is called (aka "pulse
interrupts" or "edge-triggered interrupts").

Set a bit in this bitmask to 0 to convert an interrupt line to operate
as "level sensitive" interrupt.

#### PROGADDR_RESET (default = 32'h 0000_0000)

The start address of the program.

#### PROGADDR_IRQ (default = 32'h 0000_0010)

The start address of the interrupt handler.

#### STACKADDR (default = 32'h ffff_ffff)

When this parameter has a value different from 0xffffffff, then register `x2` (the
stack pointer) is initialized to this value on reset. (All other registers remain
uninitialized.) Note that the RISC-V calling convention requires the stack pointer
to be aligned on 16 bytes boundaries (4 bytes for the RV32I soft float calling
convention).


Cycles per Instruction Performance
----------------------------------

*A short reminder: This core is optimized for size and f<sub>max</sub>, not performance.*

Unless stated otherwise, the following numbers apply to a PicoRV32-imt with
ENABLE_REGS_DUALPORT active and connected to a memory that can accommodate
requests within one clock cycle.

The average Cycles per Instruction (CPI) is approximately 4, depending on the mix of
instructions in the code. The CPI numbers for the individual instructions can
be found in the table below. The column "CPI (SP)" contains the CPI numbers for
a core built without ENABLE_REGS_DUALPORT.

| Instruction          |  CPI | CPI (SP) |
| ---------------------| ----:| --------:|
| direct jump (jal)    |    3 |        3 |
| ALU reg + immediate  |    3 |        3 |
| ALU reg + reg        |    3 |        4 |
| branch (not taken)   |    3 |        4 |
| memory load          |    5 |        5 |
| memory store         |    5 |        6 |
| branch (taken)       |    5 |        6 |
| indirect jump (jalr) |    6 |        6 |
| shift operations     | 4-14 |     4-15 |

Dhrystone benchmark results: 0.516 DMIPS/MHz (908 Dhrystones/Second/MHz)

For the Dhrystone benchmark the average CPI is 4.100.

Without using the look-ahead memory interface (usually required for max
clock speed), this results drop to 0.305 DMIPS/MHz and 5.232 CPI.


PicoRV32 Native Memory Interface
--------------------------------

The native memory interface of PicoRV32 is a simple valid-ready interface
that can run one memory transfer and one instruction transfer at a time:

    output        mem_valid
    input         mem_ready

    output [31:0] mem_addr
    output [31:0] mem_wdata
    output [ 3:0] mem_wstrb
    input  [31:0] mem_rdata
    
    output        mem_instr
    output        instr_valid
    input         instr_ready
    
    output [31:0] instr_addr
    input  [31:0] instr_rdata

The core initiates a memory transfer by asserting `mem_valid`. The valid
signal stays high until the peer asserts `mem_ready`. All core outputs
are stable over the `mem_valid` period. The instruction interface
works the same way, except that the core asserts `mem_instr`.

#### Read Transfer

In a read transfer `mem_wstrb` has the value 0 and `mem_wdata` is unused.

The memory reads the address `mem_addr` and makes the read value available on
`mem_rdata` in the cycle `mem_ready` is high.

There is no need for an external wait cycle. The memory read can be implemented
asynchronously with `mem_ready` going high in the same cycle as `mem_valid`, or
`mem_ready` being tied to constant 1.

Again, the instruction interface works the same way.

#### Write Transfer

In a write transfer `mem_wstrb` is not 0 and `mem_rdata` is unused. The memory
write the data at `mem_wdata` to the address `mem_addr` and acknowledges the
transfer by asserting `mem_ready`.

The 4 bits of `mem_wstrb` are write enables for the four bytes in the addressed
word. Only the 8 values `0000`, `1111`, `1100`, `0011`, `1000`, `0100`, `0010`,
and `0001` are possible, i.e. no write, write 32 bits, write upper 16 bits,
write lower 16, or write a single byte respectively.

There is no need for an external wait cycle. The memory can acknowledge the
write immediately  with `mem_ready` going high in the same cycle as
`mem_valid`, or `mem_ready` being tied to constant 1.

#### Look-Ahead Interface

The PicoRV32 core also provides a "Look-Ahead Memory Interface" that provides
all information about the next memory transfer one clock cycle earlier than the
normal interface.

    output        mem_la_read
    output        mem_la_write
    output [31:0] mem_la_addr
    output [31:0] mem_la_wdata
    output [ 3:0] mem_la_wstrb

In the clock cycle before `mem_valid` goes high, this interface will output a
pulse on `mem_la_read` or `mem_la_write` to indicate the start of a read or
write transaction in the next clock cycle.

*Note: The signals `mem_la_read`, `mem_la_write`, and `mem_la_addr` are driven
by combinatorial circuits within the PicoRV32 core. It might be harder to
achieve timing closure with the look-ahead interface than with the normal
memory interface described above.*

Custom Instructions for IRQ Handling
------------------------------------

This is untested for now.

*Note: The IRQ handling features in PicoRV32-imt do not follow the RISC-V
Privileged ISA specification. Instead a small set of very simple custom
instructions is used to implement IRQ handling with minimal hardware
overhead.*

The following custom instructions are only supported when IRQs are enabled
via the `ENABLE_IRQ` parameter (see above).

The PicoRV32 core has a built-in interrupt controller with 32 interrupt inputs. An
interrupt can be triggered by asserting the corresponding bit in the `irq`
input of the core.

When the interrupt handler is started, the `eoi` End Of Interrupt (EOI) signals
for the handled interrupts go high. The `eoi` signals go low again when the
interrupt handler returns.

The IRQs 0-2 can be triggered internally by the following built-in interrupt sources:

| IRQ | Interrupt Source                    |
| ---:| ------------------------------------|
|   0 | Timer Interrupt                     |
|   1 | EBREAK/ECALL or Illegal Instruction |
|   2 | BUS Error (Unalign Memory Access)   |

This interrupts can also be triggered by external sources.

The core has 4 additional 32-bit registers `q0 .. q3` that are used for IRQ
handling. When the IRQ handler is called, the register `q0` contains the return
address and `q1` contains a bitmask of all IRQs to be handled. This means one
call to the interrupt handler needs to service more than one IRQ when more than
one bit is set in `q1`.

When support for compressed instructions is enabled, then the LSB of q0 is set
when the interrupted instruction is a compressed instruction. This can be used if
the IRQ handler wants to decode the interrupted instruction.

Registers `q2` and `q3` are uninitialized and can be used as temporary storage
when saving/restoring register values in the IRQ handler.

All of the following instructions are encoded under the `custom0` opcode. The f3
and rs2 fields are ignored in all this instructions.

See [firmware/custom_ops.S](firmware/custom_ops.S) for GNU assembler macros that
implement mnemonics for this instructions.

See [firmware/start.S](firmware/start.S) for an example implementation of an
interrupt handler assembler wrapper, and [firmware/irq.c](firmware/irq.c) for
the actual interrupt handler.

#### getq rd, qs

This instruction copies the value from a q-register to a general-purpose
register.

    0000000 ----- 000XX --- XXXXX 0001011
    f7      rs2   qs    f3  rd    opcode

Example:

    getq x5, q2

#### setq qd, rs

This instruction copies the value from a general-purpose register to a
q-register.

    0000001 ----- XXXXX --- 000XX 0001011
    f7      rs2   rs    f3  qd    opcode

Example:

    setq q2, x5

#### retirq

Return from interrupt. This instruction copies the value from `q0`
to the program counter and re-enables interrupts.

    0000010 ----- 00000 --- 00000 0001011
    f7      rs2   rs    f3  rd    opcode

Example:

    retirq

#### maskirq

The "IRQ Mask" register contains a bitmask of masked (disabled) interrupts.
This instruction writes a new value to the irq mask register and reads the old
value.

    0000011 ----- XXXXX --- XXXXX 0001011
    f7      rs2   rs    f3  rd    opcode

Example:

    maskirq x1, x2

The processor starts with all interrupts disabled.

An illegal instruction or bus error while the illegal instruction or bus error
interrupt is disabled will cause the processor to halt.

#### waitirq

Pause execution until an interrupt becomes pending. The bitmask of pending IRQs
is written to `rd`.

    0000100 ----- 00000 --- XXXXX 0001011
    f7      rs2   rs    f3  rd    opcode

Example:

    waitirq x1

#### timer

Reset the timer counter to a new value. The counter counts down clock cycles and
triggers the timer interrupt when transitioning from 1 to 0. Setting the
counter to zero disables the timer. The old value of the counter is written to
`rd`.

    0000101 ----- XXXXX --- XXXXX 0001011
    f7      rs2   rs    f3  rd    opcode

Example:

    timer x1, x2


Building a pure RV32I Toolchain
-------------------------------

TL;DR: Run the following commands to build the complete toolchain:

    make download-tools
    make -j$(nproc) build-tools

The default settings in the [riscv-tools](https://github.com/riscv/riscv-tools) build
scripts will build a compiler, assembler and linker that can target any RISC-V ISA,
but the libraries are built for RV32G and RV64G targets. Follow the instructions
below to build a complete toolchain (including libraries) that target a pure RV32I
CPU.

The following commands will build the RISC-V GNU toolchain and libraries for a
pure RV32I target, and install it in `/opt/riscv32i`:

    # Ubuntu packages needed:
    sudo apt-get install autoconf automake autotools-dev curl libmpc-dev \
            libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo \
	    gperf libtool patchutils bc zlib1g-dev git libexpat1-dev

    sudo mkdir /opt/riscv32i
    sudo chown $USER /opt/riscv32i

    git clone https://github.com/riscv/riscv-gnu-toolchain riscv-gnu-toolchain-rv32i
    cd riscv-gnu-toolchain-rv32i
    git checkout 411d134
    git submodule update --init --recursive

    mkdir build; cd build
    ../configure --with-arch=rv32i --prefix=/opt/riscv32i
    make -j$(nproc)

The commands will all be named using the prefix `riscv32-unknown-elf-`, which
makes it easy to install them side-by-side with the regular riscv-tools (those
are using the name prefix `riscv64-unknown-elf-` by default).

Alternatively you can simply use one of the following make targets from PicoRV32's
Makefile to build a `RV32I[M][C]` toolchain. You still need to install all
prerequisites, as described above. Then run any of the following commands in the
PicoRV32 source directory:

| Command                                  | Install Directory  | ISA       |
|:---------------------------------------- |:------------------ |:--------  |
| `make -j$(nproc) build-riscv32i-tools`   | `/opt/riscv32i/`   | `RV32I`   |

Or simply run `make -j$(nproc) build-tools` to build and install all four tool chains.

By default calling any of those make targets will (re-)download the toolchain
sources. Run `make download-tools` to download the sources to `/var/cache/distfiles/`
once in advance.

*Note: These instructions are for git rev 411d134 (2018-02-14) of riscv-gnu-toolchain.*


Linking binaries with newlib for PicoRV32-imt
---------------------------------------------

The tool chains (see last section for install instructions) come with a version of
the newlib C standard library.

Use the linker script [firmware/riscv.ld](firmware/riscv.ld) for linking binaries
against the newlib library. Using this linker script will create a binary that
has its entry point at 0x10000. (The default linker script does not have a static
entry point, thus a proper ELF loader would be needed that can determine the
entry point at runtime while loading the program.)

Newlib comes with a few syscall stubs. You need to provide your own implementation
of those syscalls and link your program with this implementation, overwriting the
default stubs from newlib. See `syscalls.c` in [scripts/cxxdemo/](scripts/cxxdemo/)
for an example of how to do that.
