CROSS=riscv32-unknown-elf-
FMT_HEX_CMD=../scripts/fmt_hex.py
CFLAGS=-Wall -Wextra

# ---- BASYS 3 Board ----
#TODO: automate synthesis and simulation via Makefile
start.o: ../picoramsoc/start.s
	$(CROSS)as --warn -march=rv32i ../picoramsoc/start.s -o start.o

benchmark_data.o: benchmark_data.c
	$(CROSS)gcc $(CFLAGS) -march=rv32i -nostdlib -nostartfiles -ffreestanding  -c benchmark_data.c -o benchmark_data.o

basys3_benchmark.elf: ../picoramsoc/basys3_sections.lds start.o benchmark.c benchmark_data.o
	$(CROSS)gcc $(CFLAGS) -march=rv32i -nostdlib -nostartfiles -ffreestanding  -c benchmark.c -o benchmark.o
	$(CROSS)gcc $(CFLAGS) -march=rv32i -nostdlib -nostartfiles -ffreestanding  -Xlinker -T -Xlinker ../picoramsoc/basys3_sections.lds start.o benchmark.o benchmark_data.o -lgcc -o basys3_benchmark.elf

basys3_benchmark.hex: basys3_benchmark.elf
	$(CROSS)objcopy -O verilog basys3_benchmark.elf basys3_benchmark.hex
	$(FMT_HEX_CMD) basys3_benchmark.hex

basys3_benchmark.s: benchmark.c
	$(CROSS)gcc $(CFLAGS) -S -march=rv32i -nostdlib -nostartfiles -ffreestanding  -c benchmark.c -o basys3_benchmark.s

basys3_benchmark_single.elf: ../picoramsoc/basys3_sections.lds start.o benchmark_single.c benchmark_data.o
	$(CROSS)gcc $(CFLAGS) -march=rv32i -nostdlib -nostartfiles -ffreestanding  -c benchmark_single.c -o benchmark_single.o
	$(CROSS)gcc $(CFLAGS) -march=rv32i -nostdlib -nostartfiles -ffreestanding  -Xlinker -T -Xlinker ../picoramsoc/basys3_sections.lds start.o benchmark_single.o benchmark_data.o -lgcc -o basys3_benchmark_single.elf

basys3_benchmark_single.hex: basys3_benchmark_single.elf
	$(CROSS)objcopy -O verilog basys3_benchmark_single.elf basys3_benchmark_single.hex
	$(FMT_HEX_CMD) basys3_benchmark_single.hex

clean:
	rm start.o benchmark_data.o basys3_benchmark.elf basys3_benchmark.hex basys3_benchmark.s basys3_benchmark_single.elf basys3_benchmark_single.hex
