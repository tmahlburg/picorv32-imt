/*
 *  Copyright (C) 2020  Till Mahlburg
 *  Copyright (C) 2017  Clifford Wolf
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

#include <stdint.h>
#include "benchmark_data.h"

#define reg_leds (*(volatile uint32_t*)0x03000000)
#define reg_uart_clkdiv (*(volatile uint32_t*)0x02000004)
#define reg_uart_data (*(volatile uint32_t*)0x02000008)

void putchar(char c)
{
	if (c == '\n')
		putchar('\r');
	reg_uart_data = c;
}

void print_dec(uint32_t v)
{
	if (v >= 10000) {
		putchar('e');
		return;
	}

	if      (v >= 9000) { putchar('9'); v -= 9000; }
	else if (v >= 8000) { putchar('8'); v -= 8000; }
	else if (v >= 7000) { putchar('7'); v -= 7000; }
	else if (v >= 6000) { putchar('6'); v -= 6000; }
	else if (v >= 5000) { putchar('5'); v -= 5000; }
	else if (v >= 4000) { putchar('4'); v -= 4000; }
	else if (v >= 3000) { putchar('3'); v -= 3000; }
	else if (v >= 2000) { putchar('2'); v -= 2000; }
	else if (v >= 1000) { putchar('1'); v -= 1000; }

	if      (v >= 900) { putchar('9'); v -= 900; }
	else if (v >= 800) { putchar('8'); v -= 800; }
	else if (v >= 700) { putchar('7'); v -= 700; }
	else if (v >= 600) { putchar('6'); v -= 600; }
	else if (v >= 500) { putchar('5'); v -= 500; }
	else if (v >= 400) { putchar('4'); v -= 400; }
	else if (v >= 300) { putchar('3'); v -= 300; }
	else if (v >= 200) { putchar('2'); v -= 200; }
	else if (v >= 100) { putchar('1'); v -= 100; }
	else putchar('0');

	if      (v >= 90) { putchar('9'); v -= 90; }
	else if (v >= 80) { putchar('8'); v -= 80; }
	else if (v >= 70) { putchar('7'); v -= 70; }
	else if (v >= 60) { putchar('6'); v -= 60; }
	else if (v >= 50) { putchar('5'); v -= 50; }
	else if (v >= 40) { putchar('4'); v -= 40; }
	else if (v >= 30) { putchar('3'); v -= 30; }
	else if (v >= 20) { putchar('2'); v -= 20; }
	else if (v >= 10) { putchar('1'); v -= 10; }
	else putchar('0');

	if      (v >= 9) { putchar('9'); v -= 9; }
	else if (v >= 8) { putchar('8'); v -= 8; }
	else if (v >= 7) { putchar('7'); v -= 7; }
	else if (v >= 6) { putchar('6'); v -= 6; }
	else if (v >= 5) { putchar('5'); v -= 5; }
	else if (v >= 4) { putchar('4'); v -= 4; }
	else if (v >= 3) { putchar('3'); v -= 3; }
	else if (v >= 2) { putchar('2'); v -= 2; }
	else if (v >= 1) { putchar('1'); v -= 1; }
	else putchar('0');
}

int main()
{
	int result_vector[DIM];
	for (int i = 0; i < DIM; i++) {
		result_vector[i] = 0;
		for (int j = 0; j < DIM; j++) {
			result_vector[i] += matrix[j+i*DIM] * vector[i];
		}
	}
	reg_leds = 0xfff;

	/*	reg_uart_clkdiv = 86;
	for (o = 0; o < DIM; o++) {
		print_dec(result_vector[o]);
		putchar(' ');
	}
	putchar(' ');*/
	return 0;
}
