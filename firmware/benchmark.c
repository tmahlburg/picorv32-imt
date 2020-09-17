/*
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

#include <stdint.h>
// #include "benchmark_data.h"

#define reg_leds (*(volatile uint32_t*)0x03000000)
#define reg_uart_clkdiv (*(volatile uint32_t*)0x02000004)
#define reg_uart_data (*(volatile uint32_t*)0x02000008)

#define THREADS 6
#define DIM 6

int matrix[] = {4, 6, 7, 6, 7, 7,
				1, 2, 4, 4, 1, 7,
				9, 7, 4, 4, 2, 1,
				1, 9, 3, 8, 7, 5,
				8, 6, 0, 7, 1, 1,
				6, 7, 5, 7, 3, 7};
int vector[] = {7, 5, 1, 2, 1, 8};

int result_vector[DIM];

// define runtime variables, so they have different memory adresses
int i, j, k, l, m, n, o;

int done[THREADS];

void putchar(char c)
{
	if (c == '\n')
		putchar('\r');
	reg_uart_data = c;
}

void print_dec(uint32_t v)
{
	if (v >= 1000) {
		putchar('e');
		return;
	}

	if      (v >= 900) { putchar('9'); v -= 900; }
	else if (v >= 800) { putchar('8'); v -= 800; }
	else if (v >= 700) { putchar('7'); v -= 700; }
	else if (v >= 600) { putchar('6'); v -= 600; }
	else if (v >= 500) { putchar('5'); v -= 500; }
	else if (v >= 400) { putchar('4'); v -= 400; }
	else if (v >= 300) { putchar('3'); v -= 300; }
	else if (v >= 200) { putchar('2'); v -= 200; }
	else if (v >= 100) { putchar('1'); v -= 100; }

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
	// Threadswitch
    __asm__("csrr t0, 0xF14\n\t" // load current thread id into register t0
            "bnez t0, .ID_NEQ_0\n\t"// check if hart id != 0
            "j .THREAD_0\n" // if not, jump to .THREAD_0 label
            ".ID_NEQ_0:\n\t" // else
            "li t1, 1\n\t"// load 1 in register t1
            "bne t0, t1, .ID_NEQ_1\n\t"// check if thread id != 1
            "j .THREAD_1\n"// if not, jump to .THREAD_1 label
            ".ID_NEQ_1:\n\t"// else
            "li t1, 2\n\t"
            "bne t0, t1, .ID_NEQ_2\n\t"// check if thread id != 2
            "j .THREAD_2\n\t"// if not jump to .THREAD_2 label
            ".ID_NEQ_2:\n"
            "li t1, 3\n\t"// thread 3 check
            "bne t0, t1, .ID_NEQ_3\n\t"
            "j .THREAD_3\n\t"
            ".ID_NEQ_3:\n"
            "li t1, 4\n\t"// thread 4 check
            "bne t0, t1, .ID_NEQ_4\n\t"
            "j .THREAD_4\n\t"
            ".ID_NEQ_4:\n"
            "li t1, 5\n\t"// thread 5 check
            "bne t0, t1, .ID_NEQ_5\n\t"
            "j .THREAD_5\n\t"
            ".ID_NEQ_5:\n"
            "j .DONE\n");// if thread id > 5 -> go to done

    __asm__(".THREAD_0:\n\t");
	done[0] = 0;
	result_vector[0] = 0;
    for (i = 0; i < DIM; i++) {
	    result_vector[0] += matrix[i] * vector[i];
    }
	done[0] = 1;
	// print result vector, if all threads are done
    while (!done[1] || !done[2] || !done[3] || !done[4] || !done[5]) {
	    __asm__("nop\n\t");
    }

	int o;
	reg_leds = 0xfff;
	reg_uart_clkdiv = 86;
	for (o = 0; o < DIM; o++) {
		print_dec(result_vector[o]);
		putchar(' ');
	}
	putchar(' ');
    __asm__("j .DONE\n");

    __asm__(".THREAD_1:\n\t");
	done[1] = 0;
	result_vector[1] = 0;
    for (j = 0; j < DIM; j++) {
	    result_vector[1] += matrix[j+DIM] * vector[j];
    }
    done[1] = 1;
    __asm__("j .DONE\n");

    __asm__(".THREAD_2:\n\t");
	done[2] = 0;
	result_vector[2] = 0;
    for (k = 0; k < DIM; k++) {
		result_vector[2] += matrix[k+DIM*2] * vector[k];
    }
    done[2] = 1;
    __asm__("j .DONE\n");

    __asm__(".THREAD_3:\n\t");
	done[3] = 0;
	result_vector[3] = 0;
    for (l = 0; l < DIM; l++) {
	    result_vector[3] += matrix[l+DIM*3] * vector[l];
    }
    done[3] = 1;
    __asm__("j .DONE\n");

    __asm__(".THREAD_4:\n\t");
	done[4] = 0;
	result_vector[4] = 0;
    for (m = 0; m < DIM; m++) {
	    result_vector[4] += matrix[m+DIM*4] * vector[m];
    }
    done[4] = 1;
    __asm__("j .DONE\n");

    __asm__(".THREAD_5:\n\t");
	done[5] = 0;
	result_vector[5] = 0;
    for (n = 0; n < DIM; n++) {
	    result_vector[5] += matrix[n+DIM*5] * vector[n];
    }
    done[5] = 1;

    __asm__(".DONE:\n\t");
	while (1) {
		__asm__("nop\n\t");
	}
    return 0;
}
