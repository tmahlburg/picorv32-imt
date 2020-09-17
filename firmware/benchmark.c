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

#define THREADS 6

// define runtime variables, so they are guaranteed to have different memory adresses
int i, j, k, l, m, n, o;
int a, b, c, d, e, f, g;

int result_vector[DIM];
int done[THREADS];

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
	for (a = 0; a < DIM; a += THREADS) {
		result_vector[a] = 0;
	    for (i = 0; i < DIM; i++) {
		    // matrix[i+a*DIM] = A_a_i
		    result_vector[a] += matrix[i+a*DIM] * vector[i];
	    }
	}
	done[0] = 1;
	// print result vector, if all threads are done
	int all_done = 0;
	int o;
    while (all_done != THREADS) {
		all_done = 0;
		for (o = 0; o < THREADS; o++) {
		    if (done[o])
			    all_done++;
	    }
	    __asm__("nop\n\t");
    }

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
	for (b = 1; b < DIM; b += THREADS) {
		result_vector[b] = 0;
	    for (j = 0; j < DIM; j++) {
		    result_vector[b] += matrix[j+b*DIM] * vector[j];
	    }
	}
    done[1] = 1;
    __asm__("j .DONE\n");

    __asm__(".THREAD_2:\n\t");
	done[2] = 0;
	for (c = 2; c < DIM; c += THREADS) {
		result_vector[c] = 0;
	    for (k = 0; k < DIM; k++) {
		    result_vector[c] += matrix[k+c*DIM] * vector[k];
	    }
	}
    done[2] = 1;
    __asm__("j .DONE\n");

    __asm__(".THREAD_3:\n\t");
	done[3] = 0;
	for (d = 3; d < DIM; d += THREADS) {
		result_vector[d] = 0;
	    for (l = 0; l < DIM; l++) {
		    result_vector[d] += matrix[l+d*DIM] * vector[l];
	    }
	}
    done[3] = 1;
    __asm__("j .DONE\n");

    __asm__(".THREAD_4:\n\t");
	done[4] = 0;
	for (e = 4; e < DIM; e += THREADS) {
		result_vector[e] = 0;
	    for (m = 0; m < DIM; m++) {
		    result_vector[e] += matrix[m+e*DIM] * vector[m];
	    }
	}
    done[4] = 1;
    __asm__("j .DONE\n");

    __asm__(".THREAD_5:\n\t");
	done[5] = 0;
	for (f = 5; f < DIM; f += THREADS) {
		result_vector[f] = 0;
	    for (n = 0; n < DIM; n++) {
		    result_vector[f] += matrix[n+f*DIM] * vector[n];
	    }
	}
    done[5] = 1;

    __asm__(".DONE:\n\t");
	while (1) {
		__asm__("nop\n\t");
	}
    return 0;
}
