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

#define THREADS 2
#define DIM 6

int matrix[] = {	4, 6, 7, 6, 7, 7,
					1, 2, 4, 4, 1, 7,
					9, 7, 4, 4, 2, 1,
					1, 9, 3, 8, 7, 5,
					8, 6, 0, 7, 1, 1,
					6, 7, 5, 7, 3, 7};
int vector[] = {	7, 5, 1, 2, 1, 8};

int result_vector[DIM];

// define runtime variables, so they have different memory adresses
int i, j, k, l, m, n, o;

int main()
{
	// Threadswitch
    __asm__("csrr t0, 0xF14\n\t" 		// load current thread id into register t0
            "bnez t0, .ID_NEQ_0\n\t"	// check if hart id != 0
            "j .THREAD_0\n" 			// if not, jump to .THREAD_0 label
            ".ID_NEQ_0:\n\t" 			// else
            "li t1, 1\n\t"				// load 1 in register t1
            "bne t0, t1, .ID_NEQ_1\n\t"	// check if thread id != 1
            "j .THREAD_1\n"				// if not, jump to .THREAD_1 label
            ".ID_NEQ_1:\n\t"			// else
            "li t1, 2\n\t"
            "bne t0, t1, .ID_NEQ2\n\t"	// check if thread id != 2
            "j .THREAD_2\n\t"			// if not jump to .THREAD_2 label
            ".ID_NEQ2:\n"
            "li t1, 3\n\t"				// thread 3 check
            "bne t0, t1, .ID_NEQ3\n\t"
            "j .THREAD_3\n\t"
            ".ID_NEQ3:\n"
            "li t1, 4\n\t"				// thread 4 check
            "bne t0, t1, .ID_NEQ4\n\t"
            "j .THREAD_4\n\t"
            ".ID_NEQ4:\n"
            "li t1, 5\n\t"				// thread 5 check
            "bne t0, t1, .ID_NEQ5\n\t"
            "j .THREAD_5\n\t"
            ".ID_NEQ5:\n"
            "j .DONE\n");

    __asm__(".THREAD_0:\n\t");
    result_vector[0] = 0;
    for (i = 0; i < DIM; i++) {
	    result_vector[i] += matrix[i+DIM*0] * vector[i];
    }
    __asm__(".j DONE\n");
    __asm__(".THREAD_1:\n\t");
    result_vector[1] = 0;
    for (j = 0; j < DIM; j++) {
	    result_vector[j] += matrix[j+DIM*1] * vector[j];
    }
    __asm__(".j DONE\n");
    __asm__(".THREAD_2:\n\t");
    result_vector[2] = 0;
    for (k = 0; k < DIM; k++) {
	    result_vector[k] += matrix[k+DIM*2] * vector[k];
    }
    __asm__(".j DONE\n");
    __asm__(".THREAD_3:\n\t");
    result_vector[3] = 0;
    for (l = 0; l < DIM; l++) {
	    result_vector[l] += matrix[l+DIM*3] * vector[l];
    }
    __asm__(".j DONE\n");
    __asm__(".THREAD_4:\n\t");
    result_vector[4] = 0;
    for (m = 0; m < DIM; m++) {
	    result_vector[m] += matrix[m+DIM*4] * vector[m];
    }
    __asm__(".j DONE\n");
    __asm__(".THREAD_5:\n\t");
    result_vector[5] = 0;
    for (n = 0; n < DIM; n++) {
	    result_vector[n] += matrix[n+DIM*5] * vector[n];
    }

    __asm__(".DONE\n\t");
    return 0;
}
