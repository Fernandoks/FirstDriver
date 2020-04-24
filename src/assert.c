/*
 * assert.c
 *
 *  Created on: Apr 15, 2020
 *      Author: fernandoks
 */



#include <assert.h>
#include <stdlib.h>
#include <stdint.h>


#define ASSERT
#ifdef ASSERT


void
__attribute__((noreturn))
__assert_func (const char *file, int line, const char *func,
               const char *failedexpr)
{
#define SEMIHOSTING
#ifdef SEMIHOSTING
	printf("assertion \"%s\" failed: file \"%s\", line %d%s%s\n",
            failedexpr, file, line, func ? ", function: " : "",
            func ? func : "");


#endif

//#define TRACE
#ifdef TRACE
	trace_printf ("assertion \"%s\" failed: file \"%s\", line %d%s%s\n",
                failedexpr, file, line, func ? ", function: " : "",
                func ? func : "");
#endif


	abort ();

}

#endif
