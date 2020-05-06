/*******************************************************************************
* Title                 :   Circular Buffer
* Filename              :   CircularBuffer.h
* Author                :   Fernando Kaba Surjus
* Origin Date           :   01/05/2020
* Version               :   1.0.0
* Compiler              :   GNU GCC
* Target                :   ARM CORTEX M
* Notes                 :   None
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED
* OR IMPLIED WARRANTIES, INCLUDING, BUT NOTLIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL BENINGO EMBEDDED OR ITS CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/
/*************** INTERFACE CHANGE LIST **************************************
*
*    Date    Version   Author         Description
*  01/05/20   1.0.0   FernandoKS 	  Interface Created.
*
*****************************************************************************/
/** @file: CircularBuffer.h
 *  @brief This module implements a Circular Buffer to be used in Single producer
 *  	   Single consumer communications
 *
 */

#ifndef INC_CIRCULARBUFFER_H_
#define INC_CIRCULARBUFFER_H_



/******************************************************************************
* Includes
*******************************************************************************/
#include <stdint.h>				/* For portable types */
#include <stdbool.h>			/* Boolean */
#include <stdio.h> 				/* size_t  */
#include <stdlib.h>				/* Malloc  */
#include <assert.h>

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/


//circular buffer structure
typedef struct  {
	uint8_t *buffer;
	size_t 	head;
	size_t 	tail;
	size_t 	maxsize;
	bool 	isfull;
}circular_buffer_t;

//encapsulation
typedef circular_buffer_t* cbuf_handle_t;



/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/
#ifdef __cplusplus
extern "C"{
#endif

/*
 * Initialize the Circular buffer
 */
cbuf_handle_t circular_buffer_init(uint8_t* buffer, size_t size);

/*
 * Free the Structure of cbuf_handle_t
 * USER still need to free the data buffer;
 */
void circular_buffer_free(cbuf_handle_t cbuf);

/*
 * Empty the circular buffer, HEAD = TAIL
 */
void circular_buffer_reset(cbuf_handle_t cbuf);

/*
 * Add data even if the buffer is full
 * OVERWIRITTEN
 */
void circular_buffer_push(cbuf_handle_t cbuf, uint8_t data);


/*
 * Add data, but rejects new data is buffer is full
 * Return 0 if success, and -1 if buffer is full (TODO: Use type here)
 */
int circular_buffer_trypush(cbuf_handle_t cbuf, uint8_t data);

/*
 * Get data from buffer
 *  Returns 0 on success, -1 if the buffer is empty
 */
int circular_buffer_pop(cbuf_handle_t cbuf, uint8_t * data);

/*
 * Checks if the buffer is empty
 */
bool circular_buffer_empty(cbuf_handle_t cbuf);

/*
 * Checks if the buffer is full
 */
bool circular_buffer_full(cbuf_handle_t cbuf);

/*
 * Return buffer size
 */
size_t circular_buffer_capacity(cbuf_handle_t cbuf);

/*
 * Return buffer current occupation
 */
size_t circular_buffer_size(cbuf_handle_t cbuf);

/******************************************************************************
 * HELPER FUNCTIONS - STATIC
 *****************************************************************************/
static void advance_pointer(cbuf_handle_t cbuf);
static void retreat_pointer(cbuf_handle_t cbuf);


#ifdef __cplusplus
} // extern "C"
#endif


/*** End of File **************************************************************/

#endif /* INC_CIRCULARBUFFER_H_ */
