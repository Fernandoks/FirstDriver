/*******************************************************************************
* Title                 :   Circular Buffer
* Filename              :   CircularBuffer.c
* Author                :   Fernando Kaba Surjus
* Origin Date           :   01/05/2020
* Version               :   1.0.0
* Compiler              :   GNU GCC
* Target                :   ARM CORTEX M
* Notes                 :   None
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED
* OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE AUTHOR OR ITS CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
* THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/
/*************** SOURCE REVISION LOG *****************************************
*
*    Date    Version   Author         Description
*  01/05/20   1.0.0   FernandoKS 	Initial Release.
*
*******************************************************************************/
/** @file CircularBuffer.c
 *  @brief:
 *  This modules uses a bool flag and additional logic to differentiate states:
    Full state is full
    Empty state is (head == tail) && !full
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include "CircularBuffer.h"

/******************************************************************************
* Module Preprocessor Constants
*******************************************************************************/


/******************************************************************************
* Module Preprocessor Macros
*******************************************************************************/

/******************************************************************************
* Module Typedefs
*******************************************************************************/

/******************************************************************************
* Module Variable Definitions
*******************************************************************************/

/******************************************************************************
* Function Prototypes
*******************************************************************************/

/******************************************************************************
* Function Definitions
*******************************************************************************/


/******************************************************************************
* Function : circular_buffer_init()
*//**
* \b Description:
*
* This function is used to initialize the circular_buffer_ based o
*
* PRE-CONDITION: User need to allocate the buffer, and send the buffer pointer
* 				 size, and the structure circular_buffer_t
*
* POST-CONDITION: A constant pointer to the first member of the configuration
* table will be returned.
*
* @return 		A pointer to the configuration table.
*
* \b Example Example:
* @code
* 	// User provides struct
*	void circular_buffer_init(circular_buffer_t* cbuf, uint8_t* buffer, size_t size);
*
*	// Return a struct
*	circular_buffer_t circular_buffer_init(uint8_t* buffer, size_t size)
*
* @endcode
*
* @see Dio_Init
*
* <br><b> - HISTORY OF CHANGES - </b>
*
* <table align="left" style="width:800px">
* <tr><td> Date       </td><td> Software Version </td><td> Initials </td><td> Description </td></tr>
* <tr><td> 09/01/2015 </td><td> 0.5.0            </td><td> JWB      </td><td> Interface Created </td></tr>
* </table><br><br>
* <hr>
*
*******************************************************************************/

/******************************************************************************
 * Initialize the Circular buffer
*******************************************************************************/
cbuf_handle_t circular_buffer_init(uint8_t* buffer, size_t size)
{
	assert(buffer && size); //TEST if valid

	cbuf_handle_t cbuf = malloc(sizeof(circular_buffer_t));
	assert(cbuf); //Tests if NULL

	cbuf->buffer = buffer;
	cbuf->maxsize = size;
	circular_buffer_reset(cbuf);

	assert(circular_buffer_empty(cbuf));

	return cbuf;

}

/******************************************************************************
 * Free the Structure of cbuf_handle_t
 * USER still need to free the data buffer;
*******************************************************************************/
void circular_buffer_free(cbuf_handle_t cbuf)
{
	assert(cbuf);
	free(cbuf);
}

/******************************************************************************
 * Empty the circular buffer, HEAD = TAIL
*******************************************************************************/
void circular_buffer_reset(cbuf_handle_t cbuf)
{
    assert(cbuf);

    cbuf->head = 0;
    cbuf->tail = 0;
    cbuf->isfull = false;
}

/******************************************************************************
 * Add data even if the buffer is full
 * OVERWIRITTEN
*******************************************************************************/
void circular_buffer_push(cbuf_handle_t cbuf, uint8_t data)
{
	assert(cbuf && cbuf->buffer);

	cbuf->buffer[cbuf->head] = data;

	advance_pointer(cbuf);
}


/******************************************************************************
 * Add data, but rejects new data is buffer is full
 * Return 0 if success, and -1 if buffer is full (TODO: Use type here)
*******************************************************************************/
int circular_buffer_trypush(cbuf_handle_t cbuf, uint8_t data)
{
    int r = -1;

    assert(cbuf && cbuf->buffer);

    if(!circular_buffer_full(cbuf))
    {
        cbuf->buffer[cbuf->head] = data;
        advance_pointer(cbuf);
        r = 0;
    }

    return r;
}

/******************************************************************************
 * Get data from buffer
 *  Returns 0 on success, -1 if the buffer is empty
*******************************************************************************/
int circular_buffer_pop(cbuf_handle_t cbuf, uint8_t * data)
{
    assert(cbuf && data && cbuf->buffer);

    int r = -1;

    if(!circular_buffer_empty(cbuf))
    {
        *data = cbuf->buffer[cbuf->tail];
        retreat_pointer(cbuf);

        r = 0;
    }

    return r;
}

/******************************************************************************
 * Checks if the buffer is empty
*******************************************************************************/
bool circular_buffer_empty(cbuf_handle_t cbuf)
{
	assert(cbuf);
	return( (!cbuf->isfull) && (cbuf->head == cbuf->tail) );
}

/******************************************************************************
 * Checks if the buffer is full
*******************************************************************************/
bool circular_buffer_full(cbuf_handle_t cbuf)
{
	assert(cbuf);
	return ( cbuf->isfull );
}

/******************************************************************************
 * Return buffer size
*******************************************************************************/
size_t circular_buffer_capacity(cbuf_handle_t cbuf)
{
	assert(cbuf);
	return ( cbuf->maxsize );
}

/******************************************************************************
 * Return buffer current occupation
*******************************************************************************/
size_t circular_buffer_size(cbuf_handle_t cbuf)
{
	assert(cbuf);

	size_t size = cbuf->maxsize;

	if (!cbuf->isfull)
	{
		if(cbuf->head >= cbuf->tail)
		{
			size = (cbuf->head - cbuf->tail);
		}
		else
		{
			size = (cbuf->maxsize + cbuf->head - cbuf->tail);
		}
	}

	return size;

}

/******************************************************************************
 * HELPER FUNCTIONS - STATIC
 *****************************************************************************/

static void advance_pointer(cbuf_handle_t cbuf)
{
	assert(cbuf);

	if(cbuf->isfull)
   	{
		cbuf->tail = (cbuf->tail + 1) % cbuf->maxsize; //If both are equal the module is 0
	}

	cbuf->head = (cbuf->head + 1) % cbuf->maxsize;
	cbuf->isfull = (cbuf->head == cbuf->tail);
}


static void retreat_pointer(cbuf_handle_t cbuf)
{
	assert(cbuf);

	cbuf->isfull = false;
	cbuf->tail = (cbuf->tail + 1) % cbuf->maxsize;
}




/*************** END OF FUNCTIONS ***************************************************************************/
