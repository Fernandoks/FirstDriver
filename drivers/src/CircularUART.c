/*******************************************************************************
* Title                 :   Circular UART
* Filename              :   CircularUART.c
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
/** @file CircularUART.c
 *  @brief:
 *  Uses CircularBuffer to send/receive serial data
 *
 */
/******************************************************************************
* Includes
*******************************************************************************/
#include "CircularUART.h"


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

cbuf_handle_t CircularUART_Init(uint8_t* buffer, size_t size)
{
	cbuf_handle_t cbuf =  circular_buffer_init(buffer, size);
	return cbuf;
}

void CircularUART_StartTx(cbuf_handle_t cbuf, UART_Handle_t* pUART)
{
	size_t length = circular_buffer_size(cbuf);

	uint8_t* data[length];
	for (int i = 0; i < length; ++i)
	{
		if ( circular_buffer_pop(cbuf,data[i]) == -1)
		{
			break;
		}
	}

	UART_SendDataBlockIT(pUART,data,(uint32_t)length);
}

void CircularUART_StartRx(cbuf_handle_t cbuf, UART_Handle_t* pUART)
{
	uint32_t Lenght;
	uint8_t *pRxBuffer;
	//TODO: Implement ReceiveStringDataIT
	UART_ReceiveBlockDataIT(pUART,pRxBuffer, Lenght);

	//circular_buffer_push(cbuf,data);

}

void CircularUART_ClearTx(void)
{

}

void CircularUART_ClearRx(void)
{

}


uint16_t CircularUART_Send(cbuf_handle_t cbuf, UART_Handle_t* pUART)
{
		size_t length = circular_buffer_size(cbuf);

		uint8_t data[50];
		for (int i = 0; i < length; ++i)
		{
			if ( circular_buffer_pop(cbuf,&data[i]) == -1)
			{
				break;
			}
		}

		UART_SendData(pUART,&data,length );
}


uint16_t CircularUART_Receive(cbuf_handle_t cbuf, UART_Handle_t* pUART)
{
	uint8_t buffer[100];
	UART_ReceiveDataString(pUART,buffer);
	size_t length = pUART->RxLen;

	for (int i = 0; i < length; ++i)
	{
		circular_buffer_push(cbuf, buffer[i]);
	}

}


uint16_t CircularUART_GetUnsentCount(void)
{

}


uint16_t CircularUART_GetUnreadCount(void)
{

}




