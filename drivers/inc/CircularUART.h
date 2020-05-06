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
#ifndef INC_CIRCULARUART_H_
#define INC_CIRCULARUART_H_

/******************************************************************************
* Includes
*******************************************************************************/

#include "CircularBuffer.h"
#include "stm32f446xx_usart.h"

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
cbuf_handle_t CircularUART_Init(uint8_t* buffer, size_t size);
void CircularUART_StartTx(cbuf_handle_t cbuf, UART_Handle_t* pUART);
void CircularUART_StartRx(cbuf_handle_t cbuf, UART_Handle_t* pUART);
void CircularUART_ClearTx(void);
void CircularUART_ClearRx(void);
uint16_t CircularUART_Send(cbuf_handle_t cbuf, UART_Handle_t* pUART);
uint16_t CircularUART_Receive(cbuf_handle_t cbuf, UART_Handle_t* pUART);
uint16_t CircularUART_GetUnsentCount(void);
uint16_t CircularUART_GetUnreadCount(void);


#endif /* INC_CIRCULARUART_H_ */
