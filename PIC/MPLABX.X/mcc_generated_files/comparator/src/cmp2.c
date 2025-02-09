/**
 * CMP2 Generated Driver File.
 * 
 * @file cmp2.c
 * 
 * @ingroup cmp2
 * 
 * @brief This file contains the API implementation for the CMP2 driver.
 *
 * @version CMP2 Driver Version 2.13.1
*/
 /*
© [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#include <xc.h>
#include "../cmp2.h"

static void (*CMP2_InterruptHandler)(void);
static void CMP2_DefaultInterruptCallback(void);


void CMP2_Initialize(void) 
{
    // CHYS No hysteresis; CON enabled; CPOL not inverted; CSYNC asynchronous; 
    CM2CON0 = 0x80;

    // CINTN no_intFlag; CINTP no_intFlag; CSP 30ns hi_speed; 
    CM2CON1 = 0x0;
    
    // CNCH CIN0-; 
    CM2NCH = 0x0;
    
    // CPCH CIN0+; 
    CM2PCH = 0x0;    

    CMP2_InterruptCallbackRegister(CMP2_DefaultInterruptCallback);
}

void CMP2_Enable(void) 
{
    CM2CON0bits.C2EN = 1;
}

void CMP2_Disable(void) 
{
    CM2CON0bits.C2EN = 0;
}

bool CMP2_GetOutputStatus(void) 
{
    return CM2CON0bits.C2OUT; /* Note : As per datasheet, CxOUT bit in CMOUT register (if relevant) 
                                                   is mirror copy of CMxCON0.OUT */
}

void CMP2_ISR(void) 
{
    // Clear the CMP2 interrupt flag
    PIR5bits.C2IF = 0;

    if(CMP2_InterruptHandler)
    {
      CMP2_InterruptHandler();
    }
}

void CMP2_InterruptCallbackRegister(void(*callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
      CMP2_InterruptHandler = callbackHandler;
    }
}

static void CMP2_DefaultInterruptCallback(void)
{
    //Add your interrupt code here or
    //Use CMP2_InterruptCallbackRegister function to use Custom ISR

}