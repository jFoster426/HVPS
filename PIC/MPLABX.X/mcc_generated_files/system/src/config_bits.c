/**
 * CONFIGURATION BITS Generated Driver Source File
 * 
 * @file config_bits.c
 * 
 * @ingroup config_bitsdriver
 * 
 * * @brief This file contains the API prototype for the System Driver.
 *
 * @version Driver Version 1.0.0
*/

/*
� [2025] Microchip Technology Inc. and its subsidiaries.

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

// Configuration bits: selected in the GUI

//CONFIG1
#pragma config FEXTOSC = OFF     // External Oscillator Selection->Oscillator not enabled
#pragma config RSTOSC = HFINTOSC_64MHZ     // Reset Oscillator Selection->HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1

//CONFIG2
#pragma config CLKOUTEN = OFF     // Clock out Enable bit->CLKOUT function is disabled
#pragma config PR1WAY = ON     // PRLOCKED One-Way Set Enable bit->PRLOCKED bit can be cleared and set only once
#pragma config BBEN = OFF     // Boot Block enable bit->Boot block disabled
#pragma config CSWEN = OFF     // Clock Switch Enable bit->The NOSC and NDIV bits cannot be changed by user software
#pragma config FCMEN = OFF     // Fail-Safe Clock Monitor Enable bit->Fail-Safe Clock Monitor disabled
#pragma config FCMENP = OFF     // Fail-Safe Clock Monitor - Primary XTAL Enable bit->Fail-Safe Clock Monitor Disabled
#pragma config FCMENS = OFF     // Fail-Safe Clock Monitor - Secondary XTAL Enable bit->Fail-Safe Clock Monitor Disabled

//CONFIG3
#pragma config MCLRE = EXTMCLR     // MCLR Enable bit->If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR 
#pragma config PWRTS = PWRT_OFF     // Power-up timer selection bits->PWRT is disabled
#pragma config MVECEN = ON     // Multi-vector enable bit->Multi-vector enabled, Vector table used for interrupts
#pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit->IVTLOCKED bit can be cleared and set only once
#pragma config LPBOREN = OFF     // Low Power BOR Enable bit->Low-Power BOR disabled
#pragma config BOREN = SBORDIS     // Brown-out Reset Enable bits->Brown-out Reset enabled , SBOREN bit is ignored

//CONFIG4
#pragma config BORV = VBOR_2P85     // Brown-out Reset Voltage Selection bits->Brown-out Reset Voltage (VBOR) set to 2.85V
#pragma config ZCD = OFF     // ZCD Disable bit->ZCD module is disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit->PPSLOCKED bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle
#pragma config STVREN = ON     // Stack Full/Underflow Reset Enable bit->Stack full/underflow will cause Reset
#pragma config LVP = ON     // Low Voltage Programming Enable bit->Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored
#pragma config DEBUG = OFF     // Background Debugger->Background Debugger disabled
#pragma config XINST = OFF     // Extended Instruction Set Enable bit->Extended Instruction Set and Indexed Addressing Mode disabled

//CONFIG5
#pragma config WDTCPS = WDTCPS_31     // WDT Period selection bits->Divider ratio 1:65536; software control of WDTPS
#pragma config WDTE = OFF     // WDT operating mode->WDT Disabled; SWDTEN is ignored

//CONFIG6
#pragma config WDTCWS = WDTCWS_7     // WDT Window Select bits->window always open (100%); software control; keyed access not required
#pragma config WDTCCS = SC     // WDT input clock selector->Software Control

//CONFIG7
#pragma config BBSIZE = BBSIZE_128     // Boot Block Size Selection bits->Boot Block size is 128 words

//CONFIG8
#pragma config SAFSZ = SAFSZ_NONE     // SAF Block Size Selection bits->NONE

//CONFIG9
#pragma config WRTB = ON     // Boot Block Write Protection bit->Boot Block Write protected
#pragma config WRTC = ON     // Configuration Register Write Protection bit->Configuration registers Write protected
#pragma config WRTD = ON     // Data EEPROM Write Protection bit->Data EEPROM Write protected
#pragma config WRTSAF = ON     // SAF Write protection bit->SAF Write Protected
#pragma config WRTAPP = ON     // Application Block write protection bit->Application Block write protected

//CONFIG10
#pragma config CPD = ON     // Data EEPROM Code Protection bit->Data EEPROM code protection enabled

//CONFIG11
#pragma config CP = ON     // PFM Code Protection bit->PFM code protection enabled

/**
 End of File
*/