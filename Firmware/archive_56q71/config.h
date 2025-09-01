#include <xc.h>

// Used for _delay_ms()
#define _XTAL_FREQ 64000000UL

// CONFIG1
#pragma config RSTOSC = HFINTOSC_64MHZ
#pragma config FEXTOSC = OFF

// CONFIG2
#pragma config FCMENS = OFF
#pragma config FCMENP = OFF
#pragma config FCMEN = OFF
#pragma config CSWEN = OFF
#pragma config BBEN = OFF
#pragma config PR1WAY = ON
#pragma config CLKOUTEN = OFF

// CONFIG3
#pragma config BOREN = SBORDIS
#pragma config LPBOREN = OFF
#pragma config IVT1WAY = ON
#pragma config MVECEN = ON
#pragma config PWRTS = PWRT_OFF
#pragma config MCLRE = EXTMCLR

// CONFIG4
#pragma config XINST = 1 // Extended instruction set enabled when 0
#pragma config DEBUG = 1 // Debugger enabled when 0
#pragma config LVP = ON
#pragma config STVREN = ON
#pragma config PPS1WAY = ON
#pragma config ZCD = 1 // ZCD enabled when 0
#pragma config BORV = VBOR_1P9

// CONFIG5
#pragma config WDTE = OFF
#pragma config WDTCPS = WDTCPS_31

// CONFIG6
#pragma config WDTCCS = SC
#pragma config WDTCWS = WDTCWS_7

// CONFIG7
#pragma config BBSIZE = BBSIZE_128

// CONFIG8
#pragma config SAFSZ = SAFSZ_NONE

// CONFIG9
#pragma config WRTAPP = OFF
#pragma config WRTSAF = OFF
#pragma config WRTD = 0 // EEPROM write protection is enabled when 0
#pragma config WRTC = 0 // FLASH write protection is enabled when 0
#pragma config WRTB = 0 // Boot block write protection is enabled when 0

// CONFIG10
#pragma config CPD = 0 // EEPROM code protection is enabled when 0

// CONFIG11
#pragma config CP = 0 // FLASH code protection is enabled when 0