#include "config.h"

#define BSF(REG, VAL) REG |=  (1 << VAL)
#define BCF(REG, VAL) REG &= ~(1 << VAL)

void fvr_init(void)
{
    FVRCON = (1 << 7) |      // FVR is enabled
             (0b11 << 2) |   // FVR Buffer 2 is 4.096 V
             (0b11 << 0);    // FVR Buffer 1 is 4.096 V
    while ((FVRCON & (1 << 6)) == 0); // Wait for reference to stabilize
}

void dac1_init(void)
{
    BSF(TRISA,  2);          // Digital output buffer disabled on RA2
    BSF(ANSELA, 2);          // Digital input buffer disabled on RA2

    DAC1CON = (1    << 7) |  // DAC is enabled
              (0b01 << 4) |  // DAC1OUT is enabled on RA2
              (0b10 << 2) |  // PSS is connected to FVR Buffer 2
              (0    << 0);   // NSS is connected to VSS
}

void dac1_write(uint16_t data)
{
    DAC1DAT = data & 0xFFF;
}

void dac2_init(void)
{
    DAC2CON = (1    << 7) |  // DAC is enabled
              (0b00 << 4) |  // DAC2OUT is disabled
              (0b10 << 2) |  // PSS is connected to FVR Buffer 2
              (0    << 0);   // NSS is connected to VSS
}

void dac2_write(uint8_t data)
{
    DAC2DATL = data;
}

void dac3_init(void)
{
    BSF(TRISA,  2);          // Digital output buffer disabled on RA2
    BSF(ANSELA, 2);          // Digital input buffer disabled on RA2

    DAC3CON = (1    << 7) |  // DAC is enabled
              (0b00 << 4) |  // DAC3OUT is disabled
              (0b10 << 2) |  // PSS is connected to FVR Buffer 2
              (0    << 0);   // NSS is connected to VSS
}

void dac3_write(uint8_t data)
{
    DAC3DATL = data;
}

void opa1_init(void)
{
    OPA1CON0 = (1 << 7) |    // Operational amplifier is enabled
               (1 << 5) |    // Charge pump on
               (1 << 3);     // Unity gain
    OPA1CON1 = 0;            // Not used
    OPA1CON2 = 0b101;        // PCH is connected to DAC2OUT
    OPA1CON3 = (0 << 5);     // OPA1OUT is enabled
    OPA1CON4 = 0;            // Not used
    OPA1HWC  = (0 << 7);     // HW override control disabled

    BSF(TRISA,  1);          // Digital output buffer disabled on RA1
    BSF(ANSELA, 1);          // Digital input buffer disabled on RA1
}

void opa2_init(void)
{
    OPA2CON0 = (1 << 7) |    // Operational amplifier is enabled
               (1 << 5) |    // Charge pump on
               (1 << 3);     // Unity gain
    OPA2CON1 = 0;            // Not used
    OPA2CON2 = 0b110;        // PCH is connected to DAC3OUT
    OPA2CON3 = (0 << 5);     // OPA2OUT is enabled
    OPA2CON4 = 0;            // Not used
    OPA2HWC  = (0 << 7);     // HW override control disabled

    BSF(TRISB,  1);          // Digital output buffer disabled on RB1
    BSF(ANSELB, 1);          // Digital input buffer disabled on RB1
}

void clc_init(void)
{
    CLCSELECT = 0;           // Select CLC1
    CLCnCON = (1     << 7) | // CLC is enabled
              (0b011 << 0);  // Cell is SR latch
    CLCnPOL = 0;             // Gates are not inverted
    CLCnSEL0 = 0b00100101;   // Input is CLC1
    CLCnSEL1 = 0b00100110;   // Input is CLC2
    CLCnSEL2 = 0b00101111;   // Input is CLC3
    CLCnSEL3 = 0b00110000;   // Input is CLC4
    CLCnGLS0 = 0b00000010;   // Input from CLC1 is OR'ed (non-negated) into S of SR latch
    CLCnGLS1 = 0b00000000;   // Not used
    CLCnGLS2 = 0b00001000;   // Input from CLC2 is OR'ed (non-negated) into R of SR latch
    CLCnGLS3 = 0b00100000;   // Input from CLC3 is OR'ed (non-negated) into R of SR latch
}

void adc_init(void)
{
    ADCON0
    ADCON1
    ADCON2
    ADCON3
    ADCLK
    ADREF
    ADPCH
    ADNCH
    ADPRE
    ADACQ
    ADCAP
    ADRPT
    ADCNT
    ADFLTR
    ADRES
    ADPREV
    ADACC
    ADSTPT
    ADERR
    ADCP
    ADLTH
    ADUTH
    ADACT
    ADCTX
    ADCSELx
    ADCGxA
    ADCGxB
    ADCGxC
    ADCGxD
    ADCGxE
    
}

void main(void)
{

}