#include "config.h"

#define BSF(REG, VAL)  (REG) |=  (1 << (VAL))
#define BCF(REG, VAL)  (REG) &= ~(1 << (VAL))
#define BTF(REG, VAL) ((REG) &   (1 << (VAL)))

#define GRN_ON   BCF(LATB, 7)
#define GRN_OFF  BSF(LATB, 7)
#define GDRV_ON  BSF(LATC, 6)
#define GDRV_OFF BCF(LATC, 6)

void fvr_init(void)
{
    FVRCON = (1 << 7) |                 // FVR is enabled
             (0b11 << 2) |              // FVR Buffer 2 is 4.096 V
             (0b11 << 0);               // FVR Buffer 1 is 4.096 V
    while ((FVRCON & (1 << 6)) == 0);   // Wait for reference to stabilize
}

void dac1_init(void)
{
    BSF(TRISA,  2);                     // Digital output buffer disabled on RA2
    BSF(ANSELA, 2);                     // Digital input buffer disabled on RA2

    DAC1CON = (1    << 7) |             // DAC is enabled
              (0b10 << 4) |             // DAC1OUT is enabled on RA2
              (0b10 << 2) |             // PSS is connected to FVR Buffer 2
              (0    << 0);              // NSS is connected to VSS
}

void dac1_write(uint8_t data)
{
    DAC1DATL = data;
}

void dac2_init(void)
{
    DAC2CON = (1    << 7) |             // DAC is enabled
              (0b00 << 4) |             // DAC2OUT is disabled
              (0b10 << 2) |             // PSS is connected to FVR Buffer 2
              (0    << 0);              // NSS is connected to VSS
}

void dac2_write(uint8_t data)
{
    DAC2DATL = data;
}

/*

// void clc1_init(void)
// {
//     CLCSELECT = 0;              // Select CLC1
//     CLCnPOL = 0;                // Gates are not inverted
//     CLCnSEL0 = 0b00010100;      // D1S MUX selects TMR2
//     CLCnSEL1 = 0b00010100;      // D2S MUX selects TMR2
//     CLCnSEL2 = 0b00100000;      // D3S MUX selects CMP1 (ISNS comparator) output
//     CLCnSEL3 = 0b00100001;      // D4S MUX selects CMP2 (VSNS comparator) output
//     CLCnGLS0 = 0b00000010;      // lcxg1 selects input from D1S MUX (non-inverted)
//     CLCnGLS1 = 0b00000000;      // lcxg2 is not used
//     CLCnGLS2 = 0b00010000;      // lcxg3 selects input from D3S MUX (inverted)
//     CLCnGLS3 = 0b01000000;      // lcxg4 selects input from D4S MUX (inverted)
//     // lcxg1 OR lcxg2 --> S of SR latch
//     // lcxg3 OR lcxg4 --> R of SR latch
//     // BSF(ANSELF, 7);             // Disable digital input buffer on RF7 (GDRV)
//     // BCF(TRISF, 7);              // Set RF7 as digital output
//     // RF7PPS = 0x01;              // Set RF7 to CLC1OUT
//     // CLCIN0PPS = 0b00000110;     // CLCIN0PPS is connected to RA6 (VDRN_CMP)
//     // CLCIN1PPS = 0b00000111;     // CLCIN1PPS is connected to RA7 (ISNS_CMP)
//     // CLCIN4PPS = 0b00000101;     // CLCIN4PPS is connected to RA5 (VSNS_CMP)
//     // BCF(ANSELA, 5);             // Enable digital input buffer on RA5 (VSNS_CMP)
//     // BCF(ANSELA, 6);             // Enable digital input buffer on RA6 (VDRN_CMP)
//     // BCF(ANSELA, 7);             // Enable digital input buffer on RA7 (ISNS_CMP)
//     // BSF(TRISA, 5);              // Disable output buffer on RA5
//     // BSF(TRISA, 6);              // Disable output buffer on RA6
//     // BSF(TRISA, 7);              // Disable output buffer on RA7
//     CLCnCON = (1     << 7) |    // CLC is enabled
//               (0b011 << 0);     // Cell is SR latch
// }

// void clc2_init(void)
// {
//     CLCSELECT = 1;              // Select CLC2
//     CLCnPOL = 0;                // Gates are not inverted
//     CLCnSEL0 = 0b00100000;      // D1S MUX selects CMP1 (ISNS comparator) output
//     CLCnSEL1 = 0b00100101;      // D2S MUX selects CLC1 outupt
//     CLCnSEL2 = 0b00100101;      // D3S MUX selects CLC1 output
//     CLCnSEL3 = 0b00001101;      // D4S MUX selects SFINTOSC (1 MHz)
//     CLCnGLS0 = 0b10000000;      // lcxg1 selects input from D4S MUX (non-inverted)
//     CLCnGLS1 = 0b00000000;      // lcxg2 is not used
//     CLCnGLS2 = 0b00000001;      // lcxg3 selects input from D1S MUX (inverted)
//     CLCnGLS3 = 0b00001000;      // lcxg4 selects input from D2S MUX (non-inverted)
//     // lcxg2 OR lcxg4 --> D   of D latch
//     // lcgx1          --> CLK of D latch
//     // lcgx3          --> R   of D latch
//     BSF(ANSELF, 7);             // Disable digital input buffer on RF7 (GDRV)
//     BCF(TRISF, 7);              // Set RF7 as digital output
//     RF7PPS = 0x02;              // Set RF7 to CLC2OUT
//     // CLCIN0PPS = 0b00000110;     // CLCIN0PPS is connected to RA6 (VDRN_CMP)
//     // CLCIN1PPS = 0b00000111;     // CLCIN1PPS is connected to RA7 (ISNS_CMP)
//     // CLCIN4PPS = 0b00000101;     // CLCIN4PPS is connected to RA5 (VSNS_CMP)
//     // BCF(ANSELA, 5);             // Enable digital input buffer on RA5 (VSNS_CMP)
//     // BCF(ANSELA, 6);             // Enable digital input buffer on RA6 (VDRN_CMP)
//     // BCF(ANSELA, 7);             // Enable digital input buffer on RA7 (ISNS_CMP)
//     // BSF(TRISA, 5);              // Disable output buffer on RA5
//     // BSF(TRISA, 6);              // Disable output buffer on RA6
//     // BSF(TRISA, 7);              // Disable output buffer on RA7
//     CLCnCON = (1     << 7) |    // CLC is enabled
//               (0b101 << 0);     // Cell is 2-input D-FF with S and R
// }



// void adc_init(void)
// {
//     ADCLK = 31;                 // ADC operates at 1 MHz
//     ADREF = 3;                  // Connect ADC negative reference to VSS, positive reference to FVR
//     ADPCH = 0b00101000;         // RF0 selected for PCH
//     //ADNCH
//     ADPRE = 100;                // Pre-charge time is 100 ADC clocks
//     ADACQ = 100;                // Acquisition time is 100 ADC clocks
//     ADCAP = 0;                  // No additional capacitance
//     BSF(ADCP, 7);               // ADC charge pump may be used when requested by the ADC
//     BCF(PIR1, 2);               // Clear the ADC interrupt flag
//     BSF(PIE1, 2);               // Enable ADC interrupts
//     BSF(INTCON0, 7);            // GIE = 1
//     BSF(ADCON0, 7);             // ADC is enabled
// }

// void adc_start(void)
// {
//     BSF(ADCON0, 0);             // Set GO bit
// }

// void adc_wait_conversion_done(void)
// {
//     while (ADCON0 & 1);         // Wait while GO bit is set
// }

// uint16_t adc_get_res(void)
// {
//     return (uint16_t)((ADRESH << 8) | ADRESL); // Return ADC result (lower 12 bits)
// }

*/

void interrupt_init(void)
{
    BSF(INTCON0, 5);                    // Enable priority levels on interrupts
    BCF(INTCON0, 7);                    // GIE = 0

    IVTLOCK = 0x55;
    IVTLOCK = 0xAA;
    IVTLOCKbits.IVTLOCKED = 0x00;       // unlock IVT

    IVTBASEU = 0;                       // set default location of IVT base (0x0008)
    IVTBASEH = 0;
    IVTBASEL = 8;

    IVTLOCK = 0x55;
    IVTLOCK = 0xAA;
    IVTLOCKbits.IVTLOCKED = 0x01;       // lock IVT

    BSF(INTCON0, 7);                    // GIE = 1
}

void vsns_ref(float vout)
{
    // From schematic: G = 0.0045 V/V
    // VDAC = VOUT * 0.0045, DAC1DAT = VDAC * (255 / 4.096)
    // DAC1DAT = (VOUT * 0.0045) * (255 / 4.096)
    //         = VOUT * 0.2801513672
    dac1_write((uint8_t)(vout * 0.2801513672));
}

void isns_ref(float ipri)
{
    // From schematic: G = 0.077 V/A
    // VDAC = IPRI * 0.077, DAC2DAT = VDAC * (255 / 4.096)
    // DAC2DAT = (IPRI * 0.077) * (255 / 4.096)
    //         = 4.793701172
    dac2_write((uint8_t)(ipri * 4.793701172));
}

/*

// void tmr1_init(void)
// {
//     BCF(INTCON0, 7);            // GIE = 0
//     TMR1H = 0;                  // Clear high byte first
//     TMR1L = 0;                  // Then clear low byte
//     T1GCON = 0;                 // Gate disabled
//     T1GATE = 0;                 // Gate disabled
//     T1CLK = 0b00010;            // Clock source is Fosc
//     BCF(PIR3, 4);               // Clear TMR1 interrupt flag
//     BSF(PIE3, 4);               // TMR1 interrupt enable
//     BSF(IPR3, 1);               // TMR1 interrupt is high priority
//     BSF(INTCON0, 7);            // GIE = 1
//     T1CON = (0b00 << 4) |       // 1:1 prescaler
//             (1 << 0);           // Enables timer
// }

// void tmr2_init(void)
// {
//     T2TMR = 0;                  // Reset timer register
//     T2PR = 160;                 // Timer reaches this value before resetting
//     T2HLT = 0;                  // Timer external reset is not used
//     T2CLKCON = 0b00001;         // Clock source is Fosc/4
//     T2RST = 0;
//     T2CON = (1 << 7) |          // Timer is on
//             (0 << 4) |          // 1:1 prescaler
//             (0 << 0);           // 1:1 postscaler
// }

*/

void cmp1_init()
{
    // ISNS comparator
    // DAC2 output can only route to the P channel
    // N channel routes to output of CSA
    // ISNS amp output connected to RC1 (C1IN1-)
    CM1CON1 = 1;                        // Interrupt flag will be set upon a negative-going edge
    CM1NCH = 0b001;                     // N channel input is C1IN1-
    BSF(TRISC,  1);                     // Digital output buffer disabled on RC1
    BSF(ANSELC, 1);                     // Digital input buffer disabled on RC1
    CM1PCH = 0b101;                     // P channel input is DAC2 (ISNS reference DAC)
    BCF(PIR1, 4);                       // Reset comparator 1 interrupt flag
    BSF(PIE1, 4);                       // Comparator 1 interrupt enable
    BSF(INTCON0, 7);                    // GIE = 1
    CM1CON0 = (1 << 7) |                // Comparator on
              (0 << 4) |                // Output is not inverted
              (1 << 1) |                // Hysteresis enabled
              (0 << 0);                 // Asynchronous output
}

void cmp2_init(void)
{
    // VSNS comparator
    // DAC1 output routed physically to RC0 (C2IN0+)
    // N channel routes to output of FB divider
    // FB divider output connected to RC2 (C2IN2-)
    CM2CON1 = 0;                        // No interrupts needed
    CM2NCH = 0b010;                     // N channel input is C2IN2-
    BSF(TRISC,  2);                     // Digital output buffer disabled on RC2
    BSF(ANSELC, 2);                     // Digital input buffer disabled on RC2
    CM2PCH = 0b000;                     // P channel input is C2IN0+ (physically routed to VSNS reference DAC)
    BSF(TRISC, 0);                      // Digital output buffer disableed on RC0
    BSF(ANSELC, 0);                     // Digital input buffer disabled on RC0

    CM2CON0 = (1 << 7) |                // Comparator on
              (0 << 4) |                // Output is not inverted
              (1 << 1) |                // Hysteresis enabled
              (0 << 0);                 // Asynchronous output
}

void io_init(void)
{
    BCF(TRISB, 7);                      // Green LED
    BCF(ANSELB, 7);
    BCF(TRISC, 6);                      // Gate drive
    BCF(ANSELC, 6);
    GDRV_OFF;
    GRN_OFF;
}

void main(void)
{
    io_init();
    fvr_init();
    dac1_init();
    dac2_init();
    cmp1_init();
    cmp2_init();
    interrupt_init();
    __delay_ms(1000);                    // Small delay to wait for peripherals to initialize
    vsns_ref(50.0);
    isns_ref(10.0);

    GRN_ON;

    while (1)
    {
        if (CMOUT & (1 << 1)) GDRV_ON;
        __delay_us(100);
    }
    // BCF(TRISF, 5);
    // BCF(TRISF, 6);
    // BCF(TRISF, 7);
    // BCF(ANSELF, 5);
    // BCF(ANSELF, 6);
    // BCF(ANSELF, 7);
    // GRN_OFF;
    // RED_OFF;
    // GDRV_OFF;
    
    // fvr_init();

    // dac1_init();

    // dac2_init();
    // // opa1_init();
    
    // // dac3_init();
    // // opa2_init();

    // vsns_ref(100.0);
    // isns_ref(10.0);
    // // vdrn_ref();
    // // dac3_write((uint8_t)(3.0 * 255.0 / 4.096));

    // __delay_ms(500);

    // // BSF(TRISA, 5);   // VSNS_CMP = RA5, idle HIGH
    // // BSF(TRISA, 6);   // VDRN_CMP = RA6, idle LOW
    // // BSF(TRISA, 7);   // ISNS_CMP = RA7, idle HIGH
    // // BCF(ANSELA, 5);
    // // BCF(ANSELA, 6);
    // // BCF(ANSELA, 7);

    // // BCF(TRISF, 7);
    // // BCF(ANSELF, 7);
    // // BCF(LATF, 7);

    // interrupt_init();
    // // ioc_init();
    // // tmr1_init();
    // tmr2_init();
    // clc1_init();
    // clc2_init();
    // cmp1_init();
    // cmp2_init();
    // // adc_init();

    // while (1)
    // {
    //     BSF(LATF, 6);
    //     __delay_ms(100);
    //     BCF(LATF, 6);
    //     __delay_ms(100);
    // }
}

// #define TSTARTH 0x00
// #define TSTARTL 0x00

void __interrupt(irq(CMP1), base(8)) CMP1_ISR(void)
{
    if (BTF(PIR1, 4))                   // ISNS_CMP negative edge (xfmr current limit reached)
    {
        GDRV_OFF;                       // Turn off MOSFETs
        BCF(PIR1, 4);                   // Reset comparator 1 interrupt flag
        // TMR1H = TSTARTH;             // Reset timer (high byte first)
        // TMR1L = TSTARTL;             // Then low byte
    }
}

// void __interrupt(irq(TMR1), base(8)) TMR1_ISR(void)
// {
//     BCF(PIR3, 4);               // Clear TMR1 interrupt flag
//     TMR1H = TSTARTH;
//     TMR1L = TSTARTL;
// }

// void __interrupt(irq(ADCH1), base(8)) ADCH1_ISR(void)
// {
//     PIR1bits.ADCH1IF = 0; // Clear the ADC interrupt flag
// }