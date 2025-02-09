#include "config.h"

#define BSF(REG, VAL) REG |=  (1 << VAL)
#define BCF(REG, VAL) REG &= ~(1 << VAL)

void dac_init(void)
{
    DAC1CON = (1    << 7) |  // DAC is enabled
              (0b00 << 4) |  // DACOUT is disabled
              (0b10 << 2) |  // PSS is connected to FVR Buffer 2
              (0    << 0);   // NSS is connected to VSS
}

void dac_write(uint16_t data)
{
    DAC1DAT = data & 0b1111111111;
}

void cmp1_init(void)
{
    CM1CON0 = (1    << 7) | // Comparator enabled
              (0    << 6) | // Comparator output is 0 if C1VP < C1VN
              (0    << 5) | // Output is not inverted
              (0b00 << 1) | // No hysteresis
              (0    << 0);  // Comparator output is asynchronous
    CM1CON1 = (0b11 << 2) | // Comparator operates in high-speed mode 1
              (0    << 1) | // No interrupt flag set on positive edge
              (0    << 0);  // No interrupt flag set on negative edge
    CM1NCH  = 0b011;        // NCH is connected to C1IN3- (RB1 ** TO BE REWORKED ON PCB **)
    CM1PCH  = 0b011;        // PCH is connected to DAC1OUT
}

void cmp2_init(void)
{
    CM2CON0 = (1    << 7) | // Comparator enabled
              (0    << 6) | // Comparator output is 0 if C2VP < C2VN
              (0    << 5) | // Output is not inverted
              (0b00 << 1) | // No hysteresis
              (0    << 0);  // Comparator output is asynchronous
    CM2CON1 = (0b11 << 2) | // Comparator operates in high-speed mode 1
              (0    << 1) | // No interrupt flag set on positive edge
              (0    << 0);  // No interrupt flag set on negative edge
    CM2NCH  = 0b000;        // NCH is connected to C2IN0-
    CM2PCH  = 0b000;        // PCH is connected to C2IN0+
}

void clc1_init(void)
{

}

void gsx_init(void)
{
    TRISA &= ~((0 << 4) | // GS0
               (0 << 5)); // GS1
}

void gsx_write(uint8_t data)
{
    data <<= 4;
    data &= 0b00110000;
    LATA &= 0b11001111;
    LATA |= data;
}



void main(void)
{
    BCF(TRISA, 4); // Set GS0 to output
    BCF(TRISA, 5); // Set GS1 to outupt
    BCF(LATA,  4);
    BCF(LATA,  5);

    BCF(TRISA, 1); // Set GDRV to output
    BCF(LATA,  1); // Set GDRV low

    BCF(TRISB, 2); // Set #FLT to output
    BCF(TRISB, 3); // Set #CHG to output

    


    while (1)
    {
        // BCF(LATB, 2);
        // __delay_ms(100);
        // BSF(LATB, 2);
        // __delay_ms(100);
        // BCF(LATB, 3);
        // __delay_ms(100);
        // BSF(LATB, 3);
        // __delay_ms(100);

        BSF(LATA, 1);
        __delay_us(4);
        BCF(LATA, 1);
        __delay_us(1);
    }

}