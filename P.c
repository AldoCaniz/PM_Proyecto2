/* 
 * File:   slave2.c
 * Author: Aldo Caniz
 *
 * Created on 17 de mayo de 2022, 8:31 PM
 */

// PIC16F887 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ 4000000
#define  _tmr0_value 206

//VARIABLES
uint8_t pot1, pot2, pot3, pot4, pot1_t, pot2_t, pot3_t, pot4_t;
uint8_t opcion_pot, opcion_pot_t;
uint8_t pot3_ant, pot4_ant;
uint8_t boton1 = 0, boton2 = 0, boton3 = 0;
uint8_t cont, ban;
uint8_t estado = 1;
unsigned char val1, val2, val3, val4;
unsigned char gpot1[3] = {0x03, 0x04, 0x05};
unsigned char gpot2[3] = {0x06, 0x07, 0x08};
unsigned char gpot3[3] = {0x09, 0x0A, 0x0B};
unsigned char gpot4[3] = {0x0C, 0x0D, 0x0E};

//FUNCIONES
void setup(void);
unsigned char mapeo(unsigned char pot);
void serial(void);
uint8_t readEEPROM(uint8_t address);
void writeEEPROM(uint8_t data, uint8_t address);
void EEPROM(void);
int spi_escritura(int data);
void escribir(unsigned char dato);

void __interrupt() isr(void){
    if (PIR1bits.ADIF){
        if (ADCON0bits.CHS == 0){
            CCPR1L = (ADRESH >> 1) + 35;
            CCP1CONbits.DC1B1 = ADRESH % 0b01;
            CCP1CONbits.DC1B0 = (ADRESH >> 7);
        }
        else if(ADCON0bits.CHS == 1){
            CCPR2L = (ADRESH >> 1) + 35;            
            CCP2CONbits.DC2B1 = ADRESH % 0b01;
            CCP2CONbits.DC2B0 = (ADRESH >> 7);
        }
        else if(ADCON0bits.CHS == 2){
            pot3 = mapeo(ADRESH);
        }
        else if(ADCON0bits.CHS == 3){
            pot4 = mapeo(ADRESH);
        }
        PIR1bits.ADIF = 0;
    }
    
    if (INTCONbits.T0IF){
        cont++;
        if (cont == pot3)
            PORTDbits.RD0 = 0;
        if (cont == pot4)
            PORTDbits.RD1 = 0;
        if (cont == 40){
            PORTDbits.RD0 = 1;
            PORTDbits.RD1 = 1;
            cont = 0;
        }
    }
    TMR0 = _tmr0_value;
    INTCONbits.T0IF = 0;
}

void main (void){
    setup();
    ADCON0bits.GO = 1;
    while(1){
        if (RB0 == 0){
            boton1 = 1;
        }
        if (RB0 == 1 && boton1 == 1){
            estado++;
            boton1 = 0;
        }
        switch(estado){
            case(1):
                PORTE = 1;
                ADCON0bits.ADON = 1;
                break;
            case(2):
                PORTE = 2;
                EEPROM();
                break;
            case(3):
                PORTE = 4;
                ADCON0bits.ADON = 0;
                if (PIR1bits.RCIF == 1)
                    serial();
            default:
                estado = 1;
                break;
        }
        __delay_us(50);
        
        if (ADCON0bits.GO == 0){
            switch (ADCON0bits.CHS){
                case (0):
                    __delay_us(50);
                    ADCON0bits.CHS = 1;
                    break;
                case (1):
                    __delay_us(50);
                    ADCON0bits.CHS = 2;
                    break;
                case (2):
                    __delay_us(50);
                    ADCON0bits.CHS = 3;
                    break;
                case (3):
                    __delay_us(50);
                    ADCON0bits.CHS = 0;
                    break;
            }
            __delay_us(50);
            ADCON0bits.GO = 1;
        }
    }
}

void setup(void){
    ANSEL = 0b00001111;
    ANSELH = 0;
    
    TRISA = 0b00001111;
    
    TRISD = 0;
    TRISE = 0;
    TRISDbits.TRISD0 = 0;
    TRISDbits.TRISD1 = 0;
    
    TRISB = 0B0111;
    
    OPTION_REGbits.nRBPU = 0;
    
    WPUB = 0b0111;
    
    PORTE = 0;
    PORTD = 0;
    PORTB = 0;
    PORTA = 0;
    PORTC = 0;
    
    OSCCONbits.IRCF = 0b0110;
    OSCCONbits.SCS = 1;
    
    OPTION_REGbits.T0CS = 0;
    OPTION_REGbits.T0SE = 0;
    OPTION_REGbits.PSA = 0;
    OPTION_REGbits.PS2 = 0;
    OPTION_REGbits.PS1 = 0;
    OPTION_REGbits.PS0 = 0;
    TMR0 = _tmr0_value;
    
    INTCONbits.T0IF = 0;
    INTCONbits.T0IE = 1;
    
    ADCON0bits.ADCS = 0b10;
    ADCON0bits.CHS = 0;
    ADCON1bits.ADFM = 0;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    __delay_us(50);
    
    TRISCbits.TRISC2 = 1;
    TRISCbits.TRISC1 = 1;
    PR2 = 155;
    CCP1CONbits.P1M = 0;
    CCP1CONbits.CCP1M = 0b1100;
    CCP2CONbits.CCP2M = 0b1100;
    
    CCPR1L = 0x0F;
    CCP1CONbits.DC1B = 0;
    
    CCPR2L = 0x0F;
    CCP2CONbits.DC2B0 = 0;
    
    PIR1bits.TMR2IF = 0;
    T2CONbits.T2CKPS = 0b11;
    
    T2CONbits.TMR2ON = 1;
    
    while (PIR1bits.TMR2IF == 0);
    PIR1bits.TMR2IF = 0;
    
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC1 = 0;
    
    PIR1bits.RCIF = 0;
    PIE1bits.RCIE = 0;
    
    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 1;
    INTCONbits.PEIE = 1;
    INTCONbits.GIE = 1;
    
    __delay_us(50);
    ADCON0bits.ADON = 1;   
    
    return;
}

unsigned char mapeo(unsigned char pot){
    unsigned char tmr0reset;
    if (pot <= 13)
        tmr0reset = 5;
    else if (pot > 13 && pot <= 26)
        tmr0reset = 6;
    else if (pot > 26 && pot <= 39)
        tmr0reset = 7;
    else if (pot > 39 && pot <= 52)
        tmr0reset = 8;
    else if (pot > 52 && pot <= 65)
        tmr0reset = 9;
    else if (pot > 65 && pot <= 78)
        tmr0reset = 10;
    else if (pot > 78 && pot <= 91)
        tmr0reset = 11;
    else if (pot > 91 && pot <= 104)
        tmr0reset = 12;
    else if (pot > 104 && pot <= 117)
        tmr0reset = 13;
    else if (pot > 117 && pot <= 130)
        tmr0reset = 14;
    else if (pot > 130 && pot <= 143)
        tmr0reset = 15;
    else if (pot > 143 && pot <= 156)
        tmr0reset = 16;
    else if (pot > 156 && pot <= 169)
        tmr0reset = 17;
    else if (pot > 169 && pot <= 182)
        tmr0reset = 18;
    else if (pot > 195 && pot <= 208)
        tmr0reset = 19;
    else if (pot > 208 && pot <= 221)
        tmr0reset = 20;
    else if (pot > 221 && pot <= 234)
        tmr0reset = 21;
    else if (pot > 247 && pot <= 255)
        tmr0reset = 22;
    else if (pot > 255)
        tmr0reset = 23;
    
    return tmr0reset;
}

void serial(void){
    opcion_pot = RCREG;
    
    switch(opcion_pot){
        case(1):
            opcion_pot_t = 1;
            break;
        case(2):
            opcion_pot_t = 2;
            break;
        case(3):
            opcion_pot_t = 3;
            break;
        case(4):
            opcion_pot_t = 4;
            break;
    }
    
    if(opcion_pot_t == 1){
        while(!PIR1bits.RCIF){};
        pot1_t = RCREG;
        
        CCPR1L = pot1_t;
        opcion_pot = 0;
    }
    if(opcion_pot_t == 2){
        while(!PIR1bits.RCIF){};
        pot2_t = RCREG;
        
        CCPR2L = pot2_t;
        opcion_pot = 0;
    }
    if(opcion_pot_t == 3){
        while(!PIR1bits.RCIF){};
        pot3_t = RCREG;
        pot3 = pot3_t;
        opcion_pot = 0;
    }
    if(opcion_pot_t == 4){
        while(!PIR1bits.RCIF){};
        pot4_t = RCREG;
        pot4 = pot4_t;
        opcion_pot = 0;
    }
}

void writeEEPROM(uint8_t data, uint8_t address){
    EEADR = address;
    EEDAT = data;
    
    EECON1bits.EEPGD= 0;
    EECON1bits.WREN = 1;
    
    INTCONbits.GIE = 0;
    
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1;
    
    while(PIR2bits.EEIF == 0);
    PIR2bits.EEIF = 0;
    
    INTCONbits.GIE = 1;
    EECON1bits.WREN = 0;
    return;
}

uint8_t readEEPROM(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0;
    EECON1bits.RD = 1;
    uint8_t data = EEDAT;
    return data;
}

void EEPROM(void){
    if(PORTBbits.RB1 == 0)
        boton2 = 1;
    if(PORTBbits.RB1 == 1 && boton2 == 1){
        PORTAbits.RA6 = 1;
        PORTAbits.RA7 = 0;
        ADCON0bits.ADON = 1;
        writeEEPROM(CCPR1L, gpot1[0]);
        writeEEPROM(CCPR2L, gpot2[0]);
        writeEEPROM(pot3, gpot3[0]);
        writeEEPROM(pot4, gpot4[0]);
        __delay_ms(500);
        boton2 = 0;
    }
    if (PORTBbits.RB2 == 0)
        boton3 = 1;
    if (PORTBbits.RB2 == 1 && boton3 == 1){
        PORTAbits.RA6 = 0;
        PORTAbits.RA7 = 1;
        ADCON0bits.ADON = 0;
        val1 = readEEPROM(gpot1[0]);
        val2 = readEEPROM(gpot2[0]);
        val3 = readEEPROM(gpot3[0]);
        val4 = readEEPROM(gpot4[0]);
        CCPR1L = val1;
        CCPR2L = val2;
        pot3 = val3;
        pot4 = val4;
        boton3 = 0;
    }
}

int spi_escritura(int data){
    PORTCbits.RC2 = 0;
    escribir(data);
    PORTCbits.RC2 = 1;
}

void escribir(unsigned char dato){
    SSPBUF = dato;
    while(SSPSTATbits.BF == 0);
    while(PIR1bits.SSPIF == 0);
    PIR1bits.SSPIF = 0;
}