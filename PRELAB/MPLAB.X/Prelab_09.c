/* 
 * File:   Prelab_09.c
 * Author: pablo
 *
 * Created on 20 de abril de 2023, 5:14 p.m.
 */

// PIC16F887 Configuration Bit Settings
// 'C' source line config statements
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF       // RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
#pragma config CP = OFF          // Code Protection bit (Program memory code protection is enabled)
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


/**********LIBRERIAS**********/

#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <pic16f887.h>

/**********DEFINIR CONSTANTES**********/

#define _XTAL_FREQ 8000000

/**********VARIABLES GLOBALES**********/

uint8_t cont_sleep;
uint8_t sleeping = 0; //Se inicializa en 0 para que siempre este convirtiendo el ADC

/**********PROTOTIPOS**********/
void setup(void);

/**********INTERRUPCIONES**********/
void __interrupt() isr(void)
{
    if (ADIF)   //Interrupcion del ADC
    { 
        // Canal 0
        PORTD = ADRESH;
        PIR1bits.ADIF = 0;
    }
    if  (RBIF)    //PORTB
    { 
            if (PORTBbits.RB0 == 0){  //----> RB0 sleep
                sleeping = 1;
                SLEEP();
            }
            else if (PORTBbits.RB1 == 0){
                sleeping = 0;
                __delay_ms(1000);
            }
            else if (PORTBbits.RB2 == 0){
                sleeping = 0;
                __delay_ms(1000);
            }
        INTCONbits.RBIF = 0;
    }
    return;
}

/**********CÓDIGO PRINCIPAL**********/
void main(void) 
{
    setup();
    while(1)   //loop principal
    { 
        if (sleeping == 0){
            if (ADCON0bits.GO == 0){
                ADCON0bits.GO = 1;      //Iniciar a convertir
                __delay_ms(2);
            }
            PORTE = cont_sleep;
            cont_sleep ++;
            if (cont_sleep>7)
            cont_sleep = 0;
        } 
    }
}



/**********FUNCIONES**********/

void setup(void)
{
    //configuracion de entradas y salidas
    ANSELH = 0;
    
    TRISB = 0b111;  // Input los primeros 3 bits del PORTB
    TRISC = 0;
    TRISD = 0;
    TRISE = 0;
    
    PORTA = 0;
    PORTC = 0;
    PORTD = 0;
    PORTE = 0;
        
    //Configuracion PULL-UP
    OPTION_REGbits.nRBPU = 0;   //0 = PORTB pull-ups are enabled by individual PORT latch values
    WPUB = 0b111;   // Pull-up Enable para los primeros 3 bits del PORTB
    IOCB = 0b111; //Interrupt-on-change enabled para los primeros 3 bits del PORTB

    OSCCONbits.IRCF = 0b111;    //8MHz
    OSCCONbits.SCS = 1;         //Utilización del oscilador Interno
    
    //Configuración de las interrupciones
    PIR1bits.ADIF = 0;   // limpiar la bandera de interrupcion del ADC
    PIE1bits.ADIE = 1;   // habilitar interrupciones de ADC
    INTCONbits.GIE = 1;  // Usualmente la global se enciende de [ultimo]
    INTCONbits.PEIE = 1; // habilitar interrupciones perifericas
    INTCONbits.RBIF = 0; // Limpiar la bandera de interrupcion del PORTB
    INTCONbits.RBIE = 1; // Habilitar la interrupcion del PORTB
    
    //Configuracion ADC
    ANSEL = 0b00000001;
    TRISA = 0b00000001;
    ADCON0bits.ADCS = 0b10;     //Conversion Clock
    ADCON0bits.CHS = 0b0000;    //Canal 0
    ADCON1bits.ADFM = 0;    //justificado a la izquierda
    ADCON1bits.VCFG0 = 0;   //Vdd
    ADCON1bits.VCFG1 = 0;   //Vss
    ADCON0bits.ADON = 1;    //ADC enable
    ADIF =  0;
    __delay_ms(5);
    
   
    return;
}
    