/* 
 * File:   LAB_09.c
 * Author: pablo
 *
 * Created on 24 de abril de 2023, 5:49 p.m.
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

uint8_t ValPot = 0;
uint8_t address = 0x04; // Direccion de los datos en EEPROM

/**********PROTOTIPOS**********/
void setup(void);
void write_EEPROM(uint8_t address, uint8_t data); //Escritura de EEPROM
uint8_t read_EEPROM(uint8_t address); //lectura del EEPROM


/**********INTERRUPCIONES**********/
void __interrupt() isr(void){
    
    if (PIR1bits.ADIF){ // Interrupción del ADC
        if (ADCON0bits.CHS == 0){
            ValPot = ADRESH; 
            PORTC = ValPot; 
        }
        PIR1bits.ADIF = 0; //Se limpia la bandera de Interrupción del ADC
        
    }
    
    if (INTCONbits.RBIF){//Interrupción del PORTB
        if (PORTBbits.RB1 == 0){    //Modo ACTIVO
            sleeping = 0; //Apagar la bandera sleeping
        }
        
        else if (PORTBbits.RB0 == 0){
            sleeping = 1; //Encender la bandera sleeping
            SLEEP(); //Modo Sleep del PIC16f887
        }
        
        else if (PORTBbits.RB2 == 0){
            sleeping = 0 ; //Apagar la bandera sleeping
            write_EEPROM(address, ValPot); //Escribir el valor del potenciómetro en la EEPROM
        }
        INTCONbits.RBIF = 0; 

    }
    return;
    
}


/**********CÓDIGO PRINCIPAL**********/

void main(void){
    setup();
    
    while(1){
      
        if (sleeping == 0){//revisar si el pic esta en modo sleep
            if (ADCON0bits.GO == 0){//Revisión si no está funcionando el ADC
                ADCON0bits.GO = 1; //iniciar la conversion del ADC.
                __delay_us(40);
            }
            PORTE = cont_sleep; //Contador mientras no está en modo SLEEP
            cont_sleep ++;
            if (cont_sleep>7)
            cont_sleep = 0; 
        }
        PORTD = read_EEPROM(address); //Mostrar el valor guardado en la dirección establecida de la EEPROM
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

void write_EEPROM(uint8_t address, uint8_t data){
    while (WR){;}
    EEADR = address;//Dirección de memoria a Escribir
    EEDAT = data;   //Dato a escribir
    
    EECON1bits.EEPGD = 0; //Acceso a memoria de datos en la EEPROM
    EECON1bits.WREN = 1; // Habilitacion de escritura en la EEPROM
    INTCONbits.GIE = 0; //deshabilita las interrupciones 
    
    //Secuencia de escritura
    EECON2 = 0x55;
    EECON2 = 0xAA;
    EECON1bits.WR = 1; //Iniciamos la Escritura
    EECON1bits.WREN = 0; //Deshabilitamos la escritura en la EEPROM
    
    INTCONbits.RBIF = 0; // limpiamos bandera en el puerto b
    INTCONbits.GIE = 1; //habilita interrupciones globales 
            
}

uint8_t read_EEPROM(uint8_t address){
    while (WR||RD){;}   //Verifica WR, RD para saber si hay un proceso de escritura o lectura en proceso
    EEADR = address ;   //Asigna la dirección de memoria a leer
    EECON1bits.EEPGD = 0;//Leectrua a la EEPROM
    EECON1bits.RD = 1;  //Se obtien el dato de la EEPROM
    return EEDAT;       //Retorno del valor Leido
}