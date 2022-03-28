/* 
 * File:   main.c
 * Author: María José
 *
 * Created on 19 de febrero de 2022, 07:43 PM
 */

#include <stdio.h>
#include <stdlib.h>

#include "p30F4013.h"

//_FOSC(CSW_FSCM_OFF & FRC); 
#pragma config FOSFPR = FRC             // Oscillator (Internal Fast RC (No change to Primary Osc Mode bits))
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)
/********************************************************************************/
/* SE DESACTIVA EL WAvoid habilitarPerifericos()TCHDOG														*/
//_FWDT(WDT_OFF); 
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)
/********************************************************************************/
//_FBORPOR( PBOR_ON & BORV27 & PWRT_16 & MCLR_EN ); 
// FBORPOR
#pragma config FPWRT  = PWRT_16          // POR Timer Value (16ms)
#pragma config BODENV = BORV20           // Brown Out Voltage (2.7V)
#pragma config BOREN  = PBOR_ON          // PBOR Enable (Enabled)
#pragma config MCLRE  = MCLR_EN          // Master Clear Enable (Enabled)
/********************************************************************************/
//_FGS(CODE_PROT_OFF)
// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

/********************************************************************************/
/* SECCI?N DE DECLARACI?N DE CONSTANTES CON DEFINE								*/
/********************************************************************************/
#define EVER 1
//#define MUESTRAS 64

/********************************************************************************/
/* DECLARACIONES GLOBALES														*/
/********************************************************************************/
/*DECLARACI?N DE LA ISR DEL TIMER 1 USANDO __attribute__						*/
/********************************************************************************/
void __attribute__((__interrupt__)) _U2RXInterrupt( void );


//****Comandos AT****
//Para configurar el env?o de un mensaje de bienvenida cuando est? operacional
//char CMD_CSGT[] = "1,\"ready\"\r";

//char CMD_CREG[] = "AT+CREG?\r";
char CMD_AT[] = "AT\r";
char CMD_ATE0[] = "ATE1\r";
char CMD_AT_CMGF[] = "AT+CMGF=1\r";
char CMD_AT_CMGS[] = "AT+CMGS=\"#phoneNumber\"\r";
char CMD_MENSAJE[] = "Mensaje enviado desde IoT\x1A\r";

char respuestaGSM[40];
unsigned char j;

char count;

void configurarPuertos();  
void configurarUART1();
void configurarUART2();
void configurarInterrupciones();
void habilitarPerifericos();

void iniIoT_BG96();
void enviarComandoAT(char comando[]);

void printUART1(char* cadena);
void printUART2(char* cadena);

extern void RETARDO_100ms();
extern void RETARDO_1s();
extern void RETARDO_50us();

int main(void) 
{    
    configurarPuertos();
    configurarUART1();
    configurarUART2();
    configurarInterrupciones();
    habilitarPerifericos();

    printUART1("\r\n inicializando... \r\n");
    
    iniIoT_BG96();
       
    enviarComandoAT(CMD_AT);
    enviarComandoAT(CMD_ATE0);
    enviarComandoAT(CMD_AT_CMGF);
    enviarComandoAT(CMD_AT_CMGS);
    enviarComandoAT(CMD_MENSAJE);
    
    printUART1("\r\n Mensaje enviado \r\n");
    
    for(;EVER;)
    {
        asm("nop");
    }
    
    return 0;
}

void habilitarPerifericos()
{
    //Para UART1
	U1MODEbits.UARTEN=1;    //UART1 Enable bit: 1 para habilitar
    U1STAbits.UTXEN=1;      //Transmit Enable bit: 1 para habilitar
    asm("nop");
    //Para UART2
    U2MODEbits.UARTEN=1;    //UART2 Enable bit: 1 para habilitar
    U2STAbits.UTXEN=1;      //Transmit Enable bit: 1 para habilitar
    asm("nop");
}

void configurarPuertos()
{
/** RD1 : SALIDA,   CONECTADO CON PWRKEY
 *  RD3 : ENTRADA,  CONECTADO CON RI
 *  RD9 : ENTRADA,  CONECTADO CON CTS
 */    
    PORTD = 0;
    asm("nop");
    LATD = 0;
    asm("nop");
    TRISD = 0XFFFF;
    asm("nop");
    TRISDbits.TRISD1 = 0;
    asm("nop");
/** 
 *  RB0 : SALIDA,   CONECTADO A LED
 *  RB5 : ENTRADA,  CONECTADO CON STATUS
 *  RB8 : SALIDA,   CONECTADO CON RTS
 */        
    PORTB = 0;
    asm("nop");
    LATB = 0;
    asm("nop");
    TRISB = 0XFFFF;
    asm("nop");
    TRISBbits.TRISB0 = 0;
    asm("nop");
    TRISBbits.TRISB8 = 0;
    asm("nop");
    ADPCFG = 0xFFFF;
/** RC13 : SALIDA,  U1ATX
 *  RC14 : ENTRADA, U1ARX
 *  SE USA UART1, PINES ALTERNOS, PARA COMUNICACI?N CON PC
 */        
    PORTC = 0;
    asm("nop");
    LATC = 0;
    asm("nop");
    TRISC = 0XFFFF;
    asm("nop");
    TRISCbits.TRISC13 = 0;    //U1ATX
/** RF4 :   ENTRADA,  U2RX
 *  RF5 :   SALIDA,   U2TX
 *  SE USA UART2, PARA COMUNICACI?N CON MODULO IoT
 */            
    PORTF = 0;
    asm("nop");
    LATF = 0;
    asm("nop");
    TRISF = 0XFFFF;
    asm("nop");
    TRISFbits.TRISF5 = 0;    //U2TX
    asm("nop");    
}
/**  @brief: CONFIGURACION DEL UART 1. EL UART 1 SE COMUNICA CON 
 *           EL FT232 PARA COMUNICACION CON PC
 *   @baudrate: 115200 BAUDIOS
 *   @frame: 8 BITS X DATO, SIN PARIDAD, 1 BIT DE PARO
 */
void configurarUART1()
{
    U1MODE = 0X0420;    //4 para poner ALTIO en 1: usar UxATX and UxARX I/O pins; 2 para Auto Baud Enable bit
    U1STA = 0X8000;     //8 para UTXISEL: Transmission Interrupt Mode Selection bit; 
                        //1 = Interrupt when a character is transferred to the Transmit Shift register and as result, the transmit buffer becomes empty
    U1BRG = 0;          //0 para 115200 baudios
}
/**  @brief: CONFIGURACION DEL UART 2. EL UART 2 CONFIGURA EL MODULO IoT 
 *           PARA ENVIO DE COMANDOS AT Y RECEPCION DE RESPUESTAS
 *   @baudrate: 115200 BAUDIOS
 *   @frame: 8 BITS X DATO, SIN PARIDAD, 1 BIT DE PARO
 */
void configurarUART2()
{
    U2MODE = 0X0020;    //No se utiliza ALTIO
    U2STA = 0X8000;
    U2BRG = 0;          //0 para 115200 baudios
}

void configurarInterrupciones()
{    
    IFS1bits.U2RXIF=0;    //UART2 Receiver Interrupt Flag Status bit
    IEC1bits.U2RXIE=1;    //UART2 Receiver Interrupt Enable bit (Interrupt request enabled)    
}

void iniIoT_BG96()
{
    LATBbits.LATB0 = 1;     //LED
    asm("nop");
    
    LATDbits.LATD1 = 0;     //PWK = 1
    asm("nop");
    RETARDO_100ms();
    
    LATDbits.LATD1 = 1;     //PWK = 0
    asm("nop");
    RETARDO_1s();
    
    LATDbits.LATD1 = 0;     //PWK = 1
    asm("nop");
    
//    while( !PORTBbits.RB5 ); //STAT = 1 ?
    
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();
    
    LATBbits.LATB0 = 0;     //LED
}


void enviarComandoAT(char comando[])
{    
    printUART2(comando);
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();


}

/********************************************************************************/
/* DESCRICION:	ISR (INTERRUPT SERVICE ROUTINE) DEL TIMER 1						*/
/* LA RUTINA TIENE QUE SER GLOBAL PARA SER UNA ISR								*/	
/* SE USA PUSH.S PARA GUARDAR LOS REGISTROS W0, W1, W2, W3, C, Z, N Y DC EN LOS */
/* REGISTROS SOMBRA																*/
/********************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt( void )
{
    LATBbits.LATB0 = 1;     //LED
    register short int resp;
    
    resp = U2RXREG;
    U1TXREG = resp;

    LATBbits.LATB0 = 0;     //LED
    IFS1bits.U2RXIF = 0;
}

void printUART1(char* cadena) 
{
    int i;
    
    IFS0bits.U1TXIF = 0;
    for(i = 0; cadena[i] != '\0'; i++) 
    {
        U1TXREG = cadena[i];
        
        //Mientras no se genere la interrupci?n, va a esperar
        while(!IFS0bits.U1TXIF);
        IFS0bits.U1TXIF = 0;
    }
}

void printUART2(char* cadena) 
{
    int i;
    
    IFS1bits.U2TXIF = 0;
    for(i = 0; cadena[i] != '\0'; i++) 
    {
        U2TXREG = cadena[i];
        
        //Mientras no se genere la interrupci?n, va a esperar
        while(!IFS1bits.U2TXIF);
        IFS1bits.U2TXIF = 0;
    }
}

