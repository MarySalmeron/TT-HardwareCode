/* 
 * File:   main.c
 * Author: María José
 *
 * Created on 2 de marzo de 2022, 06:03 PM
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

/****Comandos para envío de mensaje*****/
char CMD_AT[] = "AT\r";
char CMD_ATE0[] = "ATE1\r";

/****Comandos para leer mensajes*****/
/*CMGR=3 PARA LEER MENSAJE*/
char CMD_AT_CMGR [] = "AT+CMGR=1\r";

/****Comandos para GPS*****/
/*char CMD_AT_QGPSCFG [] = "AT+QGPSCFG=\"nmeasrc\",1\r";
char CMD_AT_QGPS[] = "AT+QGPS=1,0\r";*/

//char CMD_AT_QGPSCFG [] = "AT+QGPSCFG=\"outport\",\"uartnmea\",115200\r";
char CMD_AT_QGPS[] = "AT+QGPS?\r";
char CMD_AT_QGPS1[] = "AT+QGPS=1\r";
char CMD_AT_QGPSLOC [] = "AT+QGPSLOC?\r"; //Solicitar ubicación
char CMD_AT_QGPSLOC1 [] = "AT+QGPSLOC?\r"; //Solicitar ubicación
char CMD_AT_QGPSGNMEA [] = "AT+QGPSGNMEA=\"GGA\"\r";
char CMD_AT_QGPSCFG0 [] = "AT+QGPSCFG=\"nmeasrc\",1\r";

char CMD_AT_QGPSEND [] = "AT+QGPSEND\r";

char respuestaGSM[40];
unsigned char j;

char dato = 0;
int error = 0;

char count;

void configurarPuertos();  
void configurarUART1();
void configurarUART2();
void configurarInterrupciones();
void habilitarPerifericos();

void turnOnGPS();
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
  
    printUART1("\r\n Inicializando... \r\n");
    
    iniIoT_BG96();
    
    turnOnGPS();
    
    //CICLO WHILE PARA QUE SE VALIDE SI LA CADENA RECIBIDA NO ES VALIDA
    while (error){
        validarGPS();
    }
    
    
    enviarComandoAT(CMD_AT_QGPSEND);
    printUART1("\r\n Comandos enviados... \r\n");
    
    
    for(;EVER;)
    {
        asm("nop");
    }
    
    return 0;
}
/*FUNCIÓN QUE ACTIVA Y EJECUTA LOS COMANDOS DE OBTENCIÓN DE COORDENADAS*/
void turnOnGPS(){
    
    enviarComandoAT(CMD_AT_QGPS);
    enviarComandoAT(CMD_AT_QGPS1);
    //RETARDO_1s();
    /*for(int j=0; j<10; j++){
        RETARDO_1s();
    }*/
    enviarComandoAT(CMD_AT_QGPSCFG0);
    enviarComandoAT(CMD_AT_QGPSGNMEA);
    
    enviarComandoAT(CMD_AT_QGPSLOC);
    /*for(int k=0;k<5;k++){
        RETARDO_1s();
        RETARDO_1s();
        RETARDO_1s();
        RETARDO_1s();
        RETARDO_1s();
        
        enviarComandoAT(CMD_AT_QGPSLOC);
    }*/
    RETARDO_1s();
    enviarComandoAT(CMD_AT_QGPS);
    
    
}
/*FUNCIÓN QUE VUELVE A SOLICITAR LAS COORDENADAS GPS*/
void validarGPS(){
    
    if (error) {      
        RETARDO_1s();
        RETARDO_1s();
        RETARDO_1s();
        RETARDO_1s();
        RETARDO_1s();
        enviarComandoAT(CMD_AT_QGPSGNMEA);
        enviarComandoAT(CMD_AT_QGPSLOC);
        
    }
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
int i=0;
int respValid=0;
//short int resp [200];
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt( void )
{
    LATBbits.LATB0 = 1;     //LED
    short int resp;
    
   resp = U2RXREG;
    U1TXREG = resp;
    
    if (resp == '6') {
        error = 1;
    }
    
    
    /*resp[i]= U2RXREG;
    U1TXREG = resp[i++];*/
   // printUART1("\r\n Recibiendo respuesta... \r\n");
    /*if(resp [i] == 13){     //<CR>
        printUART1="\r";
    }
    */
    
//    respuestaGSM[j] = resp;
//    j++;
/*    
    if(resp == 13){     //<CR>
        resp = '.';
    }
    else if(resp == 10){     //<LF>
        resp = '_';
        count--;
    }
    U1TXREG = resp;
    
    else if(resp == 62){    //'>'
        count--;
    }
    else if(resp == 32){   //' '
        resp = '-';
    }
    */
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





