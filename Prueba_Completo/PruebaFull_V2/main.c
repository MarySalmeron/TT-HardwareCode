/* 
 * File:   main.c
 * Author: María José
 *
 * Created on 11 de abril de 2022, 07:43 PM
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
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt( void );
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt( void );
void __attribute__((__interrupt__, no_auto_psv)) _INT0Interrupt( void );

/********************************************************************************/
/* COMANDOS AT                                                                  */
/********************************************************************************/
/*ENVÍO DE MENSAJES                                                             */
char CMD_AT[] = "AT\r";
char CMD_ATE0[] = "ATE1\r";
char CMD_AT_CMGF[] = "AT+CMGF=1\r";                     //Poner en modo texto
//char CMD_AT_CMGS[] = "AT+CMGS=\"5574871193\"\r";        
char CMD_AT_CMGS[] = "AT+CMGS=\"";

/*RECEPCIÓN DE MENSAJES                                                         */

char CMD_AT_GMM[]="AT+GMM\r";               //Describe el modulo
char CMD_AT_COPS[]="AT+COPS?\r";            //Checa el network 
char CMD_AT_CFUN[]="AT+CFUN?\r";            //Checa la funcionalidad
char CMD_AT_CFUN1[]="AT+CFUN=1\r";          //Activa el SMS y el CALL
char CMD_AT_CSQ[]="AT+CSQ\r";               //checa la fuerza de la señal
char CMD_AT_CSCS[]="AT+CSCS?\r";            //status de la tarjeta SIM
char CMD_AT_CREG[]="AT+CREG?\r";            //status del registro
char CMD_AT_CSCA[]="AT+CSCA?\r";            //Número del servicio
char CMD_AT_CMGL[]="AT+CMGL=\"ALL\"\r";       //Lista todos los mensajes
char CMD_AT_QINDCFG1[]= "AT+QIDNCFG=?\r";
char CMD_AT_QINDCFG[]="AT+QINDCFG=\"smsincoming\",1\r";
char CMD_AT_CMGR[]="AT+CMGR";               //Leer mensaje con index específico
char CMD_AT_CMGD[]="AT+CMGD=1,4\r";           //Borrar mensajes,

char inicioMSJ [] = "\"";
char msj_GPS[] = "\n Localizame en las siguientes coordenadas (Lat-Long): ";
char CMD_MSJFIN [] = "Me encuentro segura, en un momento me comunico contigo\x1A\r";
char parteFinMSJ [] = "\x1A\r";
char parteFinal [] = "\"\r";
char coma[] = " , ";

/*COMANDOS PARA GPS*/
char CMD_AT_QGPS1[] = "AT+QGPS=1\r";
char CMD_AT_QGPSLOC [] = "AT+QGPSLOC?\r"; //Solicitar ubicación
char CMD_AT_QGPSCFG0 [] = "AT+QGPSCFG=\"gnssconfig\"\r";
char CMD_AT_QGPSEND [] = "AT+QGPSEND\r";

/********************************************************************************/
/* DECLARACIONES DE VARIABLES													*/
/********************************************************************************/
char contacto1[15];
char contacto2[15];
char contacto3[15];
char mensaje[40];
char latitud[20];
char longitud[20];
char auxLat[20];
char auxLong[20];
unsigned char j;
unsigned char k;
unsigned char l;
unsigned char m;

int fin = 0;
int cont=0;
int act=0;
int bandFin=0;
int cont_T1;
int btn_pressed;
int cont_lat=0;
int cont_long=0;
int contComa=0;
int borra_mem=0;
int firsTime=0;
char count;
/********************************************************************************/
/* DECLARACIONES DE FUNCIONES													*/
/********************************************************************************/
void configurarPuertos();  
void configurarUART1();
void configurarUART2();
void configurarInterrupciones();
void habilitarPerifericos();
void iniTIMER(void);
void enLPOSC(void);
void iniIoT_BG96();
void enviarComandoAT(char comando[]);
void recibirMensaje();
void enviarMensaje();
void enviarMensajeFinal();
void leerMem();
void obtenerGPS();
void limpiezaCadGPS();
void printUART1(char* cadena);
void printUART2(char* cadena);
/********************************************************************************/
/* DECLARACIONES FUNCIONES EXTERNAS												*/
/********************************************************************************/
extern void RETARDO_100ms();
extern void RETARDO_1s();
extern void RETARDO_50us();

int main(void) 
{    
    configurarPuertos();
    configurarUART1();
    configurarUART2();
    iniTIMER();
    configurarInterrupciones();
    cont_T1=0;
    btn_pressed=0;
    habilitarPerifericos();
    iniIoT_BG96();
    act=1;
    recibirMensaje();
    act=0;
    firsTime=1;
    latitud[0]='%';
    printUART1("\r\n inicializando... \r\n");
    
    for(;EVER;)
    {
        if(btn_pressed == 1 ){//|| bandFin == 3){
            btn_pressed = 0;
            cont_long=0;
            cont_lat=0;
            enviarComandoAT(CMD_AT_QGPSCFG0);
            enviarComandoAT(CMD_AT_QGPS1);
            act=2;
            if( firsTime ){
                while(auxLat[0] == '%'){
                    obtenerGPS();
                }
                firsTime = 0;
            }else{
                for(int i=0; i<8;i++){
                    obtenerGPS();
                }
            }
            enviarComandoAT(CMD_AT_QGPSEND);
            limpiezaCadGPS();
            enviarMensaje();                
        }else if(cont_T1 >= 60){
            act=1;
            cont_T1=0;
            borra_mem++;
            leerMem();
            if(bandFin==3) {
                enviarMensajeFinal();
                enviarComandoAT(CMD_AT_CMGD);
                bandFin=0;
            }
            if( borra_mem == 5){
                enviarComandoAT(CMD_AT_CMGD);
                borra_mem=0;
            }
        }
        act=0;
    }
    
    return 0;
}

void limpiezaCadGPS(){
    int i=j=0;
    for(i=0, j=0; j<11;j++){
        if(j==2){
            latitud[2]=' ';
            j++;
        } 
        latitud[j]=auxLat[i];
        i++;
    }
    if(auxLong[0]=='0'){
        longitud[0]='-';
    }
    for(i=1, j=1; j<11;j++){
        
        if(j==3){
            longitud[j]=' ';
            j++;
        }
        longitud[j]=auxLong[i];
        i++;
    }
}
/****************************************************************************/
/* DESCRIPCION:	ESTA RUTINA ENVÍA UN MENSAJE UTILIZANDO COMANDOS AT			*/
/* PARAMETROS: NINGUNO                                                      */
/* RETORNO: NINGUNO															*/
/****************************************************************************/

void enviarMensaje(){
    enviarComandoAT(CMD_AT_CMGF);
    /*Cadena con las coordenadas*/
    /*ENVÍO A CONTACTO 1*/
    enviarComandoAT(CMD_AT_CMGS);
    enviarComandoAT(contacto1);
    enviarComandoAT(parteFinal);
    //enviarComandoAT(inicioMSJ);
    enviarComandoAT(mensaje);
    enviarComandoAT(msj_GPS);
    enviarComandoAT(latitud);
    enviarComandoAT(coma);
    enviarComandoAT(longitud);
    enviarComandoAT(parteFinMSJ);
    /*ENVÍO A CONTACTO 2*/
    enviarComandoAT(CMD_AT_CMGS);
    enviarComandoAT(contacto2);
    enviarComandoAT(parteFinal);
    //enviarComandoAT(inicioMSJ);
    enviarComandoAT(mensaje);
    enviarComandoAT(msj_GPS);
    enviarComandoAT(latitud);
    enviarComandoAT(coma);
    enviarComandoAT(longitud);
    enviarComandoAT(parteFinMSJ);
    /*ENVÍO A CONTACTO 3*/
    enviarComandoAT(CMD_AT_CMGS);
    enviarComandoAT(contacto3);
    enviarComandoAT(parteFinal);
    //enviarComandoAT(inicioMSJ);
    enviarComandoAT(mensaje);
    enviarComandoAT(msj_GPS);
    enviarComandoAT(latitud);
    enviarComandoAT(coma);
    enviarComandoAT(longitud);
    enviarComandoAT(parteFinMSJ);
}

/****************************************************************************/
/* DESCRIPCION:	ESTA RUTINA ENVÍA UN MENSAJE UTILIZANDO COMANDOS AT			*/
/* PARAMETROS: NINGUNO                                                      */
/* RETORNO: NINGUNO															*/
/****************************************************************************/

void enviarMensajeFinal(){
     /*Cadena final*/ 
        //ENVÍO A CONTACTO 1
        enviarComandoAT(CMD_AT_CMGS);
        enviarComandoAT(contacto1);
        enviarComandoAT(parteFinal);
        enviarComandoAT(CMD_MSJFIN);
        //ENVÍO A CONTACTO 2
        enviarComandoAT(CMD_AT_CMGS);
        enviarComandoAT(contacto2);
        enviarComandoAT(parteFinal);
        enviarComandoAT(CMD_MSJFIN);
        //ENVÍO A CONTACTO 3
        enviarComandoAT(CMD_AT_CMGS);
        enviarComandoAT(contacto3);
        enviarComandoAT(parteFinal);
        enviarComandoAT(CMD_MSJFIN);
        bandFin=0;
}

/******************************************************************************/
/* DESCRIPCION:	ESTA RUTINA RECIBE Y PROCESA LOS MENSAJES QUE RECIBE EL MÓDULO*/
/* PARAMETROS: NINGUNO                                                        */
/* RETORNO: NINGUNO                                                           */
/******************************************************************************/

void recibirMensaje(){
    enviarComandoAT(CMD_AT);
    enviarComandoAT(CMD_ATE0);
    enviarComandoAT(CMD_AT_CMGF);
    enviarComandoAT(CMD_AT_GMM);
    enviarComandoAT(CMD_AT_COPS);
    enviarComandoAT(CMD_AT_CFUN);
    enviarComandoAT(CMD_AT_CFUN1);
    enviarComandoAT(CMD_AT_CSQ);
    enviarComandoAT(CMD_AT_CSCS);
    enviarComandoAT(CMD_AT_CREG);
    enviarComandoAT(CMD_AT_CSCA);
    
    enviarComandoAT(CMD_AT_CMGF);
    //enviarComandoAT(CMD_AT_CMGD);
    enviarComandoAT(CMD_AT_CMGL);
   
    //enviarComandoAT(CMD_AT_CMGL);
}

/******************************************************************************/
/* DESCRIPCION:	ESTA RUTINA MUESTRA LOS MENSAJES RECIBIDOS                    */
/* PARAMETROS: NINGUNO                                                        */
/* RETORNO: NINGUNO                                                           */
/******************************************************************************/
void leerMem(){
    enviarComandoAT(CMD_AT_CMGL);
}

/******************************************************************************/
/* DESCRIPCION:	ESTA RUTINA RECIBE LAS COORDENADAS GPS                    */
/* PARAMETROS: NINGUNO                                                        */
/* RETORNO: NINGUNO                                                           */
/******************************************************************************/
void obtenerGPS(){
    enviarComandoAT(CMD_AT_QGPSLOC);
    
}


/******************************************************************************/
/* DESCRIPCION:	ESTA RUTINA ENVÍA LOS COMANDOS AL MÓDULO IoT                  */
/* PARAMETROS: NINGUNO                                                        */
/* RETORNO: NINGUNO                                                           */
/******************************************************************************/
void enviarComandoAT(char comando[])
{    
    printUART2(comando);
    RETARDO_1s();
    RETARDO_1s();
    RETARDO_1s();

    /*
    count = 2;
    j = 0;
    
    while(count > 0)
    {
        U1TXREG = 'x';
        
    }   //count disminuye con la interrupci?n U2RXInterrupt
    */
}

/****************************************************************************/
/* DESCRIPCION:	ESTA RUTINA CONFIGURA ENVIA LA CADENA A EL UART1       		*/
/* PARAMETROS: APUNTADOR TIPO CHAR                                          */
/* RETORNO: NINGUNO															*/
/****************************************************************************/
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

/****************************************************************************/
/* DESCRIPCION:	ESTA RUTINA CONFIGURA ENVIA LA CADENA A EL UART2      		*/
/* PARAMETROS: APUNTADOR TIPO CHAR                                          */
/* RETORNO: NINGUNO															*/
/****************************************************************************/
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

/****************************************************************************/
/* DESCRIPCION:	ESTA RUTINA CONFIGURA EL TIMER1                     		*/
/* PARAMETROS: NINGUNO                                                      */
/* RETORNO: NINGUNO															*/
/****************************************************************************/
void iniTIMER()
{
    TMR1  = 0X0000;
    PR1   = 28800;
    T1CON = 0X0020;
}

/****************************************************************************/
/* DESCRIPCION:	ESTA RUTINA HABILITA LOS PERIFERICOS PARA LOS UARTS    		*/
/* PARAMETROS: NINGUNO                                                      */
/* RETORNO: NINGUNO															*/
/****************************************************************************/
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
    //Para Timer1
    T1CONbits.TON=1;            //Se enciende el timer
    
}

/****************************************************************************/
/* DESCRIPCION:	ESTA RUTINA CONFIGURA LOS PUERTOS DE ENTRADA O SALIDA		*/
/* PARAMETROS: NINGUNO                                                      */
/* RETORNO: NINGUNO															*/
/****************************************************************************/
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
    
    PORTA = 0;
    asm("nop"); 
    LATA =0;
    asm("nop"); 
    TRISAbits.TRISA11=1;
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

/****************************************************************************/
/* DESCRIPCION:	ESTA RUTINA CONFIGURA LAS INTERRUPCIONES        			*/
/* PARAMETROS: NINGUNO                                                      */
/* RETORNO: NINGUNO															*/
/****************************************************************************/
void configurarInterrupciones()
{    
    IFS1bits.U2RXIF=0;    //UART2 Receiver Interrupt Flag Status bit
    IEC1bits.U2RXIE=1;    //UART2 Receiver Interrupt Enable bit (Interrupt request enabled)
    
    IFS0bits.INT0IF=0;      //BOTÓN
    IEC0bits.INT0IE=1;      //Habilitamos la interrupción
    INTCON2bits.INT0EP=0;   //Habilitamos el flanco de subida
    
    IFS0bits.T1IF=0;      //TIMER 1
    IEC0bits.T1IE=1;      //Habilitamos la interrupción
}

/****************************************************************************/
/* DESCRIPCION:	ESTA RUTINA INICIALIZA EL MODULO IoT                		*/
/* PARAMETROS: NINGUNO                                                      */
/* RETORNO: NINGUNO															*/
/****************************************************************************/
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

/********************************************************************************/
/* DESCRICION:	ISR (INTERRUPT SERVICE ROUTINE) DEL UART2						*/
/* LA RUTINA TIENE QUE SER GLOBAL PARA SER UNA ISR								*/	
/* SE USA PUSH.S PARA GUARDAR LOS REGISTROS W0, W1, W2, W3, C, Z, N Y DC EN LOS */
/* REGISTROS SOMBRA																*/
/********************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt( void )
{
    short int resp;
    LATBbits.LATB0 = 1;     //LED
        
    resp = U2RXREG;
    U1TXREG = resp;
    
    if(resp == 37){ //'%'
        bandFin=3;
    }
    if(act==1){
        /*INICIO DEL ANÁLISIS DE LA CADENA DE MENSAJE*/
        /*Analisis de la cadena para recepción de números de contacto y del mensaje*/
        
        if(resp == 60 ){ //'<'
            if(cont == 0){
                cont = 1;
                fin = 0;
            }else if(cont == 1){
                cont = 2;
                fin = 0;
            } else if(cont == 2){
                cont = 3;
                fin = 0;
            }  
        } else if(resp == 38) { //'&'
            cont = 4;
            fin = 0;
        }else if(cont == 1 && fin == 0){
            if(resp == 62) {// '>'
                fin = 1;
                j=0;
            }            
            else{
                contacto1[j]=resp;
                j++; 
            } 
        }  else if(cont == 2 && fin == 0){
            if(resp == 62) {         // '>'
                fin = 1;
                k=0;
            }else{
                contacto2[k]=resp;
                k++;
            }
        }  else if(cont == 3 && fin == 0){
            if(resp == 62){          // '>'
                fin = 1;
                l=0;
            }else{
                contacto3[l]=resp;
                l++;
            }
        } else if(cont == 4 && fin == 0){
            if(resp == 35){ // '#'
                fin = 1;
                cont=0;
                m=0;
            }              
            else{
                mensaje[m]=resp;
                m++;
            }
        }
        /*FIN DEL ANÁLISIS DE CADENA DE MENSAJE*/
        /*INICIO DEL ANÁLISIS DE CADENA GPS*/
    }else if(act==2){
        if (resp == 44 && contComa == 0){ //','
            contComa=1;
        }else if( resp == 44 && contComa == 1){
            contComa=2;
        }else if( resp == 44 && contComa == 2){
            contComa=3;
        }else if( contComa == 1 ){    
            auxLat[cont_lat]=resp;
            cont_lat++;
            //contComa=2;
        }else if( contComa == 2 ){
            auxLong[cont_long]=resp;
            cont_long++;
        }
    }
    /*FIN DEL ANÁLISIS DE CADENA GPS*/
    LATBbits.LATB0 = 0;     //LED
    IFS1bits.U2RXIF = 0;
}

/********************************************************************************/
/* DESCRIPCION:	ISR (INTERRUPT SERVICE ROUTINE) DEL TIMER 1						*/
/* LA RUTINA TIENE QUE SER GLOBAL PARA SER UNA ISR								*/	
/* SE USA PUSH.S PARA GUARDAR LOS REGISTROS W0, W1, W2, W3, C, Z, N Y DC EN LOS */
/* REGISTROS SOMBRA																*/
/********************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt( void )
{
    cont_T1++;
    IFS0bits.T1IF = 0;    //SE LIMPIA LA BANDERA DE INTERRUPCION DEL TIMER 1                      
}

/********************************************************************************/
/* DESCRIPCION:	ISR (INTERRUPT SERVICE ROUTINE) DEL TIMER 1						*/
/* LA RUTINA TIENE QUE SER GLOBAL PARA SER UNA ISR								*/	
/* SE USA PUSH.S PARA GUARDAR LOS REGISTROS W0, W1, W2, W3, C, Z, N Y DC EN LOS */
/* REGISTROS SOMBRA																*/
/********************************************************************************/
void __attribute__((__interrupt__, no_auto_psv)) _INT0Interrupt( void )
{
    btn_pressed=1;
    IFS0bits.INT0IF = 0;    //SE LIMPIA LA BANDERA DE INTERRUPCION DEL BOTON                      
}


