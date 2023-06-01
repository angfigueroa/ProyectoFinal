/* 
 * File:   FUNCIONAMIENTO.c
 * Author: ANGELA
 *
 * Created on 6 de mayo de 2023, 17:30
 */

#include <xc.h>
#include <stdint.h>

//Configuracion
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


//definicion de frecuencia para delay
#define _tmr0_value 176
#define _XTAL_FREQ 4000000 
#define addressEEPROM 0X00 

//VARIABLES
uint8_t VAL;
uint8_t VAL1;
uint8_t VAL2;
uint8_t VAL3;
uint8_t PWM1;
uint8_t PWM2;
uint8_t POT3;
uint8_t POT4;
uint8_t val1;
uint8_t val2;
uint8_t val3;
uint8_t val4;
uint8_t VALOR = 0;
uint8_t VALOR1;
uint8_t VALOR2;
uint8_t FLAG;
uint8_t OP;
uint8_t cont;

unsigned char I[72] = "\nBIENVENIDO, presione 1 para continuar con la comunicacion serial\n\r";
unsigned char R[60] = "\n¿Que servomotor desea mover?\n\r1) Pied \n\r2) Piei \n\r3) Cadd \n\r4) Cadi\n\r";
unsigned char M[36] = "\nIngrese un numero entre 0 y 3\n\r";

int modos;
int map (int input, int in_min, int in_max, int out_min, int out_max);
int map (int input, int in_min, int in_max, int out_min, int out_max) {
    return ((input - in_min)* (out_max - out_min)/ (in_max - in_min) + out_min);
}    
//Variables para conectar la interfaz
int serialdata[2];
char indice;
int value;
int ind;
int bandera = 0;

//PROTOTIPOS DE FUNCIONES
void setup(void);
void canales(void);
void escribir(uint8_t data, uint8_t address);
uint8_t leer(uint8_t address);
void UART(void);
void INS(void);
void OTRO(void);
void MENSAJE(void);
void MTMR0(void);



//FUNCIONES PARA ISR
void __interrupt() isr(void){
    if (PIR1bits.RCIF)
    {
        PIR1bits.RCIF = 0;
        if(ind == 0){
            serialdata[0] = RCREG;
            indice = serialdata[0];
        }
        else if(ind == 1){
            serialdata[1] = RCREG;
            value = serialdata[1];

        }
        ind ++;
        if(ind == 2){
             ind = 0;
        }
        if(bandera == 0b0011){
            if(indice == 1){
                CCPR1L = value;
             
            }
            else if(indice == 2){
                CCPR2L = value;
            }  
      
            else if(indice == 3){
                 val3 = value;
            }  
            else if(indice == 4){ 
                 val4 = value;
            } 
            }

        
        
    }
    
    if (INTCONbits.TMR0IF)
    {
        INTCONbits.TMR0IF = 0;
        cont++;
        if (cont < val3){
            PORTCbits.RC3 = 1;
        }
        if (cont >= val3){
            PORTCbits.RC3 = 0;
            }
        if (cont < val4){
            PORTCbits.RC4 = 1;
        }
        if (cont > val4){
            PORTCbits.RC4 = 0;
        }
        if (cont >= 100){
            cont = 0;
        }
        TMR0 = 206;
    }    
    
    
    if(PIR1bits.ADIF == 1){ //Interrupciones del ACD
     
        PIR1bits.ADIF = 0;
        if(ADCON0bits.CHS == 0b0000){
            CCPR1L = (ADRESH >> 1) + 124;
            POT3 = (ADRESH >> 1) + 124;
        } 
        else if (ADCON0bits.CHS == 0b0001){
            CCPR2L = (ADRESH >> 1) + 124;
            POT4 = (ADRESH >> 1) + 124;
        }
        else if (ADCON0bits.CHS == 0b0010){
            val3 = map(ADRESH,0,255,1,10);
            
        }
        else if (ADCON0bits.CHS == 0b0011){
            val4 = map(ADRESH,0,255,1,10);
        }
        
        }
 
    //INTERRUPCIONES DEL TIMER0
if(INTCONbits.T0IF == 1){ //Bandera del TMR0 encendida 
    PWM1++; //Incrementa el contador para el PWM del S1
        if(PWM1 <= POT3){ //El valor del periodo depende del POT3
            PORTCbits.RC3 = 1;//Encender el pin
        }
        else{
            PORTCbits.RC3 = 0;//Apagar el pin
        }
        if(PWM1 <= POT4){
            PORTCbits.RC4 = 1;
        }
        else{
            PORTCbits.RC4 = 0;
        }
        if(PWM1 >= 250){
            PWM1 = 0; 
        }
        TMR0 = _tmr0_value;
        INTCONbits.T0IF = 0;    
}
//INTERRUPCIONES DEL PUERTO B
if(INTCONbits.RBIF == 1){
    
        if(PORTBbits.RB0 == 0){//presionando por que son pull ups
            PORTDbits.RD0 = 1;
            PORTDbits.RD1 = 0;
            
            escribir(VALOR1, 0x10);
            escribir(VALOR2, 0x11);
            escribir(POT3, 0x12);
            escribir(POT4, 0x13);
             
            //__delay_ms(10);
        }
        if(PORTBbits.RB1 == 0){
            PORTDbits.RD0 = 0;
            PORTDbits.RD1 = 1;
         
                    
            FLAG = 1;//activar bandera del UART
            while(FLAG == 1){
                TXSTAbits.TXEN = 1;
                UART();
            }   
            TXSTAbits.TXEN = 0;
        }
        if(PORTBbits.RB2 == 0){//presionando por que son pull ups
            //ADCON0bits.ADON = 0;
            PORTDbits.RD0 = 0;
            PORTDbits.RD1 = 1;
            
            val1 = leer(0x10);
            val2 = leer(0x11);
            val3 = leer(0x12);
            val4 = leer(0x13);

            CCPR1L = val1;
            CCPR2L = val2;
            POT3 = val3;
            POT4 = val4;
            //__delay_ms(10);
            ADCON0bits.ADON = 1;    
        }
        INTCONbits.RBIF = 0;//limpiar bandera del IOCB

    }  
    //PIR1bits.TMR2IF = 0;    
  switch(indice){
 
            case(5):
                bandera = 0b0001;
                
                break;  
            case(6):
                bandera = 0b0010;
                
                break;   
            case(7):
                 bandera = 0b0011;
                 
                break;   
            case(8):
                bandera = 0b0100;
                
                break; 
        } 
}

//CONFIGURACION
void setup(void){
    //configuracion de los puertos
    ANSEL = 0B00011111;
    ANSELH = 0X00;
    
    TRISA = 0B00011111;
    TRISBbits.TRISB0 = 1;
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB2 = 1;
    TRISC = 0B10000000;
    TRISD = 0B00;
    
    PORTA = 0X00;
    PORTB = 0X00;
    PORTC = 0X00;
    PORTD = 0X00;
    
    //weak pull up
    IOCB = 0b00000111;
    OPTION_REGbits.nRBPU = 0;//internal pull up habilitado
    WPUB = 0b00000111;
    
    //configuracion del TMR0, N=176 y un overflow 
    OPTION_REG = 0B00001000;
    TMR0 = _tmr0_value;
    INTCONbits.PEIE = 1;
    INTCONbits.T0IE = 1;
    INTCONbits.T0IF = 0;
    INTCONbits.RBIE = 1;
    INTCONbits.RBIF = 0;
    INTCONbits.GIE = 1;
    
    //configuracion del oscilador
    OSCCONbits.SCS = 1;
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 0;
    PIR1bits.TMR2IF = 0;
    T2CON = 0X26;
    
    //configuracion del modulo ADC
    ADCON0bits.CHS = 0;
    ADCON0bits.CHS = 2;
    __delay_us(100);
    
    PIE1bits.ADIE = 1;
    PIR1bits.ADIF = 0;
    ADCON0bits.ADON = 1;
    ADCON0bits.ADCS = 1;
    ADCON1bits.ADFM = 0;
    ADCON1bits.VCFG0 = 0;
    ADCON1bits.VCFG1 = 0;
    
    //configuracion del PWM
    PR2 = 250;
    CCP1CON = 0B00001100;
    CCP2CON = 0B00001111;
    
    //configuracion UART
    PIR1bits.RCIF = 0;
    PIE1bits.RCIE = 0;
    PIE1bits.TXIE = 0;
    TXSTAbits.TX9 = 0;
    TXSTAbits.TXEN = 1;
    TXSTAbits.SYNC = 0;
    TXSTAbits.BRGH = 1;
    RCSTAbits.RX9 = 0;
    RCSTAbits.CREN = 1;
    RCSTAbits.SPEN = 1;
    
    //generador de bauidos del UART
    BAUDCTLbits.BRG16 = 0;
    SPBRG = 25;
    SPBRGH = 1;
    }
//LOOP PRINCIPAL
void main(void){
    setup();
    while (1){
        canales();
        //UART();
    }
}

//FUNCIONES
//limpiar a mensaje en la termianal
void UART(void){
    //__delay_ms(10);
        VALOR = 0;
        do{VALOR++;//Incrementar la variable
            TXREG = I[VALOR];
            __delay_ms(10);
        }
        while(VALOR<=72); //Cantidad de caracteres en el array
        while(RCIF == 0);
        FLAG = 0;
        INS();        //Llamamos al mensaje
}

void canales(){
    if(ADCON0bits.GO == 0){
        switch(ADCON0bits.CHS){
            case 0:
                CCPR1L = ((0.247*VAL)+62);//Funcion para el servo
                VALOR1 = CCPR1L;
                ADCON0bits.CHS = 1;//canal 3
                __delay_us(100);//delay para activar la medicion
                ADCON0bits.GO = 1;//comienza el ciclo del ADC
                break;
            case 1:
                POT4 = ((0.049*VAL1)+7);// PWM modificado
                ADCON0bits.CHS = 2;//canal 0
                __delay_us(100);
                ADCON0bits.GO = 1;
                break; 
            case 2:
                CCPR2L = ((0.247*VAL2)+62);
                VALOR2 = CCPR2L;
                ADCON0bits.CHS = 3;
                __delay_us(100);
                ADCON0bits.GO = 1;
                break; 
            case 3:
                POT3 = ((0.049*VAL3)+7);
                ADCON0bits.CHS = 0;
                __delay_us(100);
                ADCON0bits.GO = 1;
                break; 
            default:
                break;
        }
    }
}

//FUNCION PARA ESCRIBIR EN LA EEPROM
void escribir(uint8_t data,uint8_t address){
    EEADR = address;
    EEDAT = data;//valor a escribir
    
    EECON1bits.EEPGD = 0;//data memory se apunta
    EECON1bits.WREN = 1;
    INTCONbits.GIE = 0;//apagar interrupciones globales
    
    EECON2 = 0X55;
    EECON2 = 0XAA;
    EECON1bits.WR = 1;//se inicia la escritura
    
    while(PIR2bits.EEIF == 0);//se espera al final de la escritura
    PIR2bits.EEIF = 0;//se apaga la bandera
    EECON1bits.WREN = 0;
    INTCONbits.GIE = 0;
    }

//FUNCION PARA LEER DE LA EEPROM
uint8_t leer(uint8_t address){
    EEADR = address;
    EECON1bits.EEPGD = 0;
    EECON1bits.RD = 1;
    ;uint8_t data = EEDATA;
    return data;   
}

//MENSAJE 
void INS(void){
    OP = RCREG;
    switch(OP){
            case 49://se presiona 1 manualmente
                __delay_ms(10);
                VALOR = 0;
                do{VALOR++;//incrementa la variable
                    TXREG = R[VALOR];
                    __delay_ms(10);
                }
                while(VALOR<=60);//cantidad de caracteres array
                    while(RCIF == 0);
                OP = 0;//se limpia la variable que hace el cambio
                OTRO();
                break;
            case 50: // se presiona 2 para la serial
                    TXSTAbits.TXEN = 0;//se apaga la bandera de transmision
                OP = 0;
                break;
        }
}

void OTRO(void){
    OP = RCREG;
    switch(OP){
        case 49:
            MENSAJE();
            if(RCREG >= 48 && RCREG <= 57){
                VAL = RCREG;
                canales();
            }
            break;
        case 50:
            MENSAJE();
            if(RCREG >= 48 && RCREG <= 57){
                VAL1 = RCREG;
                canales();
                }
            break;
        case 51:
            MENSAJE();
            if(RCREG >= 48 && RCREG <= 57){
                VAL2 = RCREG;
                canales();
            }
            break;
        case 52:
            MENSAJE();
            if(RCREG >= 48 && RCREG <= 57){
                VAL3 = RCREG;
                canales();
            }
            break;
    }
}

void MENSAJE(void){
    __delay_ms(10);
    VALOR = 0;
    do{VALOR++;//incrementar variable
    TXREG = M[VALOR];
    __delay_ms(10);
    }
    while(VALOR<=36);
    while(RCIF == 0);
    OP = 0;//limpiar variable
}
