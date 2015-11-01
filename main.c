/* 
 * File:   main.c
 * Author: Tomasz Pliszczak
 *
 * Created on 30 pa?dziernik 2015, 21:22
 */

#include <stdio.h>
#include <stdlib.h>
#include <xc.h>

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = OFF       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR)
#pragma config CP = ON         // Flash Program Memory Code Protection (Program memory code protection is enabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = ON        // Internal/External Switchover (Internal/External Switchover mode is enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is enabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config VCAPEN = OFF     // Voltage Regulator Capacitor Enable bit (VCAP pin function disabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)

volatile unsigned int adc; 
volatile unsigned char x;
 volatile unsigned char flaga_temp;
 volatile unsigned int timer1;
 volatile unsigned long timer2;
 volatile unsigned int timer3;
 volatile unsigned int timer4;
 volatile unsigned long timer5;
 volatile unsigned int timer6;
 volatile unsigned int timer7;
 volatile unsigned char cyf_1;
 volatile unsigned char cyf_2;
 volatile unsigned char cyf_3;
 volatile unsigned char kro_1;
 volatile unsigned char kro_2;
 volatile unsigned char kro_3;
 volatile unsigned char idx;
 volatile char multi;
 volatile char PWM;


 volatile unsigned int temp;
 volatile unsigned int temp_sr;
 volatile unsigned int temp_tabela[50];


 volatile unsigned char licznik;
 
 volatile unsigned char wyswietlacz;
 volatile unsigned char case_wyswietlacz;
 volatile unsigned char cyf_1_tmp;
 volatile unsigned char cyf_2_tmp;
 volatile unsigned char cyf_3_tmp;
 volatile unsigned char kro_1_tmp;
 volatile unsigned char kro_2_tmp;
 volatile unsigned char kro_3_tmp;

  volatile unsigned char test_index;
 
  
     
  //program grzalka
    volatile unsigned int czas_s;
    volatile unsigned int czas_flash;
    volatile unsigned int czas_s_reset;
    volatile unsigned long czas_ms;
    volatile unsigned char START;
    volatile unsigned char START_Fl;
    volatile unsigned char grzalka_numer;
    volatile unsigned char czas_przerwa_ms;
    volatile unsigned char przycisk_plus_flag;
    volatile unsigned char przycisk_minus_flag;
    
        
    volatile unsigned char adr_low = 0xFD;    
    volatile unsigned char adr_low2 = 0xFE; 
    volatile unsigned char adr_hi = 0x1F;
    volatile unsigned char flash_low;
    volatile unsigned char flash_hi;
        
        

    
    //funkcje
    void wyswietl(unsigned int liczba);
    unsigned char read_flash(unsigned char adrs_hi,unsigned char adrs_lo);
    void unlock_flash(void);
    void write_flash(unsigned char adr_hi,unsigned char adr_lo, unsigned char data_lo);
    void erase_flash(unsigned char adr_hi,unsigned char adr_lo);


    #define _XTAL_FREQ 8000000

    // definicje bitów dla poszczególnych segmentów LED
    #define SEG_A (1<<0)
    #define SEG_B (1<<1)
    #define SEG_C (1<<2)
    #define SEG_D (1<<3)
    #define SEG_E (1<<4)
    #define SEG_F (1<<5)
    #define SEG_G (1<<6)
    #define SEG_DP (1<<7)
    #define NIC 10
    #define MIN 11
    #define STOP 12
    #define NAP 13


    #define Q1 LATAbits.LATA0
    #define Q2 LATAbits.LATA1
    #define Q3 LATAbits.LATA2
    #define LED LATAbits.LATA3
    #define PRZYCISK PORTBbits.RB5 
    #define PRZYCISK_PLUS PORTBbits.RB3 
    #define PRZYCISK_MINUS PORTBbits.RB4 

    // definicja tablicy zawieraj±cej definicje bitowe cyfr LED
    const unsigned char cyfry[15] = {
        (SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F),			// 0
        (SEG_B|SEG_C),						// 1
        (SEG_A|SEG_B|SEG_D|SEG_E|SEG_G),			// 2
        (SEG_A|SEG_B|SEG_C|SEG_D|SEG_G),			// 3
        (SEG_B|SEG_C|SEG_F|SEG_G),				// 4
        (SEG_A|SEG_C|SEG_D|SEG_F|SEG_G),			// 5
        (SEG_A|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G),			// 6
        (SEG_A|SEG_B|SEG_C),					// 7
        (SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G),            // 8
        (SEG_A|SEG_B|SEG_C|SEG_D|SEG_F|SEG_G),			// 9
        0,							// NIC (puste miejsce)
        SEG_G,                                                  // MIN
        (SEG_A|SEG_B|SEG_F|SEG_G),				//STOP
        (SEG_E|SEG_D|SEG_C),					//u
        (SEG_G)			// -
    };

    const unsigned char  an[2] = {0b00000001, 0b00000010 }; //USTAWIENIE ANOD 0 = zapalona

    

int main(int argc, char** argv) {

    //8MHz
    OSCCONbits.IRCF0 = 0;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF3 = 1;


    //4s watchdog
    WDTCONbits.WDTPS0=0;
    WDTCONbits.WDTPS1=0;
    WDTCONbits.WDTPS2=1;
    WDTCONbits.WDTPS3=1;
    WDTCONbits.WDTPS4=0;


    //timer0
    OPTION_REGbits.PS0=0;  //pre8 010
    OPTION_REGbits.PS1=1;
    OPTION_REGbits.PS2=0;
    OPTION_REGbits.TMR0CS = 0;  //intosc/4
    OPTION_REGbits.PSA = 0;  //prescaler

    INTCONbits.TMR0IE = 1;  //interrupt enable
    INTCONbits.GIE = 1;  //interrupt glob enable




    //RA0-7
    //IO 
    ANSELAbits.ANSA0 = 0;
    ANSELAbits.ANSA1 = 0;
    ANSELAbits.ANSA2 = 0;
    ANSELAbits.ANSA3 = 0;
    ANSELAbits.ANSA5 = 0;

    //RA0-7 wyjscia
    TRISAbits.TRISA0 = 0;  //Q1
    TRISAbits.TRISA1 = 0; //Q2
    TRISAbits.TRISA2 = 0; //Q3
    TRISAbits.TRISA3 = 0; //LED PRZYCISK
    TRISAbits.TRISA4 = 0;
    TRISAbits.TRISA5 = 0;
    TRISAbits.TRISA6 = 0;
    TRISAbits.TRISA7 = 0;
    
    //niski
    LATAbits.LATA0 = 0;
    LATAbits.LATA1 = 0;
    LATAbits.LATA2 = 0;
    LATAbits.LATA3 = 0;
    LATAbits.LATA4 = 0;
    LATAbits.LATA5 = 0;
    LATAbits.LATA6 = 0;
    LATAbits.LATA7 = 0;


    //RB0-1  anody
    //IO RB0-2
    ANSELBbits.ANSB0 = 0;
    ANSELBbits.ANSB1 = 0;

    //RB0-2 wyjscia
    TRISBbits.TRISB0 = 0;
    TRISBbits.TRISB1 = 0;
    
    LATBbits.LATB0 = 0;
    LATBbits.LATB1 = 0;
    
    
    //IO RB2-7
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0; 
    ANSELBbits.ANSB4 = 0;
    ANSELBbits.ANSB5 = 0;

    TRISBbits.TRISB4 = 0;
        
    //RB3-7 in
    TRISBbits.TRISB2 = 1; 
    TRISBbits.TRISB3 = 1; 
    TRISBbits.TRISB4 = 1;
    TRISBbits.TRISB5 = 1;
    TRISBbits.TRISB6 = 1;
    TRISBbits.TRISB7 = 1;

    //RB4-7 pull up
    OPTION_REGbits.nWPUEN = 0;  // enable all pull up
    WPUBbits.WPUB3 = 1;
    WPUBbits.WPUB4 = 1;
    
    
    // RC LED output
    TRISC = 0; // pins is output
    LATC = 0; // pins low
    ANSELC = 0;  // digital io

    //dioda led
    ANSELAbits.ANSA5 = 0;
    TRISAbits.TRISA5 = 0;
    LATAbits.LATA5 = 0;

    //temp
    FVRCONbits.TSEN = 1;
    FVRCONbits.FVREN = 0;
    FVRCONbits.TSRNG = 1;

    ADCON0bits.ADON = 1;
    ADCON0bits.CHS = 0b11110;
    ADCON1 |= 0b10000000;


    x=0;
    flaga_temp = 0;

    cyf_1 = 0;
    cyf_2 = 0;
    cyf_3 = 0;
    kro_1 = 0;
    kro_2 = 0;
    kro_3 = 0;

    timer5 = 5000;
    
    START = 0;
    czas_przerwa_ms = 100;
    czas_s_reset = 15;
    


    grzalka_numer = 1; //start od Q1
    START_Fl = 0;
                   
    //LED = 1;    
    cyf_1 = 14;
    cyf_2 = 14;
    kro_1 = 1;
    kro_2 = 1;
    __delay_ms(1000);
    
    flash_low = read_flash(adr_hi, adr_low);    
    flash_hi =  (read_flash(adr_hi, adr_low2) & 0x3fff );

    
    if ( (flash_low == 0xFF) && (flash_hi == 0xFF)) {
        flash_low = 0;
        flash_hi = 0;
        czas_flash = 0;
    }else{
        czas_s = flash_hi ;
        czas_s = (czas_s<<8);
        czas_s|= flash_low;
        
        //czas_s = flash_low;
        czas_flash = czas_s;
    }
    
    
    if (czas_s == 0) {
        czas_s = czas_s_reset;
    }

    while(1){
        
        //przyciski*************************************************************
        if (czas_s > 1800) {
            czas_s = 1800;
        }
        if (czas_s < 1) {
            czas_s = 1;
        }

        //PLUS RB3
        if ( (PRZYCISK_PLUS == 0) && (PRZYCISK_MINUS == 1 ) && ( START == 0 )){
            if (przycisk_plus_flag == 0 ){
                __delay_ms(50);
            }
            if ( (PRZYCISK_PLUS == 0) && (przycisk_plus_flag == 0) ) {
                czas_s++;
                przycisk_plus_flag = 1;
            }
        }else{
            przycisk_plus_flag = 0;
            timer1 = 1500;
        }
        //przyspieszone dodawanie 1,5s
        if ( (timer1 == 0) && (przycisk_plus_flag == 1) ) {
              czas_s++;
              __delay_ms(10);
        }
        
 
        //MINUS RB4
        if ( (PRZYCISK_PLUS == 1) && (PRZYCISK_MINUS == 0 ) && ( START == 0 )){
            if (przycisk_minus_flag == 0 ){
                __delay_ms(50);
            }
            if ( (PRZYCISK_MINUS == 0) && (przycisk_minus_flag == 0) ) {
                czas_s--;
                przycisk_minus_flag = 1;
            }
        }else{
            przycisk_minus_flag = 0;
            timer6 = 1500;
        }
        //przyspieszone dodawanie 1,5s
        if ( (timer6 == 0) && (przycisk_minus_flag == 1) ) {
              czas_s--;
              __delay_ms(10);
        }
        
               
        //reset RB3 + RB4
        if ( (PRZYCISK_PLUS == 0) && (PRZYCISK_MINUS == 0 ) && ( START == 0 )){
                   __delay_ms(1000);      
                   CLRWDT();
                   __delay_ms(1000);      
                   CLRWDT();
                   __delay_ms(1000);
                   CLRWDT();    
                   if ( (PRZYCISK_PLUS == 0) && (PRZYCISK_MINUS == 0 )){
                       czas_s = czas_s_reset;
                   }                    
                   __delay_ms(1000);                     
        }
        
        
        
        //zapis w pamieci
        if ( (czas_s!= czas_flash) && (PRZYCISK_PLUS ==1) && (PRZYCISK_MINUS == 1) ) {
            //LED = 1;
            if (timer7<4000) {
                kro_1 = 1;
                kro_2 = 1;
            }


            if (timer7 == 0 ) {           
                //todo
                //zapis flach
                
                unsigned char tmp;
                tmp = (czas_s & 0x00ff);
                erase_flash(adr_hi, adr_low);
                write_flash(adr_hi, adr_low, tmp );   //low             
                tmp =  ((czas_s >>8)&0x00ff);
                write_flash(adr_hi, adr_low2, tmp);   //high
  
                flash_low = read_flash(adr_hi, adr_low);                            
                flash_hi = (read_flash(adr_hi, adr_low2) );
    
    
                czas_flash = czas_s;
                timer7 = 5000;
            }
        }else{
            timer7 = 5000;
        }

        
        
        
        //**********************************************************************
        
        
        
        
   

        //przycisk grzalki
        if ( (PRZYCISK == 0) )  {
            if ( (START_Fl ==0) && (timer3<10) ) {
                if (START == 0 ) {
                    START = 1;
                    grzalka_numer = 1;
                }else{
                    START = 0;
                }                
                START_Fl = 1; //flaga wejscia
            }
        }else{
            START_Fl = 0;                   
            timer3 = 60;
        }
  
        //grzanie start        
        if (czas_s < 1) {
            czas_s = 1;
        }
        czas_ms = czas_s *1000UL;      
        if (grzalka_numer>6) {
            grzalka_numer =1;
        }      
        if ( (START == 1) && (timer5 ==0) ) {
            timer5 = czas_ms;
            
             switch (grzalka_numer) {
                 case 1: //Q1
                     Q1 = 1;
                     Q2 = 0;
                     Q3 = 0;
                     break;               
                 case 2: //Q1
                     timer5 = czas_przerwa_ms;
                     Q1 = 0;     
                     Q2 = 0;         
                     Q3 = 0;                    
                     break;
                 case 3: //Q2
                     Q1 = 0;
                     Q2 = 1;
                     Q3 = 0;
                     break;
                 case 4: //Q1
                     timer5 = czas_przerwa_ms;
                     Q1 = 0;     
                     Q2 = 0;         
                     Q3 = 0;
                     break;
                 case 5: //Q3                
                     Q1 = 0;
                     Q2 = 0;
                     Q3 = 1;
                     break;
                 case 6: //Q1
                     timer5 = czas_przerwa_ms;
                     Q1 = 0;     
                     Q2 = 0;         
                     Q3 = 0;
                     break;
             }
             grzalka_numer++;  
        }
        //miganie przycisku 0,2s
        if (START == 1) {
            if (timer4 ==0) {
                timer4 =200;
                LED ^= 1; 
            }
        }

        //STOP GRZALKI
        if (START == 0) { // wy?acz wszystkie             
            Q1 = 0;     
            Q2 = 0;         
            Q3 = 0;
            LED = 0;
            timer5 = 0;
        }       
        
        
        
        
                
        //wyswietlanie cyfr
        if (START ==1) {
            volatile unsigned int liczba_tmp;
            liczba_tmp = timer5/1000;
            wyswietl( liczba_tmp +1 );
        }else{
            wyswietl(czas_s);
            //wyswietl(flash_low);        
        }
        
        
        //zabezpieczenie 30 minut
        if (START == 0) {
            timer2 = 1800000;
        }else{
            if (timer2 == 0) {
                START =0; //wylacz grzalki po 30 minutach 
            }
        }

        CLRWDT();
    }

    return (EXIT_SUCCESS);
}


//funkcja
void wyswietl(unsigned int liczba)
{

    
    if ( liczba > 599 ) {
        liczba = liczba/60;
        cyf_1 = liczba/10;
        cyf_2 = liczba%10;
        kro_1 = 0;
        kro_2 = 1;
    }else if( liczba > 99 ){
        cyf_1 = liczba/60;
        kro_1 = 1;
        kro_2 = 0;
        cyf_2 = (liczba%60)/10;
    }else if (liczba > 9) { //2cyfry
        cyf_1 = liczba/10;
        cyf_2 = liczba%10;    
        kro_1 = 0;
        kro_2 = 0;
    }else{
        cyf_1 = NIC;
        cyf_2 = liczba;
        kro_1 = 0;
        kro_2 = 0;
    }

}



unsigned char read_flash( unsigned char adrs_hi,unsigned char adrs_lo) // read the flash memory
{
    
//INTCONbits.GIE = 0;
PMCON1bits.CFGS=0; // don't select configure bits.
PMADRL=adrs_lo ;//LSB bits of Flash memory address
PMADRH=adrs_hi;//MSB bits of Flash memory address

PMCON1bits.RD = 1; // Flash Read
NOP();
NOP();
//INTCONbits.GIE = 1;
return PMDATL; //return value
}




//--------------------unlock flash to write data------------------------
void unlock_flash(void)
{
PMCON2=0x55;
PMCON2=0xAA;
PMCON1bits.WR=1;
NOP();
NOP();


}


//----------------------------write falsh memory---------------
void write_flash(unsigned char adr_hi,unsigned char adr_lo,unsigned char data_lo)
{
INTCONbits.GIE = 0; // desable all itterupts
PMCON1bits.CFGS=0;

PMADRH=adr_hi; // address high
PMADRL=adr_lo; // put address low

PMCON1bits.FREE = 0;
PMCON1bits.LWLO=1;
PMCON1bits.WREN=1; // writing bit enable

PMDATL=data_lo; // data add of low - bit
PMDATH = 0;

//koniec wpisywania danych
PMCON1bits.LWLO = 0;
unlock_flash(); // writing function enable
__delay_ms(3);
PMCON1bits.WREN = 0;

INTCONbits.GIE = 1; // again set intturrput velues.
}

//----------------------------write falsh memory---------------
void erase_flash(unsigned char adr_hi,unsigned char adr_lo)
{
INTCONbits.GIE = 0; // desable all itterupts
PMCON1bits.CFGS=0;

PMADRH=adr_hi; // address high
PMADRL=adr_lo; // put address low

PMCON1bits.FREE = 1;
PMCON1bits.WREN=1; // writing bit enable
unlock_flash(); // writing function enable
__delay_ms(3);
PMCON1bits.WREN = 0;

INTCONbits.GIE = 1; // again set intturrput velues.
}


//przerwanie

void interrupt ISR(void)
{

    if ( INTCONbits.TMR0IF == 1 ) {

        if (multi>1) {
            
            LATB |= 0b00000011;  //WYGASZENIE WSZYSTKICH LED
            LATC = 255;
            //cyfra 1
            if(idx==0){
                if (kro_1 == 1) {
                    LATC =  ~(cyfry[cyf_1] | SEG_DP);
                    LATB &= ~an[idx] ; //wybor anody
                }else if( cyf_1 != 10 ){
                    LATC =  ~(cyfry[cyf_1] & ~SEG_DP);
                    LATB &= ~an[idx] ; //wybor anody
                }
            }
            //cyfra 2
            if(idx==1){
                if (kro_2 == 1) {
                    LATC =  ~(cyfry[cyf_2] | SEG_DP);
                    LATB &= ~an[idx] ; //wybor anody
                }else if( cyf_2 != 10 ){
                    LATC =  ~(cyfry[cyf_2] & ~SEG_DP);
                    LATB &= ~an[idx] ; //wybor anody
                }
            }


            if(++idx>1) idx=0;  //zmiana 0-1 dla kolejnych liczb
            multi =0;
        }
        multi++;



        if (timer1 >0) {
            timer1--;
        }
                
        if (timer2 >0) {
            timer2--;
        }
                      
        if (timer3 >0) {
            timer3--;
        }

        if (timer4 >0) {
            timer4--;
        }

        if (timer5 >0) {
            timer5--;
        }
        
        if (timer6 >0) {
            timer6--;
        }

        if (timer7 >0) {
            timer7--;
        }
        
        INTCONbits.TMR0IF = 0;
    }





}


