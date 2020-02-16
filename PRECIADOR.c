//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                SISTEMA GPS - GSM                   $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

#include "18F46K80.h"          // Microcontrolador con el que se va a trabajar
//#include "18F46K22.h"
#device *=16 HIGH_INTS=TRUE
#DEVICE WRITE_EEPROM = NOINT

#include "string.h"  // libreria para manejo de datos
#include "stdlib.h"
#include "math.h"

#Fuses INTRC_HP
#Fuses PROTECT
#FUSES NOWDT                    //No Watch Dog Timer
#FUSES WDT128                   //Watch Dog Timer uses 1:128 Postscale
#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)
#FUSES INTRC_IO                 //HS Oscilator 16-25MHz
#FUSES PLLEN                    //4X HW PLL disabled, 4X PLL enabled in software
#FUSES NOFCMEN                  //Fail-safe clock monitor disabled
#FUSES NOIESO                   //Internal External Switch Over mode disabled
#FUSES NOBROWNOUT               //No brownout reset
#FUSES WDT_SW                   //No Watch Dog Timer, enabled in Software
#FUSES NOMCLR                   //Master Clear pin used for I/O
#FUSES NOSTVREN                 //Stack full/underflow will not cause reset
#FUSES BBSIZ1K                  //1K words Boot Block size
#FUSES BORV30
#FUSES SOSC_DIG                 //Digital mode, I/O port functionality of RC0 and RC1

#use delay( clock = 16 Mhz )
#use rs232( baud = 19200, parity=N, xmit = PIN_C6, rcv = PIN_C7, bits = 8, stream = modemGSM ) // RDA1
#use rs232( baud = 19200, parity=N, xmit = PIN_D6, rcv = PIN_D7, bits = 8, stream = GPS )      // RDA2

//#use rs232( baud = 19200, parity=N, xmit = PIN_C0, rcv = PIN_C1, bits = 8, stream = DEBUGGER )

//------------------------------------------------------
//          Definir puertos del PIC 
//------------------------------------------------------
#use fast_io( A )
#use fast_io( B )
#use fast_io( C )
#use fast_io( D )
#use fast_io( E )
//--------------------------------------------------------
//          Definiciones de pines
//--------------------------------------------------------
#define LED_PILOTO      LATE1//LATA0
#define LED_Toggle      LED_PILOTO = !LED_PILOTO
#define BUZZER          LATA1


//#define ENABLE  LATB3
#define STROBE  LATB2
#define CLOCK   LATB1
#define DATA    LATB0
//--------------------------------------------------------
//          Definir variables constantes
//--------------------------------------------------------
                     

unsigned int8  const   lenbuff = 100;
unsigned int8  const   Tiempo_Muestra_Reporte = 30;

unsigned int8  const   Vel_Local_Min = 4;
unsigned int8  const   Vel_Servr_Min = 7;
//--------------------------------------------------------
//          Definir variables globales 
//--------------------------------------------------------
unsigned int8   DIGITO = 0,
                Binario = 0,
                ii=0,
                Unidad_Numero =0,
                Decena_Numero = 0,
                Centena_Numero = 0,
                Unidad_Millar_Numero = 0;

unsigned int16  Numero_16_bits= 0; 

//--------------------------------------------------------
//            Definir Funciones Globales
//--------------------------------------------------------
void setup( void );
void Numero_8_bits_a_Binario( int8 );

//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                PROGRAMA - PRINCIPAL                $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
void main()
{
    setup( );
   
    enable_interrupts( INT_RDA ); 
    enable_interrupts( INT_RDA2 );
    setup_timer_0( RTCC_INTERNAL | RTCC_DIV_256 | RTCC_8_bit );  // 13.056ms
    enable_interrupts( INT_RTCC );
    enable_interrupts( GLOBAL ); 

} 
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                       FUNCIONES                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$                                                    $$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

//--------------------------------------------------------------------------------------------------------
//$$$$$$$$$$$$$$$                                   setup( )                               $$$$$$$$$$$$$$$
//--------------------------------------------------------------------------------------------------------
void setup( )
{/*
    OSCCON  = 0b01110000;  // configuracion de oscilador para correr a 64 MHZ == 62.5ns
    OSCCON2 = 0b00000000;
    OSCTUNE = 0b01000000;   
  */  
    LATA = 0X00; 
    LATB = 0X00; 
    LATC = 0b00100000;  //0X00; 
    LATD = 0X00; 
    LATE = 0X00;  
    
    //---------------------------------------------------------------------------  
    // configuracion iniciail de estado de
    // perifericos y temporizadores 
    disable_interrupts( global );       // Desabilitar todas las interrupciones
    disable_interrupts( int_timer0 );   // Desabilitar interrupciones del timer0
    disable_interrupts( int_timer1 );   // Desabilitar interrupciones del timer1
    disable_interrupts( int_timer2 );   // Desabilitar interrupciones del timer2
    disable_interrupts( int_timer3 );   // Desabilitar interrupciones del timer3
    disable_interrupts( int_timer4 );   // Desabilitar interrupciones del timer4
    disable_interrupts( int_rda );      // Desabilitar todas las interrupciones
    disable_interrupts( int_rda2 );     // Desabilitar todas las interrupciones
    disable_interrupts( int_ext );      // Desabilitar interrupcion externa INT0
    disable_interrupts( int_ext1 );     // Desabilitar interrupcion externa INT1
    disable_interrupts( int_ext2 );     // Desabilitar interrupcion externa INT2
    setup_adc_ports   ( NO_ANALOGS );   // Desabilitar entradas analogicas
    setup_adc         ( ADC_OFF) ;      // Desabilitar el periferico ADC
    setup_spi         ( FALSE );        // Desabilitar el perferico SPI
    setup_psp         ( PSP_DISABLED ); // Desabilitar el perferico PSP
    setup_comparator  ( NC_NC_NC_NC );  // Desabilitar el perferico COMPARADOR
    port_b_pullups    ( false );        // Resistencias PULL_UP activadas PORT_B
    setup_uart        ( false );        // periferico USART desactivaso
    delay_us          ( 10 );           // Espera a que se activen. 
    //---------------------------------------------------------------------------- 

    set_tris_a( 0b00000000 );           
    set_tris_b( 0b00000000 );
    set_tris_c( 0b10100000 );
    set_tris_d( 0b10000000 );
    set_tris_e( 0b00000000 );


    //FACTOR = 240;
    //DUTY_CICLO = 850;
    //FRECUENCIAS_VELOCIDAD = 1;

    setup_ccp1(CCP_PWM); 
    setup_timer_2(T2_DIV_BY_16,255,16); //1Khz
    //setup_timer_2(T2_DIV_BY_16,255,1); //500 Hz 
    set_pwm1_duty( (int16)800 );

DIGITO = 0;

    Numero_16_bits = 2019;
    
    while( true ) 
         {
             Unidad_Millar_Numero = Numero_16_bits/1000;
             Centena_Numero = (Numero_16_bits%1000)/100;
             Decena_Numero = ((Numero_16_bits%1000)%100)/10;
             Unidad_Numero = ((Numero_16_bits%1000)%100)%10;

             STROBE = 0; 

             //Binario = Numero_8_bits_a_Binario( Unidad_Numero );
             Numero_8_bits_a_Binario( Unidad_Millar_Numero );

             for( ii = 8; ii != 0; ii-- )
                {
                    if( bit_test(Binario,(ii-1))) DATA = 1;
                      else DATA = 0;
                 
                    CLOCK = 0;
                    CLOCK = 1;           
                } 

             Numero_8_bits_a_Binario( Centena_Numero );

             for( ii = 8; ii != 0; ii-- )
                {                   
                    if( bit_test(Binario,(ii-1))) DATA = 1;
                      else DATA = 0;

                    CLOCK = 0;
                    CLOCK = 1;           
                } 

             Numero_8_bits_a_Binario( Decena_Numero );

             for( ii = 8; ii != 0; ii-- )
                {
                   if( (Centena_Numero % 2) == 0) { Binario = Binario | 0b00000001; }
                    if( bit_test(Binario,(ii-1))) DATA = 1;
                      else DATA = 0;

                    CLOCK = 0;
                 CLOCK = 1;           
                } 


             Numero_8_bits_a_Binario( Unidad_Numero );

             for( ii = 8; ii != 0; ii-- )
                {
                   if( (Unidad_Millar_Numero % 2) == 1) { Binario = Binario | 0b00000001; }
                    if( bit_test(Binario,(ii-1))) DATA = 1;
                      else DATA = 0;

                    CLOCK = 0;
                 CLOCK = 1;           
                } 


          STROBE = 1;

          delay_ms( 80 );

          Numero_16_bits++;

          if( Numero_16_bits > 9999 )
            {
                Numero_16_bits = 0;
                Numero_8_bits_a_Binario( 8 );
                
                         STROBE = 0;   
                   
                   for( ii = 8; ii != 0; ii-- )
                      {
                          if( bit_test(Binario,(ii-1))) DATA = 1;
                            else DATA = 0;
                          
                          CLOCK = 0;
                          CLOCK = 1;           
                      } 
         
                   for( ii = 8; ii != 0; ii-- )
                      {
                          if( bit_test(Binario,(ii-1))) DATA = 1;
                            else DATA = 0;

                          CLOCK = 0;
                          CLOCK = 1;           
                      } 
         


                   STROBE = 1;
                   delay_ms( 3000 );
                   
                   set_pwm1_duty( (int16)900 );
                   delay_ms( 3000 );
                   
                   set_pwm1_duty( (int16)1000 );
                   delay_ms( 3000 );
 
                    set_pwm1_duty( (int16)1023 );
                   delay_ms( 3000 );
                   
                   set_pwm1_duty( (int16)800 );
            }

      }
  
    while( true ) 
       { 
           LED_PILOTO = 1;
           delay_ms( 500 );
           
           LED_PILOTO = 0;
           delay_ms( 500 );
       } 

    LED_PILOTO = 1; 
}    
   

void Numero_8_bits_a_Binario( int8 Numero_8_Bits )
{
    switch( Numero_8_Bits )
          {      
            //-fgedcbap
              case 0: Binario = 0b10111110; break;
              case 1: Binario = 0b00001100; break;
              case 2: Binario = 0b01110110; break;
              case 3: Binario = 0b01011110; break;
              case 4: Binario = 0b11001100; break;
              case 5: Binario = 0b11011010; break;
              case 6: Binario = 0b11111010; break;
              case 7: Binario = 0b00001110; break;
              case 8: Binario = 0b11111110; break;  
              case 9: Binario = 0b11011110; break;                   
          }
}

