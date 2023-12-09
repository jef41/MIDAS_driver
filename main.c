/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules selected in the GUI.
    Generation Information :
        Product Revision  :  PIC10 / PIC12 / PIC16 / PIC18 MCUs - 1.81.7
        Device            :  PIC16F15325
        Driver Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include "mcc_generated_files/mcc.h"
#include "mcc_generated_files/examples/i2c1_master_example.h"
#include "lcd.h"
#include <string.h>  //EUSART module

//initialise some vartiables
unsigned int timeout_ct = 0, timeout_limit = 500; //LCD backlight timeout & sleep

//Init ADC, pd_values = no of photodiode readings to savee
unsigned int pd_values[40]; //could use a variable length struct rather than fixed array?
uint16_t ADCvalue = 0;
char valueAsString[0x10];
// pd_count = no of photodiode values to average was 32
// 50Hz, 20mS, 1 sample per mS
unsigned int pd_count = 40, sum = 0, average = 0;
char StoredValueAsString[0x5];

//button state - rising or falling edge
char OldState = 0;
char NewState = 0;
unsigned int Timer1Count = 0;
//reads the photodiode and averages the result out
void readPhotodiode(void)
{   
    LCD_WriteIR(CLEAR_DISPLAY);
    LCD_WriteString("reading.....", LINE_1_ADR | 0x0);
    //get readings & populate array
    
    // this should check peak to peak voltage & report error if > ?100?mV - or whatever error we allow
    IO_RA2_SetHigh(); //turn on LCD for photodiode
    __delay_ms(15); //Wait 15 ms to allow for photodiode/TIA overshoot
    sum = 0; 
    average = 0;
    for (unsigned int c = 0; c<pd_count; c++)
    {
        pd_values[c] = ADC_GetConversion(channel_ANC2);
        sum += pd_values[c];
        // 50 Hz wave, 32 values
        __delay_us(504); //(875); //1ms - sample time 46us
    }
    IO_RA2_SetLow(); //turn off LCD for photodiode
    average = sum/pd_count;
    printf("Start:\r\n");
    for (unsigned int c = 0; c<pd_count; c++)
    {
        // debug print values to usart;
        sprintf(StoredValueAsString,"%4u",pd_values[c]);
        for(uint8_t i = 0; i < strlen(StoredValueAsString); i++)
            {
            EUSART1_Write(StoredValueAsString[i]);
            }
    }
    printf("\r\n");

    //LCD_WriteString("Awake!", LINE_2_ADR);
    sprintf(valueAsString,"ADC value: %4u",average); 
    LCD_WriteIR(CLEAR_DISPLAY);
    LCD_WriteString(valueAsString, LINE_2_ADR | 0x0);
}
//timer1 ISR
void myTimer1ISR(void);
void myTimer1ISR(void)
{
   // add your TMR1 interrupt custom code
   // or set custom function using 
   ++Timer1Count;
   // Clear the TMR1 interrupt flag
   PIR4bits.TMR1IF = 0;
   TMR1_Reload();
}
/*
                         Main application
 */
void main(void)
{
    // initialize the device
    SYSTEM_Initialize();
    //TMR1_StopTimer();
    //TMR1_SetInterruptHandler (myTimer1ISR);  //Define interrupt Handler
    TMR1_SetInterruptHandler (myTimer1ISR);  //Define my own interrupt Handler

    // When using interrupts, you need to set the Global and Peripheral Interrupt Enable bits
    // Use the following macros to:

    // Enable the Global Interrupts
    INTERRUPT_GlobalInterruptEnable();

    // Enable the Peripheral Interrupts
    INTERRUPT_PeripheralInterruptEnable();

    // Disable the Global Interrupts
    //INTERRUPT_GlobalInterruptDisable();

    // Disable the Peripheral Interrupts
    //INTERRUPT_PeripheralInterruptDisable();
    
    //initialise LCD I2C    
    LCD_Init2();
    LCD_WriteString("setup done", LINE_2_ADR);
    //IO_RA1_SetHigh(); //turn on LCD backlight
    __delay_ms(500);
    LCD_WriteIR(CLEAR_DISPLAY);
    

    ADC_Initialize();                 //Initialize ADC
    printf("Startup done");
    
    //look at disabling peripherals before sleep - 10mA at present??
    while (1)
    {
        //LCD_WriteString("Going to SLEEP.....", LINE_1_ADR | 0x0);

        //if button is pressed detect rising or falling edge here
        //start timer on rising, end on falling, then act on falling
        //if(CLC1CONbits.LC1OUT){
        char NewState = CLC1CONbits.LC1OUT; //get button state
        if ((OldState == 0) & (NewState == 1)){ //button pressed
            OldState = NewState;
            TMR1_Reload();
            Timer1Count = 0;
            TMR1_StartTimer();
            __delay_ms(50); //allow time to wake before sending serial chars
            printf("button pressed");        
            //TMR1_StopTimer(); //backlight timeout timer
            //TMR1_Reload(); //reset the backlight timeout timer
            IO_RA1_SetHigh(); //turn on LCD backlight
            //__delay_ms(50); //Wait 50 ms 
            printf("\n\rAwake!!     "); //transmit a message over the EUSART that the MCU is now awake
            
            //readPhotodiode();
            timeout_ct = 0; //reset the LCD& Sleep timeout
        }
        // should there be an else if else here to catch some unwanted state?
        if ((OldState == 1) & (NewState == 0)){ //button released
            TMR1_StopTimer();
            OldState = NewState;
            __delay_ms(40); //allow time to wake before sending serial chars
            //printf("Timer1 value: %4u",Timer1Count);
            printf("button released, Timer1 value: %4u",Timer1Count);        
            IO_RA1_SetHigh(); //turn on LCD backlight
            //__delay_ms(50); //Wait 50 ms 
            if (Timer1Count <= 3) //3=1500ms
            {
                printf("\n\rshort press");
                timeout_limit = 500; // set 5 second timeout
                readPhotodiode();
            } else {
                printf("\n\rlong press");
                timeout_limit = 30000; // set 5 minute timeout - test mode
                IO_RA2_SetHigh(); //turn on LCD for photodiode
            }
            timeout_ct = 0; //reset the LCD& Sleep timeout
        }
        //either button is not pressed or we have completed a measurement
        __delay_ms(10); //wait 10 ms before looping and repeating the above
        if (timeout_ct <= timeout_limit){
            timeout_ct++;
        }else{ //we have exceeded the timeout
            timeout_ct = 0;  //reset the count
            IO_RA1_SetLow(); //turn off LCD backlight
            IO_RA2_SetLow(); //turn off LCD for photodiode (if on in test mode)
            OldState = 0; //reset button state, so wait for button press
            printf("\n\rGoing to SLEEP........"); //transmit a message over the EUSART to notify that the MCU is about to go to SLEEP
            SLEEP();
            
        }
        //TMr1 should turn off the display/sleep
        //also need an interrupt for button pressed again
    }
}

/*void myTimer1ISR(void){
    printf("TMR1 ISR\r\n");
    //IO_RA1_Toggle(); //Control LED
    //TMR1_StopTimer(); //backlight timeout timer
    
}*/
//ISR routine for timer - get long or short press
/** 
 on rising edge reset & start timer, 
 on falling edge read timer & act on button press
 
 on button press check timer value & if long or short 
 */
/*void __interrupt() my_isr_routine (void){
        //do somehting
    if(CLC1CONbits.LC1OUT) {
        printf("CLC Out High\r\n");
    }
}*/
/**
 End of File
*/