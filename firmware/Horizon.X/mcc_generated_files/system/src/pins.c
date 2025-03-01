/**
 * Generated Driver File
 * 
 * @file pins.c
 * 
 * @ingroup  pinsdriver
 * 
 * @brief This is generated driver implementation for pins. 
 *        This file provides implementations for pin APIs for all pins selected in the GUI.
 *
 * @version Driver Version 3.0.0
*/

/*
© [2025] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

#include "../pins.h"

void (*INT_IMU_InterruptHandler)(void);

void PIN_MANAGER_Initialize(void)
{
   /**
    LATx registers
    */
    LATA = 0x0;
    LATC = 0x3;

    /**
    TRISx registers
    */
    TRISA = 0xF;
    TRISC = 0x27;

    /**
    ANSELx registers
    */
    ANSELA = 0x37;
    ANSELC = 0x8;

    /**
    WPUx registers
    */
    WPUA = 0x3F;
    WPUC = 0x3F;
    OPTION_REGbits.nWPUEN = 0x0;
  
    /**
    ODx registers
    */
   
    ODCONA = 0x0;
    ODCONC = 0x0;
    /**
    SLRCONx registers
    */
    SLRCONA = 0x37;
    SLRCONC = 0x3F;
    /**
    INLVLx registers
    */
    INLVLA = 0x3F;
    INLVLC = 0x3F;

    /**
    PPS registers
    */
    RXPPS = 0x15; //RC5->EUSART:RX;
    RA5PPS = 14;  //RA5->PWM3:PWM3OUT;
    RA4PPS = 15;  //RA4->PWM4:PWM4OUT;
    RC3PPS = 20;  //RC3->EUSART:TX;
    SSPCLKPPS = 0x10;  //RC0->MSSP:SCL;
    RC0PPS = 16;  //RC0->MSSP:SCL;
    SSPDATPPS = 0x11;  //RC1->MSSP:SDA;
    RC1PPS = 17;  //RC1->MSSP:SDA;

    /**
    APFCON registers
    */

   /**
    IOCx registers 
    */
    IOCAP = 0x0;
    IOCAN = 0x0;
    IOCAF = 0x0;
    IOCCP = 0x4;
    IOCCN = 0x0;
    IOCCF = 0x0;

    INT_IMU_SetInterruptHandler(INT_IMU_DefaultInterruptHandler);

    // Enable INTCONbits.IOCIE interrupt 
    INTCONbits.IOCIE = 1; 
}
  
void PIN_MANAGER_IOC(void)
{
    // interrupt on change for pin INT_IMU}
    if(IOCCFbits.IOCCF2 == 1)
    {
        INT_IMU_ISR();  
    }
}
   
/**
   INT_IMU Interrupt Service Routine
*/
void INT_IMU_ISR(void) {

    // Add custom IOCCF2 code

    // Call the interrupt handler for the callback registered at runtime
    if(INT_IMU_InterruptHandler)
    {
        INT_IMU_InterruptHandler();
    }
    IOCCFbits.IOCCF2 = 0;
}

/**
  Allows selecting an interrupt handler for IOCCF2 at application runtime
*/
void INT_IMU_SetInterruptHandler(void (* InterruptHandler)(void)){
    INT_IMU_InterruptHandler = InterruptHandler;
}

/**
  Default interrupt handler for IOCCF2
*/
void INT_IMU_DefaultInterruptHandler(void){
    // add your INT_IMU interrupt custom code
    // or set custom function using INT_IMU_SetInterruptHandler()
}
/**
 End of File
*/