/**
 * Generated Pins header File
 * 
 * @file pins.h
 * 
 * @defgroup  pinsdriver Pins Driver
 * 
 * @brief This is generated driver header for pins. 
 *        This header file provides APIs for all pins selected in the GUI.
 *
 * @version Driver Version  3.0.0
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

#ifndef PINS_H
#define PINS_H

#include <xc.h>

#define INPUT   1
#define OUTPUT  0

#define HIGH    1
#define LOW     0

#define ANALOG      1
#define DIGITAL     0

#define PULL_UP_ENABLED      1
#define PULL_UP_DISABLED     0

// get/set IO_RA2 aliases
#define BAT_SENSE_TRIS                 TRISAbits.TRISA2
#define BAT_SENSE_LAT                  LATAbits.LATA2
#define BAT_SENSE_PORT                 PORTAbits.RA2
#define BAT_SENSE_WPU                  WPUAbits.WPUA2
#define BAT_SENSE_OD                   ODCONAbits.ODA2
#define BAT_SENSE_ANS                  ANSELAbits.ANSA2
#define BAT_SENSE_SetHigh()            do { LATAbits.LATA2 = 1; } while(0)
#define BAT_SENSE_SetLow()             do { LATAbits.LATA2 = 0; } while(0)
#define BAT_SENSE_Toggle()             do { LATAbits.LATA2 = ~LATAbits.LATA2; } while(0)
#define BAT_SENSE_GetValue()           PORTAbits.RA2
#define BAT_SENSE_SetDigitalInput()    do { TRISAbits.TRISA2 = 1; } while(0)
#define BAT_SENSE_SetDigitalOutput()   do { TRISAbits.TRISA2 = 0; } while(0)
#define BAT_SENSE_SetPullup()          do { WPUAbits.WPUA2 = 1; } while(0)
#define BAT_SENSE_ResetPullup()        do { WPUAbits.WPUA2 = 0; } while(0)
#define BAT_SENSE_SetPushPull()        do { ODCONAbits.ODA2 = 0; } while(0)
#define BAT_SENSE_SetOpenDrain()       do { ODCONAbits.ODA2 = 1; } while(0)
#define BAT_SENSE_SetAnalogMode()      do { ANSELAbits.ANSA2 = 1; } while(0)
#define BAT_SENSE_SetDigitalMode()     do { ANSELAbits.ANSA2 = 0; } while(0)
// get/set IO_RA4 aliases
#define R_PWM_TRIS                 TRISAbits.TRISA4
#define R_PWM_LAT                  LATAbits.LATA4
#define R_PWM_PORT                 PORTAbits.RA4
#define R_PWM_WPU                  WPUAbits.WPUA4
#define R_PWM_OD                   ODCONAbits.ODA4
#define R_PWM_ANS                  ANSELAbits.ANSA4
#define R_PWM_SetHigh()            do { LATAbits.LATA4 = 1; } while(0)
#define R_PWM_SetLow()             do { LATAbits.LATA4 = 0; } while(0)
#define R_PWM_Toggle()             do { LATAbits.LATA4 = ~LATAbits.LATA4; } while(0)
#define R_PWM_GetValue()           PORTAbits.RA4
#define R_PWM_SetDigitalInput()    do { TRISAbits.TRISA4 = 1; } while(0)
#define R_PWM_SetDigitalOutput()   do { TRISAbits.TRISA4 = 0; } while(0)
#define R_PWM_SetPullup()          do { WPUAbits.WPUA4 = 1; } while(0)
#define R_PWM_ResetPullup()        do { WPUAbits.WPUA4 = 0; } while(0)
#define R_PWM_SetPushPull()        do { ODCONAbits.ODA4 = 0; } while(0)
#define R_PWM_SetOpenDrain()       do { ODCONAbits.ODA4 = 1; } while(0)
#define R_PWM_SetAnalogMode()      do { ANSELAbits.ANSA4 = 1; } while(0)
#define R_PWM_SetDigitalMode()     do { ANSELAbits.ANSA4 = 0; } while(0)
// get/set IO_RA5 aliases
#define L_PWM_TRIS                 TRISAbits.TRISA5
#define L_PWM_LAT                  LATAbits.LATA5
#define L_PWM_PORT                 PORTAbits.RA5
#define L_PWM_WPU                  WPUAbits.WPUA5
#define L_PWM_OD                   ODCONAbits.ODA5
#define L_PWM_ANS                  ANSELAbits.
#define L_PWM_SetHigh()            do { LATAbits.LATA5 = 1; } while(0)
#define L_PWM_SetLow()             do { LATAbits.LATA5 = 0; } while(0)
#define L_PWM_Toggle()             do { LATAbits.LATA5 = ~LATAbits.LATA5; } while(0)
#define L_PWM_GetValue()           PORTAbits.RA5
#define L_PWM_SetDigitalInput()    do { TRISAbits.TRISA5 = 1; } while(0)
#define L_PWM_SetDigitalOutput()   do { TRISAbits.TRISA5 = 0; } while(0)
#define L_PWM_SetPullup()          do { WPUAbits.WPUA5 = 1; } while(0)
#define L_PWM_ResetPullup()        do { WPUAbits.WPUA5 = 0; } while(0)
#define L_PWM_SetPushPull()        do { ODCONAbits.ODA5 = 0; } while(0)
#define L_PWM_SetOpenDrain()       do { ODCONAbits.ODA5 = 1; } while(0)
#define L_PWM_SetAnalogMode()      do { ANSELAbits. = 1; } while(0)
#define L_PWM_SetDigitalMode()     do { ANSELAbits. = 0; } while(0)
// get/set IO_RC0 aliases
#define IO_RC0_TRIS                 TRISCbits.TRISC0
#define IO_RC0_LAT                  LATCbits.LATC0
#define IO_RC0_PORT                 PORTCbits.RC0
#define IO_RC0_WPU                  WPUCbits.WPUC0
#define IO_RC0_OD                   ODCONCbits.ODC0
#define IO_RC0_ANS                  ANSELCbits.ANSC0
#define IO_RC0_SetHigh()            do { LATCbits.LATC0 = 1; } while(0)
#define IO_RC0_SetLow()             do { LATCbits.LATC0 = 0; } while(0)
#define IO_RC0_Toggle()             do { LATCbits.LATC0 = ~LATCbits.LATC0; } while(0)
#define IO_RC0_GetValue()           PORTCbits.RC0
#define IO_RC0_SetDigitalInput()    do { TRISCbits.TRISC0 = 1; } while(0)
#define IO_RC0_SetDigitalOutput()   do { TRISCbits.TRISC0 = 0; } while(0)
#define IO_RC0_SetPullup()          do { WPUCbits.WPUC0 = 1; } while(0)
#define IO_RC0_ResetPullup()        do { WPUCbits.WPUC0 = 0; } while(0)
#define IO_RC0_SetPushPull()        do { ODCONCbits.ODC0 = 0; } while(0)
#define IO_RC0_SetOpenDrain()       do { ODCONCbits.ODC0 = 1; } while(0)
#define IO_RC0_SetAnalogMode()      do { ANSELCbits.ANSC0 = 1; } while(0)
#define IO_RC0_SetDigitalMode()     do { ANSELCbits.ANSC0 = 0; } while(0)
// get/set IO_RC1 aliases
#define IO_RC1_TRIS                 TRISCbits.TRISC1
#define IO_RC1_LAT                  LATCbits.LATC1
#define IO_RC1_PORT                 PORTCbits.RC1
#define IO_RC1_WPU                  WPUCbits.WPUC1
#define IO_RC1_OD                   ODCONCbits.ODC1
#define IO_RC1_ANS                  ANSELCbits.ANSC1
#define IO_RC1_SetHigh()            do { LATCbits.LATC1 = 1; } while(0)
#define IO_RC1_SetLow()             do { LATCbits.LATC1 = 0; } while(0)
#define IO_RC1_Toggle()             do { LATCbits.LATC1 = ~LATCbits.LATC1; } while(0)
#define IO_RC1_GetValue()           PORTCbits.RC1
#define IO_RC1_SetDigitalInput()    do { TRISCbits.TRISC1 = 1; } while(0)
#define IO_RC1_SetDigitalOutput()   do { TRISCbits.TRISC1 = 0; } while(0)
#define IO_RC1_SetPullup()          do { WPUCbits.WPUC1 = 1; } while(0)
#define IO_RC1_ResetPullup()        do { WPUCbits.WPUC1 = 0; } while(0)
#define IO_RC1_SetPushPull()        do { ODCONCbits.ODC1 = 0; } while(0)
#define IO_RC1_SetOpenDrain()       do { ODCONCbits.ODC1 = 1; } while(0)
#define IO_RC1_SetAnalogMode()      do { ANSELCbits.ANSC1 = 1; } while(0)
#define IO_RC1_SetDigitalMode()     do { ANSELCbits.ANSC1 = 0; } while(0)
// get/set IO_RC2 aliases
#define INT_IMU_TRIS                 TRISCbits.TRISC2
#define INT_IMU_LAT                  LATCbits.LATC2
#define INT_IMU_PORT                 PORTCbits.RC2
#define INT_IMU_WPU                  WPUCbits.WPUC2
#define INT_IMU_OD                   ODCONCbits.ODC2
#define INT_IMU_ANS                  ANSELCbits.ANSC2
#define INT_IMU_SetHigh()            do { LATCbits.LATC2 = 1; } while(0)
#define INT_IMU_SetLow()             do { LATCbits.LATC2 = 0; } while(0)
#define INT_IMU_Toggle()             do { LATCbits.LATC2 = ~LATCbits.LATC2; } while(0)
#define INT_IMU_GetValue()           PORTCbits.RC2
#define INT_IMU_SetDigitalInput()    do { TRISCbits.TRISC2 = 1; } while(0)
#define INT_IMU_SetDigitalOutput()   do { TRISCbits.TRISC2 = 0; } while(0)
#define INT_IMU_SetPullup()          do { WPUCbits.WPUC2 = 1; } while(0)
#define INT_IMU_ResetPullup()        do { WPUCbits.WPUC2 = 0; } while(0)
#define INT_IMU_SetPushPull()        do { ODCONCbits.ODC2 = 0; } while(0)
#define INT_IMU_SetOpenDrain()       do { ODCONCbits.ODC2 = 1; } while(0)
#define INT_IMU_SetAnalogMode()      do { ANSELCbits.ANSC2 = 1; } while(0)
#define INT_IMU_SetDigitalMode()     do { ANSELCbits.ANSC2 = 0; } while(0)
#define RC2_SetInterruptHandler  INT_IMU_SetInterruptHandler
// get/set IO_RC3 aliases
#define IO_RC3_TRIS                 TRISCbits.TRISC3
#define IO_RC3_LAT                  LATCbits.LATC3
#define IO_RC3_PORT                 PORTCbits.RC3
#define IO_RC3_WPU                  WPUCbits.WPUC3
#define IO_RC3_OD                   ODCONCbits.ODC3
#define IO_RC3_ANS                  ANSELCbits.ANSC3
#define IO_RC3_SetHigh()            do { LATCbits.LATC3 = 1; } while(0)
#define IO_RC3_SetLow()             do { LATCbits.LATC3 = 0; } while(0)
#define IO_RC3_Toggle()             do { LATCbits.LATC3 = ~LATCbits.LATC3; } while(0)
#define IO_RC3_GetValue()           PORTCbits.RC3
#define IO_RC3_SetDigitalInput()    do { TRISCbits.TRISC3 = 1; } while(0)
#define IO_RC3_SetDigitalOutput()   do { TRISCbits.TRISC3 = 0; } while(0)
#define IO_RC3_SetPullup()          do { WPUCbits.WPUC3 = 1; } while(0)
#define IO_RC3_ResetPullup()        do { WPUCbits.WPUC3 = 0; } while(0)
#define IO_RC3_SetPushPull()        do { ODCONCbits.ODC3 = 0; } while(0)
#define IO_RC3_SetOpenDrain()       do { ODCONCbits.ODC3 = 1; } while(0)
#define IO_RC3_SetAnalogMode()      do { ANSELCbits.ANSC3 = 1; } while(0)
#define IO_RC3_SetDigitalMode()     do { ANSELCbits.ANSC3 = 0; } while(0)
// get/set IO_RC4 aliases
#define ESC_EN_TRIS                 TRISCbits.TRISC4
#define ESC_EN_LAT                  LATCbits.LATC4
#define ESC_EN_PORT                 PORTCbits.RC4
#define ESC_EN_WPU                  WPUCbits.WPUC4
#define ESC_EN_OD                   ODCONCbits.ODC4
#define ESC_EN_ANS                  ANSELCbits.ANSC4
#define ESC_EN_SetHigh()            do { LATCbits.LATC4 = 1; } while(0)
#define ESC_EN_SetLow()             do { LATCbits.LATC4 = 0; } while(0)
#define ESC_EN_Toggle()             do { LATCbits.LATC4 = ~LATCbits.LATC4; } while(0)
#define ESC_EN_GetValue()           PORTCbits.RC4
#define ESC_EN_SetDigitalInput()    do { TRISCbits.TRISC4 = 1; } while(0)
#define ESC_EN_SetDigitalOutput()   do { TRISCbits.TRISC4 = 0; } while(0)
#define ESC_EN_SetPullup()          do { WPUCbits.WPUC4 = 1; } while(0)
#define ESC_EN_ResetPullup()        do { WPUCbits.WPUC4 = 0; } while(0)
#define ESC_EN_SetPushPull()        do { ODCONCbits.ODC4 = 0; } while(0)
#define ESC_EN_SetOpenDrain()       do { ODCONCbits.ODC4 = 1; } while(0)
#define ESC_EN_SetAnalogMode()      do { ANSELCbits.ANSC4 = 1; } while(0)
#define ESC_EN_SetDigitalMode()     do { ANSELCbits.ANSC4 = 0; } while(0)
// get/set IO_RC5 aliases
#define IO_RC5_TRIS                 TRISCbits.TRISC5
#define IO_RC5_LAT                  LATCbits.LATC5
#define IO_RC5_PORT                 PORTCbits.RC5
#define IO_RC5_WPU                  WPUCbits.WPUC5
#define IO_RC5_OD                   ODCONCbits.ODC5
#define IO_RC5_ANS                  ANSELCbits.ANSC5
#define IO_RC5_SetHigh()            do { LATCbits.LATC5 = 1; } while(0)
#define IO_RC5_SetLow()             do { LATCbits.LATC5 = 0; } while(0)
#define IO_RC5_Toggle()             do { LATCbits.LATC5 = ~LATCbits.LATC5; } while(0)
#define IO_RC5_GetValue()           PORTCbits.RC5
#define IO_RC5_SetDigitalInput()    do { TRISCbits.TRISC5 = 1; } while(0)
#define IO_RC5_SetDigitalOutput()   do { TRISCbits.TRISC5 = 0; } while(0)
#define IO_RC5_SetPullup()          do { WPUCbits.WPUC5 = 1; } while(0)
#define IO_RC5_ResetPullup()        do { WPUCbits.WPUC5 = 0; } while(0)
#define IO_RC5_SetPushPull()        do { ODCONCbits.ODC5 = 0; } while(0)
#define IO_RC5_SetOpenDrain()       do { ODCONCbits.ODC5 = 1; } while(0)
#define IO_RC5_SetAnalogMode()      do { ANSELCbits.ANSC5 = 1; } while(0)
#define IO_RC5_SetDigitalMode()     do { ANSELCbits.ANSC5 = 0; } while(0)
/**
 * @ingroup  pinsdriver
 * @brief GPIO and peripheral I/O initialization
 * @param none
 * @return none
 */
void PIN_MANAGER_Initialize (void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handling routine
 * @param none
 * @return none
 */
void PIN_MANAGER_IOC(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt on Change Handler for the INT_IMU pin functionality
 * @param none
 * @return none
 */
void INT_IMU_ISR(void);

/**
 * @ingroup  pinsdriver
 * @brief Interrupt Handler Setter for INT_IMU pin interrupt-on-change functionality.
 *        Allows selecting an interrupt handler for INT_IMU at application runtime.
 * @pre Pins intializer called
 * @param InterruptHandler function pointer.
 * @return none
 */
void INT_IMU_SetInterruptHandler(void (* InterruptHandler)(void));

/**
 * @ingroup  pinsdriver
 * @brief Dynamic Interrupt Handler for INT_IMU pin.
 *        This is a dynamic interrupt handler to be used together with the INT_IMU_SetInterruptHandler() method.
 *        This handler is called every time the INT_IMU ISR is executed and allows any function to be registered at runtime.
 * @pre Pins intializer called
 * @param none
 * @return none
 */
extern void (*INT_IMU_InterruptHandler)(void);

/**
 * @ingroup  pinsdriver
 * @brief Default Interrupt Handler for INT_IMU pin. 
 *        This is a predefined interrupt handler to be used together with the INT_IMU_SetInterruptHandler() method.
 *        This handler is called every time the INT_IMU ISR is executed. 
 * @pre Pins intializer called
 * @param none
 * @return none
 */
void INT_IMU_DefaultInterruptHandler(void);


#endif // PINS_H
/**
 End of File
*/