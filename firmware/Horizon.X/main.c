// Bicopter MPU Code
// Written by Matthew Santos

#include "mcc_generated_files/system/system.h"

//Program Functions
void SYSTEM_STARTUP(void);
void SYSTEM_UPDATE(void);
void APC220_Receiver_Interrupt(void);
void MPU_6050_Interrupt(void);
void ESC_Programming(uint8_t *settings);
uint16_t make16(uint8_t Left,uint8_t Right);

//Program Configuration
#define CMD_BYTES 			3
#define IMU_Addrs 			0x0068
#define IMU_Accel_Addrs 	0x3bu
#define IMU_Gyro_Addrs 		0x43u
#define IMU_Gyro_Range		0u	//0=250,1=500,2=1000,3=2000 [deg/s]
#define IMU_Accel_Range		0u	//0=2g,1=4g,2=8g,3=16g
#define BAT_Sense_Channel 	0x2
#define ESC_Brake			1
#define ESC_Timing			2
#define ESC_StartForce		1
#define ESC_CurveMode		1
#define ESC_CtrlFreq		2
#define ESC_LVProt			1
#define ESC_Cutoff			1
#define ESC_RotDir			1
#define ESC_nParameters		8

//Global Variables
char cmd[CMD_BYTES];
struct {
    bool UART_Data;
    bool IMU_Data;
    bool BAT_Data;
} Flags = {0,0,0};
uint16_t BAT_LEVEL = 0;
struct {
	uint8_t accel[6];
	uint8_t gyro[6];
} IMU_Data;
uint8_t ESC_Setting[ESC_nParameters] = {
	ESC_Brake,
	ESC_Timing,
	ESC_StartForce,
	ESC_CurveMode,
	ESC_CtrlFreq,
	ESC_LVProt,
	ESC_Cutoff,
	ESC_RotDir
};

//Main Program
int main(void){
    SYSTEM_Initialize();
    //Test Program
    //------------------
    while(1){
        BAT_LEVEL = (ADC_GetConversion(BAT_SENSE)>>6);
        //1024 = max
        PWM3_LoadDutyValue(BAT_LEVEL);
        PWM4_LoadDutyValue(BAT_LEVEL);
    }
    
    
    
    //------------------
    //Main Program
    SYSTEM_STARTUP();
    while(1){
        SYSTEM_UPDATE();
        //Note Battery still needs a trigger
    }
}

//Support Functions
//---------------------

//Perform Startup Procedure
void SYSTEM_STARTUP(void){
    //Disable Interrupts
    INTERRUPT_GlobalInterruptDisable(); 
    INTERRUPT_PeripheralInterruptDisable(); 
    
    //Setup UART Communications
    EUSART_Enable();
	EUSART_RxCompleteCallbackRegister(APC220_Receiver_Interrupt);
	EUSART_ReceiveInterruptEnable();
	EUSART_ReceiveEnable();
	//EUSART_TxCompleteCallbackRegister();
    EUSART_TransmitInterruptEnable();
	EUSART_TransmitEnable();
	
	//Setup IMU
/*	uint8_t config[4] = [
		0x1a, //Start Register Address
		0x00,	//CONFIG
		IMU_Gyro_Range<<3,	//GYRO_CONFIG
		IMU_Gyro_Range<<3,	//ACCEL_CONFIG
		];
	I2C1_Write(IMU_Addrs,config, 4);
	INT_IMU_SetInterruptHandler(MPU_6050_Interrupt);
    */
    //Enable Interrupts
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
	
	//Startup Delay
	__delay_ms(1000);
	printf("Ready\r\n");
}

//Performs System Updates
void SYSTEM_UPDATE(void){
    if(Flags.UART_Data){  //Process Received UART Command
        switch(cmd[0]){
            case 'L':   //Set Left Motor PWM
                PWM3_LoadDutyValue(make16(cmd[2],cmd[1]));
                break;
            case 'R':   //Set Right Motor PWM
                PWM4_LoadDutyValue(make16(cmd[2],cmd[1]));
                break;
            case 'E':   //Enable ESC
                ESC_EN_SetHigh();
				printf("ESC Enabled\r\n");
                break;
            case 'e':   //Disable ESC
                ESC_EN_SetLow();
				printf("ESC Disabled\r\n");
                break;
			case 'P':	//Program/Calibrate ESC
                printf("ESC Programming Start\r\n");
				ESC_Programming(ESC_Setting);
				printf("ESC Programming Complete\r\n");
				break;
            case 'B':   //Print Battery Level
                printf("%u",BAT_LEVEL);
                break;
			case 'I':	//Enable IMU Output Stream
				EXT_INT_InterruptEnable();
				printf("IMU Stream Enabled\r\n");
				break;
			case 'i':	//Disable IMU Output Stream
				EXT_INT_InterruptDisable();
				printf("IMU Stream Disabled\r\n");
				break;
            default:
				printf("CMD not Recognized\r\n");
                break;
        }
        Flags.UART_Data = 0;
    }
    else if(Flags.IMU_Data){
        //Get New IMU Data
        //I2C1_WriteRead(IMU_Addrs,IMU_Accel_Addrs,1,IMU_Data.accel,6);
        //I2C1_WriteRead(IMU_Addrs,IMU_Gyro_Addrs,1,IMU_Data.gyro,6);
        //Format IMU Data for Transmission
        int16_t ax = (int16_t) make16(IMU_Data.accel[0],IMU_Data.accel[1]);
        int16_t ay = (int16_t) make16(IMU_Data.accel[2],IMU_Data.accel[3]);
        int16_t az = (int16_t) make16(IMU_Data.accel[4],IMU_Data.accel[5]);
        int16_t x = (int16_t) make16(IMU_Data.gyro[0],IMU_Data.gyro[1]);
        int16_t y = (int16_t) make16(IMU_Data.gyro[2],IMU_Data.gyro[3]);
        int16_t z = (int16_t) make16(IMU_Data.gyro[4],IMU_Data.gyro[5]);
        //Send New IMU Data
        printf("%d,%d,%d,%d,%d,%d",x,y,z,ax,ay,az);
        Flags.IMU_Data = 0;
    }
    else if(Flags.BAT_Data){
        BAT_LEVEL = ADC_GetConversion(BAT_SENSE);	//Update Battery Data
        Flags.BAT_Data = 0;
    }
}
//APC220_Receiver_Interrupt
void APC220_Receiver_Interrupt(void){
	static uint8_t index = 0;   //Tracks Last Entry
    char msg = EUSART_Read();
    if(msg == '\r'){    //Execute Stored Command
        Flags.UART_Data = 1;
        index = 0;
    }
    else{	//Store CMD Data and Increment Index
        cmd[index] = msg;
		index = (index + 1) % CMD_BYTES;
    }
}

//MPU_6050_Interrupt
void MPU_6050_Interrupt(void){
    Flags.IMU_Data = 1;
}

//ESC Programming Mode
void ESC_Programming(uint8_t *settings){
	//Reset the ESC
    printf("Reseting ESCs\r\n");
	ESC_EN_SetLow();
	__delay_ms(2000);
	//Startup in Throttle Setting Mode
	PWM3_LoadDutyValue(0xFFFF);
	PWM4_LoadDutyValue(0xFFFF);
	__delay_ms(100);
    printf("Powering on ESCs\r\n");
	ESC_EN_SetHigh();
	//Set Max Throttle
	__delay_ms(2000); 	// (2x) beep
	//Enter Programming Mode
    printf("Entering Programming Mode\r\n");
	__delay_ms(2000);	// beep 123 123
	//Program All Parameters
	for (uint8_t i = 0; i < ESC_nParameters;i++){
        printf("Programming Parameter %u\r\n",i);
		//Select Next Parameter
        for (int c=0;c<i;c++)
            __delay_ms(1000);	
		PWM3_LoadDutyValue(0x0000);
		PWM4_LoadDutyValue(0x0000);
		//Select Parameter Value Option
        for (int c=0;c<settings[i];c++)
            __delay_ms(1000);
		PWM3_LoadDutyValue(0xFFFF);
		PWM4_LoadDutyValue(0xFFFF);
	}
	//Exit Programming Mode
	__delay_ms(1000);
	PWM3_LoadDutyValue(0x0000);
	PWM4_LoadDutyValue(0x0000);
	__delay_ms(2000);
}

//Combine two 8bit registers into a 16bit
uint16_t make16(uint8_t Left,uint8_t Right){
    uint16_t result = ((uint16_t) Left ) << 8;
    return result | Right;
}