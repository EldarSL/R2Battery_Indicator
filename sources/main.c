/********************************************************************
 FileName:      main.c
 Dependencies:  See INCLUDES section
 Processor:     PIC18, PIC24, dsPIC, and PIC32 USB Microcontrollers
 Hardware:      This demo is natively intended to be used on Microchip USB demo
                boards supported by the MCHPFSUSB stack.  See release notes for
                support matrix.  This demo can be modified for use on other 
                hardware platforms.
 Complier:      Microchip C18 (for PIC18), XC16 (for PIC24/dsPIC), XC32 (for PIC32)
 Company:       Microchip Technology, Inc.

 Software License Agreement:

 The software supplied herewith by Microchip Technology Incorporated
 (the "Company") for its PIC(R) Microcontroller is intended and
 supplied to you, the Company's customer, for use solely and
 exclusively on Microchip PIC Microcontroller products. The
 software is owned by the Company and/or its supplier, and is
 protected under applicable copyright laws. All rights are reserved.
 Any use in violation of the foregoing restrictions may subject the
 user to criminal sanctions under applicable laws, as well as to
 civil liability for the breach of the terms and conditions of this
 license.

 THIS SOFTWARE IS PROVIDED IN AN "AS IS" CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

********************************************************************
 File Description:

 Change History:
  Rev   Description
  ----  -----------------------------------------
  1.0   Initial release
  2.1   Updated for simplicity and to use common coding style
  2.8   Improvements to USBCBSendResume(), to make it easier to use.
        Added runtime check to avoid buffer overflow possibility if 
        the USB IN data rate is somehow slower than the UART RX rate.
  2.9b  Added support for optional hardware flow control.
  2.9f  Adding new part support   
  2.9j  Updates to support new bootloader features (ex: app version 
        fetching).
********************************************************************/

/** INCLUDES *******************************************************/
#include "../includes/usb/usb.h"
#include "../includes/usb/usb_function_cdc.h"
#include "../includes/HardwareProfile.h"
#include "plib.h"

#pragma config FNOSC = PRIPLL, POSCMOD = HS, FSOSCEN = OFF, OSCIOFNC = OFF
#pragma config FPLLIDIV = DIV_2, FPLLMUL = MUL_20, FPBDIV = DIV_1, FPLLODIV = DIV_2
#pragma config FWDTEN = OFF, JTAGEN = OFF, ICESEL = ICS_PGx3
#pragma config UPLLIDIV = DIV_2, UPLLEN = ON


/** I N C L U D E S **********************************************************/

#include "GenericTypeDefs.h"
#include "../includes/Compiler.h"
#include "../includes/usb/usb_config.h"
#include "../includes/usb/usb_device.h"
#include "stdint.h"
#include "math.h"
#include "../includes/R2Protocol.h"

/** C O M M A N D S ********************************************************/
#define CMD_OPEN    "O"
#define CMD_CLOSE   "C"
/** V A R I A B L E S ********************************************************/
#define PERIOD      50000   // 20 ms
#define SERVO_MIN   2000    // 1000 us
#define SERVO_REST  3750    // 1500 us
#define SERVO_MAX   5000    // 2000 us
#define SERVO_RUN_SPEED     100
#define SERVO_OPEN  (SERVO_REST + SERVO_RUN_SPEED)
#define SERVO_CLOSE (SERVO_REST - SERVO_RUN_SPEED)

#define LOG_REG_X_COEF 0.673113683
#define LOG_REG_INTCPT -13.18264525
#define V_DIVIDER 0.058823529
#define V_BRY_MAX 26.27


volatile float BRY_VOLTAGE;
volatile float BRY_CURRENT;
volatile float BRY_AMP_HRS;

volatile float V_READ;
volatile float I_READ;
volatile float AMP_HRS;
volatile float PCNT_BRY;

volatile float BRY_DATA[4] = {0};


/** P R I V A T E  P R O T O T Y P E S ***************************************/
static void InitializeSystem(void);

void initPWM(void);

void setFlapSpeed(int speed);

/******************************************************************************
 * Function:        void main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main program entry point.
 *
 * Note:            None
 *****************************************************************************/

void ADC_Config(){
    //configure and enable the ADC
    CloseADC10(); //ensure ADC is off before setting the configuration

     //define setup parameters for OpenADC10
    #define PARAM1 ADC_MODULE_ON | ADC_FORMAT_INTG | ADC_CLK_AUTO | \
    ADC_AUTO_SAMPLING_ON

    //second set of parameters needs to be switched to voltage reference when one
    //when one is available (on the PCB, not on the breadboard)
    #define PARAM2 ADC_VREF_EXT_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | \
        ADC_SAMPLES_PER_INT_2 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF

    #define PARAM3 ADC_CONV_CLK_INTERNAL_RC | ADC_SAMPLE_TIME_12

    #define PARAM4 ENABLE_AN4_ANA | ENABLE_AN5_ANA

    #define PARAM5 SKIP_SCAN_AN0 | SKIP_SCAN_AN1 | SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | \
    SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | \
    SKIP_SCAN_AN11 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | \
    SKIP_SCAN_AN15

//     configure to sample AN4 & AN5
     SetChanADC10(ADC_CH0_NEG_SAMPLEA_NVREF);

    OpenADC10(PARAM1, PARAM2, PARAM3, PARAM4, PARAM5);

    EnableADC10(); // Enable the ADC

    while (! mAD1ClearIntFlag() ) { } // wait for the first conversion to complete
}

void DVO_Config() {
mPORTBSetPinsDigitalOut(BIT_7 | BIT_8 | BIT_9 | BIT_13 | BIT_14);
mPORTBClearBits(BIT_7 | BIT_8 | BIT_9 | BIT_13 | BIT_14); //turns off all bits
mPORTBSetBits(BIT_14); //turns on Bit 7 RED LED in this case
}

void TIMER_Config(){
OpenTimer2(T2_ON | T2_PS_1_64, 62500);
ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_5);

}

void __ISR(_TIMER_2_VECTOR, ipl5) _Timer2_InterruptHandler(void){
    mT2ClearIntFlag();
    
    BRY_VOLTAGE = ReadADC10(0)*2.048/1023; // channel 4
    BRY_CURRENT = ReadADC10(1)*2.048/1023; // channel 5
    
    BRY_AMP_HRS = pow(10,BRY_VOLTAGE * LOG_REG_X_COEF + LOG_REG_INTCPT);
    
    V_READ = BRY_VOLTAGE / V_DIVIDER;
    I_READ = BRY_CURRENT*10;
    AMP_HRS = BRY_AMP_HRS / 1000;
    PCNT_BRY = BRY_VOLTAGE/V_BRY_MAX * 100;
    
    BRY_DATA[0] = V_READ;
    BRY_DATA[1] = I_READ;
    BRY_DATA[2] = AMP_HRS;
    BRY_DATA[3] = PCNT_BRY;
    

//    mAMP_HRS = (uint64_t) floor(BRY_AMP_HRS);
//    mV_READ = (uint64_t) floor(1000 * BRY_VOLTAGE / V_DIVIDER);
//    
//    
//    mI_READ = (uint64_t) floor(1000 * BRY_CURRENT*10);
//    PCNT_BRY = (uint64_t) floor(BRY_VOLTAGE/V_BRY_MAX);
    
}

int main(void)
{   
    DVO_Config(); 
    ADC_Config();
    TIMER_Config();
    
    InitializeSystem();

    #if defined(USB_INTERRUPT)
        USBDeviceAttach();
    #endif

    while(1)
    {
        #if defined(USB_POLLING)
		// Check bus status and service USB interrupts.
        USBDeviceTasks(); // Interrupt or polling method.  If using polling, must call
        				  // this function periodically.  This function will take care
        				  // of processing and responding to SETUP transactions 
        				  // (such as during the enumeration process when you first
        				  // plug in).  USB hosts require that USB devices should accept
        				  // and process SETUP packets in a timely fashion.  Therefore,
        				  // when using polling, this function should be called 
        				  // regularly (such as once every 1.8ms or faster** [see 
        				  // inline code comments in usb_device.c for explanation when
        				  // "or faster" applies])  In most cases, the USBDeviceTasks() 
        				  // function does not take very long to execute (ex: <100 
        				  // instruction cycles) before it returns.
        #endif

        OpenTimer1(T1_ON | T1_PS_1_256, 0xFFFF);
        
        
        
        struct R2ProtocolPacket comm_packet;
        uint8_t packet_data[32] = {0};
        comm_packet.data_len = 32;
        comm_packet.data = packet_data;
        
		// Application-specific tasks.
		// Application related code may be added here, or in the ProcessIO() function.
        int result = ProcessIO(&comm_packet);
        
        if (result){
            // new data available
            
            if (strncmp(comm_packet.data, "Q", 5) == 0){
                // Got "Q"
                mPORTBSetBits(BIT_14 | BIT_13 | BIT_9);
                
                //sprintf(buf, "mV_READ: %f\n\r", V_READ);
                //putsUSBUSART(buf);
                
                char buf[256];
                char source[256] = "BATTERY";
                
                char bry_reads[4] = {'V','I','A','L'};
                //char voltage[256] = "VOLTAGE";
                //char current[256] = "CURRENT";
                //char level[256] = "LEVEL";
                //char mAhrs[256] = "MILIAMPHOURS";
                // try char*
                char readings[256] =" ";
                
                int i;
                
                BRY_DATA[0] = 26.7;
                BRY_DATA[2] = 9.8;
                BRY_DATA[1] = 15555;
                BRY_DATA[3] = 13.3;
                
                
                
                for(i=0; i<4; i++){
                    
                    sprintf(buf, "%.1f", BRY_DATA[i]);
                    int newlength = strlen(buf);
                    struct R2ProtocolPacket info_packet = {
                    "BATTERY", "NUC", "", newlength, buf, ""
                    };
                    
                    readings[0] = bry_reads[i];
                    char* srcptr = &info_packet.id;
                    sprintf(srcptr, "%s", readings);
                    uint8_t output[256];
                    int len = R2ProtocolEncode(&info_packet, output, 256);
                    
                    if (len >= 0) {
                        putUSBUSART(output, len);
                                                
                        CDCTxService();
                    } else {// ending if - encoding was correct
                        putsUSBUSART("FAILED SENDING!\n\r");
                    } 
                                        
                }// ending for
                
            } // ending if request was for us
            else{
                putsUSBUSART("Garbage!\n\r");
            } // ending else - where request wasn't for us
            
            
        
        } //ending if - where there was a message
        else{
//            putsUSBUSART("hello world!\n\r");
        }
        
    }//end while
}//end main


/********************************************************************
 * Function:        static void InitializeSystem(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        InitializeSystem is a centralize initialization
 *                  routine. All required USB initialization routines
 *                  are called from here.
 *
 *                  User application initialization routine should
 *                  also be called from here.                  
 *
 * Note:            None
 *******************************************************************/
static void InitializeSystem(void)
{

    ANSELA = 0; ANSELB = 0;
    SYSTEMConfigPerformance(40000000);

//	The USB specifications require that USB peripheral devices must never source
//	current onto the Vbus pin.  Additionally, USB peripherals should not source
//	current on D+ or D- when the host/hub is not actively powering the Vbus line.
//	When designing a self powered (as opposed to bus powered) USB peripheral
//	device, the firmware should make sure not to turn on the USB module and D+
//	or D- pull up resistor unless Vbus is actively powered.  Therefore, the
//	firmware needs some means to detect when Vbus is being powered by the host.
//	A 5V tolerant I/O pin can be connected to Vbus (through a resistor), and
// 	can be used to detect when Vbus is high (host actively powering), or low
//	(host is shut down or otherwise not supplying power).  The USB firmware
// 	can then periodically poll this I/O pin to know when it is okay to turn on
//	the USB module/D+/D- pull up resistor.  When designing a purely bus powered
//	peripheral device, it is not possible to source current on D+ or D- when the
//	host is not actively providing power on Vbus. Therefore, implementing this
//	bus sense feature is optional.  This firmware can be made to use this bus
//	sense feature by making sure "USE_USB_BUS_SENSE_IO" has been defined in the
//	HardwareProfile.h file.    
    #if defined(USE_USB_BUS_SENSE_IO)
    tris_usb_bus_sense = INPUT_PIN; // See HardwareProfile.h
    #endif
    
//	If the host PC sends a GetStatus (device) request, the firmware must respond
//	and let the host know if the USB peripheral device is currently bus powered
//	or self powered.  See chapter 9 in the official USB specifications for details
//	regarding this request.  If the peripheral device is capable of being both
//	self and bus powered, it should not return a hard coded value for this request.
//	Instead, firmware should check if it is currently self or bus powered, and
//	respond accordingly.  If the hardware has been configured like demonstrated
//	on the PICDEM FS USB Demo Board, an I/O pin can be polled to determine the
//	currently selected power source.  On the PICDEM FS USB Demo Board, "RA2" 
//	is used for	this purpose.  If using this feature, make sure "USE_SELF_POWER_SENSE_IO"
//	has been defined in HardwareProfile - (platform).h, and that an appropriate I/O pin 
//  has been mapped	to it.
    #if defined(USE_SELF_POWER_SENSE_IO)
    tris_self_power = INPUT_PIN;	// See HardwareProfile.h
    #endif
    
    UserInit();

    USBDeviceInit();	//usb_device.c.  Initializes USB module SFRs and firmware
    					//variables to known states.
}//end InitializeSystem