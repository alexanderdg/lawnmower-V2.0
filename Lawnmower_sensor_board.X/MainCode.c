//code for the lawnmower sensor board
#include <xc.h>
#define _XTAL_FREQ 64000000

#define TRIGGER_PIN RC1
#define ECHO_PIN RB1


// CONFIG1L
//#pragma config FEXTOSC = OFF    // External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)
// CONFIG1H
#pragma config CLKOUTEN = OFF   // Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON      // PRLOCKED One-Way Set Enable bit (PRLOCK bit can be cleared and set only once)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)

// CONFIG2L
#pragma config MCLRE = EXTMCLR  // MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON      // Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON     // IVTLOCK bit One-way set enable bit (IVTLOCK bit can be cleared and set only once)
#pragma config LPBOREN = OFF    // Low Power BOR Enable bit (ULPBOR disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG2H
#pragma config BORV = VBOR_2P45 // Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 2.45V)
#pragma config ZCD = OFF        // ZCD Disable bit (ZCD disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config DEBUG = OFF      // Debugger Enable bit (Background debugger disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG3L
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF        // WDT operating mode (WDT enabled regardless of sleep)

// CONFIG3H
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4L
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF       // Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF      // Storage Area Flash enable bit (SAF disabled)
#pragma config WRTAPP = OFF     // Application Block write protection bit (Application Block not write protected)

// CONFIG4H
#pragma config WRTB = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-30000Bh) not write-protected)
#pragma config WRTC = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)
#pragma config WRTSAF = OFF     // SAF Write protection bit (SAF not Write Protected)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)

// CONFIG5L
#pragma config CP = OFF         // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

// CONFIG5H

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF
void initCAN(void);
void initADC(void);
void initHCSR04(void);
void initI2C(void);
int readI2C(char reg, int * conOK);
void sendTrigger1(void);
void sendTrigger2(void);
void sendTrigger3(void);
void sendTrigger4(void);
void sendTrigger5(void);
int waitForSensor1(void);
int waitForSensor2(void);
int waitForSensor3(void);
int waitForSensor4(void);
int waitForSensor5(void);
int getDistance1(void);
int getDistance2(void);
int getDistance3(void);
int getDistance4(void);
int getDistance5(void);
void wait2US(void);
void wait10US(void);
void sendCANmessage(int id, int message [8], int length);
void readCANmessage(void);
int readADC(int ch);

int ADCvalueHigh0 = 0;
int ADCvalueLow0 = 0;
int ADCvalueHigh1 = 0;
int ADCvalueLow1 = 0;
int distance1 = 0;
int distance2 = 0;
int distance3 = 0;
int distance4 = 0;
int distance5 = 0;
int perimeter = 0;
int sign_perimeter = 0;
int con_perimeter = 0;

int main() {
    //set intern oscilator frequency to 64MHz
    OSCCON1 = 0b01100000;
    initCAN();

    initADC();
    initHCSR04();
    initI2C();
    //enable global interupts
    INTCON0bits.GIEL = 1;
    //no prior in the interupts
    INTCON0bits.IPEN = 0;
    //enable CAN interupts
    PIE5bits.RXB0IE = 1;
    PIE5bits.RXB1IE = 1;
    ei();




    while (1) {
        //int a[8] = {90, 80, 70, 60, 50, 40, 30, 20};
        PORTCbits.RC1 = 0;
        wait2US();
        PORTCbits.RC1 = 1;
        wait10US();
        PORTCbits.RC1 = 0;
        if (waitForSensor1() == 1) {
            distance1 = getDistance1();
        } else distance1 = -1;
        wait10US();


        PORTCbits.RC2 = 0;
        wait2US();
        PORTCbits.RC2 = 1;
        wait10US();
        PORTCbits.RC2 = 0;
        if (waitForSensor2() == 1) {
            distance2 = getDistance2();
        } else distance2 = -1;
        wait10US();

        PORTCbits.RC7 = 0;
        wait2US();
        PORTCbits.RC7 = 1;
        wait10US();
        PORTCbits.RC7 = 0;
        if (waitForSensor3() == 1) {
            distance3 = getDistance3();
        } else distance3 = -1;
        wait10US();

        PORTAbits.RA1 = 0;
        wait2US();
        PORTAbits.RA1 = 1;
        wait10US();
        PORTAbits.RA1 = 0;
        if (waitForSensor4() == 1) {
            distance4 = getDistance4();
        } else distance4 = -1;
        wait10US();

        PORTAbits.RA2 = 0;
        wait2US();
        PORTAbits.RA2 = 1;
        wait10US();
        PORTAbits.RA2 = 0;
        if (waitForSensor5() == 1) {
            distance5 = getDistance5();
        } else distance5 = -1;
        wait10US();

        int temp0 = readADC(0);
        int temp1 = readADC(1);
        ADCvalueHigh0 = (temp0 >> 8) & 0xFF;
        ADCvalueLow0 = temp0 & 0xFF;
        ADCvalueHigh1 = (temp1 >> 8) & 0xFF;
        ADCvalueLow1 = temp1 & 0xFF;
        perimeter = readI2C(0x09, &con_perimeter);
        sign_perimeter = readI2C(0x0A, &con_perimeter);
        //readCANmessage();
    }
    return 0;
}

void __interrupt(irq(AD), high_priority) adcInt(void) {
    //ADCvalueHigh = ADRESH;
    //ADCvalueLow = ADRESL;
    //PIR1bits.ADIF = 0;
}

void __interrupt(irq(RXB0IF), high_priority) canRecInt(void) {
    //LATAbits.LA0 = COMSTATbits.RXB0OVFL;
    if (RXB0CONbits.RXFUL == 1) {
        //LATAbits.LA0 = !LATAbits.LA0;
        //haal de ontvangen data uit de registers
        int message[8];
        switch (RXB0D0) {


            case 0:
                message[0] = 1;
                sendCANmessage(0, message, 1);
                break;
            case 1:
                message[0] = ADCvalueHigh0;
                message[1] = ADCvalueLow0;
                sendCANmessage(0, message, 2);
                break;
            case 2:
                message[0] = ADCvalueHigh1;
                message[1] = ADCvalueLow1;
                sendCANmessage(0, message, 2);
                break;
            case 3:
                message[0] = distance1 >> 8 & 0xFF;
                message[1] = distance1 & 0xFF;
                sendCANmessage(0, message, 2);
                break;
            case 4:
                message[0] = distance2 >> 8 & 0xFF;
                message[1] = distance2 & 0xFF;
                sendCANmessage(0, message, 2);
                break;
            case 5:
                message[0] = distance3 >> 8 & 0xFF;
                message[1] = distance3 & 0xFF;
                sendCANmessage(0, message, 2);
                break;
            case 6:
                message[0] = distance4 >> 8 & 0xFF;
                message[1] = distance4 & 0xFF;
                sendCANmessage(0, message, 2);
                break;
            case 7:
                message[0] = distance5 >> 8 & 0xFF;
                message[1] = distance5 & 0xFF;
                sendCANmessage(0, message, 2);
                break;
            case 8:
                message[0] = perimeter;
                message[1] = sign_perimeter;
                message[2] = con_perimeter;
                sendCANmessage(0, message, 3);
                break;
        }
        //message[0] = RXB0D0 + 1;
        //message[1] = RXB0D1 + 1;
        //message[0] = distance >> 8 & 0xFF;
        //message[1] = distance & 0xFF;
        /*
        message[0] = ADCvalueHigh0;
        message[1] = ADCvalueLow0;
        message[2] = ADCvalueHigh1;
        message[3] = ADCvalueLow1;
        message[4] = RXB0D4 + 1;
        message[5] = RXB0D5 + 1;
        message[6] = RXB0D6 + 1;
        message[7] = RXB0D7 + 1;
        int length = RXB0DLCbits.DLC;
        int msbadres = RXB0SIDH;
        int lsbadres = RXB0SIDLbits.SID;
        int adres = (msbadres << 3) + lsbadres;
        //wis de RXFULL bit zodat de can module weet dat deze uitgelezen is
 
        //asm(BCF RXB0CON, RXFUL);
         */


        RXB0CONbits.RXFUL = 0;
    }
    PIR5bits.RXB0IF = 0;
    return;
}

int readI2C(char reg, int * conOK) {
    /*
     I2C1CNT = 2;
     I2C1ADB1 = 0x9E;
     I2C1TXB = 0x01;
     I2C1CON0bits.S = 1;
     while(!I2C1STAT1bits.TXBE);
     I2C1TXB = 0x01;
     while(!I2C1STAT1bits.TXBE );
    
     // Detect Stop condition  
     while(!I2C1PIRbits.PCIF);
     I2C1PIRbits.PCIF = 0;                                           
     I2C1PIRbits.SCIF = 0;
     I2C1STAT1bits.CLRBF = 1;
     */
    unsigned char data;
    * conOK = 1;
    I2C1ADB1 = 0X10;
    I2C1TXB = reg;
    while (!I2C1STAT0bits.BFRE);
    I2C1CNT = 1;
    I2C1CON0bits.RSEN = 1;
    I2C1CON0bits.S = 1;
    int counter = 0;
    while (!I2C1CON0bits.MDR && counter < 25000) {
        counter++;
    }
    if (counter >= 25000) *conOK = 0;
    //if(I2C1CON1bits.ACKT == 1) *conOK = 0;
    //if(I2C1CON1bits.ACKSTAT == 1) *conOK = 0;
    //if(I2C1PIRbits.ACKTIF == 1) *conOK = 0;
    I2C1ADB1 = (0x11);
    I2C1CNT = 1;
    I2C1CON0bits.S = 1;
    I2C1CON0bits.RSEN = 0;
    counter = 0;
    while (!I2C1STAT1bits.RXBF && counter < 25000) {
        counter++;
    }
    if (counter >= 25000) *conOK = 0;
    else data = I2C1RXB;
    //if(I2C1CON1bits.ACKSTAT == 1) *conOK = 0;
    //if(I2C1CON1bits.ACKT == 1) *conOK = 0;
    //if(I2C1PIRbits.ACKTIF == 1) *conOK = 0;
    
    // Detect Stop condition
    counter = 0;
    while (!I2C1PIRbits.PCIF && counter < 25000) {
        counter++;
    }
    if (counter >= 25000) *conOK = 0;
    //if(I2C1CON1bits.ACKT == 1) *conOK = 0;
    //if(I2C1ERRbits.NACKIF == 1) *conOK = 0;
    I2C1PIRbits.PCIF = 0;
    I2C1PIRbits.SCIF = 0;
    I2C1STAT1bits.CLRBF = 1;

    return data;
}

void initI2C(void) {
    LATCbits.LATC3 = 0; // Clear PORTC write latches
    LATCbits.LATC4 = 0;
    TRISCbits.TRISC3 = 0;
    TRISCbits.TRISC4 = 0;
    ANSELCbits.ANSELC3 = 0;
    ANSELCbits.ANSELC4 = 0;
    ODCONCbits.ODCC3 = 1;
    ODCONCbits.ODCC4 = 1;
    RC3I2C = 0x01; // Standard GPIO slew rate
    // Internal pull-ups not used
    // I2C specific thresholds
    SLRCONCbits.SLRC3 = 0; // No slew rate limiting
    RC4I2C = 0x01;
    SLRCONCbits.SLRC4 = 0;

    // PPS configuration
    int state = (unsigned char) GIE;
    GIE = 0;
    PPSLOCK = 0x55; // Unlock sequence
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x00; // unlock PPS
    RC3PPS = 0x21; // RC3->I2C1:SCL1;
    RC4PPS = 0x22; // RC4->I2C1:SDA1;
    I2C1SDAPPSbits.I2C1SDAPPS = 0x14; // RC4->I2C1:SDA1;
    I2C1SCLPPSbits.I2C1SCLPPS = 0x13; // RC3->I2C1:SCL1;
    PPSLOCK = 0x55; // Lock sequence
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0x01; // lock PPS
    GIE = state;
    I2C1CON0 = 0x04; // Master 7-bit address mode
    I2C1CON1 = 0x80; // ACKDT = ACK, ACKCNT = NACK
    I2C1CON2 = 0x24; // Enable Address Buffers
    // BFRET = 8 I2C pulses
    // FME = 1
    I2C1CLK = 0x03; // MFINTOSC (500 kHz)
    I2C1PIR = 0; // Clear all interrupt flags
    I2C1ERR = 0; // Clear all error flags
    I2C1CON0bits.EN = 1;
    I2C1ERRbits.NACKIE = 1;
}

void initADC(void) {
    //Set the output as input and turn analog function on
    TRISCbits.TRISC6 = 1;
    TRISCbits.TRISC5 = 1;
    ANSELCbits.ANSELC6 = 1;
    ANSELCbits.ANSELC5 = 1;
    //Select Fosc as clock source and prescaler of 28 (Tad = 1.1µs)
    ADCON0bits.ON = 1;
    ADCLKbits.CS = 0b011100;
    //Set data as left justified
    ADCON0bits.FM = 0;
    //set VDD and VSS as reference
    ADREFbits.NREF = 0;
    ADREFbits.PREF = 00;
    //select RA0 as input channel
    ADPCHbits.ADPCH = 0b010110;
    //set the ADC on
    ADCON0bits.ON = 1;
    //set an interupt on ADC
    //PIE1bits.ADIE = 1;
    //clear the interupt flag
    //PIR1bits.ADIF = 0;
    //timer0 is used for the automatic trigger to obtain an sample frequency of 9765Hz
    //enable timer0
    //T0CON0bits.EN = 1;
    //set the timer0 in 8bit mode
    //T0CON0bits.MD16 = 0;
    //set the post scaler on 0
    //T0CON0bits.OUTPS = 0;
    //use Fosc/4 as input clock
    //T0CON1bits.CS = 0b010;
    //set the prescaler to 1:2
    //T0CON1bits.CKPS = 0b0001;
    //move the ouput signal to an output
    //RC1PPS = 0b100101;
    //TRISCbits.TRISC1 = 0;
    //ANSELCbits.ANSELC1 = 0;
}

void initHCSR04(void) {
    T1CLK = 0x01;
    T1CONbits.CKPS = 0b11;
    TMR1H = 0x00;
    TMR1L = 0x00;
    //Disable analog function of sensor trigger pins
    ANSELCbits.ANSELC1 = 0;
    ANSELCbits.ANSELC2 = 0;
    ANSELCbits.ANSELC7 = 0;
    ANSELAbits.ANSELA1 = 0;
    ANSELAbits.ANSELA2 = 0;
    //Disable analog function of sensor echo pins
    ANSELBbits.ANSELB1 = 0;
    ANSELBbits.ANSELB0 = 0;
    ANSELAbits.ANSELA5 = 0;
    ANSELBbits.ANSELB4 = 0;
    ANSELAbits.ANSELA4 = 0;
    //Set trigger pins as output
    TRISCbits.TRISC1 = 0;
    TRISCbits.TRISC2 = 0;
    TRISCbits.TRISC7 = 0;
    TRISAbits.TRISA1 = 0;
    TRISAbits.TRISA2 = 0;
    //Set echo pins as input
    TRISBbits.TRISB1 = 1;
    TRISBbits.TRISB0 = 1;
    TRISAbits.TRISA5 = 1;
    TRISBbits.TRISB4 = 1;
    TRISAbits.TRISA4 = 1;
    //Set output low of trigger pin
    PORTCbits.RC1 = 0;
    PORTCbits.RC2 = 0;
    PORTCbits.RC7 = 0;
    PORTAbits.RA1 = 0;
    PORTAbits.RA2 = 0;
}

int waitForSensor1(void) {
    int i = 0;
    TMR1H = 0;
    TMR1L = 0;
    T1CONbits.TMR1ON = 1;
    while (!PORTBbits.RB1 && (i < 10000))
        i = (TMR1H << 8) | TMR1L; // read Timer1 and store its value in i
    if (i >= 10000)
        return 0;
    else
        return 1;
}

int waitForSensor2(void) {
    int i = 0;
    TMR1H = 0;
    TMR1L = 0;
    T1CONbits.TMR1ON = 1;
    while (!PORTBbits.RB0 && (i < 10000))
        i = (TMR1H << 8) | TMR1L; // read Timer1 and store its value in i
    if (i >= 10000)
        return 0;
    else
        return 1;
}

int waitForSensor3(void) {
    int i = 0;
    TMR1H = 0;
    TMR1L = 0;
    T1CONbits.TMR1ON = 1;
    while (!PORTAbits.RA5 && (i < 10000))
        i = (TMR1H << 8) | TMR1L; // read Timer1 and store its value in i
    if (i >= 10000)
        return 0;
    else
        return 1;
}

int waitForSensor4(void) {
    int i = 0;
    TMR1H = 0;
    TMR1L = 0;
    T1CONbits.TMR1ON = 1;
    while (!PORTBbits.RB4 && (i < 10000))
        i = (TMR1H << 8) | TMR1L; // read Timer1 and store its value in i
    if (i >= 10000)
        return 0;
    else
        return 1;
}

int waitForSensor5(void) {
    int i = 0;
    TMR1H = 0;
    TMR1L = 0;
    T1CONbits.TMR1ON = 1;
    while (!PORTAbits.RA4 && (i < 10000))
        i = (TMR1H << 8) | TMR1L; // read Timer1 and store its value in i
    if (i >= 10000)
        return 0;
    else
        return 1;
}

int getDistance1(void) {
    int ticks = 0;
    TMR1H = 0;
    TMR1L = 0;

    while (PORTBbits.RB1 && (ticks < 15000))
        ticks = (TMR1H << 8) | TMR1L; // read Timer1 value

    T1CONbits.TMR1ON = 0; // disable Timer1 module

    if (ticks >= 15000)
        return -1;

    else
        return ticks;
}

int getDistance2(void) {
    int ticks = 0;
    TMR1H = 0;
    TMR1L = 0;

    while (PORTBbits.RB0 && (ticks < 15000))
        ticks = (TMR1H << 8) | TMR1L; // read Timer1 value

    T1CONbits.TMR1ON = 0; // disable Timer1 module

    if (ticks >= 15000)
        return -1;

    else
        return ticks;
}

int getDistance3(void) {
    int ticks = 0;
    TMR1H = 0;
    TMR1L = 0;

    while (PORTAbits.RA5 && (ticks < 15000))
        ticks = (TMR1H << 8) | TMR1L; // read Timer1 value

    T1CONbits.TMR1ON = 0; // disable Timer1 module

    if (ticks >= 15000)
        return -1;

    else
        return ticks;
}

int getDistance4(void) {
    int ticks = 0;
    TMR1H = 0;
    TMR1L = 0;

    while (PORTBbits.RB4 && (ticks < 15000))
        ticks = (TMR1H << 8) | TMR1L; // read Timer1 value

    T1CONbits.TMR1ON = 0; // disable Timer1 module

    if (ticks >= 15000)
        return -1;

    else
        return ticks;
}

int getDistance5(void) {
    int ticks = 0;
    TMR1H = 0;
    TMR1L = 0;

    while (PORTAbits.RA4 && (ticks < 15000))
        ticks = (TMR1H << 8) | TMR1L; // read Timer1 value

    T1CONbits.TMR1ON = 0; // disable Timer1 module

    if (ticks >= 15000)
        return -1;

    else
        return ticks;
}

void wait2US(void) {
    for (long i = 0; i < 2; i++) {
        Nop();
    }
}

void wait10US(void) {
    for (long i = 0; i < 6; i++) {
        Nop();
    }
}

void initCAN(void) {
    //----------------------------------------------------------------------------
    // Setup for the can interface
    //----------------------------------------------------------------------------
    //assign the CANTX pin to pin RB2 with the PPS register
    RB2PPS = 0b110011;
    //set CANTX as output and CANRX as input
    TRISBbits.TRISB2 = 0;
    TRISBbits.TRISB3 = 1;
    //disable the analog option on port B
    ANSELBbits.ANSELB2 = 0;
    ANSELBbits.ANSELB3 = 0;
    //set the CAN module to the configuration mode (CANCON register)
    CANCONbits.REQOP2 = 1;
    CANCONbits.REQOP1 = 0;
    CANCONbits.REQOP0 = 0;
    //wait for the can module to go in configuration mode
    while (CANSTATbits.OPMODE != 0b100);
    //set the baud rate
    BRGCON1bits.BRP = 0b011111;
    //set the correct mask and filters
    RXF0SIDH = 0;
    RXF0SIDLbits.SID = 1;
    RXF0SIDLbits.EXIDEN = 0;
    RXM0SIDH = 255;
    RXM0SIDLbits.SID = 0b111;
    RXB0CONbits.RXM1 = 0;
    RXB0CONbits.RXM0 = 1;
    RXB0CONbits.FILHIT0 = 0;

    RXF1SIDH = 255;
    RXF1SIDLbits.SID = 0b111;
    RXF1SIDLbits.EXIDEN = 0;
    RXM1SIDH = 255;
    RXM1SIDLbits.SID = 0b111;
    RXB1CONbits.RXM1 = 0;
    RXB1CONbits.RXM0 = 1;
    RXB1CONbits.FILHIT0 = 0;
    //set the can module to normal mode
    CANCONbits.REQOP = 0b000;
    //wait till the can module is in normal mode
    while (CANSTATbits.OPMODE != 0b000);
    //----------------------------------------------------------------------------
}

void sendCANmessage(int id, int message [8], int length) {
    TXB0CONbits.TXREQ = 0;
    //select transmitbuffer
    CANCONbits.WIN = 0b000;
    //zet de hoogste prioriteit naar TXB0
    TXB0CONbits.TXPRI = 0b11;
    //write TXB0 register met adres van RX0D0
    TXB0D0 = message[0];
    TXB0D1 = message[1];
    TXB0D2 = message[2];
    TXB0D3 = message[3];
    TXB0D4 = message[4];
    TXB0D5 = message[5];
    TXB0D6 = message[6];
    TXB0D7 = message[7];
    //laad het adres in
    TXB0SIDLbits.SID = id;
    TXB0SIDH = id >> 3;
    //zet data length to 1
    TXB0DLCbits.DLC = length;
    TXB0DLCbits.TXRTR = 0;
    //request een zend actie bij de can module
    TXB0CONbits.TXREQ = 1;
    //wait till the message is sent
    while (TXB0CONbits.TXREQ == 0);
}

int readADC(int ch) {
    if (ch == 0) ADPCHbits.ADPCH = 0b010110;
    else if (ch == 1) ADPCHbits.ADPCH = 0b010101;
    ADCON0bits.GO = 1;
    while (ADCON0bits.GO);
    return (ADRESH << 8) +ADRESL;
}