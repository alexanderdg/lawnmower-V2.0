//code for the lawnmower sensor board
#include <xc.h>
#include <stdio.h>
#include <stdlib.h>

#define _XTAL_FREQ 64000000


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

enum SM_State {
    STARTMENU,
    CONF_SELFTEST,
    SELFTEST,
    SELFTEST_FAIL,
    CONF_MOWING,
    MOWING,
    CHARGING,
    RETURN_HOME,
    STUCK
};

char master_state_lut[18][20] = {
                         "INIT               ",
                         "SELF_TEST          ",
                         "RUN                ",
                         "RUN_SLOW           ",
                         "AVOID_PERI         ",
                         "TRY_LEFT           ",
                         "TRY_RIGHT          ",
                         "TRY_INSTEAD_LEFT   ",
                         "TRY_INSTEAD_RIGHT  ",
                         "TRY_LAST_TIME_LEFT ",
                         "TRY_LAST_TIME_RIGHT",
                         "TRY_BACKWARD       ",
                         "STUCK              ",
                         "FIND_PERIMETER     ",
                         "RANDOM_TURN        ",
                         "RETURN_HOME        ",
                         "CHARGING           ",
                         "SERROR             "
                     };

char charging_state_lut[7][20] = {
                         "NO_CHARGER_PRESENT ",
                         "FAST_CHARGE        ",
                         "TOP_OFF_CHARGE     ",
                         "CHARGE_COMPLETE    ",
                         "CHARGER_DISABLED   ",
                         "FAULT              ",
                         "UNDEFINED          "
                     };

void initCAN(void);
void initBuzzer(void);
void playBuzzer(void);
void initADC(void);

void sendCANmessage(int id, int message [8], int length);
void readCANmessage(void);
int readADC(int ch);

void Lcd_Port(char a);
void Lcd_Cmd(char a);
void Lcd_Clear(void);
void Lcd_Set_Cursor(char a, char b);
void Lcd_Init(void);
void Lcd_Write_Char(char a);
void Lcd_Write_String(char *a);
void Lcd_Shift_Right(void);
void Lcd_Shift_Left(void);

void MenuYesNo(void);


int ADCvalueHigh0 = 0;
int ADCvalueLow0 = 0;
int ADCvalueHigh1 = 0;
int ADCvalueLow1 = 0;
int tick_count = 0;
int max_tick_count = 1;
int min_tick_count = 0;
int status = 1;
int feedback = 0;
enum SM_State state = STARTMENU;
int enter_state = 1;
int master_state = 0;
int charging_state = 6;
float charging_voltage = 0;

void main() {
    //set intern oscilator frequency to 64MHz
    OSCCON1 = 0b01100000;
    initCAN();
    initBuzzer();
    //map external interupt to RA0
    ANSELAbits.ANSELA0 = 0;
    ANSELAbits.ANSELA1 = 0;
    ANSELAbits.ANSELA2 = 0;
    INT0PPS = 0x01;
    TRISAbits.TRISA0 = 1;
    TRISAbits.TRISA1 = 1;
    TRISAbits.TRISA2 = 1;
    PIE1bits.INT0IE = 1;
    PIR1bits.INT0IF = 0;
    INTCON0bits.INT0EDG = 1;
    //enable global interupts
    INTCON0bits.GIEL = 1;
    INTCON0bits.GIE = 1;
    //no prior in the interupts
    INTCON0bits.IPEN = 0;
    //enable CAN interupts
    PIE5bits.RXB0IE = 1;
    PIE5bits.RXB1IE = 0;
    //setup TIMER1 for escape button
    T1CONbits.CKPS = 0b11;
    T1CONbits.NOT_SYNC = 1;
    T1CONbits.RD16 = 1;
    T1CLKbits.CS = 0b00110;
    TMR1H = 0;
    TMR1L = 0;

    //tick_count = 4;
    ei();
    TRISBbits.TRISB0 = 0;
    ANSELBbits.ANSELB0 = 0;
    PORTBbits.RB0 = 1;
    //tick_count = 5;
    Lcd_Init();
    while (1) {
        switch (state) {
            case STARTMENU:
                status = state;
                if (enter_state == 1) {
                    tick_count = 0;
                    enter_state = 0;
                    Lcd_Clear();
                    Lcd_Set_Cursor(1, 1);
                    Lcd_Write_String("Welcome to the robot");
                    Lcd_Set_Cursor(2, 1);
                    Lcd_Write_String("Execute selftest?");
                }
                else {
                    MenuYesNo();
                    if (!PORTAbits.RA2) {
                        if(tick_count == 0) state = CONF_SELFTEST;
                        else state = CONF_MOWING;
                        enter_state = 1;
                        while (!PORTAbits.RA2);
                    }
                }
                break;
            case CONF_SELFTEST:
                status = state;
                if (enter_state == 1) {
                    tick_count = 1;
                    enter_state = 0;
                    Lcd_Clear();
                    Lcd_Set_Cursor(1, 1);
                    Lcd_Write_String("Robot will move");
                    Lcd_Set_Cursor(2, 1);
                    Lcd_Write_String("Are you sure??");
                }
                else {
                    MenuYesNo();
                    if (!PORTAbits.RA2) {
                        if(tick_count == 0) state = SELFTEST;
                        else state = STARTMENU;
                        enter_state = 1;
                        while (!PORTAbits.RA2);
                    }
                }
                break;
            case SELFTEST:
                status = state;
                if (enter_state == 1) {
                    enter_state = 0;
                    Lcd_Clear();
                    Lcd_Set_Cursor(1, 1);
                    Lcd_Write_String("Executing selftest");
                    Lcd_Set_Cursor(2, 1);
                    Lcd_Write_String(".....");
                }
                else {
                    if(feedback == 1) {
                        feedback = 0;
                        enter_state = 1;
                        state = SELFTEST_FAIL;
                    }
                    else if(feedback == 2) {
                        feedback = 0;
                        enter_state = 1;
                        state = CONF_MOWING;
                        status = state;
                        Lcd_Clear();
                        Lcd_Set_Cursor(1, 1);
                        Lcd_Write_String("Selftest was OK!");
                        __delay_ms(2000);
                        
                    }
                }
                break;
            case SELFTEST_FAIL:
                status = state;
                if (enter_state == 1) {
                    tick_count = 0;
                    enter_state = 0;
                    Lcd_Clear();
                    Lcd_Set_Cursor(1, 1);
                    Lcd_Write_String("Selftest failed");
                    Lcd_Set_Cursor(2, 1);
                    Lcd_Write_String("Try Again?");
                }
                else {
                    MenuYesNo();
                    if (!PORTAbits.RA2) {
                        if(tick_count == 0) state = CONF_SELFTEST;
                        else state = STARTMENU;
                        enter_state = 1;
                        while (!PORTAbits.RA2);
                    }
                }
                break;
            case CONF_MOWING:
                status = state;
                if (enter_state == 1) {
                    tick_count = 1;
                    enter_state = 0;
                    Lcd_Clear();
                    Lcd_Set_Cursor(1, 1);
                    Lcd_Write_String("Robot will mowing!!");
                    Lcd_Set_Cursor(2, 1);
                    Lcd_Write_String("Are you sure??");
                }
                else {
                    MenuYesNo();
                    if (!PORTAbits.RA2) {
                        if(tick_count == 0) state = MOWING;
                        else state = STARTMENU;
                        enter_state = 1;
                        while (!PORTAbits.RA2);
                    }
                }
                break;
            case MOWING:
                if (enter_state == 1) {
                    enter_state = 0;
                    Lcd_Clear();
                    Lcd_Set_Cursor(1, 1);
                    Lcd_Write_String("Mowing lawn");
                    Lcd_Set_Cursor(2, 1);
                    Lcd_Write_String(".....");
                    Lcd_Set_Cursor(3, 1);
                    Lcd_Write_String("Master STATE:");
                }
                else {
                    char str[20];
                    sprintf(str, "%3d", TMR1L);
                    //Lcd_Set_Cursor(3, 1);
                    //Lcd_Write_String(str);
                    Lcd_Set_Cursor(4, 1);
                    Lcd_Write_String(master_state_lut[master_state]);
                    if(!PORTAbits.RA2)
                    {
                        T1CONbits.ON = 1;
                        status = 0;
                        if(TMR1H > 50)
                        {
                            state = STARTMENU;
                            enter_state = 1;
                            Lcd_Clear();
                            Lcd_Set_Cursor(1, 1);
                            Lcd_Write_String("Release for restart");
                            while (!PORTAbits.RA2){
                                char str[20];
                                sprintf(str, "%3d", TMR1L);
                                if(TMR1H > 100) {
                                    state = RETURN_HOME;
                                    Lcd_Clear();
                                    Lcd_Set_Cursor(1, 1);
                                    Lcd_Write_String("Release for return");
                                    Lcd_Set_Cursor(2, 1);
                                    Lcd_Write_String("to charging base");
                                    while (!PORTAbits.RA2);
                                }
                            }
                            T1CONbits.ON = 0;
                            TMR1H = 0;
                            TMR1L = 0;
                        }
                    }
                    else
                    {
                        status = state;
                        T1CONbits.ON = 0;
                        TMR1L = 0;
                        TMR1H = 0;
                    }
                }
                break;
            case CHARGING:
                status = state;
                if (enter_state == 1) {
                    enter_state = 0;
                    Lcd_Clear();
                    Lcd_Set_Cursor(1, 1);
                    Lcd_Write_String("Charging robot");
                    Lcd_Set_Cursor(2, 1);
                    Lcd_Write_String("Voltage: ");
                }
                else {
                    char str[8];
                    sprintf(str, "%3.2f", charging_voltage);
                    Lcd_Set_Cursor(2, 10);
                    Lcd_Write_String(str);
                    Lcd_Set_Cursor(3, 1);
                    Lcd_Write_String(charging_state_lut[charging_state]);
                    //Lcd_Set_Cursor(3, 1);
                    //Lcd_Write_String(str);
                    
                }
                break;
            case RETURN_HOME:
                status = state;
                if (enter_state == 1) {
                    enter_state = 0;
                    Lcd_Clear();
                    Lcd_Set_Cursor(1, 1);
                    Lcd_Write_String("Returning home");
                    Lcd_Set_Cursor(2, 1);
                    Lcd_Write_String(".....");
                    Lcd_Set_Cursor(3, 1);
                    Lcd_Write_String("Master STATE:");
                }
                else {
                    char str[20];
                    sprintf(str, "%3d", TMR1L);
                    //Lcd_Set_Cursor(3, 1);
                    //Lcd_Write_String(str);
                    Lcd_Set_Cursor(4, 1);
                    Lcd_Write_String(master_state_lut[master_state]);
                    if(!PORTAbits.RA2)
                    {
                        T1CONbits.ON = 1;
                        if(TMR1H > 50)
                        {
                            state = STARTMENU;
                            enter_state = 1;
                            Lcd_Clear();
                            Lcd_Set_Cursor(1, 1);
                            Lcd_Write_String("Release for restart");
                            while (!PORTAbits.RA2){
                                
                            }
                            T1CONbits.ON = 0;
                            TMR1H = 0;
                            TMR1L = 0;
                        }
                    }
                    else
                    {
                        T1CONbits.ON = 0;
                        TMR1L = 0;
                        TMR1H = 0;
                    }
                }
                break;
            case STUCK:
                status = state;
                if (enter_state == 1) {
                    tick_count = 0;
                    enter_state = 0;
                    Lcd_Clear();
                    Lcd_Set_Cursor(1, 1);
                    Lcd_Write_String("Robot is stuck");
                    Lcd_Set_Cursor(2, 1);
                    Lcd_Write_String("Push to restart");
                }
                else {
                    if (!PORTAbits.RA2) {
                        state = STARTMENU;
                        enter_state = 1;
                        while (!PORTAbits.RA2);
                    }
                }
                break;
            default:
                status = state;
                if (enter_state == 1) {
                    enter_state = 0;
                    Lcd_Clear();
                    Lcd_Set_Cursor(1, 1);
                    Lcd_Write_String("Error statemachine");
                }
                else {
                    
                }
                break;
        }
    }
}

void __interrupt(irq(TMR0), low_priority) buzzerInt(void) {
    PORTBbits.RB1 = 0;
    T0CON0bits.EN = 0;
    PIR3bits.TMR0IF = 0;
}

void __interrupt(irq(INT0), high_priority) encInt(void) {
    if (PORTAbits.RA0 == 1) {
        if (tick_count > min_tick_count) {
            tick_count--;
            playBuzzer();
        }
    } else {
        if (tick_count < max_tick_count) {
            tick_count++;
            playBuzzer();
        }
    }
    PIR1bits.INT0IF = 0;
}

void __interrupt(irq(AD), high_priority) adcInt(void) {
    //ADCvalueHigh = ADRESH;
    //ADCvalueLow = ADRESL;
    //PIR1bits.ADIF = 0;
}

void __interrupt(irq(RXB0IF), high_priority) canRecInt2(void) {
    if (RXB0CONbits.RXFUL == 1) {
        int message[8];
        switch (RXB0D0) {
            case 0:
                message[0] = status;
                sendCANmessage(0, message, 1);
                break;
            case 1:
                feedback = RXB0D1;
                break;
            case 2:
                enter_state = 1;
                state = RXB0D1;
                status= state;
                break;
            case 3:
                master_state = RXB0D1;
                break;
            case 4:
                charging_state = RXB0D1;
                break;
            case 5:
                charging_voltage = RXB0D1 + (RXB0D2 / 100.0);
                break;
        }
        RXB0CONbits.RXFUL = 0;
    }
    PIR5bits.RXB0IF = 0;
}

void __interrupt(irq(RXB1IF), high_priority) canRecInt(void) {
    tick_count = 255;
    PIR5bits.RXB1IF = 0;
}

void Lcd_Port(char a) {
    if (a & 1)
        PORTCbits.RC2 = 1;
    else
        PORTCbits.RC2 = 0;

    if (a & 2)
        PORTCbits.RC5 = 1;
    else
        PORTCbits.RC5 = 0;

    if (a & 4)
        PORTCbits.RC6 = 1;
    else
        PORTCbits.RC6 = 0;

    if (a & 8)
        PORTCbits.RC7 = 1;
    else
        PORTCbits.RC7 = 0;
}

void Lcd_Cmd(char a) {
    PORTCbits.RC0 = 0; // => RS = 0
    Lcd_Port(a);
    PORTCbits.RC1 = 1; // => E = 1
    __delay_ms(4);
    PORTCbits.RC1 = 0; // => E = 0
}

void Lcd_Clear(void) {
    Lcd_Cmd(0);
    Lcd_Cmd(1);
}

void Lcd_Set_Cursor(char a, char b) {
    char temp, z, y;
    if (a == 1) {
        temp = 0x80 + b - 1;
        z = temp >> 4;
        y = temp & 0x0F;
        Lcd_Cmd(z);
        Lcd_Cmd(y);
    } else if (a == 2) {
        temp = 0xC0 + b - 1;
        z = temp >> 4;
        y = temp & 0x0F;
        Lcd_Cmd(z);
        Lcd_Cmd(y);
    } else if (a == 3) {
        temp = 0x94 + b - 1;
        z = temp >> 4;
        y = temp & 0x0F;
        Lcd_Cmd(z);
        Lcd_Cmd(y);
    } else if (a == 4) {
        temp = 0xD4 + b - 1;
        z = temp >> 4;
        y = temp & 0x0F;
        Lcd_Cmd(z);
        Lcd_Cmd(y);
    }
}

void Lcd_Init(void) {
    TRISC = 0x00;
    TRISBbits.TRISB0 = 0;
    ANSELBbits.ANSELB0 = 0;
    PORTBbits.RB0 = 1;
    ANSELC = 0x00;
    Lcd_Port(0x00);

    __delay_ms(20);
    Lcd_Cmd(0x03);
    __delay_ms(5);
    Lcd_Cmd(0x03);
    __delay_ms(11);
    Lcd_Cmd(0x03);
    Lcd_Cmd(0x02);
    Lcd_Cmd(0x02);
    Lcd_Cmd(0x08);
    Lcd_Cmd(0x00);
    Lcd_Cmd(0x0C);
    Lcd_Cmd(0x00);
    Lcd_Cmd(0x06);

}

void Lcd_Write_Char(char a) {
    char temp, y;
    temp = a & 0x0F;
    y = a & 0xF0;
    PORTCbits.RC0 = 1; // => RS = 1
    Lcd_Port(y >> 4); //Data transfer
    PORTCbits.RC1 = 1;
    __delay_us(40);
    PORTCbits.RC1 = 0;
    Lcd_Port(temp);
    PORTCbits.RC1 = 1;
    __delay_us(40);
    PORTCbits.RC1 = 0;
}

void Lcd_Write_String(char *a) {
    int i;
    for (i = 0; a[i] != '\0'; i++)
        Lcd_Write_Char(a[i]);
}

void Lcd_Shift_Right() {
    Lcd_Cmd(0x01);
    Lcd_Cmd(0x0C);
}

void Lcd_Shift_Left() {
    Lcd_Cmd(0x01);
    Lcd_Cmd(0x08);
}

void MenuYesNo(void) {
    char str[20];
    unsigned int value = 105;
    //sprintf(str, "%3d", tick_count);
    switch (tick_count) {
        case 0:
            Lcd_Set_Cursor(3, 2);
            Lcd_Write_String("|Yes|");
            Lcd_Set_Cursor(4, 2);
            Lcd_Write_String(" No ");
            break;
        case 1:
            Lcd_Set_Cursor(3, 2);
            Lcd_Write_String(" Yes ");
            Lcd_Set_Cursor(4, 2);
            Lcd_Write_String("|No|");
            break;
        default:
            Lcd_Set_Cursor(3, 1);
            Lcd_Write_String("Error");
            break;
    }
}

/*

void initADC(void) {
    //Setup the 2.048V voltage reference
    FVRCONbits.EN = 1;
    FVRCONbits.CDAFVR = 0b10;
    FVRCONbits.ADFVR = 0b10;
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
    ADREFbits.PREF = 11;
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
 */

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
    RXF0SIDLbits.SID = 2;
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

void initBuzzer(void) {
    //init buzzer output pin
    ANSELBbits.ANSELB1 = 0;
    TRISBbits.TRISB1 = 0;
    PORTBbits.RB1 = 1;
    T0CON0bits.EN = 1;
    T0CON0bits.MD16 = 0;
    T0CON0bits.OUTPS = 0;
    T0CON1bits.CS = 0b101;
    T0CON1bits.ASYNC = 1;
    T0CON1bits.CKPS = 0b1001;
    TMR0L = 0;
    TMR0H = 50;
    PIE3bits.TMR0IE = 1;
    PIR3bits.TMR0IF = 0;
}

void playBuzzer(void) {
    PORTBbits.RB1 = 1;
    T0CON0bits.EN = 1;
    TMR0L = 0;
}