// UART0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart0 (2).h"
#include "gpio.h"

// Pins
#define UART_TX PORTA,1
#define UART_RX PORTA,0

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initUart0(void)
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;
    _delay_cycles(3);
    enablePort(PORTA);

    // Configure UART0 pins
    selectPinPushPullOutput(UART_TX);
    selectPinDigitalInput(UART_RX);
    setPinAuxFunction(UART_TX, GPIO_PCTL_PA1_U0TX);
    setPinAuxFunction(UART_RX, GPIO_PCTL_PA0_U0RX);

    // Configure UART0 with default baud rate
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                     // use system clock (usually 40 MHz)
}

// Set baud rate as function of instruction cycle frequency
void setUart0BaudRate(uint32_t baudRate, uint32_t fcyc)
{
    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
                                                        // where r = fcyc / 16 * baudRate
    divisorTimes128 += 1;                               // add 1/128 to allow rounding
    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
    UART0_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
    UART0_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
                                                        // turn-on UART0
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart0(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0(void)
{
    while (UART0_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

void getsUart0(USER_DATA *data)
{
    int count=0;

    while(true)
    {
        char ch= getcUart0();
        int ascii_char = (int) ch;
        if((ascii_char==8 || ascii_char==127) && count>0 )
        {
            count--;
        }
        if(ascii_char==13 || count==MAX_CHARS)
        {
            data->buffer[count]='\0';
            return;
        }
        if(ascii_char>=32 && count<MAX_CHARS)
        {
            data->buffer[count]=ch;
            count++;
        }
    }
}

void parseFields(USER_DATA *data)
{
    int count=0;
    int pos_count=0;
    int prev_char = 0;
    data->fieldCount=0;
    while(data->buffer[count]!='\0')
    {
        char ch=data->buffer[count];
        int ascii_char = (int) ch;
        if((ascii_char>=65 && ascii_char<=90) || (ascii_char>=97 && ascii_char<=122))
        {
            if(prev_char==0)
            {
                data->fieldCount++;
                data->fieldPosition[pos_count]=count;
                data->fieldType[pos_count]='a';
                pos_count++;
                prev_char=1;
            }
        }
        else if(ascii_char>=48 && ascii_char<=57)
        {
            if(prev_char==0)
            {
                data->fieldCount++;
                data->fieldPosition[pos_count]=count;
                data->fieldType[pos_count]='n';
                pos_count++;
                prev_char=1;
            }
        }
        else if(data->fieldCount==MAX_FIELDS)
        {
            return;
        }
        else{
            data->buffer[count]='\0';
            prev_char=0;
        }
        count++;
    }
    data->fieldType[count]='\0';
}


char *getFieldString( USER_DATA *data,uint8_t fieldNumber)
{
    char* str;
    int count=0;
    int i=1;
    while(i<=MAX_FIELDS)
    {
        int num=data->fieldPosition[count];
        if(num==fieldNumber)
        {
            int idx=num;
            int str_idx=0;
            char final[100];
            while(data->buffer[idx]!='\0')
            {
                final[str_idx]=data->buffer[idx];
                str_idx++;
                idx++;
            }
            final[str_idx]='\0';
            return str=final;
        }
        count++;
        i++;
    }
    str="null";
    return str;
}
uint32_t getFieldInteger(USER_DATA *data,uint8_t fieldNumber)
{
    int i=1;
    int idx=0;
    int value=0;
    while(i<=data->fieldCount)
    {
        int num=data->fieldPosition[idx];
        if(num==fieldNumber && data->fieldType[idx]=='n')
        {
            while(data->buffer[num]!='\0')
            {
                char ch=data->buffer[num];
                value = value*10 +( ch-'0');
                num++;
            }
            return value;
        }
        idx++;
        i++;
}
    return 0;
}

bool isCommand(USER_DATA *data,char strcommand[],uint8_t minArguments)
{
        if(minArguments ==  (data->fieldCount -1))
        {
            int check_idx=0;
            int idx=0;
            int org_idx = data->fieldPosition[idx];
            while(data->buffer[org_idx]!='\0')
            {
                if(data->buffer[org_idx]==strcommand[check_idx])
                {
                    check_idx++;
                    org_idx++;
                    continue;
                }
                else
                {
                    return false;
                }
              }
            }
        else{
            return false;
        }
        return true;
}


// Returns the status of the receive buffer
bool kbhitUart0(void)
{
    return !(UART0_FR_R & UART_FR_RXFE);
}
