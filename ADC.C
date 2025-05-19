/*
 * ADC.C
 *
 *  Created on: Mar 17, 2024
 *      Author: Pranav Madduri
 */

#include<stdio.h>
#include<stdint.h>
#include<stdbool.h>
#include <inttypes.h>
#include <string.h>
#include "gpio.h"
#include "clock.h"
#include "wait.h"
#include "nvic.h"
#include "uart0 (2).h"
#include "rgb_led (1).h"
#include "tm4c123gh6pm.h"

#define RED_LED_MASK 2
#define BLUE_LED_MASK 4
#define GREEN_LED_MASK 8


char str[50];

int notprocessingcurrently = 1;
int Process_flag = 0;

#define BUFFER_SIZE 256

#define SAMPLE_SIZE 64

#define WINDOW_SIZE 15

#define UDMA_CHCTL_XFERSIZE 255

#pragma DATA_ALIGN(controlTable,1024)
volatile uint8_t controlTable[1024];

volatile uint16_t PRI_Buffer[BUFFER_SIZE] = {0};
volatile uint16_t ALT_Buffer[BUFFER_SIZE] = {0};

volatile uint32_t *baseAdd;

volatile uint32_t *srcAdd;
volatile uint32_t *UDMA_PRI_SRCENDP_R;
volatile uint32_t *UDMA_ALT_SRCENDP_R;
volatile uint32_t *UDMA_PRI_DSTENDP_R;
volatile uint32_t *UDMA_ALT_DSTENDP_R;
volatile uint32_t *UDMA_PCHCTL_R;
volatile uint32_t *UDMA_ACHCTL_R;

int BUFFER_STATUS = 0,STORE_STATUS = -1;
uint16_t Sample_Buffer[512] = {0};

uint16_t sample1[SAMPLE_SIZE];
uint16_t sample2[SAMPLE_SIZE];
uint16_t sample3[SAMPLE_SIZE];
uint16_t sample4[SAMPLE_SIZE];


#define length  3
uint16_t time_xy,time_yz,time_zx;
int First_hit,Second_hit,Last_hit,FL_hit;
uint16_t Angle;

int holdoff_variable = 1;

USER_DATA data;

void initRgb()
{
    // Enable clocks
    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure three LEDs
    GPIO_PORTF_DEN_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_AFSEL_R |= RED_LED_MASK | GREEN_LED_MASK | BLUE_LED_MASK;
    GPIO_PORTF_PCTL_R &= ~(GPIO_PCTL_PF1_M | GPIO_PCTL_PF2_M | GPIO_PCTL_PF3_M);
    GPIO_PORTF_PCTL_R |= GPIO_PCTL_PF1_M1PWM5 | GPIO_PCTL_PF2_M1PWM6 | GPIO_PCTL_PF3_M1PWM7;

    // Configure PWM module 1 to drive RGB LED
    // RED   on M1PWM5 (PF1), M1PWM2b
    // BLUE  on M1PWM6 (PF2), M1PWM3a
    // GREEN on M1PWM7 (PF3), M1PWM3b
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;                // reset PWM1 module
    SYSCTL_SRPWM_R = 0;                              // leave reset state
    PWM1_2_CTL_R = 0;                                // turn-off PWM1 generator 2 (drives outs 4 and 5)
    PWM1_3_CTL_R = 0;                                // turn-off PWM1 generator 3 (drives outs 6 and 7)
    PWM1_2_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 5 on PWM1, gen 2b, cmpb
    PWM1_3_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                                                     // output 6 on PWM1, gen 3a, cmpa
    PWM1_3_GENB_R = PWM_1_GENB_ACTCMPBD_ONE | PWM_1_GENB_ACTLOAD_ZERO;
                                                     // output 7 on PWM1, gen 3b, cmpb

    PWM1_2_LOAD_R = 1024;                            // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz
    PWM1_3_LOAD_R = 1024;                            // (internal counter counts down from load value to zero)

    PWM1_2_CMPB_R = 0;                               // red off (0=always low, 1023=always high)
    PWM1_3_CMPB_R = 0;                               // green off
    PWM1_3_CMPA_R = 0;                               // blue off

    PWM1_2_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 2
    PWM1_3_CTL_R = PWM_1_CTL_ENABLE;                 // turn-on PWM1 generator 3
    PWM1_ENABLE_R = PWM_ENABLE_PWM5EN | PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                                                     // enable outputs
}

void setRgbColor(uint16_t red, uint16_t green, uint16_t blue)
{
    PWM1_2_CMPB_R = red;
    PWM1_3_CMPA_R = blue;
    PWM1_3_CMPB_R = green;
}

void setupADC0()
{
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    _delay_cycles(16);
    ADC0_ACTSS_R &= ~ ADC_ACTSS_ASEN1;
    ADC0_CC_R = ADC_CC_CS_SYSPLL;
    ADC0_PC_R = ADC_PC_SR_1M;
    ADC0_EMUX_R = ADC_EMUX_EM1_ALWAYS;
    ADC0_SSCTL1_R = ADC_SSCTL1_END3;

//    ADC0_PSSI_R |= ADC_PSSI_SS1;
}

void setupAdc0Ss1()
{
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN1;
    ADC0_SSMUX1_R = 0x00;  //AIN0
    ADC0_SSMUX1_R = 1<<4 | 2<<8 | 4<<12; //AIN1,AIN2,AIN4
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN1;
}

int16_t readAdc0()
{
    return ADC0_SSFIFO1_R;
}

void setupADC1()
{
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R1;
    _delay_cycles(16);
    ADC1_ACTSS_R &= ~ ADC_ACTSS_ASEN1;
    ADC1_CC_R = ADC_CC_CS_SYSPLL;
    ADC1_PC_R = ADC_PC_SR_1M;
    ADC1_EMUX_R = ADC_EMUX_EM1_ALWAYS;
    ADC1_SSCTL1_R = ADC_SSCTL1_END3;
}

void setupAdc1Ss1()
{
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN1;
    ADC1_SSMUX1_R = 0x00;  //AIN0
    ADC1_SSMUX1_R = 1<<4 | 2<<8 | 4<<12; //AIN1,AIN2,AIN4
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN1;
}

int16_t readAdc1()
{
    return ADC1_SSFIFO1_R;
}

void TriggerCap_ISR()
{

    //Stop the DMA
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;
    UDMA_ENACLR_R |= (1<<25);
    Process_flag = 1;
    //Take out the valid samples from the buffers after a valid event is detected.
    int i,j,track = 0;
    for(i=128;i<256;i++)
    {
        Sample_Buffer[track] = PRI_Buffer[i];
        track+=1;
    }
    for(j=0;j<128;j++)
    {
        Sample_Buffer[track] = ALT_Buffer[j];
        track+=1;
    }

    int Tcount=0;
    int idx=0;
    while(Tcount<BUFFER_SIZE)
    {
        sample1[idx]=Sample_Buffer[Tcount];
        sample2[idx]=Sample_Buffer[Tcount+1];
        sample3[idx]=Sample_Buffer[Tcount+2];
        sample4[idx]=Sample_Buffer[Tcount+3];
        Tcount+=4;
        idx+=1;
    }
    UDMA_ENASET_R |= (1<<25);
}

void EventTrigger_Timer(void)
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1; //Set Clocks
    _delay_cycles(3);
    // Configure Timer 1 as the time base
    TIMER1_CTL_R &= ~TIMER_CTL_TAEN; //Disable Timer
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER; //Configure Timer as 32Bit Timer
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_1_SHOT; //one shot timer mode
    TIMER1_TAILR_R = 8000;   //200us --> 8000
    TIMER1_IMR_R = TIMER_IMR_TATOIM; //Enable Interrupt Mask

   //Enable NVIC for Timer1A
    NVIC_EN0_R = 1 << (INT_TIMER1A-16);
}

void ChannelControl_ISR()
{
    ADC1_ISC_R |= ADC_ISC_IN1;
    UDMA_CHIS_R |= 1<<25;

    //check for the Primary Buffer full
     if((*UDMA_PCHCTL_R & UDMA_CHCTL_XFERMODE_M) == 0)  //Hitting after first 4 samples only?
     {

     //Primary base control configuration(Control base address + channel number + offset of CHCTL(primary))
         *UDMA_PCHCTL_R = UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_SRCINC_NONE| UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_XFERSIZE << 4| UDMA_CHCTL_ARBSIZE_2 | UDMA_CHCTL_XFERMODE_PINGPONG;
         BUFFER_STATUS = 1;
     }

     else if((*UDMA_ACHCTL_R & UDMA_CHCTL_XFERMODE_M) == 0)
     {
         //Alternate base control configuration.(Control base address + channel number + offset of CHCTL(Alternate)).
         *UDMA_ACHCTL_R = UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_SRCINC_NONE| UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_XFERSIZE << 4| UDMA_CHCTL_ARBSIZE_2 | UDMA_CHCTL_XFERMODE_PINGPONG;

         BUFFER_STATUS = 0;
         UDMA_ENASET_R |= 1<<25;

     }
}

void setupDMA()
{
    //Enable clocks
    SYSCTL_RCGCDMA_R |= SYSCTL_RCGCDMA_R0;

    UDMA_CFG_R |= UDMA_CFG_MASTEN;
    //check for the status of the Master Enable bit.
    while(!UDMA_STAT_MASTEN);

    //select channel 25 for ADC1SS1 and assigned to primary
    UDMA_CHASGN_R = UDMA_CHASGN_SECONDARY;
    UDMA_CHMAP3_R |= 1 << UDMA_CHMAP3_CH25SEL_S;

    UDMA_PRIOCLR_R |= UDMA_PRIOCLR_CLR_M; //0X19 DISABLE         Priority clear set to default
    UDMA_ALTCLR_R |= UDMA_ALTCLR_CLR_M;   //0X19 DISBLE          Clear alternate structure and set the primary control structure, but ping-pong uses default (both).
    UDMA_USEBURSTCLR_R |= (1<<25);       //0X19 DISABLE           Sends out burst requests only, ADC are burst requests.
    UDMA_REQMASKCLR_R |= UDMA_REQMASKCLR_CLR_M;  //0X19 DISABLE  If set, this will not able to request UDMA for transfers.

    UDMA_USEBURSTSET_R |= (1<<25);       //0X19 Enable

// Control base address of UDMA using a control table from above.
    baseAdd = ((volatile uint32_t *)(&controlTable[0]));
    UDMA_CTLBASE_R = (uint32_t)baseAdd;

//setting up the primary source end pointer.(Control base address + channel number + offset of SRCENDP).
    srcAdd =((volatile uint32_t *)&ADC1_SSFIFO1_R);
    UDMA_PRI_SRCENDP_R  = ((volatile uint32_t *)(UDMA_CTLBASE_R + 0x190));
    *UDMA_PRI_SRCENDP_R =(uint32_t)srcAdd;

//setting up the primary destination end pointer.(Control base address + channel number + offset of DSTENDP)
    volatile uint32_t *PRI_DST = ((volatile uint32_t *) (&PRI_Buffer[BUFFER_SIZE-1]));
    UDMA_PRI_DSTENDP_R =((volatile uint32_t *)(UDMA_CTLBASE_R + 0x194));
    *UDMA_PRI_DSTENDP_R = (uint32_t)PRI_DST;  //64bytes transfer for each control structure. 16*64/8/8(~8samples)

//Primary base control configuration(Control base address + channel number + offset of CHCTL(primary))
     UDMA_PCHCTL_R =((volatile uint32_t*)(UDMA_CTLBASE_R + 0x198));
    *UDMA_PCHCTL_R = UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_SRCINC_NONE| UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_XFERSIZE << 4| UDMA_CHCTL_ARBSIZE_2 | UDMA_CHCTL_XFERMODE_PINGPONG;

//setting up the Alternate control structure source end pointer.(Control base address + channel number + offset of SRCENDP(ALT)).
    srcAdd =((volatile uint32_t *)&ADC1_SSFIFO1_R);
    UDMA_ALT_SRCENDP_R= ((volatile uint32_t *)(UDMA_CTLBASE_R + 0x390 + 0x00));
    *UDMA_ALT_SRCENDP_R =(uint32_t)srcAdd;

//setting up the Alternate destination end pointer.(Control base address + channel number + offset of DSTENDP)
    volatile uint32_t *ALT_DST = ((volatile uint32_t *)(&ALT_Buffer[BUFFER_SIZE-1]));
    UDMA_ALT_DSTENDP_R =((volatile uint32_t *)(UDMA_CTLBASE_R + 0x390 + 0x004));
    *UDMA_ALT_DSTENDP_R = (uint32_t)ALT_DST;

//Alternate base control configuration.(Control base address + channel number + offset of CHCTL(Alternate)).
    UDMA_ACHCTL_R =((volatile uint32_t*)(UDMA_CTLBASE_R + 0x390 + 0x008));
    *UDMA_ACHCTL_R = UDMA_CHCTL_DSTINC_16 | UDMA_CHCTL_DSTSIZE_16 | UDMA_CHCTL_SRCINC_NONE| UDMA_CHCTL_SRCSIZE_16 | UDMA_CHCTL_XFERSIZE << 4| UDMA_CHCTL_ARBSIZE_2 | UDMA_CHCTL_XFERMODE_PINGPONG;


//Trigger an Interrupt at the peripheral
    ADC1_SSCTL1_R |= ADC_SSCTL1_IE3;
    ADC1_IM_R |= ADC_IM_MASK1;
    enableNvicInterrupt(INT_ADC1SS1); // NVIC for ADC1 SS1

    UDMA_ENASET_R |= (1<<25);
}

void Holdoff_ISR(void)
{
    TIMER2_ICR_R = TIMER_ICR_TATOCINT;
    ADC0_IM_R |= ADC_IM_DCONSS1;
}

void Holdoff_Timer(uint16_t holdoff_time)
{
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2; //Set Clocks
    _delay_cycles(3);
    // Configure Timer 1 as the time base
    TIMER2_CTL_R &= ~TIMER_CTL_TAEN; //Disable Timer
    TIMER2_CFG_R = TIMER_CFG_32_BIT_TIMER; //Configure Timer as 32Bit Timer
    TIMER2_TAMR_R = TIMER_TAMR_TAMR_1_SHOT; //one shot timer mode
    TIMER2_TAILR_R = holdoff_time * 4 * 10000000;
    TIMER2_IMR_R = TIMER_IMR_TATOIM; //Enable Interrupt Mask

   //Enable NVIC for Timer1A
    NVIC_EN0_R = 1 << (INT_TIMER2A-16);
}

void DigitalCompISR()
{

    if(notprocessingcurrently == 1)
    {
        notprocessingcurrently = 0;
        TIMER1_CTL_R |= TIMER_CTL_TAEN; //Enable Timer one shot for 200us
    }
    //backoff timer enable
    ADC0_DCISC_R |=ADC_DCISC_DCINT0 | ADC_DCISC_DCINT1;
    //    ADC0_IM_R &= ~ADC_IM_DCONSS1;
//    TIMER2_CTL_R |= TIMER_CTL_TAEN;

}

void SetDigitalComp()
{

    //Setting up each sample to a digital comparator
    ADC0_SSOP1_R |= ADC_SSOP1_S0DCOP | ADC_SSOP1_S1DCOP;

    //Selecting the comparator unit for each sample.
    ADC0_SSDC1_R |= 0<<ADC_SSDC1_S0DCSEL_S |1<<ADC_SSDC1_S1DCSEL_S;

    //Comparator Unit 0,1,2,3 control registers
    ADC0_DCCTL0_R |= ADC_DCCTL1_CIE | ADC_DCCTL1_CIC_HIGH | ADC_DCCTL1_CIM_ONCE;
    ADC0_DCCTL1_R |= ADC_DCCTL1_CIE | ADC_DCCTL1_CIC_HIGH | ADC_DCCTL1_CIM_ONCE;

    //Comparator Ranges SAC + THRESHOLD
    ADC0_DCCMP0_R = 0x100<<ADC_DCCMP0_COMP0_S | 0x200<<ADC_DCCMP0_COMP1_S;
    ADC0_DCCMP1_R = 0x100<<ADC_DCCMP0_COMP0_S | 0x200<<ADC_DCCMP0_COMP1_S;


    //Enabling the Interrupts
    ADC0_IM_R |= ADC_IM_DCONSS1;
    enableNvicInterrupt(INT_ADC0SS1);
}


void Calc_Angle(void)
{
    if(notprocessingcurrently == 0 && Process_flag == 1)
    {

        int N1 = SAMPLE_SIZE;
        uint32_t xcorr_xy[(SAMPLE_SIZE - WINDOW_SIZE) + 1];
        uint32_t xcorr_yz[(SAMPLE_SIZE - WINDOW_SIZE) + 1];
        uint32_t xcorr_zx[(SAMPLE_SIZE - WINDOW_SIZE) + 1];
        int i,j;
        for(i=0;i<(SAMPLE_SIZE - WINDOW_SIZE)+1;i++)
        {
            xcorr_xy[i]=0;
            xcorr_yz[i]=0;
            xcorr_zx[i]=0;
            for(j=0;j<WINDOW_SIZE;j++)
            {
                if(j<N1)
                {
                    xcorr_xy[i]+=sample2[j]*sample3[i+j];
                    xcorr_yz[i]+=sample3[j]*sample4[i+j];
                    xcorr_zx[i]+=sample4[j]*sample2[i+j];
                }
            }
        }
        uint16_t delay_xy = 0;
        uint16_t delay_yz = 0;
        uint16_t delay_zx = 0;

        uint32_t M_xcorr_xy = xcorr_xy[0];
        uint32_t M_xcorr_yz = xcorr_yz[0];
        uint32_t M_xcorr_zx = xcorr_zx[0];
        int k;
        for(k=1;k<(SAMPLE_SIZE - WINDOW_SIZE) + 1;k++)
        {
            if(xcorr_xy[k]>M_xcorr_xy)
            {
                M_xcorr_xy = xcorr_xy[k];
                delay_xy = k;
            }
            if(xcorr_yz[k]>M_xcorr_yz)
            {
                M_xcorr_yz = xcorr_yz[k];
                delay_yz = k;
            }
            if(xcorr_zx[k]>M_xcorr_zx)
            {
                M_xcorr_zx = xcorr_zx[k];
                delay_zx = k;
            }
        }

        uint16_t d_idx[length] = {delay_xy,delay_yz,delay_zx};

        uint16_t min = d_idx[0];
        int check;
        for(check = 0;check<length;check++)
        {
            if(d_idx[check]<min)
            {
                min = d_idx[check];
            }
        }
        time_xy = (delay_xy) * 1;
        time_yz = (delay_yz) * 1;
        time_zx = (delay_zx) * 2;
        uint16_t Time_Delays[length] = {time_xy,time_yz,time_zx};

        uint16_t max = Time_Delays[0];
        int d;
        for(d=1;d<length;d++)
        {
            if(Time_Delays[d]>max)
            {
                max = Time_Delays[d];
                FL_hit = d;
            }
        }

        Second_hit = FL_hit - 1;
        if(Second_hit < 0 )
        {
            Second_hit = 2;
        }

        int ocheck,other;
        for(ocheck = 0;ocheck <length;ocheck++)
        {
            if(ocheck!= FL_hit && ocheck!=Second_hit)
            {
                other = ocheck;
            }
        }
        int k1=1, k2=0;
        if(Time_Delays[other] < Time_Delays[Second_hit])
        {
            First_hit = other;
            Last_hit = FL_hit;
            switch (First_hit)
            {
            case 0:
                Angle = 0 + (k1 * Time_Delays[Second_hit]) + k2 * (Time_Delays[Second_hit] * Time_Delays[Second_hit]);
                break;
            case 1:
                Angle = 120 + (k1 * Time_Delays[Second_hit]) + k2 * (Time_Delays[Second_hit] * Time_Delays[Second_hit]);
                break;
            case 2:
                Angle = 240 + (k1 * Time_Delays[Second_hit]) + k2 * (Time_Delays[Second_hit] * Time_Delays[Second_hit]);
                break;
            }
        }

        else if(Time_Delays[other] > Time_Delays[Second_hit])
        {
            First_hit = FL_hit;
            Last_hit = other;
            switch (First_hit)
            {
            case 0:
                Angle = 0 - (k1 * Time_Delays[Second_hit]) + k2 * (Time_Delays[Second_hit] * (Time_Delays[Second_hit]));
                break;
            case 1:
                Angle = 120 - (k1 * Time_Delays[Second_hit]) + k2 * (Time_Delays[Second_hit] * Time_Delays[Second_hit]);
                break;
            case 2:
                Angle = 240 - (k1 * Time_Delays[Second_hit]) + k2 * (Time_Delays[Second_hit] * Time_Delays[Second_hit]);
                break;
            }
        }

        if(Angle >=0 && Angle <=120)
        {
            //cyan
            setRgbColor(0, 0, 0);
            setRgbColor(0, 1023, 1023);
        }
        else if(Angle >120 && Angle <=240)
        {
            //orange
            setRgbColor(0, 0, 0);
            setRgbColor(1023, 384, 0);
        }
        else if(Angle>240)
        {
            //magenta
            setRgbColor(0, 0, 0);
            setRgbColor(1023, 0, 1023);
        }

       notprocessingcurrently = 1;
       Process_flag = 0;
    }
}
void initHw()
{
    initSystemClockTo40Mhz();
    enablePort(PORTE);
    enablePort(PORTD);
    _delay_cycles(3);

    selectPinAnalogInput(PORTE,1);
    selectPinAnalogInput(PORTE,2);
    selectPinAnalogInput(PORTE,3);
    selectPinAnalogInput(PORTD,3);
}


int main(void)
{
    initHw();
    initUart0();
    setUart0BaudRate(115200, 40e6);
    setupADC0();
    setupAdc0Ss1();
    setupADC1();
    setupAdc1Ss1();
    setupDMA();
    initRgb();
    EventTrigger_Timer();
    SetDigitalComp();

    bool Angle_Always = false;


    while(1)
    {
        Calc_Angle();
        if(kbhitUart0())
        {
            getsUart0(&data);
            putsUart0(data.buffer);
            parseFields(&data);
            if(isCommand(&data,"holdoff",1))
            {
                holdoff_variable=(uint16_t) getFieldInteger(&data,data.fieldPosition[1]);
                Holdoff_Timer(holdoff_variable);
            }
            if(isCommand(&data,"holdoff",0))
            {
                Holdoff_Timer(holdoff_variable);
            }
            if(isCommand(&data,"tdoa",0))
            {
                snprintf(str, sizeof(str), "Txy : %4"PRIu16"\n", time_xy);
                putsUart0(str);
                snprintf(str, sizeof(str), "Tyz : %4"PRIu16"\n", time_yz);
                putsUart0(str);
                snprintf(str, sizeof(str), "Tzx : %4"PRIu16"\n", time_zx);
                putsUart0(str);
            }
            if(isCommand(&data,"aoa",0))
            {
                snprintf(str, sizeof(str), "Angle : %4"PRIu16"\n", Angle);
                putsUart0(str);
            }
            if(isCommand(&data,"aoa always",0))
            {
                Angle_Always = true;
            }

        }
        if(Angle_Always)
        {
            snprintf(str, sizeof(str), "Angle : %4"PRIu16"\n", Angle);
            putsUart0(str);
        }
    }
}


