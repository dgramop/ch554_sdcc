/********************************** (C) COPYRIGHT *******************************
* File Name          : Main.C
* Author             : WCH
* Version            : V1.1
* Date               : 2017/07/05
* Description        : CH554 touch button interrupt and query mode to collect and report the current sampling channel button status, including initialization and button sampling and other demonstration functions
*******************************************************************************/
#include <stdint.h>
#include <stdio.h>

#include <ch554.h>
#include <debug.h>
#include <touchkey.h>

void main()
{
    uint8_t i;
    CfgFsys( );                                                                //CH554 clock selection configuration
    mDelaymS(5);                                                               //Modify the main frequency to recommend a slight delay to wait for the chip power supply to stabilize
    mInitSTDIO( );                                                             //Serial port 0 initialization
    UART1Setup();

    printf("start ...\n");

    P1_DIR_PU &= 0x0C;                                                         //All touch channels are set to floating input, and unused channels can be left unset.
    TouchKeyQueryCyl2ms();                                                     //TouchKey query cycle 2ms
    GetTouchKeyFree();                                                         //Get the sampling reference value
#if DE_PRINTF
    for(i=KEY_FIRST;i<(KEY_LAST+1);i++)                                        //Print sample reference value
    {
        printf("Channel %02x base sample %04x\n",(uint16_t)i,KeyFree[i]);
    }
#endif

#if INTERRUPT_TouchKey
    EA = 1;
    while(1)
    {
        if(KeyBuf)                                                               //Key_buf is non-zero, indicating that a button press is detected
        {
            printf("INT TouchKey Channel %02x \n",(uint16_t)KeyBuf);                 //Print the current button status channel
            KeyBuf	= 0;                                                           //Clear button press mark
            mDelaymS(100);                                                         //Delay is meaningless, analog microcontrollers do button processing
        }
        mDelaymS(100);                                                           //Delay is meaningless, analog microcontroller does other things
    }
#else
    while(1)
    {
        TouchKeyChannelQuery();                                                  //Query touch button status
        if(KeyBuf)                                                               //Key_buf is non-zero, indicating that a button press is detected
        {
            printf("Query TouchKey Channel %02x \n",(uint16_t)KeyBuf);              //Print the current button status channel
            KeyBuf = 0;                                                           //Clear button press mark
            mDelaymS(20);                                                         //Delay is meaningless, analog microcontrollers do button processing
        }
        //       mDelaymS(100);                                                           //Delay is meaningless, analog microcontroller does other things
    }
#endif
}
