/********************************** (C) COPYRIGHT *******************************
* File Name             : main.c
* Author                : Zhiyuan Wan
* License               : MIT
* Version               : V1.0
* Date                  : 2018/03/17
* Description           : CH551做USB转I2C，使用CDC-ACM协议
*******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define DEFAULT_ENDP0_SIZE      64
#define DEFAULT_ENDP1_SIZE      64
#include <ch554.h>
#include <ch554_usb.h>
#include <debug.h>

#include "i2c.h"

__xdata __at (0x0000) uint8_t  Ep0Buffer[DEFAULT_ENDP0_SIZE];   //Endpoint 0 OUT&IN buffer, must be an even address
__xdata __at (0x0040) uint8_t  Ep1Buffer[DEFAULT_ENDP1_SIZE];   //Endpoint 1 upload buffer
__xdata __at (0x0080) uint8_t  Ep2Buffer[2*MAX_PACKET_SIZE];    //Endpoint 2 IN & OUT buffer, must be an even address

uint16_t SetupLen;
uint8_t   SetupReq,Count,UsbConfig;
const uint8_t *  pDescr;                        //USB configuration flag
USB_SETUP_REQ   SetupReqBuf;                    //Staging the Setup package
#define UsbSetupBuf      ((PUSB_SETUP_REQ)Ep0Buffer)

#define  SET_LINE_CODING                                0X20                    // Configures DTE rate, stop-bits, parity, and number-of-character
#define  GET_LINE_CODING                                0X21                    // This request allows the host to find out the currently configured line coding.
#define  SET_CONTROL_LINE_STATE                         0X22                    // This request generates RS-232/V.24 style control signals.

uint32_t Baud = 0;

/*设备描述符*/
__code uint8_t DevDesc[] = {0x12,0x01,0x10,0x01,0x02,0x00,0x00,DEFAULT_ENDP0_SIZE,
                                                        0x86,0x1a,0x22,0x57,0x00,0x01,0x01,0x02,
                                                        0x03,0x01
                                                   };
#define SI5351_ReferenceClock           "26000000"
#define Device_Version                  "1.0.1"

__code uint8_t CfgDesc[] ={
        0x09,0x02,0x43,0x00,0x02,0x01,0x00,0xa0,0x32,                    //Configuration descriptor (two interfaces)
        //The following is the interface 0 (CDC interface) descriptor
        0x09,0x04,0x00,0x00,0x01,0x02,0x02,0x01,0x00,                    //CDC interface descriptor (one endpoint)
        //The following is the function descriptor
        0x05,0x24,0x00,0x10,0x01,                                                                //Function descriptor (header)
        0x05,0x24,0x01,0x00,0x00,                                                                //Management descriptor (no data class interface) 03 01
        0x04,0x24,0x02,0x02,                                                                      //Support Set_Line_Coding, Set_Control_Line_State, Get_Line_Coding, Serial_State
        0x05,0x24,0x06,0x00,0x01,                                                                //CDC interface numbered 0; data class interface number 1
        0x07,0x05,0x81,0x03,0x10,0x00,0x40,                                        //Interrupt upload endpoint descriptor
        //The following is the interface 1 (data interface) descriptor
        0x09,0x04,0x01,0x00,0x02,0x0a,0x00,0x00,0x00,                    //Data interface descriptor
        0x07,0x05,0x02,0x02,0x40,0x00,0x00,                                        //Endpoint descriptor
        0x07,0x05,0x82,0x02,0x40,0x00,0x00,                                        //Endpoint descriptor
};
/*字符串描述符*/
unsigned char  __code LangDes[]={0x04,0x03,0x09,0x04};             //Language descriptor
unsigned char  __code SerDes[]={                                                                 //Serial number string descriptor
                                0x14,0x03,
                                '2',0x00,'0',0x00,'1',0x00,'8',0x00,'-',0x00,
                                '3',0x00,'-',0x00,
                                '1',0x00,'7',0x00
                                };
unsigned char  __code Prod_Des[]={                                                              //Product string descriptor
                                0x16,0x03,
                                'U',0x00,'S',0x00,'B',0x00,' ',0x00,'t',0x00,'o',0x00, ' ', 0x00, 
                                'I',0x00,'2',0x00,'C',0x00,
                                };
unsigned char  __code Manuf_Des[]={
        0x18,0x03,
        'Z', 0x00, 'h', 0x00, 'i', 0x00,'Y',0x00,'u', 0x00, 'a', 0x00, 'n', 0x00, ' ', 0x00, 
        'W', 0x00, 'a', 0x00, 'n', 0x00
};

//Cdc parameter
__xdata uint8_t LineCoding[7]={0x00,0xe1,0x00,0x00,0x00,0x00,0x08};   //The initial baud rate is 57600, 1 stop bit, no parity, 8 data bits.

#define UART_REV_LEN  64                                 //Serial receive buffer size
__idata uint8_t Receive_Uart_Buf[UART_REV_LEN];   //Serial receive buffer
volatile __idata uint8_t Uart_Input_Point = 0;   //The circular buffer write pointer, the bus reset needs to be initialized to 0
volatile __idata uint8_t Uart_Output_Point = 0;  //The circular buffer fetches the pointer, and the bus reset needs to be initialized to 0.
volatile __idata uint8_t UartByteCount = 0;       //Current buffer remaining bytes to be fetched


volatile __idata uint8_t USBByteCount = 0;        //Data received on behalf of the USB endpoint
volatile __idata uint8_t USBBufOutPoint = 0;    //Data pointer

volatile __idata uint8_t UpPoint2_Busy  = 0;   //Upload endpoint is busy flag

#define BOOT_ADDR  0x3800

/* This function provided a way to access the internal bootloader */
void jump_to_bootloader()
{
        USB_INT_EN = 0;
        USB_CTRL = 0x06;

        mDelaymS(100);

        EA = 0;/* Disable all interrupts */

        __asm
                LJMP BOOT_ADDR /* Jump to bootloader */
        __endasm;
        while(1); 
}



/*******************************************************************************
* Function Name  : USBDeviceCfg()
* Description   : USB设备模式配置
* Input           : None
* Output                 : None
* Return                 : None
*******************************************************************************/
void USBDeviceCfg()
{
        USB_CTRL = 0x00;                                                                                                                   //Clear USB control register
        USB_CTRL &= ~bUC_HOST_MODE;                                                                                             //This bit is the device selection mode
        USB_CTRL |=  bUC_DEV_PU_EN | bUC_INT_BUSY | bUC_DMA_EN;                                 //USB device and internal pull-up enable, automatically return to NAK before interrupt flag is cleared during interrupt
        USB_DEV_AD = 0x00;                                                                                                               //Device address initialization
        //         USB_CTRL |= bUC_LOW_SPEED;
        //         UDEV_CTRL |= bUD_LOW_SPEED;//选择低速1.5M模式
        USB_CTRL &= ~bUC_LOW_SPEED;
        UDEV_CTRL &= ~bUD_LOW_SPEED;                                                                                     //Select full speed 12M mode, default mode
        UDEV_CTRL = bUD_PD_DIS;  // Disable DP/DM pull-down resistor
        UDEV_CTRL |= bUD_PORT_EN;                                                                                                 //Enable physical port
}
/*******************************************************************************
* Function Name  : USBDeviceIntCfg()
* Description   : USB设备模式中断初始化
* Input           : None
* Output                 : None
* Return                 : None
*******************************************************************************/
void USBDeviceIntCfg()
{
        USB_INT_EN |= bUIE_SUSPEND;                                                                                        //Enable device hang interrupt
        USB_INT_EN |= bUIE_TRANSFER;                                                                                      //Enable USB transfer completion interrupt
        USB_INT_EN |= bUIE_BUS_RST;                                                                                        //Enable device mode USB bus reset interrupt
        USB_INT_FG |= 0x1F;                                                                                                        //Clear interrupt flag
        IE_USB = 1;                                                                                                                        //Enable USB interrupt
        EA = 1;                                                                                                                            //Allow microcontroller interrupt
}
/*******************************************************************************
* Function Name  : USBDeviceEndPointCfg()
* Description   : USB设备模式端点配置，模拟兼容HID设备，除了端点0的控制传输，还包括端点2批量上下传
* Input           : None
* Output                 : None
* Return                 : None
*******************************************************************************/
void USBDeviceEndPointCfg()
{
        // TODO: Is casting the right thing here? What about endianness?
        UEP1_DMA = (uint16_t) Ep1Buffer;                                                                                                          //Endpoint 1 sends the data transfer address
        UEP2_DMA = (uint16_t) Ep2Buffer;                                                                                                          //Endpoint 2 IN data transfer address
        UEP2_3_MOD = 0xCC;                                                                                                               //Endpoint 2/3 Single Buffer Transceiver Enable
        UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;                               //Endpoint 2 automatically flips the sync flag, IN transaction returns NAK, OUT returns ACK

        UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;                                                               //Endpoint 1 automatically flips the sync flag, and IN transaction returns NAK
        UEP0_DMA = (uint16_t) Ep0Buffer;                                                                                                          //Endpoint 0 data transfer address
        UEP4_1_MOD = 0X40;                                                                                                               //Endpoint 1 upload buffer; endpoint 0 single 64 byte send and receive buffer
        UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;                                                               //Manual flip, OUT transaction returns ACK, IN transaction returns NAK
}

/*******************************************************************************
* Function Name  : DeviceInterrupt()
* Description   : CH559USB中断处理函数
*******************************************************************************/
void DeviceInterrupt(void) __interrupt (INT_NO_USB)                                        //USB interrupt service routine, using register set 1
{
        uint16_t len;
        if(UIF_TRANSFER)                                                                                                                        //USB transfer completion flag
        {
                switch (USB_INT_ST & (MASK_UIS_TOKEN | MASK_UIS_ENDP))
                {
                case UIS_TOKEN_IN | 1:                                                                                            //Endpoint 1# Endpoint interrupt upload
                        UEP1_T_LEN = 0;
                        UEP1_CTRL = UEP1_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;                  //Default response NAK
                        break;
                case UIS_TOKEN_IN | 2:                                                                                            //Endpoint 2# Endpoint bulk upload
                {
                        UEP2_T_LEN = 0;                                                                                                 //Pre-use send length must be cleared
                        UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_NAK;                  //Default response NAK
                        UpPoint2_Busy = 0;                                                                                                //Clear busy flag
                }
                        break;
                case UIS_TOKEN_OUT | 2:                                                                                          //Endpoint 3# Endpoint Batch Down
                        if ( U_TOG_OK )                                                                                                  // Out of sync packets will be dropped
                        {
                                USBByteCount = USB_RX_LEN;
                                USBBufOutPoint = 0;                                                                                      //Take data pointer reset
                                UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_NAK;          //Receive a packet of data on the NAK, the main function is processed, and the main function modifies the response mode.
                        }
                        break;
                case UIS_TOKEN_SETUP | 0:                                                                                               //SETUP transaction
                        len = USB_RX_LEN;
                        if(len == (sizeof(USB_SETUP_REQ)))
                        {
                                SetupLen = ((uint16_t)UsbSetupBuf->wLengthH<<8) | (UsbSetupBuf->wLengthL);
                                len = 0;                                                                                                          // The default is success and upload 0 length
                                SetupReq = UsbSetupBuf->bRequest;
                                if ( ( UsbSetupBuf->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )//Non-standard request
                                {
                                        switch( SetupReq )
                                        {
                                        case GET_LINE_CODING:   //0x21 currently configured
                                                pDescr = LineCoding;
                                                len = sizeof(LineCoding);
                                                len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;  // This transmission length
                                                memcpy(Ep0Buffer,pDescr,len);
                                                SetupLen -= len;
                                                pDescr += len;
                                                break;
                                        case SET_CONTROL_LINE_STATE:  //0x22 generates RS-232/V.24 style control signals
                                                break;
                                        case SET_LINE_CODING:     //0x20 Configure
                                                break;
                                        default:
                                                len = 0xFF;                                                                                                                                      /*命令不支持*/
                                                break;
                                        }
                                }
                                else                                                                                                                     //Standard request
                                {
                                        switch(SetupReq)                                                                                         //Request code
                                        {
                                        case USB_GET_DESCRIPTOR:
                                                switch(UsbSetupBuf->wValueH)
                                                {
                                                case 1:                                                                                                    //Device descriptor
                                                        pDescr = DevDesc;                                                                                //Send the device descriptor to the buffer to be sent
                                                        len = sizeof(DevDesc);
                                                        break;
                                                case 2:                                                                                                         //Configuration descriptor
                                                        pDescr = CfgDesc;                                                                                 //Send the device descriptor to the buffer to be sent
                                                        len = sizeof(CfgDesc);
                                                        break;
                                                case 3:
                                                        if(UsbSetupBuf->wValueL == 0)
                                                        {
                                                                pDescr = LangDes;
                                                                len = sizeof(LangDes);
                                                        }
                                                        else if(UsbSetupBuf->wValueL == 1)
                                                        {
                                                                pDescr = Manuf_Des;
                                                                len = sizeof(Manuf_Des);
                                                        }
                                                        else if(UsbSetupBuf->wValueL == 2)
                                                        {
                                                                pDescr = Prod_Des;
                                                                len = sizeof(Prod_Des);
                                                        }
                                                        else
                                                        {
                                                                pDescr = SerDes;
                                                                len = sizeof(SerDes);
                                                        }
                                                        break;
                                                default:
                                                        len = 0xff;                                                                                             //Unsupported command or error
                                                        break;
                                                }
                                                if ( SetupLen > len )
                                                {
                                                        SetupLen = len; //Limit total length
                                                }
                                                len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;                                                   //This transmission length
                                                memcpy(Ep0Buffer,pDescr,len);                                                             //Load upload data
                                                SetupLen -= len;
                                                pDescr += len;
                                                break;
                                        case USB_SET_ADDRESS:
                                                SetupLen = UsbSetupBuf->wValueL;                                                          //Scratch USB device address
                                                break;
                                        case USB_GET_CONFIGURATION:
                                                Ep0Buffer[0] = UsbConfig;
                                                if ( SetupLen >= 1 )
                                                {
                                                        len = 1;
                                                }
                                                break;
                                        case USB_SET_CONFIGURATION:
                                                UsbConfig = UsbSetupBuf->wValueL;
                                                break;
                                        case USB_GET_INTERFACE:
                                                break;
                                        case USB_CLEAR_FEATURE:                                                                                 //Clear Feature
                                                if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_DEVICE )                                /* 清除设备 */
                                                {
                                                        if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                                                        {
                                                                if( CfgDesc[ 7 ] & 0x20 )
                                                                {
                                                                        /* 唤醒 */
                                                                }
                                                                else
                                                                {
                                                                        len = 0xFF;                                                                             /* 操作失败 */
                                                                }
                                                        }
                                                        else
                                                        {
                                                                len = 0xFF;                                                                                     /* 操作失败 */
                                                        }
                                                }
                                                else if ( ( UsbSetupBuf->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )// End point
                                                {
                                                        switch( UsbSetupBuf->wIndexL )
                                                        {
                                                        case 0x83:
                                                                UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                                                break;
                                                        case 0x03:
                                                                UEP3_CTRL = UEP3_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                                                break;
                                                        case 0x82:
                                                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                                                break;
                                                        case 0x02:
                                                                UEP2_CTRL = UEP2_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                                                break;
                                                        case 0x81:
                                                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_T_TOG | MASK_UEP_T_RES ) | UEP_T_RES_NAK;
                                                                break;
                                                        case 0x01:
                                                                UEP1_CTRL = UEP1_CTRL & ~ ( bUEP_R_TOG | MASK_UEP_R_RES ) | UEP_R_RES_ACK;
                                                                break;
                                                        default:
                                                                len = 0xFF;                                                                              // Unsupported endpoint
                                                                break;
                                                        }
                                                }
                                                else
                                                {
                                                        len = 0xFF;                                                                                             // Not endpoints are not supported
                                                }
                                                break;
                                        case USB_SET_FEATURE:                                                                             /* Set Feature */
                                                if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_DEVICE )                                /* 设置设备 */
                                                {
                                                        if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x01 )
                                                        {
                                                                if( CfgDesc[ 7 ] & 0x20 )
                                                                {
                                                                        /* 休眠 */
#ifdef DE_PRINTF
                                                                        printf( "suspend\n" );                                                                                                                   //Sleep state
#endif
                                                                        while ( XBUS_AUX & bUART0_TX )
                                                                        {
                                                                                ;       //Waiting for transmission to complete
                                                                        }
                                                                        SAFE_MOD = 0x55;
                                                                        SAFE_MOD = 0xAA;
                                                                        WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO;                                    //USB or RXD0/1 can be woken up when there is a signal
                                                                        PCON |= PD;                                                                                                                              //Sleep
                                                                        SAFE_MOD = 0x55;
                                                                        SAFE_MOD = 0xAA;
                                                                        WAKE_CTRL = 0x00;
                                                                }
                                                                else
                                                                {
                                                                        len = 0xFF;                                                                             /* 操作失败 */
                                                                }
                                                        }
                                                        else
                                                        {
                                                                len = 0xFF;                                                                                     /* 操作失败 */
                                                        }
                                                }
                                                else if( ( UsbSetupBuf->bRequestType & 0x1F ) == USB_REQ_RECIP_ENDP )                    /* 设置端点 */
                                                {
                                                        if( ( ( ( uint16_t )UsbSetupBuf->wValueH << 8 ) | UsbSetupBuf->wValueL ) == 0x00 )
                                                        {
                                                                switch( ( ( uint16_t )UsbSetupBuf->wIndexH << 8 ) | UsbSetupBuf->wIndexL )
                                                                {
                                                                case 0x83:
                                                                        UEP3_CTRL = UEP3_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点3 IN STALL */
                                                                        break;
                                                                case 0x03:
                                                                        UEP3_CTRL = UEP3_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点3 OUT Stall */
                                                                        break;
                                                                case 0x82:
                                                                        UEP2_CTRL = UEP2_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点2 IN STALL */
                                                                        break;
                                                                case 0x02:
                                                                        UEP2_CTRL = UEP2_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点2 OUT Stall */
                                                                        break;
                                                                case 0x81:
                                                                        UEP1_CTRL = UEP1_CTRL & (~bUEP_T_TOG) | UEP_T_RES_STALL;/* 设置端点1 IN STALL */
                                                                        break;
                                                                case 0x01:
                                                                        UEP1_CTRL = UEP1_CTRL & (~bUEP_R_TOG) | UEP_R_RES_STALL;/* 设置端点1 OUT Stall */
                                                                default:
                                                                        len = 0xFF;                                                                     /* 操作失败 */
                                                                        break;
                                                                }
                                                        }
                                                        else
                                                        {
                                                                len = 0xFF;                                                                       /* 操作失败 */
                                                        }
                                                }
                                                else
                                                {
                                                        len = 0xFF;                                                                               /* 操作失败 */
                                                }
                                                break;
                                        case USB_GET_STATUS:
                                                Ep0Buffer[0] = 0x00;
                                                Ep0Buffer[1] = 0x00;
                                                if ( SetupLen >= 2 )
                                                {
                                                        len = 2;
                                                }
                                                else
                                                {
                                                        len = SetupLen;
                                                }
                                                break;
                                        default:
                                                len = 0xff;                                                                                                     //operation failed
                                                break;
                                        }
                                }
                        }
                        else
                        {
                                len = 0xff;                                                                                                              //Wrong packet length
                        }
                        if(len == 0xff)
                        {
                                SetupReq = 0xFF;
                                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;//STALL
                        }
                        else if(len <= DEFAULT_ENDP0_SIZE)                                                                                                         //Upload data or status stage returns 0 length package
                        {
                                UEP0_T_LEN = len;
                                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//The default packet is DATA1, which returns a response ACK.
                        }
                        else
                        {
                                UEP0_T_LEN = 0;  //Although it has not yet reached the status stage, it uploads 0 length data packets in advance to prevent the host from entering the status stage in advance.
                                UEP0_CTRL = bUEP_R_TOG | bUEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;//The default packet is DATA1, which returns a response ACK.
                        }
                        break;
                case UIS_TOKEN_IN | 0:                                                                                                    //Endpoint0 IN
                        switch(SetupReq)
                        {
                        case USB_GET_DESCRIPTOR:
                                len = SetupLen >= DEFAULT_ENDP0_SIZE ? DEFAULT_ENDP0_SIZE : SetupLen;                                                            //This transmission length
                                memcpy( Ep0Buffer, pDescr, len );                                                                  //Load upload data
                                SetupLen -= len;
                                pDescr += len;
                                UEP0_T_LEN = len;
                                UEP0_CTRL ^= bUEP_T_TOG;                                                                                         //Sync flag bit flip
                                break;
                        case USB_SET_ADDRESS:
                                USB_DEV_AD = USB_DEV_AD & bUDA_GP_BIT | SetupLen;
                                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                                break;
                        default:
                                UEP0_T_LEN = 0;                                                                                                   //The status phase completes the interrupt or is forced to upload 0 length packet end control transmission
                                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                                break;
                        }
                        break;
                case UIS_TOKEN_OUT | 0:  // Endpoint0 OUT
                        if(SetupReq == SET_LINE_CODING)  //Set the serial port properties
                        {
                                if( U_TOG_OK )
                                {
                                        memcpy(LineCoding,UsbSetupBuf,USB_RX_LEN);
                                        *((uint8_t *)&Baud) = LineCoding[0];
                                        *((uint8_t *)&Baud+1) = LineCoding[1];
                                        *((uint8_t *)&Baud+2) = LineCoding[2];
                                        *((uint8_t *)&Baud+3) = LineCoding[3];

                                        if(Baud > 999999) Baud = 57600;

                                        UEP0_T_LEN = 0;
                                        UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;  // Ready to upload 0 packages
                                }
                        }
                        else
                        {
                                UEP0_T_LEN = 0;
                                UEP0_CTRL |= UEP_R_RES_ACK | UEP_T_RES_ACK;  // Just ACK is fine.
                        }
                        break;



                default:
                        break;
                }
                UIF_TRANSFER = 0;                                                                                                                  //Write 0 to clear interrupt
        }
        if(UIF_BUS_RST)                                                                                                                          //Device Mode USB Bus Reset Interrupt
        {
#ifdef DE_PRINTF
                printf( "reset\n" );                                                                                                                     //Sleep state
#endif
                UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
                UEP1_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK;
                UEP2_CTRL = bUEP_AUTO_TOG | UEP_T_RES_NAK | UEP_R_RES_ACK;
                USB_DEV_AD = 0x00;
                UIF_SUSPEND = 0;
                UIF_TRANSFER = 0;
                UIF_BUS_RST = 0;                                                                                                                         //Clear interrupt flag
                Uart_Input_Point = 0;   //Circular buffer input pointer
                Uart_Output_Point = 0;  //Circular buffer read pointer
                UartByteCount = 0;        //Current buffer remaining bytes to be fetched
                USBByteCount = 0;          //Length received by the USB endpoint
                UsbConfig = 0;            //Clear configuration value
                UpPoint2_Busy = 0;
        }
        if (UIF_SUSPEND)                                                                                                                                 //USB bus suspend/wake up
        {
                UIF_SUSPEND = 0;
                if ( USB_MIS_ST & bUMS_SUSPEND )                                                                                         //Hang
                {
#ifdef DE_PRINTF
                        printf( "suspend\n" );                                                                                                                   //Sleep state
#endif
                        while ( XBUS_AUX & bUART0_TX )
                        {
                                ;       //Waiting for transmission to complete
                        }
                        SAFE_MOD = 0x55;
                        SAFE_MOD = 0xAA;
                        WAKE_CTRL = bWAK_BY_USB | bWAK_RXD0_LO | bWAK_RXD1_LO;                                    //USB or RXD0/1 can be woken up when there is a signal
                        PCON |= PD;                                                                                                                              //Sleep
                        SAFE_MOD = 0x55;
                        SAFE_MOD = 0xAA;
                        WAKE_CTRL = 0x00;
                }
        }
        else {                                                                                                                                                   //Unexpected interruption, impossible situation
                USB_INT_FG = 0xFF;                                                                                                                       //Clear interrupt flag

        }
}

void virtual_uart_tx(uint8_t tdata)
{
        Receive_Uart_Buf[Uart_Input_Point++] = tdata;
        UartByteCount++;                                        //Current buffer remaining bytes to be fetched
        if(Uart_Input_Point>=UART_REV_LEN)
        {
                Uart_Input_Point = 0;
        }
}

void v_uart_puts(char *str)
{
        while(*str)
                virtual_uart_tx(*(str++));
}

void usb_poll()
{
        uint8_t length;
        static uint8_t Uart_Timeout = 0;
        if(UsbConfig)
        {
                if(UartByteCount)
                        Uart_Timeout++;
                if(!UpPoint2_Busy)   //The endpoint is not busy (the first packet of data after idle, only used to trigger the upload)
                {
                        length = UartByteCount;
                        if(length>0)
                        {
                                if(length>39 || Uart_Timeout>100)
                                {
                                        Uart_Timeout = 0;
                                        if(Uart_Output_Point+length>UART_REV_LEN)
                                                length = UART_REV_LEN-Uart_Output_Point;
                                        UartByteCount -= length;
                                        //Write upload endpoint
                                        memcpy(Ep2Buffer+MAX_PACKET_SIZE,&Receive_Uart_Buf[Uart_Output_Point],length);
                                        Uart_Output_Point+=length;
                                        if(Uart_Output_Point>=UART_REV_LEN)
                                                Uart_Output_Point = 0;
                                        UEP2_T_LEN = length;                                                                                                    //Pre-use send length must be cleared
                                        UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_T_RES | UEP_T_RES_ACK;                       //Answer ACK
                                        UpPoint2_Busy = 1;
                                }
                        }
                }
        }
}


void uart_poll()
{/* 串口 处理程序 */
        uint8_t uart_data;
        bool i2c_ack = 0;
//        Static bool i2c_status = 0;
        static uint8_t i2c_frame_len = 0;
        static uint8_t i2c_frame_rx_len = 0;
        static uint8_t i2c_error_no = 0;
        static uint8_t uart_rx_status = 0;
        static uint8_t former_data = 0;
        static uint8_t dontstop = 0;
        uint8_t i;

        if(USBByteCount)   //USB receiving endpoint has data
        {
                uart_data = Ep2Buffer[USBBufOutPoint++];

                if(uart_rx_status == 0)
                {
                        if(uart_data == 'Q')
                        {
                                v_uart_puts(SI5351_ReferenceClock); /* 26MHz Crystal */
                                v_uart_puts("\r\n");
                        }
                        else if(uart_data == 'V')
                        { /* Version */
                                v_uart_puts(Device_Version); /* Device version */
                                v_uart_puts("\r\n");
                        }
                        else if(uart_data == 'E')
                        {
                                jump_to_bootloader();

                        }
                        else if(uart_data == 'B')
                        {
                                virtual_uart_tx(Baud / 100000 + '0');
                                virtual_uart_tx(Baud % 100000 / 10000 + '0');
                                virtual_uart_tx(Baud % 10000 / 1000 + '0');
                                virtual_uart_tx(Baud % 1000 / 100 + '0');
                                virtual_uart_tx(Baud % 100 / 10 + '0');
                                virtual_uart_tx(Baud % 10 / 1 + '0');
                                v_uart_puts("\r\n");
                        }
                        else if(uart_data == 'T' && former_data != 'A') /* BAN AT commands */
                        { /* Transmit I2C Data: T: <LEN>, 16bytes data, performing S, <AR>, <DAT>, E */

                                i2c_frame_rx_len = 0;
                                uart_rx_status = 1;
                                i2c_error_no = 0;
                                i2c_frame_len = 0;
                        }
                        else if(uart_data == 'R')
                        { /* Recieve I2C Data: R<AR><LEN> */
                                i2c_frame_rx_len = 0;
                                uart_rx_status = 3;
                                i2c_error_no = 0;
                                i2c_frame_len = 0;
                        }
                        else if(uart_data == 'T' && former_data == 'A') /* BAN AT commands */
                        {
                                v_uart_puts("OK\r\n");
                        }
                        else if(uart_data == 'A')
                        {
                        }
                        else 
                        {
                                v_uart_puts("NOT SUPPORTED\r\n");
                        }
                }
                else if(uart_rx_status == 1)
                { // 54 03 C0 02 53
                        i2c_frame_len = uart_data & 0x3f; /* 拿到长度 */
                        if(uart_data & 0x80)
                                dontstop = 1;
                        else
                                dontstop = 0;

                        i2c_start();
                        uart_rx_status = 2;
                }
                else if(uart_rx_status == 2)
                {
                        if(i2c_error_no == 0)
                        {
                                i2c_write(uart_data);
                                i2c_ack = i2c_read_ack();
                                if(i2c_ack != 1)
                                {
                                        i2c_stop();
                                        i2c_error_no = i2c_frame_rx_len + 1;
                                }
                        }
                        i2c_frame_rx_len ++;

                        if(i2c_frame_len == i2c_frame_rx_len)
                        {
                                if(i2c_error_no == 0)
                                {
                                        v_uart_puts("OK\r\n");
                                        if(dontstop == 0)
                                                i2c_stop(); /* 停止I2C */
                                }
                                else
                                {
                                        virtual_uart_tx('F');
                                        virtual_uart_tx(i2c_error_no / 10 + '0');
                                        virtual_uart_tx(i2c_error_no % 10 + '0'); /* 传输失败 */
                                        v_uart_puts("\r\n");
                                }

                                i2c_frame_len = 0;
                                i2c_frame_rx_len = 0;
                                uart_rx_status = 0;
                                i2c_error_no = 0;
                        }

                }
                else if(uart_rx_status == 3)
                {
                        i2c_start();
                        i2c_write(uart_data); /* 送地址 */
                        i2c_ack = i2c_read_ack();
                        if(i2c_ack != 1)
                        {
                                v_uart_puts("FAIL\r\n");
                                i2c_stop();
                                uart_rx_status = 0;
                        }
                        uart_rx_status = 4;
                }
                else if(uart_rx_status == 4)
                {
                        i2c_frame_len = uart_data & 0x3f;
                        for(i = 0; i < i2c_frame_len; i++)
                        {
                                virtual_uart_tx(i2c_read());
                                i2c_ack = i2c_read_ack();
                                if(i2c_ack != 1)
                                        break;
                        }
                        i2c_stop();
                        uart_rx_status = 0;
                }
                former_data = uart_data;

                USBByteCount--;

                if(USBByteCount==0)
                        UEP2_CTRL = UEP2_CTRL & ~ MASK_UEP_R_RES | UEP_R_RES_ACK;
        }
}

//Main function
main()
{
        CfgFsys( );                                                                                                                //CH559 clock selection configuration
        mDelaymS(5);                                                                                                              //Modify the main frequency and wait for the internal crystal to stabilize.
        mInitSTDIO( );                                                                                                          //Serial port 0, can be used for debugging
        //UART1Setup( );//用于CDC

#ifdef DE_PRINTF
        printf("start ...\n");
#endif
        i2c_init();

        USBDeviceCfg();
        USBDeviceEndPointCfg();                                                                                    //Endpoint configuration
        USBDeviceIntCfg();                                                                                                      //Interrupt initialization
        UEP0_T_LEN = 0;
        UEP1_T_LEN = 0;                                                                                                    //Pre-use send length must be cleared
        UEP2_T_LEN = 0;                                                                                    //Pre-use send length must be cleared

        while(1)
        {
                usb_poll();
                uart_poll();
        }
}
