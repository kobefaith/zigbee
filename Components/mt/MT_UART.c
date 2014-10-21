/***************************************************************************************************
  Filename:       MT_UART.c
  Revised:        $Date: 2009-03-12 16:25:22 -0700 (Thu, 12 Mar 2009) $
  Revision:       $Revision: 19404 $

  Description:  This module handles anything dealing with the serial port.

  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.

***************************************************************************************************/

/***************************************************************************************************
 * INCLUDES
 ***************************************************************************************************/
#include "ZComDef.h"
#include "OSAL.h"
#include "hal_uart.h"
#include "MT.h"
#include "MT_UART.h"
#include "OSAL_Memory.h"
#include "SampleApp.h"


/***************************************************************************************************
 * MACROS
 ***************************************************************************************************/

/***************************************************************************************************
 * CONSTANTS
 ***************************************************************************************************/
/* State values for ZTool protocal */
#define SOP_STATE      0x00
#define CMD_STATE1     0x01
#define CMD_STATE2     0x02
#define LEN_STATE      0x03
#define DATA_STATE     0x04
#define FCS_STATE      0x05

/***************************************************************************************************
 *                                         GLOBAL VARIABLES
 ***************************************************************************************************/
/* Used to indentify the application ID for osal task */
byte App_TaskID;

uint8  rxlen;    //接收数据长度
uint8* rbuf;  //接收数据指针
uint8 IDbuf[5];  //ID
uint8 ISID;
uint8 buflen;
uint8 pIDbuf[5];
uint16 zgID;//ID 的全局变量

/* ZTool protocal parameters */
uint8 state;
uint8  CMD_Token[2];
uint8  LEN_Token;
uint8  FSC_Token;
mtOSALSerialData_t  *pMsg;
uint8  tempDataLen;

#if defined (ZAPP_P1) || defined (ZAPP_P2)
uint16  MT_UartMaxZAppBufLen;
bool    MT_UartZAppRxStatus;
#endif


/***************************************************************************************************
 *                                          LOCAL FUNCTIONS
 ***************************************************************************************************/
static void rxCB( uint8 port, uint8 event )
{
  extern uint8 SampleApp_TaskID;
  rxlen=Hal_UART_RxBufLen(SERIAL_APP_PORT);  //接收缓冲区数据长度,字节为单位
  readbuf();  //读取buf的数值，并判断时候用ID功能  
  if(rxlen==0)
     osal_mem_free( rbuf );  //释放内存 
  else
  osal_set_event(SampleApp_TaskID,UART_RX_CB_EVT);
}

void readbuf(void)//拿出来的原因：因为只有在原来只有串口来数据才会将ID给pIDbuf，
//拿出来后就可以在初始化时调用这个函数所以在上电即可以提取ID，可以进行测试。
{
    uint8 i; 
    rbuf=osal_mem_alloc(rxlen+9);  //多分配1字节,分配如下  
    
    rbuf[0]=rxlen+8;                                     //一字节存放数据长度
    
    rbuf[1]=73;                                          //字符：I
    rbuf[2]=68;                                          //字符：D
    rbuf[3]=58;                                          //字符： ：
    Setid();
    rbuf[4]=(uint8)((zgID&0xf000)>>12)+48;                   //1
    rbuf[5]=(uint8)((zgID&0x0f00)>>8)+48;                    //2
    rbuf[6]=(uint8)((zgID&0x00f0)>>4)+48;                    //3
    rbuf[7]=(uint8)(zgID&0x000f)+48;                         //4
    rbuf[8]='\n';
    
    buflen=rxlen+9;//获得rbuf的长度，放在buflen中，在SampleApp中直接调用
    
    for(i=0;i<5;i++)//提取设备ID，存放在数组IDbuf中，本地显示
    {
      IDbuf[i]=rbuf[4+i];//从rbuf[4]开始提取
    }    
    for(i=0;i<4;i++)//发送出去作为测试用
    {
      pIDbuf[i]=IDbuf[i];
    }
    pIDbuf[4]=rbuf[8];   //最后存放了换行符 
    HalUARTRead ( SERIAL_APP_PORT, rbuf+9, rxlen); //读接收缓冲区数据到内存rbuf+9
}

void ID_Init(void)
{
  uint16 Init_ID=0x1234;       //默认PANID，自己可以任意修改
  uint16 preID;                    //这个是指向ZCD_NV_PANID的值
  uint16 ID;                       //这个想ZCD_NV_PANID1的值
  
  osal_nv_read(ZCD_NV_ID,0,2,&preID);
  osal_nv_read(ZCD_NV_ID1,0,2,&ID);
  //如果用串口调试助手修改了，则同时修改ZCD_NV_PANID和ZCD_NV_PANID1
  //并且这两个值是相同的，所以当修改后则不会进入下面这个条件语句中
  //而在修改之前通过下面的语句将默认PANIDID存入NV中，使得设备启动的
  //时候有一个固定的PANID
  if(preID!=ID)
  {
   if ( osal_nv_item_init( ZCD_NV_ID,
                              2,
                              &Init_ID ) == ZSUCCESS )
  
    {  
        osal_nv_write(ZCD_NV_ID,0,2,&Init_ID);
        osal_nv_read(ZCD_NV_ID,0,2,&Init_ID);
        zgID=Init_ID;
    } 
  }
}


void Setid(void)
{
  uint16 gu16RecBuffLen;
  
   if ( osal_nv_item_init( ZCD_NV_ID,
                              2,
                              &gu16RecBuffLen ) == ZSUCCESS )
  
  {
    osal_nv_read(ZCD_NV_ID,0,2,&gu16RecBuffLen);
  }
  zgID=gu16RecBuffLen;
}
/***************************************************************************************************
 * @fn      MT_UartInit
 *
 * @brief   Initialize MT with UART support
 *
 * @param   None
 *
 * @return  None
***************************************************************************************************/
void MT_UartInit ()
{
  halUARTCfg_t uartConfig;

  /* Initialize APP ID */
  App_TaskID = 0;

  /* UART Configuration */
  uartConfig.configured           = TRUE;
  uartConfig.baudRate             = MT_UART_DEFAULT_BAUDRATE;
  uartConfig.flowControl          = MT_UART_DEFAULT_OVERFLOW;
  uartConfig.flowControlThreshold = MT_UART_DEFAULT_THRESHOLD;
  uartConfig.rx.maxBufSize        = MT_UART_DEFAULT_MAX_RX_BUFF;
  uartConfig.tx.maxBufSize        = MT_UART_DEFAULT_MAX_TX_BUFF;
  uartConfig.idleTimeout          = MT_UART_DEFAULT_IDLE_TIMEOUT;
  uartConfig.intEnable            = TRUE;
#if defined (ZTOOL_P1) || defined (ZTOOL_P2)
  uartConfig.callBackFunc         = MT_UartProcessZToolData;
  uartConfig.callBackFunc         = rxCB; 
  HalUARTOpen (SERIAL_APP_PORT, &uartConfig);
#elif defined (ZAPP_P1) || defined (ZAPP_P2)
  uartConfig.callBackFunc         = MT_UartProcessZAppData;
#else
  uartConfig.callBackFunc         = NULL;
#endif
  HalUARTOpen (SERIAL_APP_PORT, &uartConfig);
  /* Start UART */
#if defined (MT_UART_DEFAULT_PORT)
  //HalUARTOpen (MT_UART_DEFAULT_PORT, &uartConfig);
#else
  /* Silence IAR compiler warning */
  (void)uartConfig;
#endif

  /* Initialize for ZApp */
#if defined (ZAPP_P1) || defined (ZAPP_P2)
  /* Default max bytes that ZAPP can take */
  MT_UartMaxZAppBufLen  = 1;
  MT_UartZAppRxStatus   = MT_UART_ZAPP_RX_READY;
#endif

}

/***************************************************************************************************
 * @fn      MT_SerialRegisterTaskID
 *
 * @brief   This function registers the taskID of the application so it knows
 *          where to send the messages whent they come in.
 *
 * @param   void
 *
 * @return  void
 ***************************************************************************************************/
void MT_UartRegisterTaskID( byte taskID )
{
  App_TaskID = taskID;
}

/***************************************************************************************************
 * @fn      SPIMgr_CalcFCS
 *
 * @brief   Calculate the FCS of a message buffer by XOR'ing each byte.
 *          Remember to NOT include SOP and FCS fields, so start at the CMD field.
 *
 * @param   byte *msg_ptr - message pointer
 * @param   byte len - length (in bytes) of message
 *
 * @return  result byte
 ***************************************************************************************************/
byte MT_UartCalcFCS( uint8 *msg_ptr, uint8 len )
{
  byte x;
  byte xorResult;

  xorResult = 0;

  for ( x = 0; x < len; x++, msg_ptr++ )
    xorResult = xorResult ^ *msg_ptr;

  return ( xorResult );
}


/***************************************************************************************************
 * @fn      MT_UartProcessZToolData
 *
 * @brief   | SOP | Data Length  |   CMD   |   Data   |  FCS  |
 *          |  1  |     1        |    2    |  0-Len   |   1   |
 *
 *          Parses the data and determine either is SPI or just simply serial data
 *          then send the data to correct place (MT or APP)
 *
 * @param   port     - UART port
 *          event    - Event that causes the callback
 *
 *
 * @return  None
 ***************************************************************************************************/
void MT_UartProcessZToolData ( uint8 port, uint8 event )
{
  uint8  ch;
  uint8  bytesInRxBuffer;
  
  (void)event;  // Intentionally unreferenced parameter

  while (Hal_UART_RxBufLen(port))
  {
    HalUARTRead (port, &ch, 1);

    switch (state)
    {
      case SOP_STATE:
        if (ch == MT_UART_SOF)
          state = LEN_STATE;
        break;

      case LEN_STATE:
        LEN_Token = ch;

        tempDataLen = 0;

        /* Allocate memory for the data */
        pMsg = (mtOSALSerialData_t *)osal_msg_allocate( sizeof ( mtOSALSerialData_t ) +
                                                        MT_RPC_FRAME_HDR_SZ + LEN_Token );

        if (pMsg)
        {
          /* Fill up what we can */
          pMsg->hdr.event = CMD_SERIAL_MSG;
          pMsg->msg = (uint8*)(pMsg+1);
          pMsg->msg[MT_RPC_POS_LEN] = LEN_Token;
          state = CMD_STATE1;
        }
        else
        {
          state = SOP_STATE;
          return;
        }
        break;

      case CMD_STATE1:
        pMsg->msg[MT_RPC_POS_CMD0] = ch;
        state = CMD_STATE2;
        break;

      case CMD_STATE2:
        pMsg->msg[MT_RPC_POS_CMD1] = ch;
        /* If there is no data, skip to FCS state */
        if (LEN_Token)
        {
          state = DATA_STATE;
        }
        else
        {
          state = FCS_STATE;
        }
        break;

      case DATA_STATE:

        /* Fill in the buffer the first byte of the data */
        pMsg->msg[MT_RPC_FRAME_HDR_SZ + tempDataLen++] = ch;

        /* Check number of bytes left in the Rx buffer */
        bytesInRxBuffer = Hal_UART_RxBufLen(port);

        /* If the remain of the data is there, read them all, otherwise, just read enough */
        if (bytesInRxBuffer <= LEN_Token - tempDataLen)
        {
          HalUARTRead (port, &pMsg->msg[MT_RPC_FRAME_HDR_SZ + tempDataLen], bytesInRxBuffer);
          tempDataLen += bytesInRxBuffer;
        }
        else
        {
          HalUARTRead (port, &pMsg->msg[MT_RPC_FRAME_HDR_SZ + tempDataLen], LEN_Token - tempDataLen);
          tempDataLen += (LEN_Token - tempDataLen);
        }

        /* If number of bytes read is equal to data length, time to move on to FCS */
        if ( tempDataLen == LEN_Token )
            state = FCS_STATE;

        break;

      case FCS_STATE:

        FSC_Token = ch;

        /* Make sure it's correct */
        if ((MT_UartCalcFCS ((uint8*)&pMsg->msg[0], MT_RPC_FRAME_HDR_SZ + LEN_Token) == FSC_Token))
        {
          osal_msg_send( App_TaskID, (byte *)pMsg );
        }
        else
        {
          /* deallocate the msg */
          osal_msg_deallocate ( (uint8 *)pMsg );
        }

        /* Reset the state, send or discard the buffers at this point */
        state = SOP_STATE;

        break;

      default:
       break;
    }
  }
}

#if defined (ZAPP_P1) || defined (ZAPP_P2)
/***************************************************************************************************
 * @fn      MT_UartProcessZAppData
 *
 * @brief   | SOP | CMD  |   Data Length   | FSC  |
 *          |  1  |  2   |       1         |  1   |
 *
 *          Parses the data and determine either is SPI or just simply serial data
 *          then send the data to correct place (MT or APP)
 *
 * @param   port    - UART port
 *          event   - Event that causes the callback
 *
 *
 * @return  None
 ***************************************************************************************************/
void MT_UartProcessZAppData ( uint8 port, uint8 event )
{

  osal_event_hdr_t  *msg_ptr;
  uint16 length = 0;
  uint16 rxBufLen  = Hal_UART_RxBufLen(MT_UART_DEFAULT_PORT);

  /*
     If maxZAppBufferLength is 0 or larger than current length
     the entire length of the current buffer is returned.
  */
  if ((MT_UartMaxZAppBufLen != 0) && (MT_UartMaxZAppBufLen <= rxBufLen))
  {
    length = MT_UartMaxZAppBufLen;
  }
  else
  {
    length = rxBufLen;
  }

  /* Verify events */
  if (event == HAL_UART_TX_FULL)
  {
    // Do something when TX if full
    return;
  }

  if (event & ( HAL_UART_RX_FULL | HAL_UART_RX_ABOUT_FULL | HAL_UART_RX_TIMEOUT))
  {
    if ( App_TaskID )
    {
      /*
         If Application is ready to receive and there is something
         in the Rx buffer then send it up
      */
      if ((MT_UartZAppRxStatus == MT_UART_ZAPP_RX_READY ) && (length != 0))
      {
        /* Disable App flow control until it processes the current data */
         MT_UartAppFlowControl (MT_UART_ZAPP_RX_NOT_READY);

        /* 2 more bytes are added, 1 for CMD type, other for length */
        msg_ptr = (osal_event_hdr_t *)osal_msg_allocate( length + sizeof(osal_event_hdr_t) );
        if ( msg_ptr )
        {
          msg_ptr->event = SPI_INCOMING_ZAPP_DATA;
          msg_ptr->status = length;

          /* Read the data of Rx buffer */
          HalUARTRead( MT_UART_DEFAULT_PORT, (uint8 *)(msg_ptr + 1), length );

          /* Send the raw data to application...or where ever */
          osal_msg_send( App_TaskID, (uint8 *)msg_ptr );
        }
      }
    }
  }
}

/***************************************************************************************************
 * @fn      SPIMgr_ZAppBufferLengthRegister
 *
 * @brief
 *
 * @param   maxLen - Max Length that the application wants at a time
 *
 * @return  None
 *
 ***************************************************************************************************/
void MT_UartZAppBufferLengthRegister ( uint16 maxLen )
{
  /* If the maxLen is larger than the RX buff, something is not right */
  if (maxLen <= MT_UART_DEFAULT_MAX_RX_BUFF)
    MT_UartMaxZAppBufLen = maxLen;
  else
    MT_UartMaxZAppBufLen = 1; /* default is 1 byte */
}

/***************************************************************************************************
 * @fn      SPIMgr_AppFlowControl
 *
 * @brief
 *
 * @param   status - ready to send or not
 *
 * @return  None
 *
 ***************************************************************************************************/
void MT_UartAppFlowControl ( bool status )
{

  /* Make sure only update if needed */
  if (status != MT_UartZAppRxStatus )
  {
    MT_UartZAppRxStatus = status;
  }

  /* App is ready to read again, ProcessZAppData have to be triggered too */
  if (status == MT_UART_ZAPP_RX_READY)
  {
    MT_UartProcessZAppData (MT_UART_DEFAULT_PORT, HAL_UART_RX_TIMEOUT );
  }

}

#endif //ZAPP

/***************************************************************************************************
***************************************************************************************************/
