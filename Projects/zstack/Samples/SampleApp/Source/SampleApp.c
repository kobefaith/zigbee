/**************************************************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).
**************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "OSAL_Nv.h"
#include "ZDObject.h"
#include "ioCC2530.h"   
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};


endPointDesc_t SampleApp_epDesc;


/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;
afAddrType_t SampleApp_SPI_SendData_DstAddr;
afAddrType_t SampleApp_TestID_DstAddr;
aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;
uint16 short_test;

extern uint8* rbuf;  //接收数据指针
extern uint8  rxlen;    //接收数据长度
extern uint8 buflen;
extern devStartModes_t devStartMode;
extern uint8 IDbuf[5];  //ID
extern uint8 pIDbuf[5];
extern void readbuf(void);
/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );

void SampleApp_SPI_short(void);
void SampleApp_SPI_mac(void);
void SampleApp_showhex(uint16 sixteenword);
uint8 hextoword2(uint8 t);
uint8 hextoword1(uint8 t);
void SampleApp_SPI_SendData( uint8 *buf, uint8 len );
void SampleApp_UART_EVT(void);


//测试节点ID
void CompforTESTID(uint8 *dat);

void SampleApp_Send_TestID_Message(void);
void CompforFUWEI(uint8 *dat);

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;

 #if defined ( HOLD_AUTO_START )
  
  ZDOInitDevice(0);
#endif



   // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );
   
  readbuf();//初始化，获取ID值
  
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      { 
         // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;
         
        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
               
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (SampleApp_NwkState == DEV_ZB_COORD)
              || (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
            
             osal_start_timerEx( SampleApp_TaskID, 
                              SAMPLEAPP_SEND_UART_MSG_EVT,
                              SAMPLEAPP_SEND_UART_MSG_TIMEOUT );
          }
          else
          {
            // Device is no longer in the network
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()).
  if ( events & UART_RX_CB_EVT)//发送接收所需串口函数
  {

    SampleApp_UART_EVT();
    return (events ^ UART_RX_CB_EVT);
  }
  
  if(events&SAMPLEAPP_SEND_UART_MSG_EVT)//回显所需串口函数
  {
      SampleApp_SPI_short();
      SampleApp_SPI_mac();
         
     return (events ^ SAMPLEAPP_SEND_UART_MSG_EVT);
  }
  
   if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
    // Send the periodic message
    SampleApp_SendPeriodicMessage();

    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        60000 );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }
   
    if(events & SAMLPEAPP_TESTID_MSG_EVT)
  {
     //处理测试节点请求ID事件
     SampleApp_Send_TestID_Message();
     return (events ^ SAMLPEAPP_TESTID_MSG_EVT);
  } 
  
  if(events & SAMLPEAPP_TESTBATTERY_MSG_EVT)
  {
     SystemResetSoft();
     return (events ^ SAMLPEAPP_TESTBATTERY_MSG_EVT);
  } 
  return 0;
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint8 *pointer1;  
  uint8 shortadd[5];
  uint8 checktest;
  uint8 u16NewPaind;
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_PERIODIC_CLUSTERID:   //这个是周期信息用于测试，主要是设备ID
           
     
     case SAMPLEAPP_FLASH_CLUSTERID:      
      //shortadd1[0]='\n';       
      
    if ( osal_nv_item_init( ZCD_NV_IDSET,1,&u16NewPaind ) == ZSUCCESS )  
      {
         osal_nv_read(ZCD_NV_IDSET,0,1,&u16NewPaind);
      }  
      if(u16NewPaind==1)//带ID
      {
        pointer1=&pkt->cmd.Data[1];  //接收数据指针,指向数据
        HalUARTWrite(0,pointer1,pkt->cmd.Data[0]);//cmd.Data[0]是数据的大小
       
      }
      else
      {
        pointer1=&pkt->cmd.Data[9];  //接收数据指针,指向数据
        HalUARTWrite(SERIAL_APP_PORT,pointer1,pkt->cmd.Data[0]-8);//cmd.Data[0]是数据的大小
       
      }
      break;
        
      case SAMPLEAPP_TEST_ID_CLUSTERID:
         osal_start_timerEx( SampleApp_TaskID, SAMLPEAPP_TESTID_MSG_EVT,
        ((osal_rand() & 0x00FF)) );        
      break;
      
     case SAMPLEAPP_TEST_ID_BACK_CLUSTERID://回显ID，不带换行符等任何其他的东西
       
       HalUARTWrite(0,&pkt->cmd.Data[0],39);
     break;
     
     //接收到测试节点的BATTERT请求
    case SAMPLEAPP_TEST_BATTERY_CLUSTERID:
        osal_start_timerEx( SampleApp_TaskID, SAMLPEAPP_TESTBATTERY_MSG_EVT,
        ((osal_rand() & 0x00FF)) );   
    break;
    
     
  }
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage  发送数据函数
 *
 * @brief   Send the periodic message.
            发送周期信息给协调器，协调器可以知道那个设备加入网络，那个设备离开网络
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPeriodicMessage( void )//周期函数这里用作TEST
{  uint8 testid[6]={116,101,115,116,105,100};//testid
  if ( devStartMode == MODE_HARD )//如果启动的时候是协调器，则用广播
  {
    SampleApp_SPI_SendData_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
    SampleApp_SPI_SendData_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
   SampleApp_SPI_SendData_DstAddr.addr.shortAddr = 0xFFFF;  
  }
  else//其它设备类型则用单播
  {
    
  }
  
  if ( AF_DataRequest   ( &SampleApp_SPI_SendData_DstAddr,
                          (endPointDesc_t *)&SampleApp_epDesc,
                          SAMPLEAPP_TEST_ID_CLUSTERID,
                          6, 
                          &testid[0],
                          &SampleApp_TransID, 
                          0, 
                          AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
     {
    
     }
     else
     {    
     }
}


/*********************************************************************
 * @fn      SampleApp_SendFlashMessage
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */
void SampleApp_SendFlashMessage( uint16 flashTime )
{
  uint8 buffer[3];
  buffer[0] = (uint8)(SampleAppFlashCounter++);
  buffer[1] = LO_UINT16( flashTime );
  buffer[2] = HI_UINT16( flashTime );

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
*********************************************************************/
uint8 rbuf_Free_Flag;
void SampleApp_UART_EVT(void)
{
    rbuf_Free_Flag=0;
       
    if(rbuf_Free_Flag==0)
    {
      SampleApp_SPI_SendData( rbuf, buflen); 
    }
    if(rbuf_Free_Flag==0)
    {
     CompforFUWEI(rbuf);

    } 
       
}
/*********************************************************************
 * @fn      SampleApp_SPI
 *
 * @brief   Get shortaddr and sent to UART
            这个函数主要是设备启动时显示自己的短地址
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SPI_short(void)
{ uint16 short_ddr;
  uint8 str_1[ ]="My short Address is:";
  HalUARTWrite(SERIAL_APP_PORT,&str_1[0],20);
  short_ddr=NLME_GetShortAddr();
  SampleApp_showhex(short_ddr);
}

void SampleApp_SPI_mac(void)
{ uint8 macaddr[19];
  uint8 *pointer2;
  
  uint8 str_1[ ]="My MAC Address is:";
  HalUARTWrite(SERIAL_APP_PORT,&str_1[0],18);
  macaddr[0]=48;
  macaddr[1]=120;
  macaddr[2]=hextoword1(aExtendedAddress[7]);
  macaddr[3]=hextoword2(aExtendedAddress[7]);
  macaddr[4]=hextoword1(aExtendedAddress[6]);
  macaddr[5]=hextoword2(aExtendedAddress[6]);
  macaddr[6]=hextoword1(aExtendedAddress[5]);
  macaddr[7]=hextoword2(aExtendedAddress[5]);
  macaddr[8]=hextoword1(aExtendedAddress[4]);
  macaddr[9]=hextoword2(aExtendedAddress[4]);
  macaddr[10]=hextoword1(aExtendedAddress[3]);
  macaddr[11]=hextoword2(aExtendedAddress[3]);
  macaddr[12]=hextoword1(aExtendedAddress[2]);
  macaddr[13]=hextoword2(aExtendedAddress[2]);
  macaddr[14]=hextoword1(aExtendedAddress[1]);
  macaddr[15]=hextoword2(aExtendedAddress[1]);
  macaddr[16]=hextoword1(aExtendedAddress[0]);
  macaddr[17]=hextoword2(aExtendedAddress[0]);
  macaddr[18]='\n';
  pointer2=&macaddr[0];
  HalUARTWrite(SERIAL_APP_PORT,pointer2,19);
  osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        60000 );

}
void SampleApp_showhex(uint16 sixteenword)
{  uint8 short_ddr_H;
  uint8 short_ddr_L;
  uint8 shortaddr[7];
  uint8 *pointer1;
  short_ddr_H=(uint8)((sixteenword&0xff00)>>8);
  short_ddr_L=(uint8)sixteenword;
  shortaddr[0]=48;
  shortaddr[1]=120;
  shortaddr[2]=hextoword1(short_ddr_H);
  shortaddr[3]=hextoword2(short_ddr_H);
  shortaddr[4]=hextoword1(short_ddr_L);
  shortaddr[5]=hextoword2(short_ddr_L);
  shortaddr[6]='\n';
  pointer1=&shortaddr[0];
  HalUARTWrite(SERIAL_APP_PORT,pointer1,7);
  

}
/*********************************************************************
 * @fn      hextoword
 *
 * @brief   十六进制转字符函数 (SampleApp.c)  这两个函数由网友提供.
 *
 * @param   none
 *
 * @return  none
 */
uint8 hextoword1(uint8 t )
{
  uint8 abc;
  uint8 cba;
  uint8 xx1;
  abc=t;
  cba=0xf0;
  abc=(abc&cba)>>4;
  if(abc<10)
  {
    xx1=abc+48;
  }
  else
  {
    xx1=abc+55;
  }
  return xx1;
}
uint8 hextoword2(uint8 t)
{
  uint8 abc;
  uint8 cba;
  uint8 xx2;
  abc=t;
  cba=0x0f;
  abc=abc&cba;
  if(abc<10)
  {
    xx2=abc+48;
  }
  else
  {
    xx2=abc+55;
  }
  return xx2;
}

void SampleApp_Send_TestID_Message(void)//将本节点地址回复给协调器
{    int i;
    uint16 short_add;
    uint32 Jaoyanma=0;
    uint8  Jaoyanma1=0;
    uint8 pshort_add[39];
    uint8 pshort_ddr_H;
    uint8 pshort_ddr_L;
    short_add=NLME_GetShortAddr();
    
    pshort_ddr_H=(uint8)((short_add&0xff00)>>8);
    pshort_ddr_L=(uint8)short_add;
     
    pshort_add[0]=51;//3
    pshort_add[1]=48;//0
    pshort_add[2]=73;//I
    pshort_add[3]=68;//D
    pshort_add[4]=58;//:
    pshort_add[5]=48;//0
    pshort_add[6]=120;//x
    pshort_add[7]=hextoword1(pshort_ddr_H);
    pshort_add[8]=hextoword2(pshort_ddr_H);
    pshort_add[9]=hextoword1(pshort_ddr_L);
    pshort_add[10]=hextoword2(pshort_ddr_L);
    pshort_add[11]=32;
    pshort_add[12]=77;//M
    pshort_add[13]=65;//A
    pshort_add[14]=67;//C
    pshort_add[15]=58;//:
    pshort_add[16]=48;
    pshort_add[17]=120;
    pshort_add[18]=hextoword1(aExtendedAddress[7]);
    pshort_add[19]=hextoword2(aExtendedAddress[7]);
    pshort_add[20]=hextoword1(aExtendedAddress[6]);
    pshort_add[21]=hextoword2(aExtendedAddress[6]);
    pshort_add[22]=hextoword1(aExtendedAddress[5]);
    pshort_add[23]=hextoword2(aExtendedAddress[5]);
    pshort_add[24]=hextoword1(aExtendedAddress[4]);
    pshort_add[25]=hextoword2(aExtendedAddress[4]);
    pshort_add[26]=hextoword1(aExtendedAddress[3]);
    pshort_add[27]=hextoword2(aExtendedAddress[3]);
    pshort_add[28]=hextoword1(aExtendedAddress[2]);
    pshort_add[29]=hextoword2(aExtendedAddress[2]);
    pshort_add[30]=hextoword1(aExtendedAddress[1]);
    pshort_add[31]=hextoword2(aExtendedAddress[1]);
    pshort_add[32]=hextoword1(aExtendedAddress[0]);
    pshort_add[33]=hextoword2(aExtendedAddress[0]);
    for (i=7;i>=0;i--)
    {Jaoyanma+=aExtendedAddress[i];
    }
    for (i=2;i<7;i++)
    {Jaoyanma+=pshort_add[i];
    }
    Jaoyanma+=pshort_ddr_H;
    Jaoyanma+=pshort_ddr_L;
    for (i=12;i<18;i++)
    {Jaoyanma+=pshort_add[i];
    }
    Jaoyanma1=(uint8)(Jaoyanma&0x000000ff);  
    pshort_add[34]=hextoword1(Jaoyanma1);
    pshort_add[35]=hextoword2(Jaoyanma1);  
    pshort_add[36]=49;//1
    pshort_add[37]=54;//6
    pshort_add[38]='\n'; 
  
  SampleApp_TestID_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_TestID_DstAddr.endPoint =SAMPLEAPP_ENDPOINT;
  SampleApp_TestID_DstAddr.addr.shortAddr =0x0000; 
  
  if(AF_DataRequest(&SampleApp_TestID_DstAddr,
                    (endPointDesc_t *)&SampleApp_epDesc,
                    SAMPLEAPP_TEST_ID_BACK_CLUSTERID,
                    39, 
                    &pshort_add[0],
                    &SampleApp_TransID, 
                    0, 
                    AF_DEFAULT_RADIUS)==afStatus_SUCCESS)
  {
  }
  else
  {
  }
}
/*********************************************************************
 * @fn      SampleApp_SPI_SendData
 *
 * @brief   sent the uart buffer data to the air
 *详细说明：函数的功能是如果uart的buf中有数据，则通过这个函数讲数据发送出去（通过天线发送到空中）

            1、如果这个设备启动的时候为协调器，则以广播的形式发送到所有其他设备

            2、如果这个设备不是协调器，即路由器，则以单播的形式发送到协调器

  注意：用完之后要释放内存

 * @param   *buf   the dat
            len    the length of the dat 
 *
 * @return  none
 */
void SampleApp_SPI_SendData( uint8 *buf, uint8 len )
{
  uint8 i; 
  uint8 *dat=buf;
  uint16 newadress=0,temp=0;
  

  for ( i = 0; i <9; i++ )//这几个空过去是因为有设备ID。
  {
   dat++ ;
  }
    dat++;
    dat++;  
  for ( i =0; i <4; i++ )
  { 
   if(*dat>=48&&*dat<=57)
     {
      temp=(*dat)-48;    
     }
   else if(*dat>=97&&*dat<=102)
     {
      temp=(*dat)-87;
     }
    else if(*dat>=65&&*dat<=70)
     {
      temp=(*dat)-55;
     }
   else
     return;
   
      *dat++;
     temp=temp<<((3-i)*4);
     newadress|=temp;
   } 
   if ( newadress == 0xFFFF )
  {
    SampleApp_SPI_SendData_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
    SampleApp_SPI_SendData_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_SPI_SendData_DstAddr.addr.shortAddr = 0xFFFF; 
  }
  else
    
  {
    SampleApp_SPI_SendData_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    SampleApp_SPI_SendData_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_SPI_SendData_DstAddr.addr.shortAddr = newadress; 
  }
    if ( AF_DataRequest   ( &SampleApp_SPI_SendData_DstAddr,
                         (endPointDesc_t *)&SampleApp_epDesc,
                          SAMPLEAPP_FLASH_CLUSTERID,
                          len, buf,
                          &SampleApp_TransID, 
                          0, 
                          AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    osal_mem_free( rbuf );  //必须释放内存,不然造成溢出!
  }
  else
  {
    osal_mem_free( rbuf ); //必须释放内存,不然造成溢出!    
  }  
}


void CompforFUWEI(uint8 *dat)
{
  uint8 predict[5]={102,117,119,101,105};//fuwei
  uint8 *ppredict=&predict[0];
  uint8 i; 
  uint16 newadress=0,temp=0;
  uint8 pshort_add[5]={70,85,87,69,73};//FUWEI
  if(rxlen!=9)//比较从上位机来的数据长度！！
  return;
  
  for ( i = 0; i <9; i++ )//这几个空过去是因为有设备ID。
  {
   *dat++ ;
  }
  
  for ( i = 0; i <5; i++ )
  {
   if(*dat++ == *ppredict++);
   else
       return;
  }  
  for ( i =0; i <4; i++ )
  { 
   if(*dat>=48&&*dat<=57)
     {
      temp=(*dat)-48;    
     }
   else if(*dat>=97&&*dat<=102)
     {
      temp=(*dat)-87;
     }
    else if(*dat>=65&&*dat<=70)
     {
      temp=(*dat)-55;
     }
   else
     return;
   
      *dat++;
     temp=temp<<((3-i)*4);
     newadress|=temp;
   } 
   
    SampleApp_SPI_SendData_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    SampleApp_SPI_SendData_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    SampleApp_SPI_SendData_DstAddr.addr.shortAddr = newadress; 
  
    if ( AF_DataRequest   (&SampleApp_SPI_SendData_DstAddr,
                          (endPointDesc_t *)&SampleApp_epDesc,
                          SAMPLEAPP_TEST_BATTERY_CLUSTERID,
                          2, 
                          &pshort_add[0],
                          &SampleApp_TransID, 
                          0, 
                          AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
     {
    
     }
     else
     {    
     } 
    rbuf_Free_Flag=1;
}