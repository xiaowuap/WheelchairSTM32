#include "usbh_hid_GamePad.h"
#include "usbh_hid.h"

#include "FreeRTOS.h"
#include "timers.h"

//�ֱ�֧�ְ�
#include "bsp_gamepad.h"

//xbox360�ֱ�
#include "xbox360_gamepad.h"

//���� USB PS2�ֱ�
#include "WiredPS2_gamepad.h"

//��Ϸ�ֱ�����������,��PID��VID��ͬʱ,��������������ֲ�ͬƷ��
char GamePad_Manufacturer[100] = { 0 };
char GamePad_iSerialNum[100] = { 0 } ;

//GamePad_HID����غ�������
static USBH_StatusTypeDef USBH_HID_InterfaceInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_InterfaceDeInit(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_ClassRequest(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_Process(USBH_HandleTypeDef *phost);
static USBH_StatusTypeDef USBH_HID_SOFProcess(USBH_HandleTypeDef *phost);
static void USBH_HID_ParseHIDDesc(HID_DescTypeDef *desc, uint8_t *buf);

//usb�ֱ��ĳ�ʼ������
extern USBH_StatusTypeDef USBH_HID_GamePadInit(USBH_HandleTypeDef *phost);

//������Ҫ��ʼ�����ֱ�����
void SwitchPro_GamePad_Init(USBH_HandleTypeDef *phost);

//����ps2��hid��
USBH_ClassTypeDef  GamePad_HID_Class =
{
  .Name = "HID",
  .ClassCode = USB_HID_CLASS, //Ĭ��ΪHID�豸
  .Init = USBH_HID_InterfaceInit,
  .DeInit = USBH_HID_InterfaceDeInit,
  .Requests = USBH_HID_ClassRequest,
  .BgndProcess = USBH_HID_Process,
  .SOFProcess = USBH_HID_SOFProcess,
  .pData = NULL,
};

//�Ǳ�׼HID
USBH_ClassTypeDef  GamePad_NonStdHID_Class =
{
  .Name = "HID",
  .ClassCode = 0xff, //��Щ�ֱ����ಢ���Ǳ�׼��HID�豸,����0xff
  .Init = USBH_HID_InterfaceInit,
  .DeInit = USBH_HID_InterfaceDeInit,
  .Requests = USBH_HID_ClassRequest,
  .BgndProcess = USBH_HID_Process,
  .SOFProcess = USBH_HID_SOFProcess,
  .pData = NULL,
};


/**
  * @brief  USBH_HID_InterfaceInit
  *         The function init the HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_InterfaceInit(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef status;
  HID_HandleTypeDef *HID_Handle;
  uint16_t ep_mps;
  uint8_t max_ep;
  uint8_t num = 0U;
  uint8_t interface;

	USBH_UsrLog("start find interface now");
	
	//�ӿ�ƥ��,��Ҫ��������������ӿڽ���ƥ��
	//phost->device.CfgDesc.Itf_Desc[0,1,2,3...,n...].bInterfaceSubClass

	//0xff��ʾƥ����������
	interface = USBH_FindInterface(phost, 0xFFU, 0xFFU, 0xFFU);
	
	if ((interface == 0xFFU) || (interface >= USBH_MAX_NUM_INTERFACES)) /* No Valid Interface */
	{
		USBH_DbgLog("Cannot Find the interface for %s class.", phost->pActiveClass->Name);
		return USBH_FAIL;
	}

  status = USBH_SelectInterface(phost, interface);

  if (status != USBH_OK)
  {
    return USBH_FAIL;
  }

  phost->pActiveClass->pData = (HID_HandleTypeDef *)USBH_malloc(sizeof(HID_HandleTypeDef));
  HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  if (HID_Handle == NULL)
  {
    USBH_DbgLog("Cannot allocate memory for HID Handle");
    return USBH_FAIL;
  }

  /* Initialize hid handler */
  (void)USBH_memset(HID_Handle, 0, sizeof(HID_HandleTypeDef));

  HID_Handle->state = USBH_HID_ERROR;

  //����HID�豸�� PID��VID���в�ͬ���豸����ʶ��
  if( phost->device.DevDesc.idProduct == Wired_PS2_PID && phost->device.DevDesc.idVendor==Wired_PS2_VID )
  {	  //����PS2�ֱ���PID/VID
	  USBH_UsrLog("Wired PS2 device found!");
	  gamepad_brand = PS2_USB_Wired;
	  GamePadInterface = &Wired_USB_PS2Gamepad;
	  HID_Handle->Init = USBH_HID_GamePadInit;
  }
  else if( phost->device.DevDesc.idProduct == WiredV2_PS2_PID && phost->device.DevDesc.idVendor==WiredV2_PS2_VID )
  {
	  //�ڶ���usb����
	  USBH_UsrLog("Wired PS2 V2 device found!");
	  gamepad_brand = PS2_USB_WiredV2;
	  GamePadInterface = &Wired_USB_PS2Gamepad;
	  HID_Handle->Init = USBH_HID_GamePadInit;
  }
  
//  else if( phost->device.DevDesc.idProduct == Wireless_PC_PS2_PID && phost->device.DevDesc.idVendor==Wireless_PC_PS2_VID )
//  {	  //����PS2�ֱ� PCģʽ PID/VID
//	  USBH_UsrLog("Wireless PC PS2 device found!");
//	  gamepad_brand = PS2_Wiredless_PC;
//	  HID_Handle->Init = USBH_HID_GamePadInit;
//  }
  
  else if( phost->device.DevDesc.idProduct == Xbox360_GamePad_PID && phost->device.DevDesc.idVendor==Xbox360_GamePad_VID )
  {	
		//����������������ͬ����PID��VID��Ʒ
		if( strcmp(GamePad_Manufacturer,"Flydigi") == 0 )
		{
			gamepad_brand = Xbox360;
		}
		else
		{
			gamepad_brand = PS2_USB_Wiredless;
		}
		//XBOX360ƽ̨
		USBH_UsrLog("XBox360 device found!");
		GamePadInterface = &Xbox360Gamepad;
		HID_Handle->Init = USBH_HID_GamePadInit;
  }
  
  else
  {
	gamepad_brand = UnKnown_Dev;//δ֪���豸����
	USBH_UsrLog("Protocol not supported.PID:%X\tVID:%X\r\n",phost->device.DevDesc.idProduct,phost->device.DevDesc.idVendor);
    return USBH_FAIL;
  }

  HID_Handle->state     = USBH_HID_INIT;
  HID_Handle->ctl_state = USBH_HID_REQ_INIT;
  HID_Handle->ep_addr   = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bEndpointAddress;
  HID_Handle->length    = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].wMaxPacketSize;
  HID_Handle->poll      = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[0].bInterval;
  
  if (HID_Handle->poll < HID_MIN_POLL)
  {
    HID_Handle->poll = HID_MIN_POLL;
  }

  /* Check of available number of endpoints */
  /* Find the number of EPs in the Interface Descriptor */
  /* Choose the lower number in order not to overrun the buffer allocated */
  max_ep = ((phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints <= USBH_MAX_NUM_ENDPOINTS) ?
            phost->device.CfgDesc.Itf_Desc[interface].bNumEndpoints : USBH_MAX_NUM_ENDPOINTS);


  /* Decode endpoint IN and OUT address from interface descriptor */
  for (num = 0U; num < max_ep; num++)
  {
//	USBH_UsrLog("Found endpoint %d: Address = 0x%02X, Type = 0x%02X", num,
//				phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress,
//				phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bmAttributes);
	  
    if ((phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress & 0x80U) != 0U)
    {
      HID_Handle->InEp = (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress);
      HID_Handle->InPipe = USBH_AllocPipe(phost, HID_Handle->InEp);
      ep_mps = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].wMaxPacketSize;

      /* Open pipe for IN endpoint */
      (void)USBH_OpenPipe(phost, HID_Handle->InPipe, HID_Handle->InEp, phost->device.address,
                          phost->device.speed, USB_EP_TYPE_INTR, ep_mps);

      (void)USBH_LL_SetToggle(phost, HID_Handle->InPipe, 0U);
    }
    else
    {
      HID_Handle->OutEp = (phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].bEndpointAddress);
      HID_Handle->OutPipe = USBH_AllocPipe(phost, HID_Handle->OutEp);
      ep_mps = phost->device.CfgDesc.Itf_Desc[interface].Ep_Desc[num].wMaxPacketSize;

      /* Open pipe for OUT endpoint */
      (void)USBH_OpenPipe(phost, HID_Handle->OutPipe, HID_Handle->OutEp, phost->device.address,
                          phost->device.speed, USB_EP_TYPE_INTR, ep_mps);

      (void)USBH_LL_SetToggle(phost, HID_Handle->OutPipe, 0U);
    }
  }
  
  return USBH_OK;
}

/**
  * @brief  USBH_HID_InterfaceDeInit
  *         The function DeInit the Pipes used for the HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_InterfaceDeInit(USBH_HandleTypeDef *phost)
{
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  if (HID_Handle->InPipe != 0x00U)
  {
    (void)USBH_ClosePipe(phost, HID_Handle->InPipe);
    (void)USBH_FreePipe(phost, HID_Handle->InPipe);
    HID_Handle->InPipe = 0U;     /* Reset the pipe as Free */
  }

  if (HID_Handle->OutPipe != 0x00U)
  {
    (void)USBH_ClosePipe(phost, HID_Handle->OutPipe);
    (void)USBH_FreePipe(phost, HID_Handle->OutPipe);
    HID_Handle->OutPipe = 0U;     /* Reset the pipe as Free */
  }

  if ((phost->pActiveClass->pData) != NULL)
  {
    USBH_free(phost->pActiveClass->pData);
    phost->pActiveClass->pData = 0U;
  }

  //�豸�γ�ʱ,�����з���ʼ��
  USB_GamePad_PullOutCallback();
  
  return USBH_OK;
}

/**
  * @brief  USBH_HID_ClassRequest
  *         The function is responsible for handling Standard requests
  *         for HID class.
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_ClassRequest(USBH_HandleTypeDef *phost)
{

  USBH_StatusTypeDef status         = USBH_BUSY;
  USBH_StatusTypeDef classReqStatus = USBH_BUSY;
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  /* Switch HID state machine */
  switch (HID_Handle->ctl_state)
  {
    case USBH_HID_REQ_INIT:
    case USBH_HID_REQ_GET_HID_DESC:

      USBH_HID_ParseHIDDesc(&HID_Handle->HID_Desc, phost->device.CfgDesc_Raw);

      HID_Handle->ctl_state = USBH_HID_REQ_GET_REPORT_DESC;

      break;
    case USBH_HID_REQ_GET_REPORT_DESC:

      /* Get Report Desc */
      classReqStatus = USBH_HID_GetHIDReportDescriptor(phost, HID_Handle->HID_Desc.wItemLength);
      if (classReqStatus == USBH_OK)
      {
        /* The descriptor is available in phost->device.Data */
        HID_Handle->ctl_state = USBH_HID_REQ_SET_IDLE;
      }
      else if (classReqStatus == USBH_NOT_SUPPORTED)
      {
        USBH_ErrLog("Control error: HID: Device Get Report Descriptor request failed");
        status = USBH_OK;
      }
      else
      {
        /* .. */
      }

      break;

    case USBH_HID_REQ_SET_IDLE:

      classReqStatus = USBH_HID_SetIdle(phost, 0U, 0U);

      /* set Idle */
      if (classReqStatus == USBH_OK)
      {
        HID_Handle->ctl_state = USBH_HID_REQ_SET_PROTOCOL;
      }
      else
      {
        if (classReqStatus == USBH_NOT_SUPPORTED)
        {
          HID_Handle->ctl_state = USBH_HID_REQ_SET_PROTOCOL;
        }
      }
      break;

    case USBH_HID_REQ_SET_PROTOCOL:
      /* set protocol */
      classReqStatus = USBH_HID_SetProtocol(phost, 0U);
      if (classReqStatus == USBH_OK)
      {
        HID_Handle->ctl_state = USBH_HID_REQ_IDLE;

        /* all requests performed */
        phost->pUser(phost, HOST_USER_CLASS_ACTIVE);
        status = USBH_OK;
      }
      else if (classReqStatus == USBH_NOT_SUPPORTED)
      {
		  USBH_ErrLog("TODO:Cannel:-->Control error: HID: Device Set protocol request failed");
        status = USBH_OK;
      }
      else
      {
        /* .. */
      }
      break;

    case USBH_HID_REQ_IDLE:
    default:
      break;
  }

  return status;
}

/**
  * @brief  USBH_HID_Process
  *         The function is for managing state machine for HID data transfers
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_Process(USBH_HandleTypeDef *phost)
{
  USBH_StatusTypeDef status = USBH_OK;
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
  uint32_t XferSize;

  switch (HID_Handle->state)
  {
    case USBH_HID_INIT:
      status = HID_Handle->Init(phost);

      if (status == USBH_OK)
      {
        HID_Handle->state = USBH_HID_IDLE;
      }
      else
      {
        USBH_ErrLog("HID Class Init failed");
        HID_Handle->state = USBH_HID_ERROR;
        status = USBH_FAIL;
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
      break;

    case USBH_HID_IDLE:
		
	//����GetReport��������HID�豸��������.�ڴ�֮ǰ HID_Handle->pData ������ɳ�ʼ��,��һ��ͨ���� HID_Handle->Init �������,�����޷�����ɹ�
	
      status = USBH_HID_GetReport(phost, 0x01U, 0U, HID_Handle->pData, (uint8_t)HID_Handle->length);

      if (status == USBH_OK)
      {
        HID_Handle->state = USBH_HID_SYNC;
      }
      else if (status == USBH_BUSY)
      {
        HID_Handle->state = USBH_HID_IDLE;
        status = USBH_OK;
      }
      else if (status == USBH_NOT_SUPPORTED)
      {
        HID_Handle->state = USBH_HID_SYNC;
        status = USBH_OK;
      }
      else
      {
        HID_Handle->state = USBH_HID_ERROR;
        status = USBH_FAIL;
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
      break;

    case USBH_HID_SYNC:
      /* Sync with start of Even Frame */
      if ((phost->Timer & 1U) != 0U)
      {
        HID_Handle->state = USBH_HID_GET_DATA;
      }

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
      break;

    case USBH_HID_GET_DATA:
		
	//����HID�豸������,������ HID_Handle->pData
      (void)USBH_InterruptReceiveData(phost, HID_Handle->pData,
                                      (uint8_t)HID_Handle->length,
                                      HID_Handle->InPipe);

      HID_Handle->state = USBH_HID_POLL;
      HID_Handle->timer = phost->Timer;
      HID_Handle->DataReady = 0U;
      break;

    case USBH_HID_POLL:
      if (USBH_LL_GetURBState(phost, HID_Handle->InPipe) == USBH_URB_DONE)
      {
        XferSize = USBH_LL_GetLastXferSize(phost, HID_Handle->InPipe);

        if ((HID_Handle->DataReady == 0U) && (XferSize != 0U) && (HID_Handle->fifo.buf != NULL))
        {
		  //��������HID_Handle->pData��HID�豸����д�뵽fifo�У�����ͨ��fifo����ȡhid�豸������
          (void)USBH_HID_FifoWrite(&HID_Handle->fifo, HID_Handle->pData, HID_Handle->length); 
          HID_Handle->DataReady = 1U;
          USBH_HID_EventCallback(phost);
	  
#if (USBH_USE_OS == 1U)
          phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
          (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
          (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
        }
      }
      else
      {
        /* IN Endpoint Stalled */
        if (USBH_LL_GetURBState(phost, HID_Handle->InPipe) == USBH_URB_STALL)
        {
          /* Issue Clear Feature on interrupt IN endpoint */
          if (USBH_ClrFeature(phost, HID_Handle->ep_addr) == USBH_OK)
          {
            /* Change state to issue next IN token */
            HID_Handle->state = USBH_HID_GET_DATA;
          }
        }
      }
      break;

    default:
      break;
  }

  return status;
}

/**
  * @brief  USBH_HID_SOFProcess
  *         The function is for managing the SOF Process
  * @param  phost: Host handle
  * @retval USBH Status
  */
static USBH_StatusTypeDef USBH_HID_SOFProcess(USBH_HandleTypeDef *phost)
{
  HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;

  if (HID_Handle->state == USBH_HID_POLL)
  {
    if ((phost->Timer - HID_Handle->timer) >= HID_Handle->poll)
    {
      HID_Handle->state = USBH_HID_GET_DATA;

#if (USBH_USE_OS == 1U)
      phost->os_msg = (uint32_t)USBH_URB_EVENT;
#if (osCMSIS < 0x20000U)
      (void)osMessagePut(phost->os_event, phost->os_msg, 0U);
#else
      (void)osMessageQueuePut(phost->os_event, &phost->os_msg, 0U, 0U);
#endif
#endif
    }
  }
  return USBH_OK;
}

/**
  * @brief  USBH_ParseHIDDesc
  *         This function Parse the HID descriptor
  * @param  desc: HID Descriptor
  * @param  buf: Buffer where the source descriptor is available
  * @retval None
  */
static void USBH_HID_ParseHIDDesc(HID_DescTypeDef *desc, uint8_t *buf)
{
  USBH_DescHeader_t *pdesc = (USBH_DescHeader_t *)buf;
  uint16_t CfgDescLen;
  uint16_t ptr;

  CfgDescLen = LE16(buf + 2U);

  if (CfgDescLen > USB_CONFIGURATION_DESC_SIZE)
  {
    ptr = USB_LEN_CFG_DESC;

    while (ptr < CfgDescLen)
    {
      pdesc = USBH_GetNextDesc((uint8_t *)pdesc, &ptr);

      if (pdesc->bDescriptorType == USB_DESC_TYPE_HID)
      {
        desc->bLength = *(uint8_t *)((uint8_t *)pdesc + 0U);
        desc->bDescriptorType = *(uint8_t *)((uint8_t *)pdesc + 1U);
        desc->bcdHID = LE16((uint8_t *)pdesc + 2U);
        desc->bCountryCode = *(uint8_t *)((uint8_t *)pdesc + 4U);
        desc->bNumDescriptors = *(uint8_t *)((uint8_t *)pdesc + 5U);
        desc->bReportDescriptorType = *(uint8_t *)((uint8_t *)pdesc + 6U);
        desc->wItemLength = LE16((uint8_t *)pdesc + 7U);
        break;
      }
    }
  }
}

///////////////////////// ��ʼ�������ݽ��� ///////////////////////////////
//�ֱ�ͨ�����뱨��
static uint8_t GamePadReportData[128] = { 0 }; //���ڴ��HID�豸���͹���������

//ps2��ʼ������
USBH_StatusTypeDef USBH_HID_GamePadInit(USBH_HandleTypeDef *phost)
{
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	
	//HID_Handle->length�ڳ�ʼ���豸ʱ��HID�豸ȷ��
	if (HID_Handle->length > sizeof(GamePadReportData))
	{
		HID_Handle->length = (uint16_t)sizeof(GamePadReportData);
	}
	
	//��ʼ��pData,���벽��.
	HID_Handle->pData = GamePadReportData;
	
	if ((HID_QUEUE_SIZE * sizeof(GamePadReportData)) > sizeof(phost->device.Data))
	{	//sizeof(phost->device.Data) ��С�� USBH_MAX_DATA_BUFFER ����
		return USBH_FAIL;
	}
	else
	{
		//��ʼ��fifo
		USBH_HID_FifoInit(&HID_Handle->fifo, phost->device.Data, (uint16_t)(HID_QUEUE_SIZE * sizeof(GamePadReportData)));
	}
	
	//USB����
	USB_GamePad_InsertCallback();
	
	//switch pro�ֱ�,��Ҫ�����ʼ��
	if( gamepad_brand == SwitchPro )
	{
		SwitchPro_GamePad_Init(phost);
	}
	
	return USBH_OK;
}

//ps2���ݽ���,��ڲ���ΪHID�豸.���л���Ϊ����.
USBH_StatusTypeDef USBH_HID_PS2_Decode(USBH_HandleTypeDef *phost)
{
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	
	//����Ƿ�ɹ�ʶ���ֱ�����
	if( gamepad_brand == UnKnown_Dev ) return USBH_FAIL;
	
	//���hid�豸�Ƿ�������
	if ((HID_Handle->length == 0U) || (HID_Handle->fifo.buf == NULL))
	{
		return USBH_FAIL;
	}
	
	//��fifo�ж�ȡhid���ݷ���GamePadReportData�������н���
	if (USBH_HID_FifoRead(&HID_Handle->fifo, (uint8_t* )GamePadReportData, HID_Handle->length) == HID_Handle->length)
	{
		/* ����ʵ�ʽ���Ĳ�ͬ��Ϸ�ֱ�,���ò�ͬ�Ĵ����� */
		GamePadDebug.ready = 1;//�ֱ�����,��ʾ�����ݽ���
		
		//xbox360��ʽ����
		//LX  GamePadReportData[7]  ���128-0,�Ҷ�0-127
		//LY GamePadReportData[9]  �¶�128-0,�϶�0-127 
		//RX GamePadReportData[11]  ���128-0,�Ҷ�0-127
		//RY GamePadReportData[13]  �¶�128-0,�϶�0-127 
		//LT GamePadReportData[4] 0-255
		//RT GamePadReportData[5] 0-255
		// GamePadReportData[3] 1:LB 2:RB 3:logo 4:? 5:A 6:B 7:X 8:Y
		// GamePadReportData[2] 1234:�������� 5:start 6:seltec 7:Lrock 8:Rrock

		if( gamepad_brand == Xbox360 || gamepad_brand == PS2_USB_Wiredless )
		{
			Xbox360_gamepad_Decode(phost,GamePadReportData,HID_Handle->length);
		}
		else if( gamepad_brand == PS2_USB_Wired )
		{
			Wired_USB_PS2gamepad_Decode(phost,GamePadReportData,HID_Handle->length);
		}
		else if( gamepad_brand == PS2_USB_WiredV2 )
		{
			Wired_USB_V2_PS2gamepad_Decode(phost,GamePadReportData,HID_Handle->length);
		}
		
		return USBH_OK;
	}

	return   USBH_FAIL;

}

//SwitchPro�ֱ���ʼ��
void SwitchPro_GamePad_Init(USBH_HandleTypeDef *phost)
{
	#define DelayTime 100
	HID_HandleTypeDef *HID_Handle = (HID_HandleTypeDef *) phost->pActiveClass->pData;
	
	USBH_UsrLog("switch pro init now...");
	
	/* ����������Դ�����򹤳�ץ��,������Ậ�� */
	
	uint8_t initcode[64] = { 0 };
	initcode[0] = 0x80; initcode[1] = 0x01;
	while( USBH_OK!= USBH_InterruptSendData(phost,initcode,64,HID_Handle->OutPipe) ) ; 
	USBH_Delay(DelayTime);
	
	initcode[1] = 0x02;
	while( USBH_OK!= USBH_InterruptSendData(phost,initcode,64,HID_Handle->OutPipe) ) ; 
	USBH_Delay(DelayTime);
	
	initcode[1] = 0x03;
	while( USBH_OK!= USBH_InterruptSendData(phost,initcode,64,HID_Handle->OutPipe) ) ; 
	USBH_Delay(DelayTime);
	
	initcode[1] = 0x02;
	while( USBH_OK!= USBH_InterruptSendData(phost,initcode,64,HID_Handle->OutPipe) ) ; 
	USBH_Delay(DelayTime);
	
	initcode[1] = 0x04;
	while( USBH_OK!= USBH_InterruptSendData(phost,initcode,64,HID_Handle->OutPipe) ) ;
	USBH_Delay(DelayTime);

	USBH_UsrLog("switch pro init ok!");
	
	/* д��������Ϣ��,switch pro�ֱ��Żἤ��������� */
}

