/* USB CDC-ACM "virtual COM port" for the FRDM-MCXA153's second (target)
 * USB connector, wired to the MCX A153's own USB0 (KHCI) controller --
 * separate from the MCU-Link debug USB port.
 *
 * Behavior: echoes back whatever bytes the host sends, so opening the
 * enumerated /dev/ttyACMx (or COMx) port and typing something proves the
 * link works end-to-end. Adapted from NXP's official
 * usb_device_cdc_vcom "bm" (bare-metal) example for this exact board,
 * with the debug-console/board/clock_config scaffolding stripped out and
 * replaced by the RTT logger already used by the blinky demo.
 */
#include <string.h>
#include "fsl_device_registers.h"
#include "rtt.h"

#include "usb_device_config.h"
#include "usb.h"
#include "usb_device.h"

#include "usb_device_class.h"
#include "usb_device_cdc_acm.h"
#include "usb_device_ch9.h"

#include "usb_device_descriptor.h"
#include "virtual_com.h"

/* Heartbeat LED (same red channel used by the blinky demo, GPIO3 pin 12,
 * active-low) so it's obvious the firmware is alive even with nothing
 * plugged into the USB port yet. Driven from SysTick so it keeps blinking
 * at a steady rate regardless of how busy the USB polling loop is. */
#define LED_RED_PIN 12U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void USB_DeviceClockInit(void);
void USB_DeviceIsrEnable(void);
usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param);
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param);

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern usb_device_endpoint_struct_t g_UsbDeviceCdcVcomDicEndpoints[];
extern usb_device_class_struct_t g_UsbDeviceCdcVcomConfig;

usb_cdc_vcom_struct_t s_cdcVcom;

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t s_lineCoding[LINE_CODING_SIZE] = {
    (LINE_CODING_DTERATE >> 0U) & 0x000000FFU,
    (LINE_CODING_DTERATE >> 8U) & 0x000000FFU,
    (LINE_CODING_DTERATE >> 16U) & 0x000000FFU,
    (LINE_CODING_DTERATE >> 24U) & 0x000000FFU,
    LINE_CODING_CHARFORMAT,
    LINE_CODING_PARITYTYPE,
    LINE_CODING_DATABITS};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t s_abstractState[COMM_FEATURE_DATA_SIZE] = {(STATUS_ABSTRACT_STATE >> 0U) & 0x00FFU,
                                                           (STATUS_ABSTRACT_STATE >> 8U) & 0x00FFU};

USB_DMA_INIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE)
static uint8_t s_countryCode[COMM_FEATURE_DATA_SIZE] = {(COUNTRY_SETTING >> 0U) & 0x00FFU,
                                                         (COUNTRY_SETTING >> 8U) & 0x00FFU};

USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static usb_cdc_acm_info_t s_usbCdcAcmInfo;
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_currRecvBuf[DATA_BUFF_SIZE];
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_currSendBuf[DATA_BUFF_SIZE];
volatile static uint32_t s_recvSize = 0;
volatile static uint32_t s_sendSize = 0;

static usb_device_class_config_struct_t s_cdcAcmConfig[1] = {{
    USB_DeviceCdcVcomCallback,
    0,
    &g_UsbDeviceCdcVcomConfig,
}};

static usb_device_class_config_list_struct_t s_cdcAcmConfigList = {
    s_cdcAcmConfig,
    USB_DeviceCallback,
    1,
};

/*******************************************************************************
 * Board / clock / IRQ glue
 ******************************************************************************/
void USB0_IRQHandler(void)
{
    USB_DeviceKhciIsrFunction(s_cdcVcom.deviceHandle);
}

void SysTick_Handler(void)
{
    GPIO3->PTOR = (1U << LED_RED_PIN);
}

static void LED_Init(void)
{
    SYSCON->CLKUNLOCK &= ~SYSCON_CLKUNLOCK_UNLOCK_MASK;
    MRCC0->MRCC_GLB_CC1_SET  = MRCC_MRCC_GLB_CC1_PORT3_MASK | MRCC_MRCC_GLB_CC1_GPIO3_MASK;
    MRCC0->MRCC_GLB_RST1_SET = MRCC_MRCC_GLB_RST1_PORT3_MASK | MRCC_MRCC_GLB_RST1_GPIO3_MASK;
    SYSCON->CLKUNLOCK |= SYSCON_CLKUNLOCK_UNLOCK_MASK;

    PORT3->PCR[LED_RED_PIN] = PORT_PCR_MUX(0U);
    GPIO3->PSOR             = (1U << LED_RED_PIN); /* off (active-low) */
    GPIO3->PDDR |= (1U << LED_RED_PIN);

    /* Toggle every ~250 ms. */
    SysTick_Config(SystemCoreClock / 4U);
}

void USB_DeviceClockInit(void)
{
    RESET_PeripheralReset(kUSB0_RST_SHIFT_RSTn);
    CLOCK_EnableUsbfsClock();
}

void USB_DeviceIsrEnable(void)
{
    uint8_t usbDeviceKhciIrq[] = USBFS_IRQS;
    uint8_t irqNumber          = usbDeviceKhciIrq[CONTROLLER_ID - kUSB_ControllerKhci0];

    NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
    EnableIRQ((IRQn_Type)irqNumber);
}

/*******************************************************************************
 * CDC ACM class callback (line coding, control line state, rx/tx events)
 ******************************************************************************/
usb_status_t USB_DeviceCdcVcomCallback(class_handle_t handle, uint32_t event, void *param)
{
    uint32_t len;
    uint8_t *uartBitmap;
    usb_device_cdc_acm_request_param_struct_t *acmReqParam;
    usb_device_endpoint_callback_message_struct_t *epCbParam;
    usb_status_t error          = kStatus_USB_InvalidRequest;
    usb_cdc_acm_info_t *acmInfo = &s_usbCdcAcmInfo;
    acmReqParam                 = (usb_device_cdc_acm_request_param_struct_t *)param;
    epCbParam                   = (usb_device_endpoint_callback_message_struct_t *)param;

    switch (event)
    {
        case kUSB_DeviceCdcEventSendResponse:
            if (epCbParam->length == USB_CANCELLED_TRANSFER_LENGTH)
            {
                error = kStatus_USB_Success;
            }
            else if ((epCbParam->length != 0) &&
                     (0U == (epCbParam->length % g_UsbDeviceCdcVcomDicEndpoints[0].maxPacketSize)))
            {
                error = USB_DeviceCdcAcmSend(handle, USB_CDC_VCOM_BULK_IN_ENDPOINT, NULL, 0);
            }
            else if ((1U == s_cdcVcom.attach) && (1U == s_cdcVcom.startTransactions))
            {
                if ((epCbParam->buffer != NULL) || ((epCbParam->buffer == NULL) && (epCbParam->length == 0)))
                {
                    error = USB_DeviceCdcAcmRecv(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
                                                  g_UsbDeviceCdcVcomDicEndpoints[1].maxPacketSize);
                }
            }
            break;
        case kUSB_DeviceCdcEventRecvResponse:
            if (epCbParam->length == USB_CANCELLED_TRANSFER_LENGTH)
            {
                error = kStatus_USB_Success;
            }
            else if ((1U == s_cdcVcom.attach) && (1U == s_cdcVcom.startTransactions))
            {
                s_recvSize = epCbParam->length;
                error      = kStatus_USB_Success;
                if (0U == s_recvSize)
                {
                    error = USB_DeviceCdcAcmRecv(handle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
                                                  g_UsbDeviceCdcVcomDicEndpoints[1].maxPacketSize);
                }
            }
            break;
        case kUSB_DeviceCdcEventSerialStateNotif:
            ((usb_device_cdc_acm_struct_t *)handle)->hasSentState = 0;
            error                                                 = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventSendEncapsulatedCommand:
        case kUSB_DeviceCdcEventGetEncapsulatedResponse:
            break;
        case kUSB_DeviceCdcEventSetCommFeature:
            if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == acmReqParam->setupValue)
            {
                if (1U == acmReqParam->isSetup)
                {
                    *(acmReqParam->buffer) = s_abstractState;
                    *(acmReqParam->length) = sizeof(s_abstractState);
                }
                error = kStatus_USB_Success;
            }
            else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == acmReqParam->setupValue)
            {
                if (1U == acmReqParam->isSetup)
                {
                    *(acmReqParam->buffer) = s_countryCode;
                    *(acmReqParam->length) = sizeof(s_countryCode);
                }
                error = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceCdcEventGetCommFeature:
            if (USB_DEVICE_CDC_FEATURE_ABSTRACT_STATE == acmReqParam->setupValue)
            {
                *(acmReqParam->buffer) = s_abstractState;
                *(acmReqParam->length) = COMM_FEATURE_DATA_SIZE;
                error                  = kStatus_USB_Success;
            }
            else if (USB_DEVICE_CDC_FEATURE_COUNTRY_SETTING == acmReqParam->setupValue)
            {
                *(acmReqParam->buffer) = s_countryCode;
                *(acmReqParam->length) = COMM_FEATURE_DATA_SIZE;
                error                  = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceCdcEventClearCommFeature:
            break;
        case kUSB_DeviceCdcEventGetLineCoding:
            *(acmReqParam->buffer) = s_lineCoding;
            *(acmReqParam->length) = LINE_CODING_SIZE;
            error                  = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventSetLineCoding:
            if (1U == acmReqParam->isSetup)
            {
                *(acmReqParam->buffer) = s_lineCoding;
                *(acmReqParam->length) = sizeof(s_lineCoding);
            }
            error = kStatus_USB_Success;
            break;
        case kUSB_DeviceCdcEventSetControlLineState:
        {
            s_usbCdcAcmInfo.dteStatus = acmReqParam->setupValue;
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_CARRIER_ACTIVATION)
            {
                acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
            }
            else
            {
                acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_TX_CARRIER;
            }
            if (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE)
            {
                acmInfo->uartState |= USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
            }
            else
            {
                acmInfo->uartState &= (uint16_t)~USB_DEVICE_CDC_UART_STATE_RX_CARRIER;
            }
            acmInfo->dtePresent = (acmInfo->dteStatus & USB_DEVICE_CDC_CONTROL_SIG_BITMAP_DTE_PRESENCE) ? true : false;

            acmInfo->serialStateBuf[0] = NOTIF_REQUEST_TYPE;
            acmInfo->serialStateBuf[1] = USB_DEVICE_CDC_NOTIF_SERIAL_STATE;
            acmInfo->serialStateBuf[2] = 0x00;
            acmInfo->serialStateBuf[3] = 0x00;
            acmInfo->serialStateBuf[4] = 0x00;
            acmInfo->serialStateBuf[5] = 0x00;
            acmInfo->serialStateBuf[6] = UART_BITMAP_SIZE;
            acmInfo->serialStateBuf[7] = 0x00;
            acmInfo->serialStateBuf[4] = acmReqParam->interfaceIndex;
            uartBitmap    = (uint8_t *)&acmInfo->serialStateBuf[NOTIF_PACKET_SIZE + UART_BITMAP_SIZE - 2];
            uartBitmap[0] = acmInfo->uartState & 0xFFu;
            uartBitmap[1] = (acmInfo->uartState >> 8) & 0xFFu;

            len = (uint32_t)(NOTIF_PACKET_SIZE + UART_BITMAP_SIZE);
            if (0U == ((usb_device_cdc_acm_struct_t *)handle)->hasSentState)
            {
                error = USB_DeviceCdcAcmSend(handle, USB_CDC_VCOM_INTERRUPT_IN_ENDPOINT, acmInfo->serialStateBuf, len);
                if (kStatus_USB_Success != error)
                {
                    rtt_puts("kUSB_DeviceCdcEventSetControlLineState error!\r\n");
                }
                ((usb_device_cdc_acm_struct_t *)handle)->hasSentState = 1;
            }

            if (1U == s_cdcVcom.attach)
            {
                s_cdcVcom.startTransactions = 1;
                rtt_puts("host opened the port\r\n");
            }
            error = kStatus_USB_Success;
        }
        break;
        case kUSB_DeviceCdcEventSendBreak:
            break;
        default:
            break;
    }

    return error;
}

/*******************************************************************************
 * USB device (chapter 9) callback
 ******************************************************************************/
usb_status_t USB_DeviceCallback(usb_device_handle handle, uint32_t event, void *param)
{
    usb_status_t error = kStatus_USB_InvalidRequest;
    uint16_t *temp16    = (uint16_t *)param;
    uint8_t *temp8      = (uint8_t *)param;

    switch (event)
    {
        case kUSB_DeviceEventBusReset:
            s_cdcVcom.attach               = 0;
            s_cdcVcom.currentConfiguration = 0U;
            error                          = kStatus_USB_Success;
            break;
        case kUSB_DeviceEventSetConfiguration:
            if (0U == (*temp8))
            {
                s_cdcVcom.attach               = 0;
                s_cdcVcom.currentConfiguration = 0U;
                error                          = kStatus_USB_Success;
            }
            else if (USB_CDC_VCOM_CONFIGURE_INDEX == (*temp8))
            {
                s_cdcVcom.attach               = 1;
                s_cdcVcom.currentConfiguration = *temp8;
                error                          = kStatus_USB_Success;
                USB_DeviceCdcAcmRecv(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_OUT_ENDPOINT, s_currRecvBuf,
                                      g_UsbDeviceCdcVcomDicEndpoints[1].maxPacketSize);
            }
            break;
        case kUSB_DeviceEventSetInterface:
            if (0U != s_cdcVcom.attach)
            {
                uint8_t interface        = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                uint8_t alternateSetting = (uint8_t)(*temp16 & 0x00FFU);
                if (interface == USB_CDC_VCOM_COMM_INTERFACE_INDEX)
                {
                    if (alternateSetting < USB_CDC_VCOM_COMM_INTERFACE_ALTERNATE_COUNT)
                    {
                        s_cdcVcom.currentInterfaceAlternateSetting[interface] = alternateSetting;
                        error                                                = kStatus_USB_Success;
                    }
                }
                else if (interface == USB_CDC_VCOM_DATA_INTERFACE_INDEX)
                {
                    if (alternateSetting < USB_CDC_VCOM_DATA_INTERFACE_ALTERNATE_COUNT)
                    {
                        s_cdcVcom.currentInterfaceAlternateSetting[interface] = alternateSetting;
                        error                                                = kStatus_USB_Success;
                    }
                }
            }
            break;
        case kUSB_DeviceEventGetConfiguration:
            if (NULL != param)
            {
                *temp8 = s_cdcVcom.currentConfiguration;
                error  = kStatus_USB_Success;
            }
            break;
        case kUSB_DeviceEventGetInterface:
            if (NULL != param)
            {
                uint8_t interface = (uint8_t)((*temp16 & 0xFF00U) >> 0x08U);
                if (interface < USB_CDC_VCOM_INTERFACE_COUNT)
                {
                    *temp16 = (*temp16 & 0xFF00U) | s_cdcVcom.currentInterfaceAlternateSetting[interface];
                    error   = kStatus_USB_Success;
                }
            }
            break;
        case kUSB_DeviceEventGetDeviceDescriptor:
            if (NULL != param)
            {
                error = USB_DeviceGetDeviceDescriptor(handle, (usb_device_get_device_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetConfigurationDescriptor:
            if (NULL != param)
            {
                error = USB_DeviceGetConfigurationDescriptor(
                    handle, (usb_device_get_configuration_descriptor_struct_t *)param);
            }
            break;
        case kUSB_DeviceEventGetStringDescriptor:
            if (NULL != param)
            {
                error = USB_DeviceGetStringDescriptor(handle, (usb_device_get_string_descriptor_struct_t *)param);
            }
            break;
        default:
            break;
    }

    return error;
}

/*******************************************************************************
 * Application init / task / main
 ******************************************************************************/
static void APPInit(void)
{
    LED_Init();
    USB_DeviceClockInit();

    s_cdcVcom.speed        = USB_SPEED_FULL;
    s_cdcVcom.attach       = 0;
    s_cdcVcom.cdcAcmHandle = (class_handle_t)NULL;
    s_cdcVcom.deviceHandle = NULL;

    if (kStatus_USB_Success != USB_DeviceClassInit(CONTROLLER_ID, &s_cdcAcmConfigList, &s_cdcVcom.deviceHandle))
    {
        rtt_puts("USB device init failed\r\n");
    }
    else
    {
        rtt_puts("USB device CDC virtual com demo\r\n");
        s_cdcVcom.cdcAcmHandle = s_cdcAcmConfigList.config->classHandle;
    }

    USB_DeviceIsrEnable();

    /* Let D+ stay released long enough for the host to notice the previous
     * disconnect before we pull it up again. */
    SDK_DelayAtLeastUs(5000, 48000000U);
    USB_DeviceRun(s_cdcVcom.deviceHandle);
}

static void APPTask(void)
{
    usb_status_t error = kStatus_USB_Error;

    if ((1U == s_cdcVcom.attach) && (1U == s_cdcVcom.startTransactions))
    {
        if ((0 != s_recvSize) && (USB_CANCELLED_TRANSFER_LENGTH != s_recvSize))
        {
            uint32_t sr = DisableGlobalIRQ();
            if ((0U != s_recvSize) && (USB_CANCELLED_TRANSFER_LENGTH != s_recvSize))
            {
                memcpy(s_currSendBuf, s_currRecvBuf, s_recvSize);
                s_sendSize = s_recvSize;
                s_recvSize = 0;
            }
            EnableGlobalIRQ(sr);
        }

        if (0U != s_sendSize)
        {
            uint32_t size = s_sendSize;
            s_sendSize    = 0;
            error = USB_DeviceCdcAcmSend(s_cdcVcom.cdcAcmHandle, USB_CDC_VCOM_BULK_IN_ENDPOINT, s_currSendBuf, size);
            (void)error;
        }
    }
}

int main(void)
{
    rtt_init();
    rtt_puts("USB CDC virtual COM starting\r\n");

    APPInit();

    while (1)
    {
        APPTask();
    }
}
