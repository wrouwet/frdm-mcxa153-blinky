/* USB CDC-ACM "virtual COM port" for the FRDM-MCXA153's second (target)
 * USB connector, wired to the MCX A153's own USB0 (KHCI) controller --
 * separate from the MCU-Link debug USB port.
 *
 * Behavior: accepts simple text commands over the CDC port and bridges
 * them to an LPI2C0 master transaction, so a host PC can drive I2C
 * devices on some other board through this one. See usage comment above
 * process_command() for the command syntax. Adapted from NXP's official
 * usb_device_cdc_vcom "bm" (bare-metal) example for this exact board,
 * with the debug-console/board/clock_config scaffolding stripped out and
 * replaced by the RTT logger already used by the blinky demo.
 */
#include <string.h>
#include "fsl_device_registers.h"
#include "fsl_lpi2c.h"
#include "fsl_port.h"
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
/* Sized independently of DATA_BUFF_SIZE (which is tied to the USB
 * endpoint's max packet size, 64 bytes, and only actually constrains
 * s_currRecvBuf below): USB_DeviceCdcAcmSend() already transparently
 * spans multiple packets for a longer buffer (see the ZLP-handling logic
 * in kUSB_DeviceCdcEventSendResponse), so the real ceiling on a reply is
 * just this array's size, not one packet. Needs to be big enough for the
 * longest reply process_command() can produce ("OK" + N * " xx" + "\r\n"
 * for N = I2C_CMD_MAX_DATA bytes -- see that constant's comment for why
 * it's sized the way it is). 400 comfortably covers "OK" + 128*" xx" +
 * "\r\n" (388) with headroom. */
#define SEND_BUF_SIZE 400U
USB_DMA_NONINIT_DATA_ALIGN(USB_DATA_ALIGN_SIZE) static uint8_t s_currSendBuf[SEND_BUF_SIZE];
volatile static uint32_t s_sendSize = 0;

/* Command line assembly. Bytes arrive here as USB packets (possibly one
 * character at a time, from an interactive terminal); s_lineReady is set
 * once a full line has accumulated, and cleared again by APPTask() once
 * it has built a reply. While a completed line is waiting to be answered,
 * no further receive is armed (simple half-duplex command/response).
 * Needs to hold the longest *input* line too, not just replies -- a
 * worst-case "XS <addr> <count> <128 hex data bytes>" write is close to
 * 400 characters, same headroom reasoning as SEND_BUF_SIZE. */
#define LINE_BUF_SIZE 400U
static char s_lineBuf[LINE_BUF_SIZE];
static uint32_t s_lineLen             = 0;
volatile static bool s_lineReady      = false;

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

/* Bumped once per SysTick period (~250 ms, see the calibration note in
 * LED_Init()) so other code can bound a wait without needing an accurate
 * millisecond time base -- good enough for "give up eventually", not for
 * precise timing. */
volatile static uint32_t s_tickCount = 0;

void SysTick_Handler(void)
{
    GPIO3->PTOR = (1U << LED_RED_PIN);
    s_tickCount++;
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

    /* Neither SystemCoreClock nor CLOCK_GetCoreSysClkFreq() (48 MHz) match
     * the clock that actually drives SysTick's CLKSOURCE=1 "processor
     * clock" input on this chip/board -- both produced a visibly faster
     * blink than intended. This reload was calibrated by eye against the
     * real board to give a ~2 Hz blink (full on/off cycle twice a second).
     * Note SysTick->LOAD is only 24 bits wide (max ~16.7M), so any fix
     * using a queried clock value needs a software tick counter rather
     * than a single reload once the desired period gets much longer. */
    SysTick_Config(12000000UL);
}

void USB_DeviceClockInit(void)
{
    RESET_PeripheralReset(kUSB0_RST_SHIFT_RSTn);
    CLOCK_EnableUsbfsClock();
}

/* LPI2C0 master, routed to PORT3 pins 27 (SCL) / 28 (SDA) via mux ALT2,
 * which fan out to the board's mikroBUS header (J5) and Pmod connector
 * (J7, unpopulated) -- confirmed against NXP's own lpi2c/polling_b2b
 * example for this board. Clocked from the 12 MHz FRO (undivided), same
 * clock-attach pattern as CLOCK_EnableUsbfsClock() uses for USB. */
#define I2C_MASTER_CLOCK_HZ 12000000UL
#define I2C_MASTER_BAUD_HZ  100000U

static void I2C_Init(void)
{
    CLOCK_EnableClock(kCLOCK_GateLPI2C0);
    CLOCK_EnableClock(kCLOCK_GatePORT3);
    RESET_ReleasePeripheralReset(kLPI2C0_RST_SHIFT_RSTn);
    RESET_ReleasePeripheralReset(kPORT3_RST_SHIFT_RSTn);

    CLOCK_AttachClk(kFRO12M_to_LPI2C0);
    CLOCK_SetClockDiv(kCLOCK_DivLPI2C0, 1U);

    /* Enable the pins' internal pull-ups (matching NXP's own lpi2c
     * reference config for these pins). I2C needs pull-ups to work at
     * all; the internal ones are weak (~kOhm range, fine at 100 kHz) and
     * mainly matter for bench-testing this bus with nothing wired to it
     * yet -- an external target board will normally supply its own,
     * stronger pull-ups once connected. */
    const port_pin_config_t i2cPinConfig = {
        .pullSelect          = kPORT_PullUp,
        .pullValueSelect     = kPORT_LowPullResistor,
        .slewRate            = kPORT_FastSlewRate,
        .passiveFilterEnable = kPORT_PassiveFilterDisable,
        .openDrainEnable     = kPORT_OpenDrainDisable,
        .driveStrength       = kPORT_LowDriveStrength,
        .driveStrength1      = kPORT_NormalDriveStrength,
        .mux                 = kPORT_MuxAlt2,
        .inputBuffer         = kPORT_InputBufferEnable,
        .invertInput         = kPORT_InputNormal,
        .lockRegister        = kPORT_UnlockRegister,
    };
    PORT_SetPinConfig(PORT3, 27U, &i2cPinConfig);
    PORT_SetPinConfig(PORT3, 28U, &i2cPinConfig);

    lpi2c_master_config_t config;
    LPI2C_MasterGetDefaultConfig(&config);
    config.baudRate_Hz = I2C_MASTER_BAUD_HZ;
    LPI2C_MasterInit(LPI2C0, &config, I2C_MASTER_CLOCK_HZ);
}

/* Some I2C targets (e.g. an OpenBIC controller speaking IPMB) don't
 * answer a request by being read from -- they reply by becoming bus
 * MASTER themselves and writing the response out to whatever
 * "requester address" was named in the request. To see that response,
 * we have to temporarily become an I2C SLAVE ourselves at that address
 * and catch the incoming write, then switch back to master mode
 * afterwards. Polled rather than interrupt-driven, matching the rest of
 * this file's fully-synchronous per-command design. */
static bool I2C_SlaveWaitForWrite(uint8_t ourAddr, uint8_t *buf, uint32_t bufMax, uint32_t *outLen,
                                   uint32_t timeoutTicks)
{
    LPI2C_MasterEnable(LPI2C0, false);

    lpi2c_slave_config_t slaveConfig;
    LPI2C_SlaveGetDefaultConfig(&slaveConfig);
    slaveConfig.address0 = ourAddr;
    LPI2C_SlaveInit(LPI2C0, &slaveConfig, I2C_MASTER_CLOCK_HZ);

    uint32_t len       = 0;
    bool sawAddress    = false;
    bool gotStop       = false;
    uint32_t startTick = s_tickCount;

    while ((s_tickCount - startTick) <= timeoutTicks)
    {
        uint32_t status = LPI2C_SlaveGetStatusFlags(LPI2C0);

        if (0U != (status & (uint32_t)kLPI2C_SlaveAddressValidFlag))
        {
            sawAddress = true;
            (void)LPI2C0->SASR; /* reading SASR clears AVF */
        }

        if (0U != (status & (uint32_t)kLPI2C_SlaveRxReadyFlag))
        {
            uint32_t data = LPI2C0->SRDR;
            if ((0U == (data & LPI2C_SRDR_RXEMPTY_MASK)) && (len < bufMax))
            {
                buf[len++] = (uint8_t)(data & LPI2C_SRDR_DATA_MASK);
            }
        }

        if (0U != (status & (uint32_t)kLPI2C_SlaveStopDetectFlag))
        {
            LPI2C_SlaveClearStatusFlags(LPI2C0, (uint32_t)kLPI2C_SlaveStopDetectFlag);
            if (sawAddress)
            {
                gotStop = true;
                break;
            }
        }
    }

    LPI2C_SlaveEnable(LPI2C0, false);
    I2C_Init(); /* back to master mode, with pins/clocks freshly reconfigured */

    rtt_puts(sawAddress ? "slave: address was matched\r\n" : "slave: no address match seen\r\n");

    *outLen = len;
    return gotStop;
}

/* This driver has no automatic bus-recovery: if a transfer times out
 * (e.g. the target device is holding SCL or SDA low mid-transaction), the
 * bus stays wedged and every subsequent transfer -- even a plain write --
 * keeps failing until something forces the lines free again. This is the
 * standard I2C recovery procedure: temporarily drive the pins as plain
 * GPIO, clock SCL up to 9 times (enough for any slave stuck mid-byte to
 * finish and release SDA), then drive a manual STOP condition, before
 * switching the pins back to LPI2C0 and reinitializing the peripheral. */
static void I2C_BusRecover(void)
{
    PORT3->PCR[27] = PORT_PCR_MUX(0U) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* SCL, GPIO, pull-up */
    PORT3->PCR[28] = PORT_PCR_MUX(0U) | PORT_PCR_PE_MASK | PORT_PCR_PS_MASK; /* SDA, GPIO, pull-up */

    GPIO3->PDDR |= (1U << 27);  /* SCL: output */
    GPIO3->PDDR &= ~(1U << 28); /* SDA: input, so we can see when it's released */
    GPIO3->PSOR = (1U << 27);   /* SCL idle high */

    for (int i = 0; i < 9; i++)
    {
        if (0U != (GPIO3->PDIR & (1U << 28)))
        {
            break; /* SDA already released */
        }
        GPIO3->PCOR = (1U << 27);
        for (volatile int d = 0; d < 200; d++)
        {
        }
        GPIO3->PSOR = (1U << 27);
        for (volatile int d = 0; d < 200; d++)
        {
        }
    }

    /* Manual STOP condition: SDA low-to-high while SCL is high. */
    GPIO3->PDDR |= (1U << 28);
    GPIO3->PCOR = (1U << 28);
    for (volatile int d = 0; d < 200; d++)
    {
    }
    GPIO3->PSOR = (1U << 27);
    for (volatile int d = 0; d < 200; d++)
    {
    }
    GPIO3->PSOR = (1U << 28);
    for (volatile int d = 0; d < 200; d++)
    {
    }
    GPIO3->PDDR &= ~(1U << 28);

    I2C_Init();
}

/* NAK is a normal "no device at this address" response and doesn't mean
 * the bus is wedged, so only run recovery for the other failure classes. */
static void i2c_recover_if_needed(status_t status)
{
    if (status != kStatus_Success && status != kStatus_LPI2C_Nak)
    {
        I2C_BusRecover();
    }
}

void USB_DeviceIsrEnable(void)
{
    uint8_t usbDeviceKhciIrq[] = USBFS_IRQS;
    uint8_t irqNumber          = usbDeviceKhciIrq[CONTROLLER_ID - kUSB_ControllerKhci0];

    NVIC_SetPriority((IRQn_Type)irqNumber, USB_DEVICE_INTERRUPT_PRIORITY);
    EnableIRQ((IRQn_Type)irqNumber);
}

/*******************************************************************************
 * UART-to-I2C command bridge
 *
 * Line-based text protocol, one command per line (LF or CRLF terminated).
 * <addr> is a 7-bit I2C address, <byte> a data byte -- both 1-2 hex
 * digits, no "0x" prefix. <n> is a decimal byte count.
 *
 *   W <addr> <byte> [byte ...]     write bytes to <addr>
 *   R <addr> <n>                   read <n> bytes from <addr>
 *   X <addr> <n> <byte> [byte ...] write bytes, repeated-start, then
 *                                  read <n> bytes (register-read pattern)
 *   WS/RS/XS                       SMBus flavor of W/R/X: adds/checks a
 *                                  trailing PEC (Packet Error Check,
 *                                  CRC-8) byte per the SMBus spec, same
 *                                  arguments otherwise. See the smbus_pec_*
 *                                  helpers below for exactly which bytes
 *                                  the CRC covers for each. RS/XS report
 *                                  "ERR pec" (instead of the usual "OK
 *                                  <bytes>") if the device's PEC byte
 *                                  doesn't check out, and otherwise return
 *                                  just the requested <n> data bytes --
 *                                  the trailing PEC byte itself is
 *                                  verified, not handed back to the
 *                                  caller. The `S` modifier is only
 *                                  recognized directly after `W`/`R`/`X`
 *                                  (no space) -- it's unrelated to the
 *                                  standalone `S` scan command below.
 *   S                              scan the bus, list responding addresses
 *   I <addr> <ourAddr> <byte> [byte ...]
 *                                  write bytes to <addr> (e.g. an IPMB
 *                                  request), then briefly become an I2C
 *                                  slave at <ourAddr> and capture whatever
 *                                  the target writes back -- for devices
 *                                  (like OpenBIC/IPMB) that respond by
 *                                  becoming bus master themselves rather
 *                                  than being read from
 *   L <ourAddr>                    like I, but with no write of our own
 *                                  first -- just listen; useful for
 *                                  independently testing the slave-mode
 *                                  RX path against some other master
 *
 * Replies (also LF/CRLF terminated):
 *   OK                             W succeeded
 *   OK <byte> [byte ...]           R/X/I/L succeeded, with the data
 *   OK <addr> [addr ...]           S succeeded (may be empty if nothing found)
 *   ERR <reason>                   something went wrong
 *
 * Example: reading 2 bytes starting at register 0x00 of a device at 0x50:
 *   > X 50 2 00
 *   < OK a1 b2
 ******************************************************************************/
/* Ceiling on how much data a single command can move, both what W/X can
 * write and what R/X/I/L can read/capture. Constrained only by
 * SEND_BUF_SIZE/LINE_BUF_SIZE now ("OK" + N * " xx" + "\r\n" <=
 * SEND_BUF_SIZE, and a worst-case input line <= LINE_BUF_SIZE -- see
 * both constants' comments), not by a single USB packet -- multi-packet
 * transfers work fine either direction.
 *
 * Originally sized for IPMI/IPMB traffic (a smaller 16 silently
 * truncated a standard 18-byte Get Device ID response, dropping the
 * trailing checksum byte and making every response look
 * checksum-invalid even though the actual exchange was fine -- caught
 * 2026-08-24; 32 covered that plus the longer Aux-FW-Revision variant).
 *
 * Bumped from 32 to 128 to comfortably fit a full, unfragmented MCTP
 * packet for future MCTP-over-SMBus testing: per DSP0236 (the MCTP base
 * spec), every compliant endpoint must support a 64-byte baseline MTU
 * (payload), and the MCTP transport header adds 4 more bytes on top of
 * that -- 68 bytes minimum, plus 1 more for this bridge's own SMBus PEC
 * byte (see WS/RS/XS above) = 69 bytes minimum needed for a single
 * baseline-MTU MCTP request/response to round-trip through this bridge
 * without fragmentation-handling logic here. 128 leaves real headroom
 * above that minimum (some MCTP profiles negotiate larger MTUs) without
 * costing much RAM on a chip this size. If a future need exceeds even
 * this, the right fix is proper multi-packet reassembly in whatever's
 * driving the bridge, not another one-off bump here. */
#define I2C_CMD_MAX_DATA 128U

static const char *skip_spaces(const char *p)
{
    while (*p == ' ')
    {
        p++;
    }
    return p;
}

static bool parse_hex_u32(const char **pp, uint32_t *out, uint32_t maxDigits)
{
    const char *p = skip_spaces(*pp);
    uint32_t value = 0;
    uint32_t ndig  = 0;
    while (ndig < maxDigits)
    {
        char c = *p;
        uint32_t digit;
        if (c >= '0' && c <= '9')
        {
            digit = (uint32_t)(c - '0');
        }
        else if (c >= 'a' && c <= 'f')
        {
            digit = (uint32_t)(c - 'a' + 10);
        }
        else if (c >= 'A' && c <= 'F')
        {
            digit = (uint32_t)(c - 'A' + 10);
        }
        else
        {
            break;
        }
        value = (value << 4) | digit;
        ndig++;
        p++;
    }
    if (ndig == 0U)
    {
        return false;
    }
    *out = value;
    *pp  = p;
    return true;
}

static bool parse_dec_u32(const char **pp, uint32_t *out)
{
    const char *p = skip_spaces(*pp);
    uint32_t value = 0;
    uint32_t ndig  = 0;
    while (*p >= '0' && *p <= '9')
    {
        value = (value * 10U) + (uint32_t)(*p - '0');
        p++;
        ndig++;
    }
    if (ndig == 0U)
    {
        return false;
    }
    *out = value;
    *pp  = p;
    return true;
}

static uint32_t append_str(char *buf, uint32_t pos, const char *s)
{
    while (*s)
    {
        buf[pos++] = *s++;
    }
    return pos;
}

static uint32_t append_hex_byte(char *buf, uint32_t pos, uint8_t v)
{
    static const char hexdigits[] = "0123456789abcdef";
    buf[pos++]                    = hexdigits[(v >> 4) & 0xFU];
    buf[pos++]                    = hexdigits[v & 0xFU];
    return pos;
}

/* I2C_RETRY_TIMES is overridden (see Makefile) to a finite value so a
 * floating/unwired bus (no target attached, no pull-ups) fails fast with
 * kStatus_LPI2C_Timeout instead of hanging the whole command loop forever. */
static const char *i2c_error_str(status_t status)
{
    switch (status)
    {
        case kStatus_LPI2C_Nak:
            return "ERR nak";
        case kStatus_LPI2C_Timeout:
            return "ERR timeout (bus never went idle -- check wiring/pull-ups)";
        case kStatus_LPI2C_PinLowTimeout:
            return "ERR pin low timeout (SCL or SDA stuck low -- check wiring/short/target power)";
        case kStatus_LPI2C_ArbitrationLost:
            return "ERR arbitration lost (bus contention -- another master, or SDA/SCL swapped/shorted?)";
        case kStatus_LPI2C_BitError:
            return "ERR bit error (bus noise or a wire not making contact)";
        case kStatus_LPI2C_FifoError:
            return "ERR fifo error";
        case kStatus_LPI2C_Busy:
            return "ERR busy";
        default:
            return "ERR i2c (unknown status)";
    }
}

/* SMBus PEC (Packet Error Check): a CRC-8 with polynomial x^8+x^2+x+1
 * (0x07), MSB-first, no reflection, initial value 0 -- computed over
 * every byte actually seen on the wire for a transaction, per the SMBus
 * spec. Deliberately a plain bit-by-bit implementation rather than a
 * lookup table: our transactions are at most a few dozen bytes, so the
 * table's ROM/RAM cost isn't worth it here.
 *
 * Critically, this covers the address+R/W byte(s) too -- but those never
 * appear in our own data[]/rdata[] buffers, since
 * LPI2C_MasterTransferBlocking() generates and sends them itself as part
 * of issuing START (and repeated START). So every call site below feeds
 * (addr << 1) | rw by hand into the running CRC at the right point(s),
 * in addition to whatever's actually in a data buffer -- get the byte
 * ordering/count wrong here and PEC will silently never match, even
 * though the underlying I2C bytes are all correct. */
static uint8_t smbus_pec_byte(uint8_t crc, uint8_t b)
{
    crc = (uint8_t)(crc ^ b);
    for (int i = 0; i < 8; i++)
    {
        crc = (crc & 0x80U) ? (uint8_t)((uint32_t)(crc << 1) ^ 0x07U) : (uint8_t)(crc << 1);
    }
    return crc;
}

static uint8_t smbus_pec_buf(uint8_t crc, const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++)
    {
        crc = smbus_pec_byte(crc, buf[i]);
    }
    return crc;
}

/* Processes one received command line (not including its line ending) and
 * writes a reply (not including its line ending) into outBuf. Returns the
 * reply length. outBuf must be at least DATA_BUFF_SIZE bytes. */
static uint32_t process_command(const char *line, uint32_t lineLen, char *outBuf)
{
    const char *p = skip_spaces(line);
    char cmd      = *p;
    if (cmd >= 'a' && cmd <= 'z')
    {
        cmd = (char)(cmd - 'a' + 'A');
    }
    p++;

    if (cmd != 'S' && cmd != 'W' && cmd != 'R' && cmd != 'X' && cmd != 'I' && cmd != 'L')
    {
        return append_str(outBuf, 0, "ERR bad command");
    }

    if (cmd == 'S')
    {
        /* Probe each address with a zero-length write, the standard I2C
         * scan idiom: it's just a START + address byte + STOP, and
         * LPI2C_MasterTransferBlocking() correctly waits for and reports
         * a NAK. (The lower-level LPI2C_MasterStart() alone does *not* --
         * it only enqueues the START command into the hardware FIFO and
         * returns, without waiting to see whether the address was
         * actually acknowledged on the bus -- so it can't be used to
         * detect a NAK by itself.)
         *
         * Unlike every other command below, this loop originally never
         * called i2c_recover_if_needed() on a failed probe -- meaning a
         * scan issued while the bus happened to be wedged (e.g. right
         * after two closely-spaced master writes collided, one seeing
         * arbitration loss) silently came back completely empty and
         * *stayed* wedged, since nothing here ever recovered it. Every
         * other command (W/R/X/I) already self-heals this way; a plain
         * scan reasonably needs to as well, since it's often the first
         * thing run in a new session and has no other error path to
         * surface the problem through (found 2026-08-24 via the test
         * framework: a scan run immediately after a deliberate
         * back-to-back-write stress test came back empty, while a
         * subsequent W-based command succeeded normally because it, not
         * the scan, happened to trigger recovery). */
        uint32_t pos = append_str(outBuf, 0, "OK");
        for (uint32_t addr = 0x08U; addr <= 0x77U; addr++)
        {
            lpi2c_master_transfer_t probe = {0};
            probe.slaveAddress             = (uint16_t)addr;
            probe.direction                = kLPI2C_Write;
            probe.flags                    = kLPI2C_TransferDefaultFlag;

            status_t status = LPI2C_MasterTransferBlocking(LPI2C0, &probe);
            if (kStatus_Success == status)
            {
                outBuf[pos++] = ' ';
                pos           = append_hex_byte(outBuf, pos, (uint8_t)addr);
            }
            else
            {
                i2c_recover_if_needed(status);
            }
        }
        return pos;
    }

    if (cmd == 'L')
    {
        /* Listen only, with no write of our own first -- useful on its own
         * for independently verifying the slave-mode RX path (e.g. against
         * a master that isn't us), decoupled from whatever an "I" command's
         * own write step might be doing. */
        uint32_t ourAddr;
        if (!parse_hex_u32(&p, &ourAddr, 2U) || ourAddr > 0x7FU)
        {
            return append_str(outBuf, 0, "ERR bad address");
        }

        uint8_t rdata[I2C_CMD_MAX_DATA];
        uint32_t rlen = 0;
        bool got = I2C_SlaveWaitForWrite((uint8_t)ourAddr, rdata, I2C_CMD_MAX_DATA, &rlen, 16U);
        if (!got)
        {
            return append_str(outBuf, 0, "ERR timeout waiting for a write");
        }

        uint32_t pos = append_str(outBuf, 0, "OK");
        for (uint32_t i = 0; i < rlen; i++)
        {
            outBuf[pos++] = ' ';
            pos           = append_hex_byte(outBuf, pos, rdata[i]);
        }
        return pos;
    }

    /* Only W/R/X reach this point (S and L both return above), so this
     * modifier check can't misfire on the standalone S/L commands. No
     * space between the command letter and this 'S' -- "WS 50 ..." not
     * "W S 50 ...' -- so it reads naturally as one token, "the SMBus
     * flavor of W", rather than looking like a separate argument. */
    bool smbusPec = false;
    if (*p == 'S' || *p == 's')
    {
        smbusPec = true;
        p++;
    }

    uint32_t addr;
    if (!parse_hex_u32(&p, &addr, 2U) || addr > 0x7FU)
    {
        return append_str(outBuf, 0, "ERR bad address");
    }

    if (cmd == 'W' || cmd == 'X')
    {
        uint32_t readCount = 0;
        if (cmd == 'X')
        {
            if (!parse_dec_u32(&p, &readCount) || readCount > I2C_CMD_MAX_DATA)
            {
                return append_str(outBuf, 0, "ERR bad count");
            }
        }

        uint8_t data[I2C_CMD_MAX_DATA];
        uint32_t dataLen = 0;
        uint32_t byte;
        /* A plain SMBus write (WS) needs room for one extra, computed PEC
         * byte appended after the caller's data below -- reserve it up
         * front so the parse loop can't fill the buffer completely and
         * leave no space for it. X's write phase never gets its own PEC
         * (a combined write+read transaction has exactly one PEC, after
         * the read phase, covering the whole thing -- see below), so it
         * doesn't need this reservation. */
        uint32_t dataMax = (smbusPec && cmd == 'W') ? (I2C_CMD_MAX_DATA - 1U) : I2C_CMD_MAX_DATA;
        while (dataLen < dataMax && parse_hex_u32(&p, &byte, 2U))
        {
            data[dataLen++] = (uint8_t)byte;
        }

        /* Appending a plain write's PEC has to happen here, before the
         * transfer, since it must go out as part of the SAME START..STOP
         * transaction as the rest of the data -- not a second, separate
         * one. */
        if (smbusPec && cmd == 'W')
        {
            uint8_t pec = smbus_pec_byte(0U, (uint8_t)((addr << 1) | 0U));
            pec         = smbus_pec_buf(pec, data, dataLen);
            data[dataLen++] = pec;
        }

        lpi2c_master_transfer_t xfer = {0};
        xfer.slaveAddress             = (uint16_t)addr;
        xfer.direction                = kLPI2C_Write;
        xfer.data                     = data;
        xfer.dataSize                 = dataLen;
        xfer.flags                    = (cmd == 'X') ? kLPI2C_TransferNoStopFlag : kLPI2C_TransferDefaultFlag;

        status_t status = LPI2C_MasterTransferBlocking(LPI2C0, &xfer);
        if (status != kStatus_Success)
        {
            i2c_recover_if_needed(status);
            return append_str(outBuf, 0, i2c_error_str(status));
        }

        if (cmd == 'W')
        {
            return append_str(outBuf, 0, "OK");
        }

        /* X (and XS): read phase. For XS, PEC covers the WHOLE
         * transaction -- write address+data, repeated start, read
         * address+data -- as one continuous CRC, with the PEC byte
         * itself appended by the target after its last real data byte.
         * Ask for one extra byte so we can read and check it, but only
         * ever report the requested readCount data bytes back to the
         * caller -- the PEC byte itself is consumed here, not handed
         * back. */
        uint32_t readLen = readCount + (smbusPec ? 1U : 0U);
        if (readLen > I2C_CMD_MAX_DATA)
        {
            return append_str(outBuf, 0, "ERR bad count");
        }

        uint8_t rdata[I2C_CMD_MAX_DATA];
        lpi2c_master_transfer_t rxfer = {0};
        rxfer.slaveAddress             = (uint16_t)addr;
        rxfer.direction                = kLPI2C_Read;
        rxfer.data                     = rdata;
        rxfer.dataSize                 = readLen;
        rxfer.flags                    = kLPI2C_TransferRepeatedStartFlag;

        status = LPI2C_MasterTransferBlocking(LPI2C0, &rxfer);
        if (status != kStatus_Success)
        {
            i2c_recover_if_needed(status);
            return append_str(outBuf, 0, i2c_error_str(status));
        }

        if (smbusPec)
        {
            /* dataLen here already reflects only the write-phase bytes
             * actually sent -- X's write phase never got its own PEC
             * appended above (that only happens for a plain W), so this
             * correctly covers just the real write data before folding
             * in the repeated-start read address and read data. */
            uint8_t pec = smbus_pec_byte(0U, (uint8_t)((addr << 1) | 0U));
            pec         = smbus_pec_buf(pec, data, dataLen);
            pec         = smbus_pec_byte(pec, (uint8_t)((addr << 1) | 1U));
            pec         = smbus_pec_buf(pec, rdata, readCount);
            if (pec != rdata[readCount])
            {
                return append_str(outBuf, 0, "ERR pec (SMBus packet error check failed)");
            }
        }

        uint32_t pos = append_str(outBuf, 0, "OK");
        for (uint32_t i = 0; i < readCount; i++)
        {
            outBuf[pos++] = ' ';
            pos           = append_hex_byte(outBuf, pos, rdata[i]);
        }
        return pos;
    }

    if (cmd == 'R')
    {
        uint32_t count;
        if (!parse_dec_u32(&p, &count) || count > I2C_CMD_MAX_DATA)
        {
            return append_str(outBuf, 0, "ERR bad count");
        }

        /* See the X/XS read phase above for why one extra byte is
         * requested when smbusPec is set, and why it's checked against
         * I2C_CMD_MAX_DATA again here (count alone can pass the check
         * above yet still overflow rdata[] once the trailing PEC byte
         * is added on top of it). */
        uint32_t readLen = count + (smbusPec ? 1U : 0U);
        if (readLen > I2C_CMD_MAX_DATA)
        {
            return append_str(outBuf, 0, "ERR bad count");
        }

        uint8_t rdata[I2C_CMD_MAX_DATA];
        lpi2c_master_transfer_t xfer = {0};
        xfer.slaveAddress             = (uint16_t)addr;
        xfer.direction                = kLPI2C_Read;
        xfer.data                     = rdata;
        xfer.dataSize                 = readLen;
        xfer.flags                    = kLPI2C_TransferDefaultFlag;

        status_t status = LPI2C_MasterTransferBlocking(LPI2C0, &xfer);
        if (status != kStatus_Success)
        {
            i2c_recover_if_needed(status);
            return append_str(outBuf, 0, i2c_error_str(status));
        }

        if (smbusPec)
        {
            uint8_t pec = smbus_pec_byte(0U, (uint8_t)((addr << 1) | 1U));
            pec         = smbus_pec_buf(pec, rdata, count);
            if (pec != rdata[count])
            {
                return append_str(outBuf, 0, "ERR pec (SMBus packet error check failed)");
            }
        }

        uint32_t pos = append_str(outBuf, 0, "OK");
        for (uint32_t i = 0; i < count; i++)
        {
            outBuf[pos++] = ' ';
            pos           = append_hex_byte(outBuf, pos, rdata[i]);
        }
        return pos;
    }

    if (cmd == 'I')
    {
        uint32_t ourAddr;
        if (!parse_hex_u32(&p, &ourAddr, 2U) || ourAddr > 0x7FU)
        {
            return append_str(outBuf, 0, "ERR bad requester address");
        }

        uint8_t data[I2C_CMD_MAX_DATA];
        uint32_t dataLen = 0;
        uint32_t byte;
        while (dataLen < I2C_CMD_MAX_DATA && parse_hex_u32(&p, &byte, 2U))
        {
            data[dataLen++] = (uint8_t)byte;
        }

        lpi2c_master_transfer_t xfer = {0};
        xfer.slaveAddress             = (uint16_t)addr;
        xfer.direction                = kLPI2C_Write;
        xfer.data                     = data;
        xfer.dataSize                 = dataLen;
        xfer.flags                    = kLPI2C_TransferDefaultFlag;

        status_t status = LPI2C_MasterTransferBlocking(LPI2C0, &xfer);
        if (status != kStatus_Success)
        {
            i2c_recover_if_needed(status);
            return append_str(outBuf, 0, i2c_error_str(status));
        }

        uint8_t rdata[I2C_CMD_MAX_DATA];
        uint32_t rlen = 0;
        /* ~1s: a handful of SysTick periods (~250 ms each, see LED_Init()). */
        bool got = I2C_SlaveWaitForWrite((uint8_t)ourAddr, rdata, I2C_CMD_MAX_DATA, &rlen, 16U);
        if (!got)
        {
            return append_str(outBuf, 0, "ERR timeout waiting for response");
        }

        uint32_t pos = append_str(outBuf, 0, "OK");
        for (uint32_t i = 0; i < rlen; i++)
        {
            outBuf[pos++] = ' ';
            pos           = append_hex_byte(outBuf, pos, rdata[i]);
        }
        return pos;
    }

    (void)lineLen;
    return append_str(outBuf, 0, "ERR bad command");
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
                error = kStatus_USB_Success;

                bool lineDone = false;
                for (uint32_t i = 0; i < epCbParam->length && !lineDone; i++)
                {
                    char c = (char)s_currRecvBuf[i];
                    if (c == '\n')
                    {
                        lineDone = true;
                    }
                    else if (c != '\r')
                    {
                        /* Reserve the last slot for the NUL terminator below. */
                        if (s_lineLen < LINE_BUF_SIZE - 1U)
                        {
                            s_lineBuf[s_lineLen++] = c;
                        }
                        else
                        {
                            lineDone = true; /* overflow: treat as a (too-long) line so we reply and reset */
                        }
                    }
                }

                if (lineDone)
                {
                    /* NUL-terminate so the hand-rolled hex/decimal parsers in
                     * process_command() stop at the real end of the line
                     * instead of reading stale bytes left over in this
                     * static buffer from a previous, longer command. */
                    s_lineBuf[s_lineLen] = '\0';
                    s_lineReady           = true; /* APPTask() will reply, then re-arm the next receive */
                }
                else
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

    I2C_Init();
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
        if (s_lineReady && (0U == s_sendSize))
        {
            uint32_t replyLen = process_command(s_lineBuf, s_lineLen, (char *)s_currSendBuf);
            s_currSendBuf[replyLen++] = '\r';
            s_currSendBuf[replyLen++] = '\n';

            s_lineLen   = 0;
            s_lineReady = false;
            s_sendSize  = replyLen;
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
