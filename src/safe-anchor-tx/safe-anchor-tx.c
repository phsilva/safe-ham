/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "../common/button_boot.h"

#include <stdio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include <libopencm3/cm3/nvic.h> // for sys_tick_handler
#include <libopencm3/cm3/systick.h>

#include "deca_device_api.h"
#include "deca_regs.h"

#include "FreeRTOS.h"
#include "task.h"

#define MOSI GPIO7
#define MISO GPIO6
#define CLK GPIO5
#define CS GPIO4

// TODO: add printf support (USART/USB)
// TODO: add FreeRTOS (task manaager, USB, DW1000, others)

// TODO: clear interruptions while sending SPI to DW1000
//  CM_ATOMIC_BLOCK in cortex.h

// understand interrupts
// 	nvic_set_priority(NVIC_DMA1_CHANNEL7_IRQ, 0);
//  nvic_enable_irq(NVIC_DMA1_CHANNEL7_IRQ);
//  convention defines the interrupt service routines name (irq_name__isr)
//  per chip defined in vector_nvic.h

// understand USB CDC
// undestand systick
// understand data format DW1000
// how to delays and execute?

static void msleep(uint32_t delay)
{
    vTaskDelay(delay / portTICK_PERIOD_MS);
}

void ledOn(void)
{
    gpio_clear(GPIOA, GPIO8);
}

static void ledOff(void)
{
    gpio_set(GPIOA, GPIO8);
}

static void toggleLed()
{
    gpio_toggle(GPIOA, GPIO8);
}

static void debugLed(uint8_t n)
{
    for (uint8_t i = 0; i < n; i++)
    {
        ledOn();
        msleep(300);
        ledOff();
        msleep(300);
    }

    msleep(1000);
}

decaIrqStatus_t decamutexon(void)
{
    return 0;
}

void decamutexoff(decaIrqStatus_t s)
{
}

void deca_sleep(unsigned int time_ms)
{
    msleep(time_ms);
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer)
{
    gpio_clear(GPIOA, CS);

    for (size_t i = 0; i < headerLength; i++)
        (void)spi_xfer(SPI1, headerBuffer[i]);

    for (size_t i = 0; i < readlength; i++)
        readBuffer[i] = spi_xfer(SPI1, 0x00);

    gpio_set(GPIOA, CS);
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodylength, const uint8 *bodyBuffer)
{
    gpio_clear(GPIOA, CS);

    for (size_t i = 0; i < headerLength; i++)
        (void)spi_xfer(SPI1, headerBuffer[i]);

    for (size_t i = 0; i < bodylength; i++)
        (void)spi_xfer(SPI1, bodyBuffer[i]);

    gpio_set(GPIOA, CS);
}

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0483,  // STMicroelectronics
    .idProduct = 0x5740, // STM32F407
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec it's
 * optional, but its absence causes a NULL pointer dereference in the
 * Linux cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x01,
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize = 64,
        .bInterval = 1,
    },
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x82,
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize = 64,
        .bInterval = 1,
    }};

static const struct
{
    struct usb_cdc_header_descriptor header;
    struct usb_cdc_call_management_descriptor call_mgmt;
    struct usb_cdc_acm_descriptor acm;
    struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
    .header =
        {
            .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
            .bDescriptorType = CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
            .bcdCDC = 0x0110,
        },
    .call_mgmt =
        {
            .bFunctionLength =
                sizeof(struct usb_cdc_call_management_descriptor),
            .bDescriptorType = CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
            .bmCapabilities = 0,
            .bDataInterface = 1,
        },
    .acm =
        {
            .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
            .bDescriptorType = CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_TYPE_ACM,
            .bmCapabilities = 0,
        },
    .cdc_union = {
        .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
    }};

static const struct usb_interface_descriptor comm_iface[] = {
    {.bLength = USB_DT_INTERFACE_SIZE,
     .bDescriptorType = USB_DT_INTERFACE,
     .bInterfaceNumber = 0,
     .bAlternateSetting = 0,
     .bNumEndpoints = 1,
     .bInterfaceClass = USB_CLASS_CDC,
     .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
     .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
     .iInterface = 0,

     .endpoint = comm_endp,

     .extra = &cdcacm_functional_descriptors,
     .extralen = sizeof(cdcacm_functional_descriptors)}};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,

    .endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {
    {
        .num_altsetting = 1, .altsetting = comm_iface,
    },
    {
        .num_altsetting = 1, .altsetting = data_iface,
    }};

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,

    .interface = ifaces,
};

static const char *usb_strings[] = {
    "Hamilton Positioning", "SafeHAM-TX", "DEMO",
};

static usbd_device *globalUSB;

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static int cdcacm_control_request(
    usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
    uint16_t *len,
    void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
    (void)complete;
    (void)buf;
    (void)usbd_dev;

    switch (req->bRequest)
    {
    case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
    {
        /*
       * This Linux cdc_acm driver requires this to be implemented
       * even though it's optional in the CDC spec, and we don't
       * advertise it in the ACM functional descriptor.
       */
        return 1;
    }
    case USB_CDC_REQ_SET_LINE_CODING:
        if (*len < sizeof(struct usb_cdc_line_coding))
        {
            return 0;
        }
        return 1;
    }
    return 0;
}

int _write(int file, char *ptr, int len)
{
    if (len)
    {
        // FIXME: how to deal with more than 64 bytes?
        usbd_ep_write_packet(globalUSB, 0x82, ptr, len);
        return len;
    }

    return -1;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
    (void)ep;

    char buf[64];
    int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

    uint32_t dwId = dwt_readdevid();

    buf[0] = (dwId >> 24) & 0xff;
    buf[1] = (dwId >> 16) & 0xff;
    buf[2] = (dwId >> 8) & 0xff;
    buf[3] = dwId & 0xff;

    printf("DevId: 0x%02x%02x%02x%02x\n", buf[0], buf[1], buf[2], buf[3]);
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
    (void)wValue;

    // 0x8n means IN (IN is host IN, thus uC OUT)

    // OUT (data from host)
    usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
    // IN (data to host)
    usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
    // IN (interrupt data to host)
    usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

    usbd_register_control_callback(
        usbd_dev, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, cdcacm_control_request);
}

static void setup_boot_button(void) { button_boot(); }

static void setup_clock(void)
{
    rcc_clock_setup_hse_3v3(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
}

static void setup_leds(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO8);
    gpio_set(GPIOA, GPIO8);
}

static void setup_usb(usbd_device **usbd_dev)
{
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9 | GPIO11 | GPIO12);
    gpio_set_af(GPIOA, GPIO_AF10, GPIO9 | GPIO11 | GPIO12);

    *usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config, usb_strings, 3,
                          usbd_control_buffer, sizeof(usbd_control_buffer));

    usbd_register_set_config_callback(*usbd_dev, cdcacm_set_config);
}

static void setup_spi(void)
{
    /*
    MOSI: PA7
    MISO: PA6
    CLK : PA5
    CS  : PA4
  */

    /* chip select */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, CS);
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, CLK | MISO | MOSI);

    rcc_periph_clock_enable(RCC_SPI1);

    /* set to high which is not-selected */
    gpio_set(GPIOA, CS);

    gpio_set_af(GPIOA, GPIO_AF5, CLK | MISO | MOSI);

    spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256,
                    /* high or low for the peripheral device */
                    SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
                    /* CPHA: Clock phase: read on rising edge of clock */
                    SPI_CR1_CPHA_CLK_TRANSITION_1,
                    /* DFF: Date frame format (8 or 16 bit) */
                    SPI_CR1_DFF_8BIT,
                    /* Most or Least Sig Bit First */
                    SPI_CR1_MSBFIRST);

    spi_enable(SPI1);
}

static void setup_systick(void)
{
    /* clock rate / 1000 to get 1mS interrupt rate */
    systick_set_reload(168000);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    /* this done last */
    systick_interrupt_enable();
}

static StaticTask_t xMainTask;
static StackType_t xMainStack[configMINIMAL_STACK_SIZE];

static StaticTask_t xDWMTask;
static StackType_t xDWMStack[configMINIMAL_STACK_SIZE];

#define BLINK_FRAME_SN_IDX 1

/* Inter-frame delay period, in milliseconds. */
#define TX_DELAY_MS 1000

static void set_spi_slow()
{
    spi_set_baudrate_prescaler(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_256);
}

static void set_spi_fast()
{
    spi_set_baudrate_prescaler(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_16);
}

static void dwm_task(void *pvParameters)
{
    msleep(5000);

    /* Default communication configuration. We use here EVK1000's default mode (mode 3). */
    dwt_config_t config = {
        2,               /* Channel number. */
        DWT_PRF_64M,     /* Pulse repetition frequency. */
        DWT_PLEN_1024,   /* Preamble length. Used in TX only. */
        DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
        9,               /* TX preamble code. Used in TX only. */
        9,               /* RX preamble code. Used in RX only. */
        1,               /* 0 to use standard SFD, 1 to use non-standard SFD. */
        DWT_BR_110K,     /* Data rate. */
        DWT_PHRMODE_STD, /* PHY header mode. */
        (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    };

    /* The frame sent in this example is an 802.15.4e standard blink. It is a 12-byte frame composed of the following fields:
 *     - byte 0: frame type (0xC5 for a blink).
 *     - byte 1: sequence number, incremented for each new frame.
 *     - byte 2 -> 9: device ID, see NOTE 1 below.
 *     - byte 10/11: frame check-sum, automatically set by DW1000.  */
    static uint8 tx_msg[] = {0xC5, 0, 'D', 'E', 'C', 'A', 'W', 'A', 'V', 'E', 0, 0};
    /* Index to access to sequence number of the blink frame in the tx_msg array. */

    msleep(5000);

    set_spi_slow();

    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0);
    gpio_clear(GPIOA, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);

    msleep(5);

    printf("reset state: 0x%02x", gpio_get(GPIOA, GPIO0));

    if (dwt_initialise(DWT_LOADNONE) == DWT_ERROR)
    {
        printf("INIT FAILED\n");
        while (1)
        {
        }
    }

    set_spi_fast();

    printf("init ready!\n");

    /* Configure DW1000. See NOTE 3 below. */
    dwt_configure(&config);

    printf("configure ready!\n");

    /* Loop forever sending frames periodically. */
    while (1)
    {
        /* Write frame data to DW1000 and prepare transmission. See NOTE 4 below.*/
        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0); /* Zero offset in TX buffer. */
        dwt_writetxfctrl(sizeof(tx_msg), 0, 0);     /* Zero offset in TX buffer, no ranging. */

        /* Start transmission. */
        printf("about to tx\n");
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        /* Poll DW1000 until TX frame sent event set. See NOTE 5 below.
         * STATUS register is 5 bytes long but, as the event we are looking at is in the first byte of the register, we can use this simplest API
         * function to access it.*/
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
        {
            printf("waiting for tx ready\n");
        };

        /* Clear TX frame sent event. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

        toggleLed();

        /* Execute a delay between transmissions. */
        msleep(TX_DELAY_MS);

        /* Increment the blink frame sequence number (modulo 256). */
        tx_msg[BLINK_FRAME_SN_IDX]++;
    }
}

static void main_task(void *pvParameters)
{
    usbd_device *usbd_dev = NULL;
    setup_usb(&usbd_dev);

    globalUSB = usbd_dev;

    while (1)
    {
        usbd_poll(usbd_dev);
    }
}

int main(void)
{
    setup_boot_button();

    setup_clock();
    setup_leds();
    setup_spi();
    setup_systick();

    // Setup main task
    xTaskCreateStatic(main_task, "main", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, xMainStack, &xMainTask);
    xTaskCreateStatic(dwm_task, "dwm", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES - 2, xDWMStack, &xDWMTask);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    return 0;
}

// Freertos required callbacks
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[configMINIMAL_STACK_SIZE];
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void vAssertCalled(unsigned long ulLine, const char *const pcFileName)
{
    printf("Assert failed at %s:%lu", pcFileName, ulLine);
    while (1)
        ;
}
