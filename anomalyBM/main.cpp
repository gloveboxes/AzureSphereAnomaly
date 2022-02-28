/* Copyright (c) Microsoft Corporation. All rights reserved.
 * Licensed under the MIT License.
 *
 * This example is built on the Azure Sphere DevX library.
 *   1. DevX is an Open Source community-maintained implementation of the Azure Sphere SDK samples.
 *   2. DevX is a modular library that simplifies common development scenarios.
 *        - You can focus on your solution, not the plumbing.
 *   3. DevX documentation is maintained at https://github.com/gloveboxes/AzureSphereDevX/wiki
 *	 4. The DevX library is not a substitute for understanding the Azure Sphere SDK Samples.
 *          - https://github.com/Azure/azure-sphere-samples
 *
 * DEVELOPER BOARD SELECTION
 *
 * The following developer boards are supported.
 *
 *	 1. AVNET Azure Sphere Starter Kit.
 *   2. AVNET Azure Sphere Starter Kit Revision 2.
 *	 3. Seeed Studio Azure Sphere MT3620 Development Kit aka Reference Design Board or rdb.
 *	 4. Seeed Studio Seeed Studio MT3620 Mini Dev Board.
 *
 *
 ************************************************************************************************/

#include "intercore.h"
#include "intercore_contract.h"

#include "os_hal_uart.h"
#include "os_hal_gpt.h"
#include "os_hal_i2c.h"
#include "nvic.h"

#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include "lsm6dso_driver.h"
#include "lsm6dso_reg.h"

#include "ei_run_classifier.h"

void *__dso_handle = (void *)&__dso_handle;

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER
#error "Incompatible model, this example is only compatible with accelerometer data"
#endif

/* I2C */
static const i2c_num i2c_port_num = OS_HAL_I2C_ISU2;
static const i2c_speed_kHz i2c_speed = I2C_SCL_50kHz;
static const uint8_t i2c_lsm6dso_addr = LSM6DSO_I2C_ADD_L >> 1;
static uint8_t *i2c_tx_buf;
static uint8_t *i2c_rx_buf;

// Edge Impulse
static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
static float inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};

#define I2C_MAX_LEN 64

uint8_t mbox_local_buf[MBOX_BUFFER_LEN_MAX];
BufferHeader *outbound, *inbound;
volatile u8 blockDeqSema;
volatile u8 blockFifoSema;

const ComponentId hlAppId = {.data1 = 0x8E24FC73,
                             .data2 = 0xA329,
                             .data3 = 0x4D99,
                             .data4 = {0x9b, 0x37, 0x2e, 0xe5, 0x4d, 0xc7, 0x95, 0xe7},
                             .reserved_word = 0};

/* Bitmap for IRQ enable. bit_0 and bit_1 are used to communicate with HL_APP */
uint32_t mbox_irq_status = 0x3;
size_t payloadStart = 20; /* UUID 16B, Reserved 4B */

volatile bool refresh_data_trigger;
volatile bool update_dso_trigger;

struct os_gpt_int gpt0_int;
struct os_gpt_int gpt1_int;
// struct os_gpt_int gpt3_int;

/******************************************************************************/
/* Timers */
/******************************************************************************/
static const uint8_t gpt_task_scheduler = OS_HAL_GPT0;
static const uint8_t gpt_timer = OS_HAL_GPT2;

static const uint32_t gpt_task_scheduler_timer_val = 1; /* 1ms */

u32 mbox_shared_buf_size = 0;

static const uint8_t uart_port_num = OS_HAL_UART_ISU3;

/******************************************************************************/
/* Applicaiton Hooks */
/******************************************************************************/
/* Hook for "printf". */
extern "C" void _putchar(char character)
{
    // mtk_os_hal_uart_put_char(uart_port_num, character);
    // if (character == '\n')
    //     mtk_os_hal_uart_put_char(uart_port_num, '\r');
}

void read_dso(void)
{
    // roll the buffer -3 points so we can overwrite the last one
    numpy::roll(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, -3);

    float x, y, z;
    lsm6dso_read(&x, &y, &z);

    buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3] = x / 100.0f;
    buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2] = y / 100.0f;
    buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1] = z / 100.0f;
}

/******************************************************************************/
/* Functions */
/******************************************************************************/
int32_t i2c_write(int *fD, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (buf == NULL)
        return -1;

    if (len > (I2C_MAX_LEN - 1))
        return -1;

    i2c_tx_buf[0] = reg;
    if (buf && len)
        memcpy(&i2c_tx_buf[1], buf, len);
    mtk_os_hal_i2c_write(i2c_port_num, i2c_lsm6dso_addr, i2c_tx_buf, len + 1);
    return 0;
}

int32_t i2c_read(int *fD, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (buf == NULL)
        return -1;

    if (len > (I2C_MAX_LEN))
        return -1;

    mtk_os_hal_i2c_write_read(i2c_port_num, i2c_lsm6dso_addr,
                              &reg, i2c_rx_buf, 1, len);
    memcpy(buf, i2c_rx_buf, len);
    return 0;
}

void i2c_enum(void)
{
    uint8_t i;
    uint8_t data;

    printf("[ISU%d] Enumerate I2C Bus, Start\n", i2c_port_num);
    for (i = 0; i < 0x80; i += 2)
    {
        printf("[ISU%d] Address:0x%02X, ", i2c_port_num, i);
        if (mtk_os_hal_i2c_read(i2c_port_num, i, &data, 1) == 0)
            printf("Found 0x%02X\n", i);
    }
    printf("[ISU%d] Enumerate I2C Bus, Finish\n\n", i2c_port_num);
}

int i2c_init(void)
{
    /* Allocate I2C buffer */
    i2c_tx_buf = (uint8_t *)malloc(I2C_MAX_LEN);
    i2c_rx_buf = (uint8_t *)malloc(I2C_MAX_LEN);
    if (i2c_tx_buf == NULL || i2c_rx_buf == NULL)
    {
        printf("Failed to allocate I2C buffer!\n");
        return -1;
    }

    /* MT3620 I2C Init */
    mtk_os_hal_i2c_ctrl_init(i2c_port_num);
    mtk_os_hal_i2c_speed_init(i2c_port_num, i2c_speed);

    return 0;
}

static void send_intercore_msg(size_t length)
{
    uint32_t dataSize;

    // copy high level appid to first 20 bytes
    memcpy((void *)mbox_local_buf, &hlAppId, sizeof(hlAppId));
    dataSize = payloadStart + length;

    EnqueueData(inbound, outbound, mbox_shared_buf_size, mbox_local_buf, dataSize);
}

static void process_inbound_message()
{
    static int msgId = 0;
    u32 mbox_local_buf_len;
    int result;
    INTER_CORE_BLOCK *in_data;
    INTER_CORE_BLOCK *out_data;

    mbox_local_buf_len = MBOX_BUFFER_LEN_MAX;
    result =
        DequeueData(outbound, inbound, mbox_shared_buf_size, mbox_local_buf, &mbox_local_buf_len);

    if (result == 0 && mbox_local_buf_len > payloadStart)
    {

        in_data = (INTER_CORE_BLOCK *)(mbox_local_buf + payloadStart);

        switch (in_data->cmd)
        {

        case IC_ECHO:
            out_data = (INTER_CORE_BLOCK *)(mbox_local_buf + payloadStart);

            out_data->msgId = msgId++;
            memcpy(out_data->message, in_data->message, sizeof(in_data->message));

            send_intercore_msg(sizeof(INTER_CORE_BLOCK));
            break;
        default:
            break;
        }
    }
}

static void task_scheduler(void *cb_data)
{
    static size_t refresh_data_tick_counter = SIZE_MAX;

    // read the dso every millisecond
    

    if (refresh_data_tick_counter++ >= 2000) // 2 seconds
    {
        refresh_data_tick_counter = 0;
        refresh_data_trigger = true;
        update_dso_trigger = true;
    }
}

extern "C" _Noreturn void RTCoreMain(void)
{

    /* Init Vector Table */
    NVIC_SetupVectorTable();

    volatile bool wait = true;
    while (wait)
        ;

    /* Init I2C Master/Slave */
    mtk_os_hal_i2c_ctrl_init(i2c_port_num);

    /* Enumerate I2C Bus*/
    i2c_enum();

    /* MT3620 I2C Init */
    if (i2c_init())
    {
        printf("failed to init i2c\n");
    }

    /* LSM6DSO Init */
    if (lsm6dso_init((void *)i2c_write, (void *)i2c_read))
    {
        printf("failed to init lsm6dso\n");
    }

    /* Init GPT */
    gpt0_int.gpt_cb_hdl = task_scheduler;
    gpt0_int.gpt_cb_data = NULL;

    gpt1_int.gpt_cb_hdl = NULL;
    gpt1_int.gpt_cb_data = NULL;

    mtk_os_hal_gpt_init();

    /* configure GPT0 clock speed (as 1KHz) */
    /* and register GPT0 user interrupt callback handle and user data. */
    mtk_os_hal_gpt_config((gpt_num)gpt_task_scheduler, false, &gpt0_int);

    // Configure GPT2 (in free-run mode, as 1KHz):
    mtk_os_hal_gpt_config((gpt_num)gpt_timer, false, NULL);

    /* configure GPT0 timeout as 1ms and repeat mode. */
    mtk_os_hal_gpt_reset_timer((gpt_num)gpt_task_scheduler, gpt_task_scheduler_timer_val, true);

    /* configure GPT2 timeout as 1ms and free-run mode. Note, auto-repeat is ignored */
    mtk_os_hal_gpt_reset_timer((gpt_num)gpt_timer, gpt_task_scheduler_timer_val, true);

    initialise_intercore_comms();

    /* start timer */
    mtk_os_hal_gpt_start((gpt_num)gpt_task_scheduler);
    mtk_os_hal_gpt_start((gpt_num)gpt_timer);

    for (;;)
    {

        if (blockDeqSema > 0)
        {
            process_inbound_message();
        }

        if (refresh_data_trigger)
        {
            refresh_data_trigger = false;
            // refresh_data();
        }

        if (update_dso_trigger){
            update_dso_trigger = false;
            read_dso();
        }
    }
}
