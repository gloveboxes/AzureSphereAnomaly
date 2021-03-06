#include <cstdio>
#include <stdio.h>

#include "intercore_contract.h"

#include "FreeRTOS.h"
#include "task.h"
#include <semphr.h>
#include "printf.h"
#include "mt3620.h"
#include "ctype.h"

#include "os_hal_gpio.h"
#include "os_hal_uart.h"
#include "os_hal_i2c.h"
#include "os_hal_mbox.h"
#include "os_hal_mbox_shared_mem.h"

#include "lsm6dso_driver.h"
#include "lsm6dso_reg.h"

#include "ei_run_classifier.h"

void *__dso_handle = (void *)&__dso_handle;

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_ACCELEROMETER
#error "Incompatible model, this example is only compatible with accelerometer data"
#endif

/******************************************************************************/
/* Configurations */
/******************************************************************************/
/* Intercore */

/* Maximum mailbox buffer len.
 *    Maximum message len: 1024B
 *                         1024 is the maximum value when HL_APP invoke send().
 *    Component UUID len : 16B
 *    Reserved data len  : 4B
 */
#define MBOX_BUFFER_LEN_MAX 1044

INTERCORE_BLOCK intercore_block;

/* Bitmap for IRQ enable. bit_0 and bit_1 are used to communicate with HL_APP */
static const uint32_t mbox_irq_status = 0x3;

SemaphoreHandle_t blockDeqSema;
SemaphoreHandle_t blockFifoSema;
SemaphoreHandle_t blockFifoSema_M4;
SemaphoreHandle_t SwIntSema_M4;
static const u32 pay_load_start_offset = 20; /* UUID 16B, Reserved 4B */

u8 *mbox_buf;
u32 allocated_buf_size;
u32 mbox_buf_len;
BufferHeader *outbound, *inbound;
u32 shared_buf_size;
size_t payloadStart = 20; /* UUID 16B, Reserved 4B */

/// <summary>
///     When sending a message, this is the recipient HLApp's component ID.
///     When receiving a message, this is the sender HLApp's component ID.
/// </summary>
typedef struct
{
    /// <summary>4-byte little-endian word</summary>
    uint32_t data1;
    /// <summary>2-byte little-endian half</summary>
    uint16_t data2;
    /// <summary>2-byte little-endian half</summary>
    uint16_t data3;
    /// <summary>2 bytes (big-endian) followed by 6 bytes (big-endian)</summary>
    uint8_t data4[8];
    /// <summary> 8 bytes reserved </summary>
    uint32_t reserved_word;
} ComponentId;

static const ComponentId hlAppId = {.data1 = 0x25025d2c,
                                    .data2 = 0x66da,
                                    .data3 = 0x4448,
                                    .data4 = {0xba, 0xe1, 0xac, 0x26, 0xfc, 0xdd, 0x36, 0x27},
                                    .reserved_word = 0};

/* UART */
static const UART_PORT uart_port_num = OS_HAL_UART_ISU0;

/* GPIO */
static const os_hal_gpio_pin gpio_led_red = OS_HAL_GPIO_8;
static const os_hal_gpio_pin gpio_led_green = OS_HAL_GPIO_9;

/* I2C */
static const i2c_num i2c_port_num = OS_HAL_I2C_ISU2;
static const i2c_speed_kHz i2c_speed = I2C_SCL_50kHz;
static const uint8_t i2c_lsm6dso_addr = LSM6DSO_I2C_ADD_L >> 1;
static uint8_t *i2c_tx_buf;
static uint8_t *i2c_rx_buf;

#define I2C_MAX_LEN 64
#define APP_STACK_SIZE_BYTES 1024 / 4

// Edge Impulse
static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
static float inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};

// To prevent false positives we smoothen the results, with readings=10 and time_between_readings=200
// we look at 2 seconds of data + (length of window (e.g. also 2 seconds)) for the result

// We use N number of readings to smoothen the results over
#define SMOOTHEN_OVER_READINGS 10
// Time between readings in milliseconds
#define SMOOTHEN_TIME_BETWEEN_READINGS 200

/******************************************************************************/
/* Application Hooks */
/******************************************************************************/
/* Hook for "stack over flow". */
extern "C" void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    printf("%s: %s\n", __func__, pcTaskName);
}

/* Hook for "memory allocation failed". */
extern "C" void vApplicationMallocFailedHook(void)
{
    printf("%s\n", __func__);
}

/* Hook for "printf". */
extern "C" void _putchar(char character)
{
    mtk_os_hal_uart_put_char(uart_port_num, character);
    if (character == '\n')
        mtk_os_hal_uart_put_char(uart_port_num, '\r');
}

/******************************************************************************/
/* Functions */
/******************************************************************************/
/* Mailbox Fifo Interrupt handler.
 * Mailbox Fifo Interrupt is triggered when mailbox fifo been R/W.
 *     data->event.channel: Channel_0 for A7, Channel_1 for the other M4.
 *     data->event.ne_sts: FIFO Non-Empty.interrupt
 *     data->event.nf_sts: FIFO Non-Full interrupt
 *     data->event.rd_int: Read FIFO interrupt
 *     data->event.wr_int: Write FIFO interrupt
 */
void mbox_fifo_cb(struct mtk_os_hal_mbox_cb_data *data)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (data->event.channel == OS_HAL_MBOX_CH0)
    {
        /* A7 core write data to mailbox fifo. */
        if (data->event.wr_int)
        {
            xSemaphoreGiveFromISR(blockFifoSema, &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }
}

/* SW Interrupt handler.
 * SW interrupt is triggered when:
 *    1. A7 read/write the shared memory.
 *    2. The other M4 triggers SW interrupt.
 *     data->swint.swint_channel: Channel_0 for A7, Channel_1 for the other M4.
 *     Channel_0:
 *         data->swint.swint_sts bit_0: A7 read data from mailbox
 *         data->swint.swint_sts bit_1: A7 write data to mailbox
 *     Channel_1:
 *         data->swint.swint_sts bit_0: M4 sw interrupt
 */
void mbox_swint_cb(struct mtk_os_hal_mbox_cb_data *data)
{
    BaseType_t higher_priority_task_woken = pdFALSE;

    if (data->swint.channel == OS_HAL_MBOX_CH0)
    {
        if (data->swint.swint_sts & (1 << 1))
        {
            xSemaphoreGiveFromISR(blockDeqSema, &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }
}

void mbox_print_and_convert_buf(u8 *mbox_buf, u32 mbox_data_len)
{
    u32 payload_len;
    u32 i;

    printf("\n\nCM4_A receives message from CA7 (%d bytes):\n", mbox_data_len);
    printf("  Component Id (16 bytes): %02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X\n",
           mbox_buf[3], mbox_buf[2], mbox_buf[1], mbox_buf[0],
           mbox_buf[5], mbox_buf[4], mbox_buf[7], mbox_buf[6],
           mbox_buf[8], mbox_buf[9], mbox_buf[10], mbox_buf[11],
           mbox_buf[12], mbox_buf[13], mbox_buf[14], mbox_buf[15]);

    /* Print reserved field as little-endian 4-byte integer. */
    printf("  Reserved (4 bytes): 0x%02X %02X %02X %02X\n", mbox_buf[19], mbox_buf[18], mbox_buf[17], mbox_buf[16]);

    /* Print message as hex. */
    payload_len = mbox_data_len - pay_load_start_offset;
    printf("  Payload (%d bytes as hex): ", payload_len);
    for (i = pay_load_start_offset; i < mbox_data_len; ++i)
        printf("0x%02X ", mbox_buf[i]);
    printf("\n");

    /* Print message as text. */
    printf("  Payload (%d bytes as text): ", payload_len);
    for (i = pay_load_start_offset; i < mbox_data_len; ++i)
        printf("%c", mbox_buf[i]);
    printf("\n");

    /* Convert payload, upper to lower, lower to upper.*/
    for (i = pay_load_start_offset; i < mbox_data_len; ++i)
    {
        if (isupper(mbox_buf[i]))
            mbox_buf[i] = tolower(mbox_buf[i]);
        else if (islower(mbox_buf[i]))
            mbox_buf[i] = toupper(mbox_buf[i]);
    }

    printf("CM4_A sends message to CA7:\n");
    printf("  Payload (%d bytes as text): ", payload_len);
    for (i = pay_load_start_offset; i < mbox_data_len; ++i)
        printf("%c", mbox_buf[i]);
    printf("\n");
}

void InitaliseIntercore(void)
{
    struct mbox_fifo_event mask;

    printf("MBOX_A Task Started\n");

    blockDeqSema = xSemaphoreCreateBinary();
    blockFifoSema = xSemaphoreCreateBinary();
    blockFifoSema_M4 = xSemaphoreCreateBinary();
    SwIntSema_M4 = xSemaphoreCreateBinary();

    /* Register interrupt callback */
    mask.channel = OS_HAL_MBOX_CH0;
    mask.ne_sts = 0; /* FIFO Non-Empty interrupt */
    mask.nf_sts = 0; /* FIFO Non-Full interrupt */
    mask.rd_int = 0; /* Read FIFO interrupt */
    mask.wr_int = 1; /* Write FIFO interrupt */
    mtk_os_hal_mbox_fifo_register_cb(OS_HAL_MBOX_CH0, mbox_fifo_cb, &mask);
    mtk_os_hal_mbox_sw_int_register_cb(OS_HAL_MBOX_CH0, mbox_swint_cb, mbox_irq_status);

    /* Get mailbox shared buffer size, defined by Azure Sphere OS. */
    if (GetIntercoreBuffers(&outbound, &inbound, &shared_buf_size) == -1)
    {
        printf("GetIntercoreBuffers failed\n");
        return;
    }

    /* Allocate the M4 buffer for mailbox communication */
    allocated_buf_size = shared_buf_size;
    if (allocated_buf_size > MBOX_BUFFER_LEN_MAX)
        allocated_buf_size = MBOX_BUFFER_LEN_MAX;
    mbox_buf = (u8 *)pvPortMalloc(allocated_buf_size);
    if (mbox_buf == NULL)
    {
        printf("pvPortMalloc failed\n");
        return;
    }

    // printf("shared buf size = %d\n", shared_buf_size);
    // printf("allocated buf size = %d\n", allocated_buf_size);
}

// void MBOXTask_A(void *pParameters)
// {
//     int result;

//     InitaliseIntercore();

//     while (1)
//     {
//         // vTaskDelay(pdMS_TO_TICKS(10));

//         /* Handle A7 <--> M4 Communication */
//         /* Init buffer */
//         mbox_buf_len = allocated_buf_size;
//         memset(mbox_buf, 0, allocated_buf_size);

//         /* Read from A7, dequeue from mailbox */
//         result = DequeueData(outbound, inbound, shared_buf_size, mbox_buf, &mbox_buf_len);
//         if (result == -1 || mbox_buf_len < pay_load_start_offset)
//         {
//             xSemaphoreTake(blockDeqSema, portMAX_DELAY);
//             continue;
//         }

//         // send_intercore_msg((void*)"hello world", 11);

//         // /* Print the received message and convert it.*/
//         // mbox_print_and_convert_buf(mbox_buf, mbox_buf_len);

//         // /* Write to A7, enqueue to mailbox */
//         // EnqueueData(inbound, outbound, shared_buf_size, mbox_buf, mbox_buf_len);

//         ///* Wait for M4 SW interrupt 0 */
//         // xSemaphoreTake(SwIntSema_M4, portMAX_DELAY);
//         // printf("CM4_A received SW interrupt from CM4_B\n");
//     }
// }

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
    i2c_tx_buf = (uint8_t *)pvPortMalloc(I2C_MAX_LEN);
    i2c_rx_buf = (uint8_t *)pvPortMalloc(I2C_MAX_LEN);
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

static void send_intercore_msg(INTERCORE_BLOCK *data, size_t length)
{
    uint32_t dataSize;
    memcpy((void *)mbox_buf, &hlAppId, sizeof(hlAppId)); // copy high level appid to first 20 bytes
    memcpy(mbox_buf + payloadStart, data, length);
    dataSize = payloadStart + length;
    EnqueueData(inbound, outbound, shared_buf_size, mbox_buf, dataSize);
}

void update_intercore(const char *prediction)
{
    static char previous_prediction[64] = {0};

    if (strcmp(prediction, "uncertain") != 0)
    {
        strcpy(previous_prediction, prediction);
        // memset(&intercore_block, 0x00, sizeof(INTERCORE_BLOCK));
        intercore_block.cmd = IC_PREDICTION;
        strcpy(intercore_block.PREDICTION, prediction);

        send_intercore_msg(&intercore_block, sizeof(intercore_block));
    }
}

void inference_task(void *pParameters)
{
    InitaliseIntercore();

    // struct that smoothens out the readings over time, to avoid misclassification if a single frame
    // happened to overlap with one of the classes in our training set
    ei_classifier_smoothen_t smoothen;

    ei_classifier_smoothen_init(&smoothen, SMOOTHEN_OVER_READINGS, SMOOTHEN_OVER_READINGS * 0.7,
                                0.8 /* confidence */, 0.3 /* max anomaly score */);

    static bool first_reading = true;

    // wait until we have a full frame of data
    vTaskDelay(pdMS_TO_TICKS((EI_CLASSIFIER_INTERVAL_MS * EI_CLASSIFIER_RAW_SAMPLE_COUNT)));
    mtk_os_hal_gpio_set_output(gpio_led_green, OS_HAL_GPIO_DATA_LOW);

    while (1)
    {
        
        // copy into working buffer (other buffer is used by other task)
        memcpy(inference_buffer, buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE * sizeof(float));

        // Turn the raw buffer in a signal which we can the classify
        signal_t signal;
        int err = numpy::signal_from_buffer(inference_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0)
        {
            ei_printf("Failed to create signal from buffer (%d)\n", err);
            return;
        }

        ei_impulse_result_t result = {0};

        // invoke the impulse
        EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
        if (res != 0)
        {
            printf("run_classifier returned: %d\n", res);
            return;
        }

        if (first_reading)
        {
            printf("Timing = (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)\n",
                   result.timing.dsp, result.timing.classification, result.timing.anomaly);
            first_reading = false;
        }

        const char *prediction = ei_classifier_smoothen_update(&smoothen, &result);
        // printf("%s", prediction);

        update_intercore(prediction);

        // printf(" [ ");
        // for (size_t ix = 0; ix < smoothen.count_size; ix++)
        // {
        //     printf("%u", smoothen.count[ix]);
        //     if (ix != smoothen.count_size + 1)
        //     {
        //         printf(", ");
        //     }
        // }
        // printf("]\n");

        vTaskDelay(pdMS_TO_TICKS(SMOOTHEN_TIME_BETWEEN_READINGS));
    }

    ei_classifier_smoothen_free(&smoothen);
}

void i2c_task(void *pParameters)
{
    /* Enumerate I2C Bus*/
    i2c_enum();

    /* MT3620 I2C Init */
    if (i2c_init())
        return;

    /* LSM6DSO Init */
    if (lsm6dso_init((void *)i2c_write, (void *)i2c_read))
        return;

    // Create inference task
    xTaskCreate(inference_task, "Inferencing Task", APP_STACK_SIZE_BYTES * 4, NULL, 2, NULL);

    mtk_os_hal_gpio_set_output(gpio_led_red, OS_HAL_GPIO_DATA_LOW);

    while (1)
    {
        // roll the buffer -3 points so we can overwrite the last one
        numpy::roll(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, -3);

        float x, y, z;
        lsm6dso_read(&x, &y, &z);

        buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3] = x / 100.0f;
        buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2] = y / 100.0f;
        buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1] = z / 100.0f;
    }
}

extern "C" _Noreturn void RTCoreMain(void)
{
    /* Setup Vector Table */
    NVIC_SetupVectorTable();

    /* Init UART */
    mtk_os_hal_uart_ctlr_init(uart_port_num);

    /* Init GPIO */
    mtk_os_hal_gpio_set_direction(gpio_led_red, OS_HAL_GPIO_DIR_OUTPUT);
    mtk_os_hal_gpio_set_direction(gpio_led_green, OS_HAL_GPIO_DIR_OUTPUT);

    mtk_os_hal_gpio_set_output(gpio_led_red, OS_HAL_GPIO_DATA_HIGH);
    mtk_os_hal_gpio_set_output(gpio_led_green, OS_HAL_GPIO_DATA_HIGH);

    printf("\nFreeRTOS I2C LSM6DSO Demo %d\n", 1337);

    /* Init I2C Master/Slave */
    mtk_os_hal_i2c_ctrl_init(i2c_port_num);

    /* Open the MBOX channel of A7 <-> M4 */
    mtk_os_hal_mbox_open_channel(OS_HAL_MBOX_CH0);

    /* Create I2C Master/Slave Task */
    xTaskCreate(i2c_task, "I2C Task", APP_STACK_SIZE_BYTES, NULL, 4, NULL);

    vTaskStartScheduler();
    for (;;)
        __asm__("wfi");
}
