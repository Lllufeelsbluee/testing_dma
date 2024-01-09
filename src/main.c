#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"

#define I2S_NUM         (0)
#define DMA_BUF_COUNT   (2)
#define DMA_BUF_LEN     (1024)

static QueueHandle_t i2s_event_queue;

static void IRAM_ATTR i2s_isr_callback(void* arg)
{
    i2s_event_t event;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    event.type = I2S_EVENT_RX_DONE;
    xQueueSendFromISR(i2s_event_queue, &event, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

static void i2s_data_handler_task(void* arg)
{
    while (1) {
        i2s_event_t event;
        if (xQueueReceive(i2s_event_queue, &event, portMAX_DELAY)) {
            if (event.type == I2S_EVENT_RX_DONE) {
                // Handle I2S data reception here
                size_t bytes_read;
                char* data = malloc(DMA_BUF_LEN);
                i2s_read(I2S_NUM, data, DMA_BUF_LEN, &bytes_read, portMAX_DELAY);
                // Process data here
                free(data);
            }
        }
    }
}
void i2s_isr_register(
    i2s_port_t i2s_num,
    void (*isr_handler)(void*),
    void* arg,
    int intr_alloc_flags,
    intr_handle_t* handle
);

void app_main()
{
    i2s_config_t i2s_config = 
    {
        .mode = I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN,
        .sample_rate = 44100,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_MSB,
        .dma_buf_count = DMA_BUF_COUNT,
        .dma_buf_len = DMA_BUF_LEN,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    
    i2s_driver_install(I2S_NUM, &i2s_config, 1, &i2s_event_queue);
    i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_0);
    i2s_adc_enable(I2S_NUM);

    i2s_isr_register(I2S_NUM, i2s_isr_callback, NULL, ESP_INTR_FLAG_LEVEL1, NULL);

    xTaskCreate(i2s_data_handler_task, "i2s_data_handler_task", 2048, NULL, 1, NULL);
}

