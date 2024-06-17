#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include <stdint.h>
#pragma pack(1)

#define TX_PIN 27
#define RX_PIN 25
#define BAUD_RATE 9600
#define BUF_SIZE (1024)

const uart_port_t uart_num = UART_NUM_2;
const char *TAG = "UART";
QueueHandle_t uart_queue;
uint8_t *dt;
int uart_retries = 0;
uint16_t crc_a = 0;
uint16_t crc_b = 0;

// typedef struct
// {
//     union
//     {
//         struct
//         {
//             uint8_t abnt_cmd;
//             uint8_t num_serie_msb;
//             uint8_t num_serie;
//             uint8_t num_serie_lsb;
//             uint32_t null_sector_1;
//             uint32_t null_sector_2;
//             uint32_t null_sector_3;
//             uint32_t null_sector_4;
//             uint32_t null_sector_5;
//             uint32_t null_sector_6;
//             uint32_t null_sector_7;
//             uint32_t null_sector_8;
//             uint32_t null_sector_9;
//             uint32_t null_sector_10;
//             uint32_t null_sector_11;
//             uint32_t null_sector_12;
//             uint32_t null_sector_13;
//             uint16_t null_sector_14;
//             uint16_t null_sector_15;
//             uint16_t null_sector_16;
//             uint8_t null_sector_17;
//             uint8_t crs_lsb;
//             uint8_t crs_msb;
//         };
//         uint8_t buf[65];  
//     };
// }data_to_send;

uint8_t ds[66];

// typedef struct
// {
//     union
//     {
//         struct
//         {
            
//         };
//         uint8_t buf[258];
//     };
// }data_to_receive;

// uint16_t ax25crc16(unsigned char *data_p, uint16_t length) {
//     uint16_t crc = 0xFFFF;
//     uint16_t crc16_table[] = {
//             0x0000, 0x1081, 0x2102, 0x3183,
//             0x4204, 0x5285, 0x6306, 0x7387,
//             0x8408, 0x9489, 0xa50a, 0xb58b,
//             0xc60c, 0xd68d, 0xe70e, 0xf78f
//     };

//     while(length--){
//         crc = ( crc >> 4 ) ^ crc16_table[(crc & 0xf) ^ (*data_p & 0xf)];
//         crc = ( crc >> 4 ) ^ crc16_table[(crc & 0xf) ^ (*data_p++ >> 4)];
//     }

//     //uint32_t data = crc;
//     //crc = (crc << 8) | (data >> 8 & 0xff); // do byte swap here that is needed by AX25 standard
//     return (~crc);
// }

uint16_t crc16arc_bit(uint16_t crc, void const *mem, size_t len) {
    unsigned char const *data = mem;
    if (data == NULL)
        return 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (unsigned k = 0; k < 8; k++) {
            crc = crc & 1 ? (crc >> 1) ^ 0xa001 : crc >> 1;
        }
    }

    uint32_t tmp = crc;
    crc = (crc << 8) | (tmp >> 8 & 0xff);
    return crc;
}

uint16_t replaceByte(uint16_t value, uint8_t b, uint16_t pos)
{
    return (value & ~(0xFF << (pos * 8))) | ((b & 0xFF) << (pos * 8));
}

void setup()
{
    dt = (uint8_t *)calloc(BUF_SIZE, sizeof(uint8_t));
    for(int i = 0; i < 64; i++)
    {
        switch (i)
        {
        case 0:
            ds[i] = 0x14;
            break;

        case 1:
            ds[i] = 0x12;
            break;

        case 2:
            ds[i] = 0x34;
            break;
                    
        case 3:
            ds[i] = 0x56;
            break;
        
        default:
            ds[i] = 0x00;
            break;
        }
    }
    
    uint16_t a = 0;
    a = crc16arc_bit(a, ds, 64);
    ds[64] = (a >> (1 * 8)) & 0xFF;
    ds[65] = (a >> (0 * 8)) & 0xFF;

    for(int i = 0; i < 66; i++)
    {
        if(i < 65)
        {
            printf("%x | ", ds[i]);
        }
        else
        {
            printf("%x\n", ds[i]);
        }
    }
}

void uart_t()
{
    //setup();
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
    intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, intr_alloc_flags));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_2, &uart_config));
    uart_set_line_inverse(UART_NUM_2, UART_SIGNAL_TXD_INV | UART_SIGNAL_RXD_INV);
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 18, 5, 4, 2));
    while(1)
    {
        uart_write_bytes(UART_NUM_2, (void *)ds, 66);
        int code = uart_read_bytes(UART_NUM_2, dt, (BUF_SIZE - 1), 100 / portTICK_PERIOD_MS);

        if(code > -1)
        {
            for(int i = 0; i < 258; i++)
            {
                if(i < 257)
                {
                    printf("%x | ", dt[i]);
                }
                else
                {
                    printf("%x\n", dt[i]);
                }
            }
            crc_b = replaceByte(crc_b, dt[256], 1);
            crc_b = replaceByte(crc_b, dt[257], 0);
            crc_a = crc16arc_bit(0, dt, 256);

            if(crc_a == crc_b)
            {
                ESP_LOGI(TAG, "Message validated!");
            }
            else
            {
                ESP_LOGE(TAG, "Message invalid");
            }
        }
        else
        {
            uart_retries++;
            ESP_LOGE(TAG,"Couldn't get UART message. num of retries: %i", uart_retries);
        }
        vTaskDelay(500);
    }
}

void app_main(void)
{
    setup();
    xTaskCreate(uart_t, "uart_task", 5000, NULL, 0, NULL);
}