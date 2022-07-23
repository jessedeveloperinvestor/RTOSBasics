
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/uart.h"

/*

#include <stdio.h>  // Entradas e saídas do C
#include <string.h>
#include <stdlib.h>  // iterações do C
#include <math.h>  // lib math

#include "freertos/FreeRTOS.h"  // Incluindo RTOS
#include "freertos/task.h"  // Criar tasks e etc
#include "freertos/queue.h" // queue em RTOS
#include "driver/gpio.h"   // inportando gpio
#include "driver/uart.h"  // terminal de monitoramento

*/
//================== LED ===========================//

#define LED_ESP32 (GPIO_NUM_14) // PINO LED

//================== UART ===========================//

#define TXD_UART1 (GPIO_NUM_1) // PINO TX UART1 = GPIO1
#define RXD_UART1 (GPIO_NUM_3) // PINO RX UART1 = GPIO3
#define RTS_UART (UART_PIN_NO_CHANGE)
#define CTS_UART (UART_PIN_NO_CHANGE)
#define BUF_SIZE 500 // Alocacao do buffer do uart na RAM

TaskHandle_t xLedHandle = NULL;

QueueHandle_t QUEUE_DATA;

void xLED(void *arg)
{

    bool led = false;
    char rx = 0;
    uint32_t CMD = 0;

    while (1) {
        CMD = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(portMAX_DELAY));

        xQueuePeek(QUEUE_DATA, &rx, pdMS_TO_TICKS(0));

        if(rx == 'A') {
            led = true;
        } else if(rx == 'B') {
            led = false;
        }
        gpio_set_level(LED_ESP32, led);

        rx = 0;

    }
}
void xUART(void *arg)
{

    uint8_t *data = (uint8_t*) malloc(BUF_SIZE);

    uart_write_bytes(UART_NUM_1, "\n\rola setor P&D\n\r", 17);
    uart_write_bytes(UART_NUM_1, "\r\n", 2); // Final do Frame
    vTaskDelay(100 / portTICK_PERIOD_MS);

    while (1)
    {

        vTaskDelay(pdMS_TO_TICKS(500));

        int RxUART = uart_read_bytes(UART_NUM_1, data, BUF_SIZE, pdMS_TO_TICKS(10));

        if (RxUART > 0)
        {

            uart_write_bytes(UART_NUM_1, (const char *)data, RxUART);
            if (data[0] == 'A' || data[0] == 'B') {
                xQueueOverwrite(QUEUE_DATA, &data[0]);
                xTaskNotify(xLedHandle, 2, eSetValueWithOverwrite);
            }
        }
    }
}

void app_main()
{

    uart_config_t uart1_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, //
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

    uart_param_config(UART_NUM_1, &uart1_config);
    uart_set_pin(UART_NUM_1, TXD_UART1, RXD_UART1, RTS_UART, CTS_UART);
    uart_driver_install(UART_NUM_1, BUF_SIZE, 0, 0, NULL, 0);

    //================================GPIO LED=================================//

    gpio_config_t io_conf;                     // Ponteiro de configuração dos pinos
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE; // Desabilita Interrupçoes
    io_conf.mode = GPIO_MODE_OUTPUT;           // set as output mode
    io_conf.pin_bit_mask = (1ULL << LED_ESP32);
    io_conf.pull_down_en = 0; // Desabilita modo PULL-DOWN
    io_conf.pull_up_en = 0;   // Desabilita modo PULL-UP
    gpio_config(&io_conf);    // Configura GPIO com as configurações para OUTPUT

    //==========================================================================//

    //========================criação da queue==================================//

    QUEUE_DATA = xQueueCreate(1, sizeof(char));

    //==========================================================================//

    //=========================CRIAÇÃO DE TASK==================================//

    xTaskCreatePinnedToCore(xUART, "xUART", 2000, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(xLED, "xLED", 2000, NULL, 10, &xLedHandle, 1);

    //==========================================================================//
}