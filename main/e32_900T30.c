#include "e32_900t30d.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "E32-900T30D"

// Konfiguration der Pins
#define E32_M0_GPIO    10
#define E32_M1_GPIO    11
#define E32_AUX_GPIO   14
#define E32_TXD_GPIO   13
#define E32_RXD_GPIO   12

// UART Konfiguration
#define E32_UART_PORT  UART_NUM_1
#define BUF_SIZE       1024

void e32_set_mode(uint8_t m0, uint8_t m1) {
    gpio_set_level(E32_M0_GPIO, m0);
    gpio_set_level(E32_M1_GPIO, m1);
    vTaskDelay(pdMS_TO_TICKS(50)); // Warten bis Moduswechsel abgeschlossen
}

static e32_receive_callback_t user_callback = NULL;

static void e32_receive_task(void *pvParameters) {
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
    while (1) {
        int len = uart_read_bytes(E32_UART_PORT, data, BUF_SIZE, pdMS_TO_TICKS(500));
        if (len > 0 && user_callback != NULL) {
            user_callback(data, len);
        }
    }
    free(data);
    vTaskDelete(NULL);
}


void e32_init() {
    // GPIOs initialisieren
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << E32_M0_GPIO) | (1ULL << E32_M1_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // AUX Pin als Input konfigurieren (optional)
    io_conf.pin_bit_mask = (1ULL << E32_AUX_GPIO);
    io_conf.mode = GPIO_MODE_INPUT;
    gpio_config(&io_conf);

    // UART initialisieren
    uart_config_t uart_config = {
        .baud_rate = 9600, // Standard-Baudrate des Moduls
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(E32_UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(E32_UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(E32_UART_PORT, E32_TXD_GPIO, E32_RXD_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Normal Mode (M0=0, M1=0)
    e32_set_mode(0, 0);
    
    ESP_LOGI(TAG, "E32-900T30D initialisiert");
}
void e32_send_data(const uint8_t *data, size_t len) {
    uart_write_bytes(E32_UART_PORT, (const char *)data, len);
    ESP_LOGI(TAG, "%d Bytes gesendet", len);
}

void e32_set_receive_callback(e32_receive_callback_t callback) {
    user_callback = callback;
}

void e32_configure(void) {
    e32_set_mode(1, 1); // Sleep Mode
    vTaskDelay(pdMS_TO_TICKS(100));
    
    uint8_t config_cmd[6] = {0xC0, 0x00, 0x08, 0x00, 0x00, 0x44};
    e32_send_data(config_cmd, sizeof(config_cmd));
    
    e32_set_mode(0, 0); // Zurück zu Normal Mode
}