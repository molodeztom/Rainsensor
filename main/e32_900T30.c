#include "e32_900t30d.h"
#include "driver/uart.h"
#include "driver/gpio.h"
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

#define CONFIG_CMD_LEN 6
#define RESPONSE_LEN   6

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
  
    
    ESP_LOGI(TAG, "E32-900T30D initialisiert");
}


// Daten senden
esp_err_t e32_send_data(const uint8_t *data, size_t len) {
    int bytes_written = uart_write_bytes(E32_UART_PORT, (const char *)data, len);
    return (bytes_written == len) ? ESP_OK : ESP_FAIL;
}

void e32_set_receive_callback(e32_receive_callback_t callback) {
    user_callback = callback;
}

void e32_configure(void) {
 // 1. In den Konfigurationsmodus wechseln (Sleep Mode)
 e32_set_mode(1, 1);
    
 // 2. Warten bis Moduswechsel abgeschlossen ist
 // (Das Modul benötigt typisch 30-100ms)
 vTaskDelay(pdMS_TO_TICKS(100));
 
 // 3. Konfigurationsbefehl senden
 // Byte 0: C0 = Write Configuration
 // Byte 1-5: Konfigurationsparameter
 uint8_t config_cmd[6] = {
     0xC0,    // Write Configuration Command
     0x00,    // Address High Byte
     0x08,    // Baudrate/Parity/Air Rate
     0x00,    // Channel
     0x00,    // Option Flags
     0x44     // CRC Checksum
 };
 

     // Configuration Command senden (0xC1)
     if (e32_send_data(config_cmd, sizeof(config_cmd)) != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Senden des Konfigurationsbefehls");
        return;
    }
 
 // 4. Kurz warten bis Befehl verarbeitet wurde
 vTaskDelay(pdMS_TO_TICKS(50));
 
 // 5. Zurück in den Normalmodus
 e32_set_mode(0, 0);
 ESP_LOGI(TAG, "E32-900T30D konfiguriert");

}



// Daten empfangen mit Timeout
esp_err_t e32_receive_data(uint8_t *buffer, size_t len, uint32_t timeout_ms) {
    int bytes_read = uart_read_bytes(E32_UART_PORT, buffer, len, pdMS_TO_TICKS(timeout_ms));
    return (bytes_read == len) ? ESP_OK : ESP_FAIL;
}

// Parameter auslesen und anzeigen
void e32_read_and_display_parameters() {

    uint8_t read_cmd[CONFIG_CMD_LEN] = {0xC1, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t response[RESPONSE_LEN] = {0};
    
    // 1. In den Konfigurationsmodus wechseln
    e32_set_mode(1, 1);
    vTaskDelay(pdMS_TO_TICKS(150)); // Längere Verzögerung
    
    // 2. AUX-Pin Status prüfen (wenn angeschlossen)
    #ifdef E32_AUX_GPIO
    int retries = 0;
    while (gpio_get_level(E32_AUX_GPIO) == 0 && retries++ < 10) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (retries >= 10) {
        ESP_LOGE(TAG, "AUX-Pin nicht bereit - Modul antwortet nicht");
        return;
    }
    #endif
    
    // 3. UART-Puffer leeren
    uint8_t dummy;
    while (uart_read_bytes(E32_UART_PORT, &dummy, 1, 20 / portTICK_PERIOD_MS) > 0) {}
    
    // 4. Read Configuration Command senden (0xC1)
    ESP_ERROR_CHECK(e32_send_data(read_cmd, CONFIG_CMD_LEN));
    vTaskDelay(pdMS_TO_TICKS(50)); // Warten bis Befehl verarbeitet
    
  
      // Antwort empfangen
      esp_err_t result = e32_receive_data(response, RESPONSE_LEN, 200);
      if (result != ESP_OK) {
        ESP_LOGE(TAG, "Fehler beim Empfang der Konfiguration");
        handle_e32_error(result, "Konfigurationsantwort");
        return;
    }

    
  
    
    // 6. Zurück zum Normalmodus
    e32_set_mode(0, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
  
        
    // Parameter dekodieren und anzeigen
    ESP_LOGI(TAG, "Aktuelle E32-900T30D Konfiguration:");
    ESP_LOGI(TAG, "----------------------------------");
    
    // 1. Byte: HEAD (should be 0xC1 for response)
    ESP_LOGI(TAG, "HEAD: 0x%02X", response[0]);
    
    // 2. Byte: ADDH (Address high byte)
    ESP_LOGI(TAG, "ADDH (Address High): 0x%02X", response[1]);
    
    // 3. Byte: ADDL (Address low byte)
    ESP_LOGI(TAG, "ADDL (Address Low): 0x%02X", response[2]);
    
    // 4. Byte: SPEED (SPED)
    uint8_t speed = response[3];
    ESP_LOGI(TAG, "SPED (Speed): 0x%02X", speed);
    ESP_LOGI(TAG, "  - UART Baudrate: %s", 
             (speed & 0x03) == 0x00 ? "1200" :
             (speed & 0x03) == 0x01 ? "2400" :
             (speed & 0x03) == 0x02 ? "4800" :
             (speed & 0x03) == 0x03 ? "9600" : "19200");
    ESP_LOGI(TAG, "  - UART Parity: %s", 
             ((speed >> 2) & 0x03) == 0x00 ? "8N1" :
             ((speed >> 2) & 0x03) == 0x01 ? "8O1" :
             ((speed >> 2) & 0x03) == 0x02 ? "8E1" : "8N1");
    ESP_LOGI(TAG, "  - Air Data Rate: %s kbps", 
             ((speed >> 4) & 0x07) == 0x00 ? "0.3" :
             ((speed >> 4) & 0x07) == 0x01 ? "1.2" :
             ((speed >> 4) & 0x07) == 0x02 ? "2.4" : "4.8/9.6/19.2");
    
    // 5. Byte: CHAN (Channel)
    uint8_t channel = response[4];
    float frequency = 902.0 + (channel * 0.125);
    ESP_LOGI(TAG, "CHAN: 0x%02X (%.3f MHz)", channel, frequency);
    
    // 6. Byte: OPTION
    uint8_t option = response[5];
    ESP_LOGI(TAG, "OPTION: 0x%02X", option);
    ESP_LOGI(TAG, "  - Transmission Mode: %s", 
             (option & 0x01) ? "Fixed" : "Transparent");
    ESP_LOGI(TAG, "  - IO Drive Mode: %s", 
             ((option >> 1) & 0x01) ? "Open collector" : "Push-pull");
    ESP_LOGI(TAG, "  - Wireless Wakeup: %d ms", 
             ((option >> 2) & 0x07) == 0 ? 250 :
             ((option >> 2) & 0x07) == 1 ? 500 :
             ((option >> 2) & 0x07) == 2 ? 750 :
             ((option >> 2) & 0x07) == 3 ? 1000 :
             ((option >> 2) & 0x07) == 4 ? 1250 :
             ((option >> 2) & 0x07) == 5 ? 1500 :
             ((option >> 2) & 0x07) == 6 ? 1750 : 2000);
    ESP_LOGI(TAG, "  - FEC: %s", 
             ((option >> 5) & 0x01) ? "Enabled" : "Disabled");
    ESP_LOGI(TAG, "  - Transmission Power: %s", 
             ((option >> 6) & 0x03) == 0x00 ? "30dBm" :
             ((option >> 6) & 0x03) == 0x01 ? "27dBm" :
             ((option >> 6) & 0x03) == 0x02 ? "24dBm" : "21dBm");
    
    ESP_LOGI(TAG, "----------------------------------");
}

void handle_e32_error(esp_err_t err, const char *operation) {
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "%s erfolgreich", operation);
        return;
    }
    
    // Speziell für UART-Fehler
    if (err == ESP_FAIL) {
        ESP_LOGE(TAG, "%s fehlgeschlagen (allgemeiner Fehler)", operation);
        return;
    }
    
    // Dekodierung des Fehlercodes
    const char *err_name = esp_err_to_name(err);
    const char *err_desc = "";
    
    // Benutzerdefinierte Fehlerbeschreibungen
    switch(err) {
        case ESP_ERR_INVALID_ARG:
            err_desc = "Ungültiges Argument";
            break;
        case ESP_ERR_TIMEOUT:
            err_desc = "Timeout aufgetreten";
            break;
        case ESP_ERR_NOT_SUPPORTED:
            err_desc = "Funktion nicht unterstützt";
            break;
        case ESP_ERR_NO_MEM:
            err_desc = "Kein Speicher verfügbar";
            break;
        default:
            err_desc = "Unbekannter Fehler";
    }
    
    ESP_LOGE(TAG, "Fehler bei %s: %s (0x%x) - %s", 
             operation, err_name, err, err_desc);
    
    // Bei kritischen Fehlern evtl. Neustart
    if (err == ESP_ERR_NO_MEM || err == ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Kritischer Fehler, Neustart...");
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }
}

