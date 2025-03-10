/**************************************************************************
RainSensor


  Hardware:
  ESP32-S3-DevKitC-1 mit Wroom N16R8
  LoRa E32-900T30D connected M0 M1 and Rx Tx
  Hall Sensor with Adapter

  Try sending a message to remote LoRa
  Sleep for a certain time
  Put LoRa to sleep mode
  Setup LoRa to defined parameters
  Measure Temperature
  Send Temperature and Time Stamp

  History: master if not shown otherwise
  20240525  V0.1: Wakeup with timer and count boots in RTC memory (Copy from ESP32-S3 Test V0.2)
  20250109  V0.2: LED blinks on wakeup
  20250209  V0.3: Use GPIO8 as input for counter
  20250214  V0.4 Interrupt Test
  20250215  V0.5B: ISR registered usw.
  20250216  V0.5.1B: Zählt mit und ohne laufender CPU allerdings müsste man sicherheitshalber die überprüfungen in wake up wieder einschalten
  20250225  V0.5.2B: Stack Size erhöht, unnötigen Code entfernt. Funktioniert so gut. 
  20250227  V0.6
  */

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "ulp.h"
#include "ulp_main.h"
#include "led_strip.h"
// #include "esp_intr_alloc.h"
#include "driver/rtc_cntl.h"

#include "esp_log.h"
// definitions
static const char *TAG = "rainsens";
#define BLINK_GPIO CONFIG_BLINK_GPIO

// external references
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

// forward declarations
static void init_ulp_program(void);
static void update_pulse_count(void);
static void configure_led(void);
static led_strip_handle_t led_strip;
static TaskHandle_t ulp_task_handle = NULL;

static uint32_t interrupt_count = 0;

void signal_from_ulp()
{
    ESP_LOGI(TAG, "ULP triggered an interrupt! Calling specific function...");
    interrupt_count++;
    printf("Interrupt Counter %5" PRIu32 "\n", interrupt_count);
    update_pulse_count();
}

static void IRAM_ATTR ulp_isr_handler(void *arg)
{
    SET_PERI_REG_MASK(RTC_CNTL_INT_CLR_REG, RTC_CNTL_ULP_CP_INT_CLR_M);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(ulp_task_handle, 0, eNoAction, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken)
    {
        portYIELD_FROM_ISR();
    }
}

void ulp_task(void *arg)
{
    while (1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Warte auf Benachrichtigung
        printf("Task notified \n");
        size_t freeMemory = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        printf("Free Memory: %d bytes\n", freeMemory);
        // Überprüfe den Stack-Verbrauch
        UBaseType_t highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        printf("Stack High Water Mark: %d\n", highWaterMark);
        signal_from_ulp(); // Führe die spezifische Funktion aus
        led_strip_set_pixel(led_strip, 0, 0, 0, 200);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip); // Überprüfe den freien Speicher

        freeMemory = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        printf("Free Memory: %d bytes\n", freeMemory);
        // Überprüfe den Stack-Verbrauch
        highWaterMark = uxTaskGetStackHighWaterMark(NULL);
        printf("Stack High Water Mark: %d\n", highWaterMark);
    }
}

void setup_ulp_interrupt()
{
    esp_err_t err = rtc_isr_register(ulp_isr_handler, NULL, RTC_CNTL_ULP_CP_INT_ENA_M, ESP_INTR_FLAG_IRAM);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to register ULP interrupt handler: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Create ulp_task");
    xTaskCreate(ulp_task, "ulp_task", 4096, NULL, 5, &ulp_task_handle);
    // ULP-Interrupt aktivieren, required!
    SET_PERI_REG_MASK(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
    ESP_LOGI(TAG, "ULP interrupt enabled");
}

void app_main(void)
{
    /* If user is using USB-serial-jtag then idf monitor needs some time to
     *  re-connect to the USB port. We wait 1 sec here to allow for it to make the reconnection
     *  before we print anything. Otherwise the chip will go back to sleep again before the user
     *  has time to monitor any output.
     */
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_log_level_set("*", ESP_LOG_INFO);
    printf("rainsensor V0.5.2B\n\n");
    printf("Firmware Version: %s\n", APP_VERSION);

    /* Configure the peripheral according to the LED type */
    configure_led();
    printf("Interrupt Counter %5" PRIu32 "\n", interrupt_count);

    led_strip_set_pixel(led_strip, 0, 0, 200, 0);
    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);
    setup_ulp_interrupt();
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP)
    {
        printf("Not ULP wakeup, initializing ULP\n");
        init_ulp_program();
    }
    else
    {
        printf("ULP wakeup, saving pulse count\n");
        update_pulse_count();
    }

    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    // wait some time to get a chance to call interrupt from usp instead of wakeup
    vTaskDelay(pdMS_TO_TICKS(10000));
    printf("Entering deep sleep\n\n");
    led_strip_clear(led_strip);
    esp_deep_sleep_start();
}

static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
                                    (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* GPIO used for pulse counting. */
    gpio_num_t gpio_num = GPIO_NUM_8;
    int rtcio_num = rtc_io_number_get(gpio_num);
    assert(rtc_gpio_is_valid_gpio(gpio_num) && "GPIO used for pulse counting must be an RTC IO");

    /* Initialize some variables used by ULP program.
     * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
     * These variables are declared in an auto generated header file,
     * 'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
     * These variables are located in RTC_SLOW_MEM and can be accessed both by the
     * ULP and the main CPUs.
     *
     * Note that the ULP reads only the lower 16 bits of these variables.
     */
    ulp_debounce_counter = 2;
    ulp_debounce_max_count = 3;
    ulp_next_edge = 0;
    ulp_io_number = rtcio_num; /* map from GPIO# to RTC_IO# */
    ulp_edge_count_to_wake_up = 10;

    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
    rtc_gpio_init(gpio_num);
    rtc_gpio_set_direction(gpio_num, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num);
    rtc_gpio_pullup_dis(gpio_num);
    rtc_gpio_hold_en(gpio_num);

#if CONFIG_IDF_TARGET_ESP32
    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors on modules which have these (e.g. ESP32-WROVER)
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
#endif // CONFIG_IDF_TARGET_ESP32

    esp_deep_sleep_disable_rom_logging(); // suppress boot messages

    /* Set ULP wake up period to T = 20ms.
     * Minimum pulse width has to be T * (ulp_debounce_counter + 1) = 80ms.
     */
    ulp_set_wakeup_period(0, 4000);

    /* Start the program */
    err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

static void update_pulse_count(void)
{
    const char *nvs_namespace = "plusecnt";
    const char *count_key = "count";
    ;
    ESP_ERROR_CHECK(nvs_flash_init());
    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open(nvs_namespace, NVS_READWRITE, &handle));
    uint32_t pulse_count = 0;
    esp_err_t err = nvs_get_u32(handle, count_key, &pulse_count);
    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
    printf("Read pulse count from NVS: %5" PRIu32 "\n", pulse_count);

    /* ULP program counts signal edges, convert that to the number of pulses */
    uint32_t pulse_count_from_ulp = (ulp_edge_count & UINT16_MAX) / 2;
    /* In case of an odd number of edges, keep one until next time */
    ulp_edge_count = ulp_edge_count % 2;
    printf("Pulse count from ULP: %5" PRIu32 "\n", pulse_count_from_ulp);
    printf("Interrupt Counter %5" PRIu32 "\n", interrupt_count);
    /* Save the new pulse count to NVS */
    pulse_count += pulse_count_from_ulp;
    ESP_ERROR_CHECK(nvs_set_u32(handle, count_key, pulse_count));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);
    printf("Wrote updated pulse count to NVS: %5" PRIu32 "\n", pulse_count);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");

    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };

    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
}
