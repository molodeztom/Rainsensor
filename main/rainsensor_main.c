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
  20250227  V0.6.3.3 Timer  Only:  3 Variablen um die Timer werte zu zeigen. Zählt sauber hoch.
  20250308  V0.6.3.4        Calculate time from 3 variables
  20250308  V0.6.3.5        Print Time in hh:mm:ss
  20250308  V0.6.3.6        ulp wakeup time longer, call wakeup from wake_up.S File
  20250309  V0.6.3.8        Timer triggert einen test puls increment wenn die Zeit abgelaufen ist
  20250310  V0.6.3.9        interval to ulp via variable, subtraction in ulp using overflow to find when time is up
  20250310  V0.6.3.10       Clean code part one remove unneeded instructions reduce jump instructions
  20250310  V0.6.4          Clean code use assembler subroutine, improved comments
  20250327  V0.6.5          Debug out edge_count
  20250330  V0.6.5.1        Now working fine, still need to wake on timer only when a pulse was detected
  20250330  V0.6.5.2        Only wake on timer when at least one pulse detected
  20250330  V0.6.5.3        Clean code
  20250401  V0.7.0          Merged with main
  20250401  V0.7.1          Add interrupt handler for ulp wakeup
  20250404  V0.8.0          Add blink task to check if a longer task in main is working and blocks deep sleep
  20250405  V0.8.1          Add E32-900T30D LoRa module to send data
  */

#include <stdio.h>
#include <inttypes.h>
#include <string.h>
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
#include "e32_900t30d.h"
#include "esp_log.h"
// definitions
static const char *TAG = "rainsens";
#define BLINK_GPIO CONFIG_BLINK_GPIO
#define RTC_SLOW_CLK_FREQ 136000 // when RTC_CLCK Source = internal 136 kHz oscillator
// #define RTC_SLOW_CLK_FREQ 68359 //when RTC_CLCK Source = internal 17.5 MHz oscillator / 256
#define ulp_wakeup_period 1000                    // after ulp is halted it sleeps until next wakeup period
static const double wakeup_interval_seconds = 20; // time to wake cpu if at minimum one input pulse detected
// external references
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

EventGroupHandle_t blink_event_group;
#define TASK_DONE_BIT (1 << 0) // Bitmask for the event group

// forward declarations
static void init_ulp_program(void);
static void update_timer_count(void);
static void configure_led(void);
uint32_t calculate_time_ms(uint64_t ticks);
uint64_t calculate_ticks_from_seconds(double seconds);
uint16_t calculate_increments_for_interval(double interval_seconds);
void format_time(uint32_t ms, int *hours, int *minutes, int *seconds);
void signal_from_ulp();
void ulp_task(void *arg);
void BlinkTask(void *arg);
void setup_ulp_interrupt();
static void update_pulse_count(void);
static void  reset_counter(void);


static led_strip_handle_t led_strip;
static TaskHandle_t ulp_task_handle = NULL;

static uint32_t interrupt_count = 0;



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

// Callback-Funktion für empfangene Daten
static void handle_received_data(const uint8_t *data, size_t len) {
    ESP_LOGI(TAG, "Empfangene Daten (%d Bytes): %.*s", len, len, data);
    // Hier können Sie die empfangenen Daten weiterverarbeiten
}


void app_main(void)
{
    /* If user is using USB-serial-jtag then idf monitor needs some time to
     *  re-connect to the USB port. We wait 1 sec here to allow for it to make the reconnection
     *  before we print anything. Otherwise the chip will go back to sleep again before the user
     *  has time to monitor any output.
     */
    TaskHandle_t xBlinkTask = NULL;
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_log_level_set("*", ESP_LOG_WARN); // Set log level for all components to INFO
    esp_log_level_set("E32-900T30D", ESP_LOG_INFO);
    esp_log_level_set("rainsens", ESP_LOG_INFO);
    printf("rainsensor V0.8.1 \n\n");
    printf("Firmware Version: %s\n", APP_VERSION);

    ESP_LOGI("rainsens", "rainsensor V0.8.1 started");
    ESP_LOGI("E32-900T30D", "E32-900T30D example started");
    blink_event_group = xEventGroupCreate(); // Create the event group for task synchronization
    if (blink_event_group == NULL)
    {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }

     // E32-Modul initialisieren
     e32_init();
    
     // Callback für empfangene Daten registrieren
     e32_set_receive_callback(handle_received_data);
     
     // Beispiel: Nachricht senden
     char *test_msg = "Hello LoRa World!";

     e32_send_data((uint8_t *)test_msg, strlen(test_msg));
  
    /* Initialize NVS */

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
        update_timer_count();
        update_pulse_count();
    }

    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
   
    xTaskCreate(BlinkTask, "blink_task", 4096, NULL, 5, &xBlinkTask);  
    xEventGroupWaitBits(blink_event_group, TASK_DONE_BIT, pdTRUE, pdTRUE, portMAX_DELAY); // Wait for the task to finish

    // wait some time to get a chance to call interrupt from usp instead of wakeup
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("Entering deep sleep\n\n");
    led_strip_clear(led_strip);
    //reset_counter();//TODO remove
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
    ulp_debounce_counter = 3;
    ulp_debounce_max_count = 3;
    ulp_io_number = rtcio_num; /* map from GPIO# to RTC_IO# */
    ulp_edge_count_to_wake_up = 8;

    // ulp_timer_count_low_l = 0;
    ulp_timer_count_low_h = 0;
    ulp_timer_count_high = 0;
    // ulp_edge_count = 0;

    // Calculate the number of timer_count_low_h increments for the desired interval
    uint16_t increments = calculate_increments_for_interval(wakeup_interval_seconds);

    // Pass the increments value to the ULP program
    ulp_time_to_wake_CPU = increments;
    printf("Wake-up interval: %.2f seconds -> Increments: %u\n", wakeup_interval_seconds, increments);

    printf("time to wake CPU from ULP: %5" PRIu32 "\n", ulp_time_to_wake_CPU);

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
     time in ms
     */
    ulp_set_wakeup_period(0, ulp_wakeup_period);

    /* Start the program */
    err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

static void update_pulse_count(void)
{
    const char *nvs_namespace = "plusecnt";
    const char *count_key = "count";

    ESP_ERROR_CHECK(nvs_flash_init());
    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open(nvs_namespace, NVS_READWRITE, &handle));
    uint32_t pulse_count = 0;
    esp_err_t err = nvs_get_u32(handle, count_key, &pulse_count);
    assert(err == ESP_OK || err == ESP_ERR_NVS_NOT_FOUND);
    printf("Read pulse count from NVS: %5" PRIu32 "\n", pulse_count);

    /* ULP program counts signal edges, convert that to the number of pulses */
    uint32_t pulse_count_from_ulp = (ulp_edge_count & UINT16_MAX) / 2;
    /* In case of an odd number of edges, keep one until next time, result is 0 or 1! */
    ulp_edge_count = ulp_edge_count % 2;
    printf("pulse count from ULP: %5" PRIu32 "\n", pulse_count_from_ulp);

    /* Save the new pulse count to NVS */
    pulse_count += pulse_count_from_ulp;
    ESP_ERROR_CHECK(nvs_set_u32(handle, count_key, pulse_count));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);
    printf("Wrote updated pulse count to NVS: %5" PRIu32 "\n", pulse_count);
}

static void update_timer_count(void)
{
    const char *nvs_namespace = "plusecnt";
    const char *count_key = "pulse";
    ;
    ESP_ERROR_CHECK(nvs_flash_init());
    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open(nvs_namespace, NVS_READONLY, &handle));
    uint32_t timer_count = 1;
    esp_err_t err = nvs_get_u32(handle, count_key, &timer_count);

    
    nvs_close(handle);
    uint32_t ulp_TIME_TO_WAKEUP_CPU = (ulp_time_to_wake_CPU & UINT16_MAX);
    printf("time to wake CPU from ULP: %5" PRIu32 "\n", ulp_time_to_wake_CPU);

    uint32_t ulp_TIMER_LOW_H = (ulp_timer_count_low_h & UINT16_MAX);
    printf("timer count high from ULP: %5" PRIu32 "\n", ulp_TIMER_LOW_H);

    uint32_t ulp_TIMER_HIGH = (ulp_timer_count_high & UINT16_MAX);
    printf("timer count upper from ULP: %5" PRIu32 "\n", ulp_TIMER_HIGH);

    uint32_t ulp_START_TIME_LOW_H = (ulp_start_time_low_h & UINT16_MAX);
    printf("start_time_low_h: %5" PRIu32 "\n", ulp_START_TIME_LOW_H);

    uint32_t ulp_START_TIME_HIGH = (ulp_start_time_high & UINT16_MAX);
    printf("start_time_high: %5" PRIu32 "\n", ulp_START_TIME_HIGH);

    uint32_t ulp_TIMER_COUNT = (ulp_timer_count & UINT16_MAX);
    printf("timer_count: %5" PRIu32 "\n", ulp_TIMER_COUNT);

    uint32_t ulp_TEST_DIV = (ulp_test_div & UINT16_MAX);
    printf("Test Value: %5" PRIu32 "\n", ulp_TEST_DIV);

    uint32_t ulp_EDGE_COUNT = (ulp_edge_count & UINT16_MAX);
    printf("Edge Count: %5" PRIu32 "\n", ulp_EDGE_COUNT);

    uint32_t ulp_EDGE_COUNT_TO_WAKE_UP = (ulp_edge_count_to_wake_up & UINT16_MAX);
    printf("Edge Count to wake up: %5" PRIu32 "\n", ulp_EDGE_COUNT_TO_WAKE_UP);


    uint64_t timer_value = ((uint64_t)ulp_TIMER_HIGH << 32) |
                           ((uint32_t)ulp_TIMER_LOW_H << 16);
    printf("ULP Timerwert: %llu Ticks\n", timer_value);

    uint32_t ms = calculate_time_ms(timer_value);

    // Convert milliseconds to hours, minutes, and seconds
    int hours, minutes, seconds;
    format_time(ms, &hours, &minutes, &seconds);

    // Output the elapsed time in h:mm:ss format
    printf("Elapsed Time: %02d:%02d:%02d\n", hours, minutes, seconds);
}

// Function to calculate the elapsed time in milliseconds
uint32_t calculate_time_ms(uint64_t ticks)
{
    // Each tick corresponds to approximately 6.67 microseconds, so we use the formula to convert ticks to milliseconds.
    // Calculation in milliseconds: (Ticks / RTC_SLOW_CLK_FREQ) * 1000
    return (uint32_t)((ticks * 1000) / RTC_SLOW_CLK_FREQ); // Ticks -> milliseconds
}

uint64_t calculate_ticks_from_seconds(double seconds)
{
    // Convert seconds to ticks using the formula: Ticks = seconds * RTC_SLOW_CLK_FREQ
    return (uint64_t)(seconds * RTC_SLOW_CLK_FREQ);
}

// Function to convert milliseconds into hours, minutes, and seconds
void format_time(uint32_t ms, int *hours, int *minutes, int *seconds)
{
    *hours = ms / 3600000;             // Calculate hours (ms / 3600000)
    *minutes = (ms % 3600000) / 60000; // Calculate minutes ((ms % 3600000) / 60000)
    *seconds = (ms % 60000) / 1000;    // Calculate seconds ((ms % 60000) / 1000)
}

// Function to calculate the number of timer_count_low_h increments for a given interval
uint16_t calculate_increments_for_interval(double interval_seconds)
{
    // Each timer_count_low_h increment corresponds to 65536 / 136000 ≈ 0.482 seconds
    double increments = interval_seconds / (65536.0 / RTC_SLOW_CLK_FREQ);

    // Manual rounding
    if (increments - (uint16_t)increments >= 0.5)
    {
        return (uint16_t)increments + 1;
    }
    else
    {
        return (uint16_t)increments;
    }
}

void signal_from_ulp()
{
    ESP_LOGI(TAG, "ULP triggered an interrupt! Calling specific function...");
    interrupt_count++;
    printf("Interrupt Counter %5" PRIu32 "\n", interrupt_count);
    update_pulse_count();
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

void BlinkTask(void *arg)
{
    for (int i = 0; i < 15; i++)
    {
        led_strip_set_pixel(led_strip, 0, 230, 0, 0);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000));
        led_strip_set_pixel(led_strip, 0, 0, 230, 0);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    led_strip_set_pixel(led_strip, 0, 200, 200, 200);
    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(1000));
    xEventGroupSetBits(blink_event_group, TASK_DONE_BIT); // Set the event group bit to signal task completion
    led_strip_clear(led_strip); // Clear the LED strip

    ESP_LOGI(TAG, "BlinkTask finished, deleting task");
    vTaskDelete(NULL); // Delete the task when done     
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

static void reset_counter(void)
{
    const char *nvs_namespace = "plusecnt";
    const char *count_key = "count";
    ESP_ERROR_CHECK(nvs_flash_init());
    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open(nvs_namespace, NVS_READWRITE, &handle));
    uint32_t pulse_count = 0;
    uint32_t timer_count = 0;
    ESP_ERROR_CHECK(nvs_set_u32(handle, count_key, pulse_count));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);
}

