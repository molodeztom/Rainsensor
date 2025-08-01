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
  20250625  V0.8.2          E32 module replaced now using E32_Lora_Lib
  20250635  V0.9.1          add task to receive data (did not do that because it is all in a sequence)
  20250720  V0.9.2          use separate while loop for receiving data
  20250720  V0.9.3          filter with magic bytes turned off, pull up resistor for RX activated
  20250720  V0.9.4          test for garbage bytes and print number of errors
  20250721  V0.9.6          Add comment at beginning, update date and version number in comment and across file
  20250721  V0.9.7          Add debug output for received data and from ulp
  20250721  V0.9.8          Send number of pulses in message
  20250721  V0.9.9          Send a packed struct with pulse count and timers
  20250721  V0.9.10         BlinkTask duration reduced to 3 seconds, debug output for send counter and checksum, struct size check before LoRa send, checksum bug fix prompt for receiver
  20250722  V0.9.11         Test without interrupt
  20250723  V0.9.12         Test with interrupt worked, all functions not needed outcommented
  20250724  V0.9.13         Code cleanup interrupt functions removed
  20250724  V0.9.14         Code cleanup, removed unused interrupt counter, optimize functions called only if ulp wakeup
  20250724  V0.9.15         Message if Rainsensor initialized
  20250727  V0.9.16         New send structure with messageID and eventID, no time string anymore
  20250727  V0.9.17         Use NVS to store messageID, increment and store messageID in NVS
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
#include "ulp.h"
#include "ulp_main.h"
#include "led_strip.h"
// #include "esp_intr_alloc.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/rtc_cntl.h"
#include "esp_log.h"
#include "E32_Lora_Lib.h"
#include "../include/communication.h"

// =====================
// Application Parameters
// =====================

// LoRa communication parameters
#define LORA_RX_BUFFER_SIZE 128    // Buffer size for received LoRa messages
#define LORA_REPLY_TIMEOUT_MS 5000 // Timeout (ms) to wait for a reply after sending a message
#define LORA_RECEIVE_POLL_MS 200   // Polling interval (ms) when waiting for a reply

// LED and ULP parameters
#define BLINK_GPIO CONFIG_BLINK_GPIO              // GPIO for addressable LED strip
#define ulp_wakeup_period 1000                    // ULP wakeup period in ms
static const double wakeup_interval_seconds = 60; // Time to wake CPU if at least one input pulse detected
#define RTC_SLOW_CLK_FREQ 136000                  // RTC slow clock frequency (internal 136 kHz oscillator)
// #define RTC_SLOW_CLK_FREQ 68359   // RTC slow clock frequency (17.5 MHz oscillator / 256)
#define TASK_DONE_BIT (1 << 0) // Bitmask for event group
static const char *nvs_namespace = "pulsecnt";


// external references
extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

EventGroupHandle_t blink_event_group;
#define TASK_DONE_BIT (1 << 0) // Bitmask for the event group

// forward declarations
static void print_buffer_hex(const uint8_t *buf, size_t len);
static void test_for_garbage_bytes(const uint8_t *buf, size_t len);
static void init_ulp_program(void);
static void update_timer_count(void);
static void configure_led(void);
static void update_pulse_count(void);
static void reset_counter(void);
static void BlinkTask(void *arg);
static uint32_t calculate_time_ms(uint64_t ticks);
static uint64_t calculate_ticks_from_seconds(double seconds);
static uint16_t calculate_increments_for_interval(double interval_seconds);
static void format_time(uint32_t ms, int *hours, int *minutes, int *seconds);
static void send_lora_message(uint32_t pulse_count, int hours, int minutes, int seconds, uint32_t elapsed_ms, int send_counter);
static void receive_lora_message(void);
static uint16_t read_messageId_fromNVS();
static void increment_and_store_messageId(uint16_t *messageId);

static const char *TAG = "rainsens";
#define RAINSENSOR_VERSION "V0.9.17"

static led_strip_handle_t led_strip;

void app_main(void)
{
    /* If user is using USB-serial-jtag then idf monitor needs some time to
     *  re-connect to the USB port. We wait 1 sec here to allow for it to make the reconnection
     *  before we print anything. Otherwise the chip will go back to sleep again before the user
     *  has time to monitor any output.
     */
    uint32_t ms = 0;
    int hours = 0, minutes = 0, seconds = 0;
    static uint16_t msg_id = 0; // Message ID for LoRa messages
    TaskHandle_t xBlinkTask = NULL;
    vTaskDelay(pdMS_TO_TICKS(1000));
    /*     esp_log_level_set("*", ESP_LOG_WARN); // Set log level for all components to INFO
        esp_log_level_set("E32-900T30D", ESP_LOG_INFO); */
        esp_log_level_set("rainsens", ESP_LOG_INFO); 
    printf("rainsensor %s\n\n", RAINSENSOR_VERSION);
    printf("Firmware Version: %s\n", APP_VERSION);
    printf("E32_Lora_Lib version: %s\n", e32_lora_lib_get_version());
    printf("E32_Lora_Lib git version: %s\n", E32_LORA_LIB_GIT_VERSION);

    e32_config_t config; // E32 configuration structure
    uint8_t rx_buffer[128];
    size_t received = 0;
    void initLibrary();
    init_io();                // initialize IO pins
    e32_init_config(&config); // initialize E32 configuration structure
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());

    config.OPTION.transmissionPower = TRANSMISSION_POWER_21dBm;    // set transmission power to 30 dBm
    config.OPTION.wirelessWakeupTime = WIRELESS_WAKEUP_TIME_500MS; // set wakeup time to 250ms
    config.OPTION.fec = FEC_ENABLE;
    config.CHAN = 0x06;         // set channel to 6 (902.875MHz)
    sendConfiguration(&config); // E32 configuration structure

    vTaskDelay(pdMS_TO_TICKS(WAIT_FOR_PROCESSING_LIB)); // wait for command to be processed

    blink_event_group = xEventGroupCreate(); // Create the event group for task synchronization
    if (blink_event_group == NULL)
    {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }
    // int send_counter = 1;
    uint32_t pulse_count = 0;

    /* Initialize NVS */

    /* Configure the peripheral according to the LED type */
    configure_led();
    led_strip_set_pixel(led_strip, 0, 0, 200, 0);
    /* Refresh the strip to send data */
    led_strip_refresh(led_strip);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP)
    {
        printf("Not ULP wakeup, initializing ULP\n");
        init_ulp_program();
        // TODO: change use lora_send_message (add an event code to  struct first)
        const char *message = "Rainsensor initialized";
        size_t message_length = strlen(message) + 1; // Include null terminator
        esp_err_t result = e32_send_data((uint8_t *)message, message_length);
        // Check if the transmission was successful
        if (result == ESP_OK)
        {
            ESP_LOGI("Rainsensor", "Initialization message sent successfully.");
        }
        else
        {
            ESP_LOGE("Rainsensor", "Failed to send initialization message: %s", esp_err_to_name(result));
        }
    }
    else
    {
        printf("ULP wakeup, saving pulse count\n");
        update_timer_count();
        update_pulse_count();
        // Read updated pulse_count from NVS
        nvs_handle_t handle;
        ESP_ERROR_CHECK(nvs_open(nvs_namespace, NVS_READONLY, &handle));
        ESP_ERROR_CHECK(nvs_get_u32(handle, "count", &pulse_count));
        nvs_close(handle);
        // Calculate elapsed time for payload
        // After update_timer_count(), ms/hours/minutes/seconds are set
        ms = calculate_time_ms(((uint64_t)ulp_timer_count_high << 32) | ((uint32_t)ulp_timer_count_low_h << 16));
        format_time(ms, &hours, &minutes, &seconds);
        xTaskCreate(BlinkTask, "blink_task", 4096, NULL, 5, &xBlinkTask);
        xEventGroupWaitBits(blink_event_group, TASK_DONE_BIT, pdTRUE, pdTRUE, portMAX_DELAY); // Wait for the task to finish

        // Use ulp_timer_count as send_counter since timer_count is undeclared
        send_lora_message(pulse_count, hours, minutes, seconds, ms, (uint32_t)ulp_timer_count);
        receive_lora_message();
    }

    // ... process answer or timeout ...
    // Optional: sleep before next cycle
    vTaskDelay(pdMS_TO_TICKS(4000));

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
    ulp_debounce_counter = 3;
    ulp_debounce_max_count = 3;
    ulp_io_number = rtcio_num; /* map from GPIO# to RTC_IO# */
    ulp_edge_count_to_wake_up = 8;

    // Debug: Print ULP config variables
    printf("[DEBUG] ULP io_number (RTC IO): %d (should match GPIO8 RTC IO number)\n", (int)ulp_io_number);
    printf("[DEBUG] ULP edge_count_to_wake_up: %d\n", (int)ulp_edge_count_to_wake_up);
    printf("[DEBUG] ULP debounce_counter: %d\n", (int)ulp_debounce_counter);
    printf("[DEBUG] ULP debounce_max_count: %d\n", (int)ulp_debounce_max_count);
    printf("[DEBUG] ULP edge_count (before sleep): %d\n", (int)ulp_edge_count);

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

    const char *count_key = "count";

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

    // Debug: Print ULP variables after update
    printf("[DEBUG] ULP edge_count (after wakeup): %d\n", (int)ulp_edge_count);
    printf("[DEBUG] ULP io_number (RTC IO): %d\n", (int)ulp_io_number);
    printf("[DEBUG] ULP edge_count_to_wake_up: %d\n", (int)ulp_edge_count_to_wake_up);
}

static void update_timer_count(void)
{
    //const char *nvs_namespace = "pulsecnt";
    const char *count_key = "pulse";

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
static uint32_t calculate_time_ms(uint64_t ticks)
{
    // Each tick corresponds to approximately 6.67 microseconds, so we use the formula to convert ticks to milliseconds.
    // Calculation in milliseconds: (Ticks / RTC_SLOW_CLK_FREQ) * 1000
    return (uint32_t)((ticks * 1000) / RTC_SLOW_CLK_FREQ); // Ticks -> milliseconds
}

static uint64_t calculate_ticks_from_seconds(double seconds)
{
    // Convert seconds to ticks using the formula: Ticks = seconds * RTC_SLOW_CLK_FREQ
    return (uint64_t)(seconds * RTC_SLOW_CLK_FREQ);
}

// Function to convert milliseconds into hours, minutes, and seconds
static void format_time(uint32_t ms, int *hours, int *minutes, int *seconds)
{
    *hours = ms / 3600000;             // Calculate hours (ms / 3600000)
    *minutes = (ms % 3600000) / 60000; // Calculate minutes ((ms % 3600000) / 60000)
    *seconds = (ms % 60000) / 1000;    // Calculate seconds ((ms % 60000) / 1000)
}

// Function to calculate the number of timer_count_low_h increments for a given interval
static uint16_t calculate_increments_for_interval(double interval_seconds)
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

static volatile bool send_lora_on_ulp = false;

static void BlinkTask(void *arg)
{
    // Blink for 3 seconds (3 cycles of 1s)
    for (int i = 0; i < 3; i++)
    {
        led_strip_set_pixel(led_strip, 0, 230, 0, 0);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(500));
        led_strip_set_pixel(led_strip, 0, 0, 230, 0);
        led_strip_refresh(led_strip);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    led_strip_set_pixel(led_strip, 0, 200, 200, 200);
    led_strip_refresh(led_strip);
    vTaskDelay(pdMS_TO_TICKS(500));
    xEventGroupSetBits(blink_event_group, TASK_DONE_BIT);
    led_strip_clear(led_strip);
    ESP_LOGI(TAG, "BlinkTask finished, deleting task");
    vTaskDelete(NULL);
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
    //const char *nvs_namespace = "pulsecnt";
    const char *count_key = "count";

    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open(nvs_namespace, NVS_READWRITE, &handle));
    uint32_t pulse_count = 0;
    uint32_t timer_count = 0;
    ESP_ERROR_CHECK(nvs_set_u32(handle, count_key, pulse_count));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);
}

// Optional: Test function to check for garbage bytes in received buffer
static int garbage_error_counter = 0;
static int message_counter = 0;
static void test_for_garbage_bytes(const uint8_t *buf, size_t len)
{
    // Garbage: any non-printable ASCII before first printable char
    size_t i = 0;
    while (i < len && (buf[i] < 0x20 || buf[i] > 0x7E))
    {
        i++;
    }
    if (i > 0)
    {
        garbage_error_counter++;
        ESP_LOGW("rainsens", "Garbage bytes detected at start of message (%d bytes):", (int)i);
        printf("[GARBAGE HEX]: ");
        for (size_t j = 0; j < i; ++j)
        {
            printf("%02X ", buf[j]);
        }
        printf("\n");
        ESP_LOGW("rainsens", "Total garbage errors so far: %d", garbage_error_counter);
    }
    message_counter++;
    if (message_counter % 10 == 0)
    {
        printf("[INFO] Garbage error counter after %d messages: %d\n", message_counter, garbage_error_counter);
    }
}

static void print_buffer_hex(const uint8_t *buf, size_t len)
{
    printf("[HEX]: ");
    for (size_t i = 0; i < len; i++)
    {
        printf("%02X ", buf[i]);
    }
    printf("\n");
}

void send_lora_message(uint32_t pulse_count, int hours, int minutes, int seconds, uint32_t elapsed_ms, int send_counter)
{
    ESP_LOGI(TAG, "send packed struct payload");
    /* message structure
    uint16_t messageID;          // Message ID
    uint16_t lora_eventID;      // Event ID see below
    uint32_t elapsed_time_ms;      // Elapsed time in ms
    uint32_t pulse_count;          // Number of pulses
    uint16_t checksum;             // Checksum (sum of all bytes except checksum field)
    */
    char elapsed_time_str[9];
    uint16_t messageID = 0;
    messageID = read_messageId_fromNVS();
    lora_payload_t payload;
    const size_t max_payload_size = 58; // E32-900T30D max payload size
    if (sizeof(lora_payload_t) > max_payload_size)
    {
        ESP_LOGE(TAG, "ERROR: lora_payload_t size (%u bytes) exceeds LoRa E32 max payload size (%u bytes). Not sending!", (unsigned)sizeof(lora_payload_t), (unsigned)max_payload_size);
        return;
    }
    snprintf(elapsed_time_str, sizeof(elapsed_time_str), "%02d:%02d:%02d", hours, minutes, seconds);
    printf("messageID before increment: %u", messageID);
    payload.messageID = messageID; // Use messageID from NVS
    increment_and_store_messageId(&messageID); // Increment and store new messageID in NVS
    printf("messageID after increment: %u", messageID);
    payload.lora_eventID = 0x0001; // Example event ID, change as needed
    payload.elapsed_time_ms = elapsed_ms;
    payload.pulse_count = pulse_count;
    payload.checksum = lora_payload_checksum(&payload);

    ESP_LOGI(TAG, "Payload: time %s, ms %lu, pulses %lu, messageID %lu, checksum 0x%04X",
             elapsed_time_str,
             (unsigned long)payload.elapsed_time_ms,
             (unsigned long)payload.pulse_count,
             (unsigned long)payload.messageID,
             payload.checksum);
    printf("[DEBUG] Send counter: %lu\n", (unsigned long)payload.messageID);
    printf("[DEBUG] Checksum: 0x%04X\n", payload.checksum);
    ESP_ERROR_CHECK(e32_send_data((uint8_t *)&payload, sizeof(payload)));
}

static void receive_lora_message()
{
// Wait for reply up to 5 seconds, polling every 200ms
#define LORA_RX_BUFFER_SIZE 128
#define LORA_REPLY_TIMEOUT_MS 5000
#define LORA_RECEIVE_POLL_MS 200
    uint8_t rx_buffer[LORA_RX_BUFFER_SIZE];
    size_t total_received = 0;
    uint32_t waited_ms = 0;
    esp_err_t err = ESP_OK;
    bool got_terminator = false;
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Waiting for reply up to %d ms...", LORA_REPLY_TIMEOUT_MS);
    while (waited_ms < LORA_REPLY_TIMEOUT_MS && !got_terminator && total_received < LORA_RX_BUFFER_SIZE)
    {
        size_t received = 0;
        err = e32_receive_data(rx_buffer + total_received, LORA_RX_BUFFER_SIZE - total_received, &received);
        if (err == ESP_OK && received > 0)
        {
            // Check for terminator in newly received data
            for (size_t i = 0; i < received; i++)
            {
                if (rx_buffer[total_received + i] == '!')
                {
                    got_terminator = true;
                    total_received += i + 1; // include the terminator
                    break;
                }
            }
            if (!got_terminator)
            {
                total_received += received;
            }
        }
        if (!got_terminator)
        {
            vTaskDelay(pdMS_TO_TICKS(LORA_RECEIVE_POLL_MS));
            waited_ms += LORA_RECEIVE_POLL_MS;
        }
    }
    if (got_terminator)
    {
        printf("Received message: ");
        for (size_t i = 0; i < total_received; i++)
        {
            printf("%c", rx_buffer[i]);
        }
        printf("\n");
        print_buffer_hex(rx_buffer, total_received);
        // Test for garbage bytes at start
        test_for_garbage_bytes(rx_buffer, total_received);
    }
    else if (total_received > 0)
    {
        printf("Partial message received (no terminator): ");
        for (size_t i = 0; i < total_received; i++)
        {
            printf("%c", rx_buffer[i]);
        }
        printf("\n");
        print_buffer_hex(rx_buffer, total_received);
        // Test for garbage bytes at start
        test_for_garbage_bytes(rx_buffer, total_received);
    }
    else
    {
        printf("No reply received within timeout\n");
    }
}

uint16_t read_messageId_fromNVS()
{
    ESP_LOGI(TAG, "Reading messageID from NVS");
    nvs_handle_t handle;
    uint16_t messageId = 0; // default 0 on first run
    esp_err_t err = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
    if (err == ESP_OK)
    {
        err = nvs_get_u16(handle, "messageID", &messageId);
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            messageId = 0; // not found, start at 0
            printf("No messageID found in NVS, starting at 0\n");
        }
        else if (err != ESP_OK)
        {
            printf("Error reading messageID from NVS: %s\n", esp_err_to_name(err));
            // handle error (log etc)
        }
        nvs_close(handle);
    }
    else
    {
        printf("Error opening NVS: %s\n", esp_err_to_name(err));
        // handle error opening NVS
    }
    return messageId;
}

void increment_and_store_messageId(uint16_t *messageID)
{
    ESP_LOGI(TAG, "Incrementing and storing messageID in NVS %u", *messageID);
    *messageID = *messageID + 1; // Increment messageID

    printf("New messageID: %u\n", *messageID);
    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open(nvs_namespace, NVS_READWRITE, &handle));
    ESP_ERROR_CHECK(nvs_set_u16(handle, "messageID", *messageID));
    ESP_ERROR_CHECK(nvs_commit(handle));
    nvs_close(handle);
}