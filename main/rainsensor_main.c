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
  20250802  V0.9.18         Code cleanup, remove old debug info
  20250803  V0.9.20         Improved error handling: added explicit error checks and recovery mechanisms
  20250803  V0.9.21         Use ESP_LOGD for debug output, removed ESP_LOGI for debug output where it was not needed
  20250803  V0.9.22         Add constants for all magic numbers, use them in code
  20250803  V0.9.23         Remove unused debug calculations when in release mode
  20250803  V0.9.24         Add Kconfig option to enable/disable debug output, move main receive lora messsage functionality to E32_Lora_Lib
  20250803  V0.9.25         Add version number to E32_Lora_Lib.h and E32_Lora_Lib.c, use it in initLibrary
  20250803  V0.9.26         remove unused variables, use e32_lora_lib_get_version() to get version number, replace deprecated "driver/rtc_cntl.h"
  20250804  V0.9.27         Restructure app_main to make it more readable. 
  20250804  V0.9.28         Test receive a struct with checksum, not using terminator because we have a fixed size message
  */

/*
To enable or disable debug output:
Run idf.py menuconfig
Navigate to "Example Configuration" → "Enable detailed debug output"
Toggle the option as needed
Save the configuration and rebuild the project
When disabled, the detailed debug output in the update_timer_count function will not be compiled, resulting in smaller code size and better performance in release builds.
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
#include "esp_system.h"

/* Old deprecated include - kept for reference */
// #include "driver/rtc_cntl.h"

/* New recommended includes for ESP-IDF v5.4.1+ */
#include "esp_sleep.h"      // For sleep-related functions
#include "driver/rtc_io.h"  // For RTC GPIO functions
#include "esp_log.h"
#include "E32_Lora_Lib.h"
#include "../include/communication.h"

// =====================
// Application Parameters
// =====================

// Use the version number extracted from Git tags if available, otherwise use a default
#ifdef APP_VERSION_NUMBER
#define RAINSENSOR_VERSION "V " APP_VERSION_NUMBER
#else
#define RAINSENSOR_VERSION "V0.9.27"
#endif

// LoRa communication parameters
#define LORA_RX_BUFFER_SIZE 128    // Buffer size for received LoRa messages
#define LORA_REPLY_TIMEOUT_MS 8000 // Timeout (ms) to wait for a reply after sending a message
#define LORA_RECEIVE_POLL_MS 200   // Polling interval (ms) when waiting for a reply

// LoRa configuration constants
#define LORA_CHANNEL_DEFAULT 0x06    // Default channel (902.875MHz)
#define LORA_EVENT_ID_DEFAULT 0x0001 // Default event ID
#define LORA_MAX_PAYLOAD_SIZE 58     // E32-900T30D max payload size in bytes
#define LORA_MESSAGE_TERMINATOR '!'  // Character that marks the end of a LoRa message

// Buffer and formatting constants
#define HEX_CHARS_PER_BYTE 3  // Each byte becomes 2 hex chars + 1 space in string representation

// Time-related constants
#define STARTUP_DELAY_MS 1000      // Delay at startup to allow monitor reconnection
#define SHUTDOWN_DELAY_MS 4000     // Delay before entering deep sleep
#define LED_BLINK_INTERVAL_MS 500  // Interval between LED color changes
#define LORA_RECEIVE_DELAY_MS 2000 // Delay before waiting for LoRa reply
#define MS_PER_SECOND 1000         // Milliseconds in a second
#define MS_PER_MINUTE 60000        // Milliseconds in a minute
#define MS_PER_HOUR 3600000        // Milliseconds in an hour
#define TIMER_COUNTER_MAX 65536.0  // Maximum value of timer counter (2^16)

// ULP pulse counting parameters
#define ULP_PULSE_COUNTING_GPIO GPIO_NUM_8 // GPIO pin used for pulse counting (Hall sensor input)
#define ULP_DEBOUNCE_COUNT 3               // Debounce counter initial value - filters out noise by requiring signal to be stable for this many samples
#define ULP_DEBOUNCE_MAX_COUNT 3           // Maximum debounce count - with ULP wakeup period of 20ms, this creates a minimum pulse width of 80ms (20ms * (3+1))
#define ULP_EDGES_PER_PULSE 2              // Number of edges per complete pulse (rising + falling) - used to convert edge count to pulse count
#define ULP_PULSES_TO_WAKE_UP 4            // Number of complete pulses (rain drops) required to wake up the CPU

// ULP memory access constants
#define ULP_16BIT_MASK UINT16_MAX // Mask for accessing 16-bit values from ULP memory (ULP only reads lower 16 bits)

// LED and ULP parameters
#define BLINK_GPIO CONFIG_BLINK_GPIO              // GPIO for addressable LED strip
#define ulp_wakeup_period 1000                    // ULP wakeup period in ms
static const double wakeup_interval_seconds = 60; // Time to wake CPU if at least one input pulse detected
#define RTC_SLOW_CLK_FREQ 136000                  // RTC slow clock frequency (internal 136 kHz oscillator)
// #define RTC_SLOW_CLK_FREQ 68359   // RTC slow clock frequency (17.5 MHz oscillator / 256)
#define TASK_DONE_BIT (1 << 0) // Bitmask for event group
static const char *nvs_namespace = "pulsecnt";

// LED color constants (R, G, B values)
#define LED_COLOR_RED_R 230
#define LED_COLOR_RED_G 0
#define LED_COLOR_RED_B 0

#define LED_COLOR_GREEN_R 0
#define LED_COLOR_GREEN_G 200
#define LED_COLOR_GREEN_B 0

#define LED_COLOR_WHITE_R 200
#define LED_COLOR_WHITE_G 200
#define LED_COLOR_WHITE_B 200

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
static void initialize_led(void);
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
static void task_delay_callback(uint32_t ms);
static void handle_ulp_wakeup(uint32_t *pulse_count, uint32_t *ms, int *hours, int *minutes, int *seconds);
static void handle_normal_startup(void);
static void prepare_for_deep_sleep(void);
static bool initialize_system(void);
static void initialize_led(void);
static void handle_normal_startup(void);
static void handle_ulp_wakeup(uint32_t *pulse_count, uint32_t *ms, int *hours, int *minutes, int *seconds);
static void prepare_for_deep_sleep(void);

static const char *TAG = "rainsens";



static led_strip_handle_t led_strip;

// Forward declarations for helper functions


void app_main(void)
{
    uint32_t ms = 0;
    int hours = 0, minutes = 0, seconds = 0;
       /* If user is using USB-serial-jtag then idf monitor needs some time to
     *  re-connect to the USB port. We wait 1 sec here to allow for it to make the reconnection
     *  before we print anything. Otherwise the chip will go back to sleep again before the user
     *  has time to monitor any output.
     */

    vTaskDelay(pdMS_TO_TICKS(STARTUP_DELAY_MS));

    if (!initialize_system()) {
        return;
    }
    uint32_t pulse_count = 0;

    /* Initialize LED */
    initialize_led();

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause != ESP_SLEEP_WAKEUP_ULP)
    {
        handle_normal_startup();
    }
    else
    {
        handle_ulp_wakeup(&pulse_count, &ms, &hours, &minutes, &seconds);
    }

    prepare_for_deep_sleep();
}

static bool initialize_system(void)
{
    // Configure logging
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(TAG, ESP_LOG_DEBUG);
    esp_log_level_set("LORA_LIB", ESP_LOG_DEBUG);
    esp_log_level_set("gpio", ESP_LOG_ERROR);

    ESP_LOGI(TAG, "Rainsensor Firmware Version: %s", APP_VERSION);
    ESP_LOGI(TAG, "E32_Lora_Lib version: %s", e32_lora_lib_get_version());

    // Initialize components
    if (nvs_flash_init() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize NVS flash");
        return false;
    }

    if (esp_sleep_enable_ulp_wakeup() != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable ULP wakeup");
        return false;
    }

    // Initialize LoRa
    e32_config_t config;
    initLibrary();
    e32_init_config(&config);
    config.OPTION.transmissionPower = TRANSMISSION_POWER_21dBm;
    config.OPTION.wirelessWakeupTime = WIRELESS_WAKEUP_TIME_500MS;
    config.OPTION.fec = FEC_ENABLE;
    config.CHAN = LORA_CHANNEL_DEFAULT;
    sendConfiguration(&config);
     // Note: sendConfiguration doesn't return an error code
    vTaskDelay(pdMS_TO_TICKS(WAIT_FOR_PROCESSING_LIB));

    blink_event_group = xEventGroupCreate();
    if (blink_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return false;
    }

    return true;
}

static void handle_ulp_wakeup(uint32_t *pulse_count, uint32_t *ms, int *hours, int *minutes, int *seconds)
{
    ESP_LOGD(TAG, "ULP wakeup, saving pulse count");
    update_timer_count();
    update_pulse_count();
    
    // Read pulse count from NVS
    nvs_handle_t handle;
    if (nvs_open(nvs_namespace, NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_u32(handle, "count", pulse_count);
        nvs_close(handle);
    }

    // Calculate elapsed time
    *ms = calculate_time_ms(((uint64_t)ulp_timer_count_high << 32) | ((uint32_t)ulp_timer_count_low_h << 16));
    format_time(*ms, hours, minutes, seconds);
    
    // Blink LED and send LoRa message
    TaskHandle_t xBlinkTask = NULL;
    xTaskCreate(BlinkTask, "blink_task", 4096, NULL, 5, &xBlinkTask);
    xEventGroupWaitBits(blink_event_group, TASK_DONE_BIT, pdTRUE, pdTRUE, portMAX_DELAY);
    //send data to lora WLAN bridge until message is acknowledged
    bool msgAck = false; 
    int numRetry = 0;
    #define numRetryMax 2

    while((msgAck == false) && (numRetry <= numRetryMax)){
        send_lora_message(*pulse_count, *hours, *minutes, *seconds, *ms, (uint32_t)ulp_timer_count);
        receive_lora_message();
        msgAck = true;
    }

}

static void handle_normal_startup(void)
{
    ESP_LOGD(TAG, "Not ULP wakeup, initializing ULP");
    init_ulp_program();
    
    const char *message = "Rainsensor initialized";
    esp_err_t result = e32_send_data((uint8_t *)message, strlen(message) + 1);
    if (result == ESP_OK) {
        ESP_LOGD(TAG, "Initialization message sent successfully.");
    } else {
        ESP_LOGE(TAG, "Failed to send initialization message: %s", esp_err_to_name(result));
    }
}

static void prepare_for_deep_sleep(void)
{
    // Optional: sleep before next cycle
    vTaskDelay(pdMS_TO_TICKS(SHUTDOWN_DELAY_MS));

    ESP_LOGI(TAG, "Entering deep sleep");
    esp_err_t led_err = led_strip_clear(led_strip);
    if (led_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to clear LED strip: %s", esp_err_to_name(led_err));
    }

    esp_deep_sleep_start();
}

static void init_ulp_program(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
                                    (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to load ULP binary: %s", esp_err_to_name(err));
        return;
    }

    /* GPIO used for pulse counting. */
    int rtcio_num = rtc_io_number_get(ULP_PULSE_COUNTING_GPIO);
    assert(rtc_gpio_is_valid_gpio(ULP_PULSE_COUNTING_GPIO) && "GPIO used for pulse counting must be an RTC IO");

    /* Initialize some variables used by ULP program.
     * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
     * These variables are declared in an auto generated header file,
     * 'ulp_main.h', name of this file is defined in component.mk as ULP_APP_NAME.
     * These variables are located in RTC_SLOW_MEM and can be accessed both by the
     * ULP and the main CPUs.
     *
     * Note that the ULP reads only the lower 16 bits of these variables.
     */
    ulp_debounce_counter = ULP_DEBOUNCE_COUNT;
    ulp_debounce_max_count = ULP_DEBOUNCE_MAX_COUNT;
    ulp_io_number = rtcio_num; /* map from GPIO# to RTC_IO# */                                              /* map from GPIO# to RTC_IO# */
    ulp_edge_count_to_wake_up = ULP_PULSES_TO_WAKE_UP * ULP_EDGES_PER_PULSE; // Set wake-up threshold based on pulse count

    // Debug: Print ULP config variables
    ESP_LOGD(TAG, "ULP io_number (RTC IO): %d (should match GPIO8 RTC IO number)", (int)ulp_io_number);
    ESP_LOGD(TAG, "ULP edge_count_to_wake_up: %d", (int)ulp_edge_count_to_wake_up);
    ESP_LOGD(TAG, "ULP debounce_counter: %d", (int)ulp_debounce_counter);
    ESP_LOGD(TAG, "ULP debounce_max_count: %d", (int)ulp_debounce_max_count);
    ESP_LOGD(TAG, "ULP edge_count (before sleep): %d", (int)ulp_edge_count);

    // ulp_timer_count_low_l = 0;
    ulp_timer_count_low_h = 0;
    ulp_timer_count_high = 0;
    // ulp_edge_count = 0;

    // Calculate the number of timer_count_low_h increments for the desired interval
    uint16_t increments = calculate_increments_for_interval(wakeup_interval_seconds);

    // Pass the increments value to the ULP program
    ulp_time_to_wake_CPU = increments;
    ESP_LOGD(TAG, "Wake-up interval: %.2f seconds -> Increments: %u", wakeup_interval_seconds, increments);

    ESP_LOGD(TAG, "Time to wake CPU from ULP: %5" PRIu32, ulp_time_to_wake_CPU);

    /* Initialize selected GPIO as RTC IO, enable input, disable pullup and pulldown */
    rtc_gpio_init(ULP_PULSE_COUNTING_GPIO);
    rtc_gpio_set_direction(ULP_PULSE_COUNTING_GPIO, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(ULP_PULSE_COUNTING_GPIO);
    rtc_gpio_pullup_dis(ULP_PULSE_COUNTING_GPIO);
    rtc_gpio_hold_en(ULP_PULSE_COUNTING_GPIO);

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
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start ULP program: %s", esp_err_to_name(err));
        return;
    }
}

static void update_pulse_count(void)
{

    const char *count_key = "count";

    nvs_handle_t handle;
    esp_err_t err = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
        return;
    }

    uint32_t pulse_count = 0;
    err = nvs_get_u32(handle, count_key, &pulse_count);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "Error reading pulse count from NVS: %s", esp_err_to_name(err));
        nvs_close(handle);
        return;
    }

    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGI(TAG, "No pulse count found in NVS, starting at 0");
    }
    else
    {
        ESP_LOGD(TAG, "Read pulse count from NVS: %5" PRIu32, pulse_count);
    }

    /* ULP program counts signal edges, convert that to the number of pulses */
    uint32_t pulse_count_from_ulp = (ulp_edge_count & ULP_16BIT_MASK) / ULP_EDGES_PER_PULSE;
    /* In case of an odd number of edges, keep one until next time, result is 0 or 1! */
    ulp_edge_count = ulp_edge_count % ULP_EDGES_PER_PULSE;
    ESP_LOGI(TAG, "Pulse count from ULP: %5" PRIu32, pulse_count_from_ulp);

    /* Save the new pulse count to NVS */
    pulse_count += pulse_count_from_ulp;
    err = nvs_set_u32(handle, count_key, pulse_count);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write pulse count to NVS: %s", esp_err_to_name(err));
        nvs_close(handle);
        return;
    }

    err = nvs_commit(handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit changes to NVS: %s", esp_err_to_name(err));
    }
    else
    {
        ESP_LOGI(TAG, "Wrote updated pulse count to NVS: %5" PRIu32, pulse_count);
    }

    nvs_close(handle);

    // Debug: Print ULP variables after update
    ESP_LOGD(TAG, "ULP edge_count (after wakeup): %d", (int)ulp_edge_count);
    ESP_LOGD(TAG, "ULP io_number (RTC IO): %d", (int)ulp_io_number);
    ESP_LOGD(TAG, "ULP edge_count_to_wake_up: %d", (int)ulp_edge_count_to_wake_up);
}

static void update_timer_count(void)
{
    // const char *nvs_namespace = "pulsecnt";
    const char *count_key = "pulse";

    nvs_handle_t handle;
    esp_err_t err = nvs_open(nvs_namespace, NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
        return;
    }

    uint32_t timer_count_nvs = 1;
    err = nvs_get_u32(handle, count_key, &timer_count_nvs);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
    {
        ESP_LOGE(TAG, "Error reading timer count from NVS: %s", esp_err_to_name(err));
    }
    nvs_close(handle);

    // These variables are needed for the timer calculation regardless of debug mode
    uint32_t timer_low_h = (ulp_timer_count_low_h & ULP_16BIT_MASK);
    uint32_t timer_high = (ulp_timer_count_high & ULP_16BIT_MASK);

    // Combine high and low timer values to get the full 48-bit timer value
    uint64_t timer_value = ((uint64_t)timer_high << 32) |
                           ((uint32_t)timer_low_h << 16);

#ifdef CONFIG_ENABLE_DEBUG_OUTPUT
    // Detailed debug output - only compiled in debug builds
    uint32_t time_to_wakeup = (ulp_time_to_wake_CPU & ULP_16BIT_MASK);
    ESP_LOGD(TAG, "Time to wake CPU from ULP: %5" PRIu32, time_to_wakeup);
    ESP_LOGD(TAG, "Timer count high from ULP: %5" PRIu32, timer_low_h);
    ESP_LOGD(TAG, "Timer count upper from ULP: %5" PRIu32, timer_high);

    uint32_t start_time_low_h = (ulp_start_time_low_h & ULP_16BIT_MASK);
    ESP_LOGD(TAG, "Start_time_low_h: %5" PRIu32, start_time_low_h);

    uint32_t start_time_high = (ulp_start_time_high & ULP_16BIT_MASK);
    ESP_LOGD(TAG, "Start_time_high: %5" PRIu32, start_time_high);

    uint32_t timer_count = (ulp_timer_count & ULP_16BIT_MASK);
    ESP_LOGD(TAG, "Timer_count: %5" PRIu32, timer_count);

    uint32_t test_div = (ulp_test_div & ULP_16BIT_MASK);
    ESP_LOGD(TAG, "Test Value: %5" PRIu32, test_div);

    uint32_t edge_count = (ulp_edge_count & ULP_16BIT_MASK);
    ESP_LOGD(TAG, "Edge Count: %5" PRIu32, edge_count);

    uint32_t edge_count_to_wake_up = (ulp_edge_count_to_wake_up & ULP_16BIT_MASK);
    ESP_LOGD(TAG, "Edge Count to wake up: %5" PRIu32, edge_count_to_wake_up);

    ESP_LOGD(TAG, "ULP Timerwert: %llu Ticks", timer_value);
#endif // CONFIG_ENABLE_DEBUG_OUTPUT

    uint32_t ms = calculate_time_ms(timer_value);

    // Convert milliseconds to hours, minutes, and seconds
    int hours, minutes, seconds;
    format_time(ms, &hours, &minutes, &seconds);

    // Output the elapsed time in h:mm:ss format
    ESP_LOGI(TAG, "Elapsed Time: %02d:%02d:%02d", hours, minutes, seconds);
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
    *hours = ms / MS_PER_HOUR;                       // Calculate hours
    *minutes = (ms % MS_PER_HOUR) / MS_PER_MINUTE;   // Calculate minutes
    *seconds = (ms % MS_PER_MINUTE) / MS_PER_SECOND; // Calculate seconds
}

// Function to calculate the number of timer_count_low_h increments for a given interval
static uint16_t calculate_increments_for_interval(double interval_seconds)
{
    // Each timer_count_low_h increment corresponds to 65536 / 136000 ≈ 0.482 seconds
    double increments = interval_seconds / (TIMER_COUNTER_MAX / RTC_SLOW_CLK_FREQ);

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

//static volatile bool send_lora_on_ulp = false;

static void BlinkTask(void *arg)
{
    // Blink for 3 seconds (3 cycles of 1s)
    for (int i = 0; i < 3; i++)
    {
        esp_err_t led_err = led_strip_set_pixel(led_strip, 0, LED_COLOR_RED_R, LED_COLOR_RED_G, LED_COLOR_RED_B);
        if (led_err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set LED pixel (red): %s", esp_err_to_name(led_err));
        }

        led_err = led_strip_refresh(led_strip);
        if (led_err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to refresh LED strip: %s", esp_err_to_name(led_err));
        }

        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_INTERVAL_MS));

        led_err = led_strip_set_pixel(led_strip, 0, LED_COLOR_GREEN_R, LED_COLOR_GREEN_G, LED_COLOR_GREEN_B);
        if (led_err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to set LED pixel (green): %s", esp_err_to_name(led_err));
        }

        led_err = led_strip_refresh(led_strip);
        if (led_err != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to refresh LED strip: %s", esp_err_to_name(led_err));
        }
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_INTERVAL_MS));
    }
    esp_err_t led_err = led_strip_set_pixel(led_strip, 0, LED_COLOR_WHITE_R, LED_COLOR_WHITE_G, LED_COLOR_WHITE_B);
    if (led_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set LED pixel (white): %s", esp_err_to_name(led_err));
    }

    led_err = led_strip_refresh(led_strip);
    if (led_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to refresh LED strip: %s", esp_err_to_name(led_err));
    }

    vTaskDelay(pdMS_TO_TICKS(LED_BLINK_INTERVAL_MS));
    xEventGroupSetBits(blink_event_group, TASK_DONE_BIT);

    led_err = led_strip_clear(led_strip);
    if (led_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to clear LED strip: %s", esp_err_to_name(led_err));
    }
    ESP_LOGD(TAG, "BlinkTask finished, deleting task");
    vTaskDelete(NULL);
}

static void initialize_led(void)
{
    /* Configure the peripheral according to the LED type */
    configure_led();
    esp_err_t led_err = led_strip_set_pixel(led_strip, 0, LED_COLOR_GREEN_R, LED_COLOR_GREEN_G, LED_COLOR_GREEN_B);
    if (led_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set LED pixel: %s", esp_err_to_name(led_err));
    }

    /* Refresh the strip to send data */
    led_err = led_strip_refresh(led_strip);
    if (led_err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to refresh LED strip: %s", esp_err_to_name(led_err));
    }
}

static void configure_led(void)
{
    ESP_LOGD(TAG, "Example configured to blink addressable LED!");

    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };

    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    esp_err_t err = led_strip_new_spi_device(&strip_config, &spi_config, &led_strip);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create LED strip device: %s", esp_err_to_name(err));
        // Continue without LED functionality
    }
}

static void reset_counter(void)
{
    // const char *nvs_namespace = "pulsecnt";
    const char *count_key = "count";

    nvs_handle_t handle;
    esp_err_t err = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
        return;
    }

    uint32_t pulse_count = 0;
//    uint32_t timer_count = 0;

    err = nvs_set_u32(handle, count_key, pulse_count);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write pulse count to NVS: %s", esp_err_to_name(err));
        nvs_close(handle);
        return;
    }

    err = nvs_commit(handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit changes to NVS: %s", esp_err_to_name(err));
    }

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
        ESP_LOGW(TAG, "Garbage bytes detected at start of message (%d bytes):", (int)i);

        // Create a buffer for the hex string
        char hex_str[i * HEX_CHARS_PER_BYTE + 1]; // Space for hex representation + null terminator
        for (size_t j = 0; j < i; ++j)
        {
            sprintf(hex_str + j * HEX_CHARS_PER_BYTE, "%02X ", buf[j]);
        }
        ESP_LOGW(TAG, "GARBAGE HEX: %s", hex_str);

        ESP_LOGW(TAG, "Total garbage errors so far: %d", garbage_error_counter);
    }
    message_counter++;
    if (message_counter % 10 == 0)
    {
        ESP_LOGD(TAG, "Garbage error counter after %d messages: %d", message_counter, garbage_error_counter);
    }
}

static void print_buffer_hex(const uint8_t *buf, size_t len)
{
    // Create a buffer for the hex string
    char hex_str[len * HEX_CHARS_PER_BYTE + 1]; // Space for hex representation + null terminator
    for (size_t i = 0; i < len; i++)
    {
        sprintf(hex_str + i * HEX_CHARS_PER_BYTE, "%02X ", buf[i]);
    }
    ESP_LOGD(TAG, "HEX: %s", hex_str);
}

void send_lora_message(uint32_t pulse_count, int hours, int minutes, int seconds, uint32_t elapsed_ms, int send_counter)
{
    ESP_LOGD(TAG, "send packed struct payload");
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
    if (sizeof(lora_payload_t) > LORA_MAX_PAYLOAD_SIZE - 2) // Account for delimiter
    {
        ESP_LOGE(TAG, "ERROR: lora_payload_t size (%u bytes) exceeds LoRa E32 max payload size (%u bytes). Not sending!",
                (unsigned)sizeof(lora_payload_t), (unsigned)(LORA_MAX_PAYLOAD_SIZE - 2));
        return;
    }
    snprintf(elapsed_time_str, sizeof(elapsed_time_str), "%02d:%02d:%02d", hours, minutes, seconds);
    ESP_LOGI(TAG, "MessageID: %u", messageID);
    payload.messageID = messageID;                // Use messageID from NVS
    payload.lora_eventID = LORA_EVENT_ID_DEFAULT; // Example event ID, change as needed
    payload.elapsed_time_ms = elapsed_ms;
    payload.pulse_count = pulse_count;
    payload.checksum = lora_payload_checksum(&payload);

    ESP_LOGD(TAG, "Payload: time %s, ms %lu, pulses %lu, messageID %lu, checksum 0x%04X",
             elapsed_time_str,
             (unsigned long)payload.elapsed_time_ms,
             (unsigned long)payload.pulse_count,
             (unsigned long)payload.messageID,
             payload.checksum);
    ESP_LOGD(TAG, "Send counter: %lu", (unsigned long)payload.messageID);
    ESP_LOGD(TAG, "Checksum: 0x%04X", payload.checksum);
    
    // Use the new function to send with delimiter
    esp_err_t err = e32_send_message_with_delimiter((uint8_t *)&payload, sizeof(payload));
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to send LoRa message: %s", esp_err_to_name(err));
        // Continue execution, as this is a non-critical error
    }
    increment_and_store_messageId(&messageID);    // Increment and store new messageID in NVS
}

/**
 * @brief Callback function for task delays
 *
 * This function is passed to the E32 library to implement delays using FreeRTOS task delay.
 *
 * @param ms Delay time in milliseconds
 */
static void task_delay_callback(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief Receive and process LoRa messages
 *
 * This function uses the E32 library's message reception with terminator functionality
 * and processes the received messages. It handles:
 * 1. Message reception with timeout and terminator detection
 * 2. Processing of complete and partial messages
 * 3. Checking for garbage bytes at the start of messages
 *
 * The function delegates the polling and message accumulation to the library while
 * keeping the application-specific processing in this function.
 */
static void receive_lora_message(void)
{
    uint8_t rx_buffer[LORA_RX_BUFFER_SIZE];
    size_t total_received = 0;
    esp_err_t err = ESP_OK;
    ESP_LOGD(TAG, "receive message");
    
    // Initial delay before starting to receive
    vTaskDelay(pdMS_TO_TICKS(LORA_RECEIVE_DELAY_MS));
    
    // Use the enhanced function to receive a message with delimiter
    uint32_t start_time = pdTICKS_TO_MS(xTaskGetTickCount()); // Get time in ms using FreeRTOS
    err = e32_receive_message(
        rx_buffer,
        LORA_RX_BUFFER_SIZE,
        &total_received,
        LORA_REPLY_TIMEOUT_MS,
        LORA_RECEIVE_POLL_MS,
        task_delay_callback
    );
    
    uint32_t end_time = pdTICKS_TO_MS(xTaskGetTickCount()); // Get time in ms using FreeRTOS
    ESP_LOGI(TAG, "Message reception took %u ms", (unsigned)(end_time - start_time));
    
    // Process the received message
    if (total_received > 0 && total_received >= sizeof(lora_payload_t))
    {
        ESP_LOGD(TAG, "Received %u bytes, processing as lora_payload_t", (unsigned)total_received);
        
        // Copy buffer into struct
        lora_payload_t payload;
        memcpy(&payload, rx_buffer, sizeof(lora_payload_t));
        
        // Validate checksum
        uint16_t calc_checksum = lora_payload_checksum(&payload);
        bool checksum_ok = (calc_checksum == payload.checksum);
        
        // Print all fields
        ESP_LOGD(TAG, "Received Payload: ms %lu, pulses %lu, messageID %lu, eventID %lu, checksum 0x%04X %s",
                (unsigned long)payload.elapsed_time_ms,
                (unsigned long)payload.pulse_count,
                (unsigned long)payload.messageID,
                (unsigned long)payload.lora_eventID,
                payload.checksum,
                checksum_ok ? "(valid)" : "(INVALID)");
        
        ESP_LOGD(TAG, "Calc Checksum: 0x%04X", calc_checksum);
        
        // Print hex representation for debugging
        print_buffer_hex(rx_buffer, total_received);
    }
    else
    {
        ESP_LOGD(TAG, "No reply received or message too short (%u bytes)", (unsigned)total_received);
    }
}

uint16_t read_messageId_fromNVS()
{
    ESP_LOGD(TAG, "Reading messageID from NVS");
    nvs_handle_t handle;
    uint16_t messageId = 0; // default 0 on first run
    esp_err_t err = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
    if (err == ESP_OK)
    {
        err = nvs_get_u16(handle, "messageID", &messageId);
        if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            messageId = 0; // not found, start at 0
            ESP_LOGI(TAG, "No messageID found in NVS, starting at 0");
        }
        else if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error reading messageID from NVS: %s", esp_err_to_name(err));
            // handle error (log etc)
        }
        nvs_close(handle);
    }
    else
    {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        // handle error opening NVS
    }
    return messageId;
}

void increment_and_store_messageId(uint16_t *messageID)
{
    ESP_LOGD(TAG, "Incrementing and storing messageID in NVS %u", *messageID);
    *messageID = *messageID + 1; // Increment messageID

    nvs_handle_t handle;
    esp_err_t err = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to open NVS handle: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_u16(handle, "messageID", *messageID);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write messageID to NVS: %s", esp_err_to_name(err));
        nvs_close(handle);
        return;
    }

    err = nvs_commit(handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to commit changes to NVS: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
}