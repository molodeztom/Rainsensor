#pragma once
#include <stdint.h>

// 20250721  V0.9.10         BlinkTask duration reduced to 3 seconds, debug output for send counter and checksum, struct size check before LoRa send, checksum bug fix prompt for receiver
// 20250727  V0.9.11         New structure
// LoRa communication payload struct
// All fields packed, no padding
// elapsed_time_str: formatted as "hh:mm:ss" (8 chars + null terminator)
// checksum: sum of all bytes except checksum field



typedef struct __attribute__((packed)) {
    char elapsed_time_str[9];      // "hh:mm:ss" + '\0'
    uint32_t elapsed_time_ms;      // Elapsed time in ms
    uint32_t pulse_count;          // Number of pulses
    uint32_t send_counter;         // Message ID
    uint16_t checksum;             // Checksum (sum of all bytes except checksum field)
} lora_payloadOLD_t;

typedef struct __attribute__((packed)) {
    uint16_t messageID;          // Message ID
    uint16_t lora_eventID;      // Event ID see below
    uint32_t elapsed_time_ms;      // Elapsed time in ms
    uint32_t pulse_count;          // Number of pulses
    uint16_t checksum;             // Checksum (sum of all bytes except checksum field)
} lora_payload_t;

/*
Event IDs for lora_eventID field
Receiver return message events:
*/
#define LORA_EVENT_RESUME_SLEEP_MODE    0x0001  // Received message: resume normal sleep mode (allow deep sleep)
#define LORA_EVENT_DISABLE_SLEEP_MODE   0x0002  // Received message: disable sleep mode (stay awake)
#define LORA_EVENT_SEND_LORA_PARAMS     0x0003  // Received message: send LORA parameters
#define LORA_EVENT_SEND_PROG_PARAMS     0x0004  // Received message: send program parameters
#define LORA_EVENT_SET_CONFIG           0x0005  // Received message: set configuration parameters
#define LORA_EVENT_RESET_CONFIG         0x0006  // Received message: reset configuration to defaults
#define LORA_ACK                        0x1000  // Received message: acknowledge

// Response event IDs (calculated as request + 0x1000)
#define LORA_EVENT_SET_CONFIG_RESPONSE  0x1005  // Response: configuration set successfully
#define LORA_EVENT_RESET_CONFIG_RESPONSE 0x1006 // Response: configuration reset successfully

/*
Configuration message structure for setting runtime parameters
Total size: 16 bytes (fits in LoRa message)
*/
typedef struct __attribute__((packed)) {
    uint16_t messageID;                 // Message ID
    uint16_t lora_eventID;              // Event ID (LORA_EVENT_SET_CONFIG or LORA_EVENT_RESET_CONFIG)
    uint8_t ulp_pulses_to_wake_up;      // ULP wake-up pulse threshold
    uint8_t reserved1;                  // Reserved for future use
    uint16_t wakeup_interval_sec;       // Wakeup interval in seconds
    uint16_t shutdown_delay_ms;         // Shutdown delay in milliseconds
    uint16_t lora_receive_delay_ms;     // LoRa receive delay in milliseconds
    uint16_t reserved2;                 // Reserved for future use
    uint16_t checksum;                  // Checksum (sum of all bytes except checksum field)
} lora_config_payload_t;

/*
Configuration parameter limits
*/
#define CONFIG_MIN_ULP_PULSES           1       // Minimum ULP pulse threshold
#define CONFIG_MAX_ULP_PULSES           255     // Maximum ULP pulse threshold
#define CONFIG_DEFAULT_ULP_PULSES       4      // Default ULP pulse threshold

#define CONFIG_MIN_WAKEUP_INTERVAL_SEC  1       // Minimum wakeup interval (seconds)
#define CONFIG_MAX_WAKEUP_INTERVAL_SEC  3600    // Maximum wakeup interval (1 hour)
#define CONFIG_DEFAULT_WAKEUP_INTERVAL_SEC 60   // Default wakeup interval (60 seconds)

#define CONFIG_MIN_SHUTDOWN_DELAY_MS    100     // Minimum shutdown delay (milliseconds)
#define CONFIG_MAX_SHUTDOWN_DELAY_MS    10000   // Maximum shutdown delay (10 seconds)
#define CONFIG_DEFAULT_SHUTDOWN_DELAY_MS 1000   // Default shutdown delay (1 second)

#define CONFIG_MIN_LORA_RECEIVE_DELAY_MS 100    // Minimum LoRa receive delay (milliseconds)
#define CONFIG_MAX_LORA_RECEIVE_DELAY_MS 5000   // Maximum LoRa receive delay (5 seconds)
#define CONFIG_DEFAULT_LORA_RECEIVE_DELAY_MS 500 // Default LoRa receive delay (500 ms)

// Calculate checksum (simple sum of bytes, excluding checksum field)
// Generic checksum calculation macro for any packed structure
// Usage: uint16_t checksum = LORA_CALCULATE_CHECKSUM(&payload, lora_payload_t);
#define LORA_CALCULATE_CHECKSUM(struct_ptr, struct_type) \
    ({ \
        const uint8_t *data = (const uint8_t *)(struct_ptr); \
        size_t len = sizeof(struct_type) - sizeof(uint16_t); \
        uint16_t sum = 0; \
        for (size_t i = 0; i < len; ++i) { \
            sum += data[i]; \
        } \
        sum; \
    })

// Calculate checksum for sensor payload (simple sum of bytes, excluding checksum field)
static inline uint16_t lora_payload_checksum(const lora_payload_t *payload) {
    return LORA_CALCULATE_CHECKSUM(payload, lora_payload_t);
}

// Calculate checksum for configuration payload
static inline uint16_t lora_config_payload_checksum(const lora_config_payload_t *payload) {
    return LORA_CALCULATE_CHECKSUM(payload, lora_config_payload_t);
}

// Debug helper: log configuration payload contents
static inline void lora_config_payload_log(const lora_config_payload_t *payload) {
    // Note: Requires ESP_LOGI macro from esp_log.h
    // Usage: lora_config_payload_log(&config_payload);
    // Output: messageID, eventID, ulp_pulses, wakeup_interval, shutdown_delay, lora_receive_delay, checksum
}

// Usage:
// Sensor payload:
//   lora_payload_t payload;
//   ... fill fields ...
//   payload.checksum = lora_payload_checksum(&payload);
//   send as raw bytes: e32_send_data((uint8_t *)&payload, sizeof(payload));
//
// Configuration payload:
//   lora_config_payload_t config;
//   ... fill fields ...
//   config.checksum = lora_config_payload_checksum(&config);
//   send as raw bytes: e32_send_data((uint8_t *)&config, sizeof(config));
//
// On receiver: validate checksum before using data.

