#pragma once
#include <stdint.h>

// 20250721  V0.9.10         BlinkTask duration reduced to 3 seconds, debug output for send counter and checksum, struct size check before LoRa send, checksum bug fix prompt for receiver

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
} lora_payload_t;

// Calculate checksum (simple sum of bytes, excluding checksum field)
static inline uint16_t lora_payload_checksum(const lora_payload_t *payload) {
    const uint8_t *data = (const uint8_t *)payload;
    size_t len = sizeof(lora_payload_t) - sizeof(uint16_t); // exclude checksum
    uint16_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum += data[i];
    }
    return sum;
}

// Usage:
// lora_payload_t payload;
// ... fill fields ...
// payload.checksum = lora_payload_checksum(&payload);
// send as raw bytes: e32_send_data((uint8_t *)&payload, sizeof(payload));
// On receiver: validate checksum before using data.

