# Configuration Message Implementation Plan (Final)

## Event Response Policy

**All received event IDs MUST send an acknowledgment response:**
- Configuration events → Send config response with actual values
- Control events (sleep mode) → Send simple ACK with event ID
- This ensures receiver knows command was received and processed

## Critical Design Review

### Issues with Previous Approach
1. ❌ **Bit packing too complex** - Hard to debug, error-prone
2. ❌ **Min/Max values in wrong file** - Should be in communication.h for receiver access
3. ❌ **Reusing send function unclear** - Separate functions are clearer
4. ✅ **Runtime config uncompressed** - Correct approach
5. ✅ **Error handling** - Good, keep it
6. ✅ **Validation needed** - Uses min/max constants

### Revised Approach
1. ✅ **Use separate, clean config structure** - No bit packing
2. ✅ **Min/Max in communication.h** - Shared between sender/receiver
3. ✅ **Separate send functions** - Clear purpose, shared helper
4. ✅ **Debug-friendly unpacking** - Log functions for troubleshooting
5. ✅ **Keep validation** - Enforces min/max at runtime

### Clarification: Two Types of "Packing"

**Type 1: Bit-Level Packing (REJECTED ❌)**
```c
// Complex bit operations to squeeze data:
uint32_t data = (value1 << 0) | (value2 << 8) | (value3 << 16);
// To read: uint8_t value1 = (data >> 0) & 0xFF;  // Hard to debug!
```
**Problems:** Complex, error-prone, hard to debug

**Type 2: Struct Packing (ACCEPTED ✅)**
```c
// Simple struct with __attribute__((packed)):
typedef struct __attribute__((packed)) {
    uint8_t value1;   // Direct access: payload.value1
    uint16_t value2;  // Direct access: payload.value2
} my_struct_t;
```
**Benefits:** Direct field access, easy debugging, no bit operations

**What This Plan Uses:**
- ✅ Struct packing (`__attribute__((packed))`) - just removes padding
- ❌ NOT bit-level packing - all fields directly accessible
- ✅ Runtime config stored uncompressed in RAM
- ✅ Only "packed" when sent over LoRa (as a struct)

---

## Implementation Steps

### Step 1: Define Configuration Structure and Constants

**File:** `include/communication.h`

#### 1.1 Add Configuration Limits (Shared with Receiver)
```c
// Configuration parameter limits (shared between sender and receiver)
#define CONFIG_MIN_ULP_PULSES 1
#define CONFIG_MAX_ULP_PULSES 100
#define CONFIG_MIN_WAKEUP_SEC 10
#define CONFIG_MAX_WAKEUP_SEC 3600
#define CONFIG_MIN_SHUTDOWN_MS 1000
#define CONFIG_MAX_SHUTDOWN_MS 30000
#define CONFIG_MIN_LORA_DELAY_MS 1000
#define CONFIG_MAX_LORA_DELAY_MS 30000

// Default configuration values
#define CONFIG_DEFAULT_ULP_PULSES 4
#define CONFIG_DEFAULT_WAKEUP_SEC 60
#define CONFIG_DEFAULT_SHUTDOWN_MS 4000
#define CONFIG_DEFAULT_LORA_DELAY_MS 6000
```

#### 1.2 Add New Event IDs
```c
// Configuration event IDs
#define LORA_EVENT_SET_CONFIG           0x0005  // Received: set configuration parameters
#define LORA_EVENT_SET_CONFIG_RESPONSE  0x1005  // Response: configuration applied (0x0005 + 0x1000)
#define LORA_EVENT_RESET_CONFIG         0x0006  // Received: reset configuration to defaults

// Response event ID encoding: Add 0x1000 to request event ID
// Examples:
//   LORA_EVENT_DISABLE_SLEEP_MODE (0x0002) → Response: 0x1002
//   LORA_EVENT_RESUME_SLEEP_MODE (0x0001) → Response: 0x1001
//   LORA_EVENT_SET_CONFIG (0x0005) → Response: 0x1005
```

#### 1.3 Define Clean Configuration Message Structure
```c
/**
 * @brief Configuration message structure
 * 
 * Used for LORA_EVENT_SET_CONFIG and LORA_EVENT_SET_CONFIG_RESPONSE
 * Total size: 16 bytes (well within 58 byte LoRa limit)
 */
typedef struct __attribute__((packed)) {
    uint16_t messageID;                 // Message ID
    uint16_t lora_eventID;             // LORA_EVENT_SET_CONFIG or LORA_EVENT_SET_CONFIG_RESPONSE
    uint8_t ulp_pulses_to_wake_up;     // Number of pulses to wake CPU (1-100)
    uint8_t reserved1;                  // Reserved for future use
    uint16_t wakeup_interval_sec;      // Wakeup interval in seconds (10-3600)
    uint16_t shutdown_delay_ms;        // Shutdown delay in milliseconds (1000-30000)
    uint16_t lora_receive_delay_ms;    // LoRa receive delay in milliseconds (1000-30000)
    uint16_t reserved2;                 // Reserved for future use
    uint16_t checksum;                  // Checksum (sum of all bytes except checksum field)
} lora_config_payload_t;
```

#### 1.4 Add Generic Checksum Macro
```c
/**
 * @brief Generic checksum calculation macro
 * 
 * Reuses the same checksum algorithm as lora_payload_checksum().
 * Works for any structure with a uint16_t checksum field at the end.
 * 
 * @param struct_ptr Pointer to structure
 * @param struct_type Type of the structure
 * @return Checksum value
 */
#define LORA_CALCULATE_CHECKSUM(struct_ptr, struct_type) ({ \
    const uint8_t *data = (const uint8_t *)(struct_ptr); \
    size_t len = sizeof(struct_type) - sizeof(uint16_t); \
    uint16_t sum = 0; \
    for (size_t i = 0; i < len; ++i) { \
        sum += data[i]; \
    } \
    sum; \
})

// Usage examples:
// lora_payload_t payload;
// payload.checksum = LORA_CALCULATE_CHECKSUM(&payload, lora_payload_t);
//
// lora_config_payload_t config;
// config.checksum = LORA_CALCULATE_CHECKSUM(&config, lora_config_payload_t);

// Keep existing function for backward compatibility
// (already defined, no changes needed)
```

#### 1.5 Add Debug Helper Function
```c
/**
 * @brief Log configuration payload contents for debugging
 * 
 * @param payload Pointer to configuration payload
 * @param tag Log tag to use
 */
static inline void lora_config_payload_log(const lora_config_payload_t *payload, const char *tag) {
    ESP_LOGI(tag, "Config Payload:");
    ESP_LOGI(tag, "  messageID: %u", payload->messageID);
    ESP_LOGI(tag, "  eventID: 0x%04X", payload->lora_eventID);
    ESP_LOGI(tag, "  ulp_pulses: %u", payload->ulp_pulses_to_wake_up);
    ESP_LOGI(tag, "  wakeup_sec: %u", payload->wakeup_interval_sec);
    ESP_LOGI(tag, "  shutdown_ms: %u", payload->shutdown_delay_ms);
    ESP_LOGI(tag, "  lora_delay_ms: %u", payload->lora_receive_delay_ms);
    ESP_LOGI(tag, "  checksum: 0x%04X", payload->checksum);
}
```

---

### Step 2: Create Runtime Configuration Structure

**File:** `main/rainsensor_main.c`

#### 2.1 Define Runtime Configuration Structure
```c
/**
 * @brief Runtime configuration structure
 * 
 * Stores uncompressed configuration values used during operation.
 * Values are loaded from NVS at startup and can be updated via LoRa messages.
 */
typedef struct {
    uint8_t ulp_pulses_to_wake_up;      // Number of pulses to wake CPU
    uint16_t wakeup_interval_seconds;   // Wakeup interval in seconds
    uint16_t shutdown_delay_ms;         // Shutdown delay in milliseconds
    uint16_t lora_receive_delay_ms;     // LoRa receive delay in milliseconds
} rainsensor_config_t;

// Global configuration instance (initialized with defaults from communication.h)
static rainsensor_config_t g_config = {
    .ulp_pulses_to_wake_up = CONFIG_DEFAULT_ULP_PULSES,
    .wakeup_interval_seconds = CONFIG_DEFAULT_WAKEUP_SEC,
    .shutdown_delay_ms = CONFIG_DEFAULT_SHUTDOWN_MS,
    .lora_receive_delay_ms = CONFIG_DEFAULT_LORA_DELAY_MS
};
```

---

### Step 3: Implement NVS Configuration Functions

**File:** `main/rainsensor_main.c`

#### 3.1 Add Forward Declarations
```c
static void load_config_from_nvs(void);
static void save_config_to_nvs(void);
static void reset_config_to_defaults(void);
static void send_config_response(void);
static void send_simple_ack(uint16_t request_event_id);
static bool validate_and_apply_config(rainsensor_config_t *config);
static void log_config(const char *prefix);
static void handle_set_config_event(const uint8_t *rx_buffer, size_t total_received);
static void handle_reset_config_event(void);
static void handle_disable_sleep_event(void);
static void handle_resume_sleep_event(void);
```

#### 3.2 Implement Configuration Validation
```c
/**
 * @brief Validate and apply configuration parameters
 * 
 * Validates configuration against limits defined in communication.h.
 * Invalid values are clamped to valid range or replaced with defaults.
 * 
 * @param config Configuration to validate and apply
 * @return true if all values were valid, false if any were corrected
 */
static bool validate_and_apply_config(rainsensor_config_t *config)
{
    bool all_valid = true;
    
    // Validate ulp_pulses_to_wake_up
    if (config->ulp_pulses_to_wake_up < CONFIG_MIN_ULP_PULSES || 
        config->ulp_pulses_to_wake_up > CONFIG_MAX_ULP_PULSES) {
        ESP_LOGW(TAG, "Invalid ulp_pulses: %u (range: %u-%u), using default: %u",
                 config->ulp_pulses_to_wake_up, 
                 CONFIG_MIN_ULP_PULSES, CONFIG_MAX_ULP_PULSES,
                 CONFIG_DEFAULT_ULP_PULSES);
        config->ulp_pulses_to_wake_up = CONFIG_DEFAULT_ULP_PULSES;
        all_valid = false;
    }
    
    // Validate wakeup_interval_seconds
    if (config->wakeup_interval_seconds < CONFIG_MIN_WAKEUP_SEC || 
        config->wakeup_interval_seconds > CONFIG_MAX_WAKEUP_SEC) {
        ESP_LOGW(TAG, "Invalid wakeup_sec: %u (range: %u-%u), using default: %u",
                 config->wakeup_interval_seconds,
                 CONFIG_MIN_WAKEUP_SEC, CONFIG_MAX_WAKEUP_SEC,
                 CONFIG_DEFAULT_WAKEUP_SEC);
        config->wakeup_interval_seconds = CONFIG_DEFAULT_WAKEUP_SEC;
        all_valid = false;
    }
    
    // Validate shutdown_delay_ms
    if (config->shutdown_delay_ms < CONFIG_MIN_SHUTDOWN_MS || 
        config->shutdown_delay_ms > CONFIG_MAX_SHUTDOWN_MS) {
        ESP_LOGW(TAG, "Invalid shutdown_ms: %u (range: %u-%u), using default: %u",
                 config->shutdown_delay_ms,
                 CONFIG_MIN_SHUTDOWN_MS, CONFIG_MAX_SHUTDOWN_MS,
                 CONFIG_DEFAULT_SHUTDOWN_MS);
        config->shutdown_delay_ms = CONFIG_DEFAULT_SHUTDOWN_MS;
        all_valid = false;
    }
    
    // Validate lora_receive_delay_ms
    if (config->lora_receive_delay_ms < CONFIG_MIN_LORA_DELAY_MS || 
        config->lora_receive_delay_ms > CONFIG_MAX_LORA_DELAY_MS) {
        ESP_LOGW(TAG, "Invalid lora_delay_ms: %u (range: %u-%u), using default: %u",
                 config->lora_receive_delay_ms,
                 CONFIG_MIN_LORA_DELAY_MS, CONFIG_MAX_LORA_DELAY_MS,
                 CONFIG_DEFAULT_LORA_DELAY_MS);
        config->lora_receive_delay_ms = CONFIG_DEFAULT_LORA_DELAY_MS;
        all_valid = false;
    }
    
    return all_valid;
}
```

#### 3.3 Implement Configuration Logging
```c
/**
 * @brief Log current configuration
 * 
 * @param prefix Prefix string for log message
 */
static void log_config(const char *prefix)
{
    ESP_LOGI(TAG, "%s: pulses=%u, wakeup=%us, shutdown=%ums, lora_delay=%ums",
             prefix,
             g_config.ulp_pulses_to_wake_up,
             g_config.wakeup_interval_seconds,
             g_config.shutdown_delay_ms,
             g_config.lora_receive_delay_ms);
}
```

#### 3.4 Implement Load Configuration
```c
/**
 * @brief Load configuration from NVS
 * 
 * Follows the same pattern as read_messageId_fromNVS().
 * Uses defaults if values not found in NVS.
 */
static void load_config_from_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(nvs_namespace, NVS_READONLY, &handle);
    
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for config load: %s, using defaults", 
                 esp_err_to_name(err));
        log_config("Default config");
        return;
    }
    
    // Load each configuration parameter (use defaults if not found)
    uint8_t pulses;
    if (nvs_get_u8(handle, "cfg_pulses", &pulses) == ESP_OK) {
        g_config.ulp_pulses_to_wake_up = pulses;
    }
    
    uint16_t wakeup_sec;
    if (nvs_get_u16(handle, "cfg_wakeup", &wakeup_sec) == ESP_OK) {
        g_config.wakeup_interval_seconds = wakeup_sec;
    }
    
    uint16_t shutdown_ms;
    if (nvs_get_u16(handle, "cfg_shutdown", &shutdown_ms) == ESP_OK) {
        g_config.shutdown_delay_ms = shutdown_ms;
    }
    
    uint16_t lora_delay_ms;
    if (nvs_get_u16(handle, "cfg_lora_dly", &lora_delay_ms) == ESP_OK) {
        g_config.lora_receive_delay_ms = lora_delay_ms;
    }
    
    nvs_close(handle);
    
    // Validate loaded configuration
    validate_and_apply_config(&g_config);
    log_config("Loaded config");
}
```

#### 3.5 Implement Save Configuration
```c
/**
 * @brief Save configuration to NVS
 * 
 * Follows the same pattern as increment_and_store_messageId().
 */
static void save_config_to_nvs(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(nvs_namespace, NVS_READWRITE, &handle);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS for config save: %s", esp_err_to_name(err));
        return;
    }
    
    // Save each configuration parameter
    bool save_failed = false;
    
    err = nvs_set_u8(handle, "cfg_pulses", g_config.ulp_pulses_to_wake_up);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save cfg_pulses: %s", esp_err_to_name(err));
        save_failed = true;
    }
    
    err = nvs_set_u16(handle, "cfg_wakeup", g_config.wakeup_interval_seconds);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save cfg_wakeup: %s", esp_err_to_name(err));
        save_failed = true;
    }
    
    err = nvs_set_u16(handle, "cfg_shutdown", g_config.shutdown_delay_ms);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save cfg_shutdown: %s", esp_err_to_name(err));
        save_failed = true;
    }
    
    err = nvs_set_u16(handle, "cfg_lora_dly", g_config.lora_receive_delay_ms);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save cfg_lora_dly: %s", esp_err_to_name(err));
        save_failed = true;
    }
    
    if (!save_failed) {
        err = nvs_commit(handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to commit config to NVS: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI(TAG, "Configuration saved to NVS successfully");
            log_config("Saved config");
        }
    }
    
    nvs_close(handle);
}
```

#### 3.6 Implement Reset Configuration
```c
/**
 * @brief Reset configuration to default values
 */
static void reset_config_to_defaults(void)
{
    ESP_LOGI(TAG, "Resetting configuration to defaults");
    
    g_config.ulp_pulses_to_wake_up = CONFIG_DEFAULT_ULP_PULSES;
    g_config.wakeup_interval_seconds = CONFIG_DEFAULT_WAKEUP_SEC;
    g_config.shutdown_delay_ms = CONFIG_DEFAULT_SHUTDOWN_MS;
    g_config.lora_receive_delay_ms = CONFIG_DEFAULT_LORA_DELAY_MS;
    
    log_config("Reset config");
    save_config_to_nvs();
    send_config_response();
}
```

---

### Step 4: Implement Event Handler Functions

**File:** `main/rainsensor_main.c`

#### 4.1 Implement Configuration Set Handler
```c
/**
 * @brief Handle LORA_EVENT_SET_CONFIG event
 * 
 * @param rx_buffer Received message buffer
 * @param total_received Number of bytes received
 */
static void handle_set_config_event(const uint8_t *rx_buffer, size_t total_received)
{
    ESP_LOGI(TAG, "Handling LORA_EVENT_SET_CONFIG");
    
    // Check if we received enough data for config message
    if (total_received < sizeof(lora_config_payload_t)) {
        ESP_LOGW(TAG, "Config message too short (%u bytes, expected %u) - ignoring",
                 (unsigned)total_received, (unsigned)sizeof(lora_config_payload_t));
        return;
    }
    
    lora_config_payload_t config_msg;
    memcpy(&config_msg, rx_buffer, sizeof(lora_config_payload_t));
    
    // Log received configuration for debugging
    lora_config_payload_log(&config_msg, TAG);
    
    // Validate checksum using generic macro
    uint16_t calc_checksum = LORA_CALCULATE_CHECKSUM(&config_msg, lora_config_payload_t);
    if (calc_checksum != config_msg.checksum) {
        ESP_LOGW(TAG, "Config message checksum invalid (calc: 0x%04X, recv: 0x%04X) - ignoring",
                 calc_checksum, config_msg.checksum);
        return;
    }
    
    ESP_LOGI(TAG, "Checksum valid, applying configuration");
    
    // Create temporary config for validation
    rainsensor_config_t temp_config = {
        .ulp_pulses_to_wake_up = config_msg.ulp_pulses_to_wake_up,
        .wakeup_interval_seconds = config_msg.wakeup_interval_sec,
        .shutdown_delay_ms = config_msg.shutdown_delay_ms,
        .lora_receive_delay_ms = config_msg.lora_receive_delay_ms
    };
    
    // Validate and apply
    bool all_valid = validate_and_apply_config(&temp_config);
    g_config = temp_config;
    
    if (all_valid) {
        ESP_LOGI(TAG, "All configuration values valid");
    } else {
        ESP_LOGW(TAG, "Some configuration values were corrected");
    }
    
    log_config("Applied config");
    
    // Save to NVS
    save_config_to_nvs();
    
    // Send response with actual applied values
    send_config_response();
}
```

#### 4.2 Implement Configuration Reset Handler
```c
/**
 * @brief Handle LORA_EVENT_RESET_CONFIG event
 */
static void handle_reset_config_event(void)
{
    ESP_LOGI(TAG, "Handling LORA_EVENT_RESET_CONFIG");
    reset_config_to_defaults();  // This already calls send_config_response()
}
```

#### 4.3 Implement Sleep Mode Control Handlers
```c
/**
 * @brief Handle LORA_EVENT_DISABLE_SLEEP_MODE event
 */
static void handle_disable_sleep_event(void)
{
    ESP_LOGI(TAG, "Handling LORA_EVENT_DISABLE_SLEEP_MODE - disabling sleep mode");
    skip_deep_sleep = true;
    send_simple_ack(LORA_EVENT_DISABLE_SLEEP_MODE);
}

/**
 * @brief Handle LORA_EVENT_RESUME_SLEEP_MODE event
 */
static void handle_resume_sleep_event(void)
{
    ESP_LOGI(TAG, "Handling LORA_EVENT_RESUME_SLEEP_MODE - resuming sleep mode");
    skip_deep_sleep = false;
    send_simple_ack(LORA_EVENT_RESUME_SLEEP_MODE);
}
```

#### 4.4 Update receive_lora_message() Switch Statement
Replace the existing switch cases with clean function calls:

```c
// In receive_lora_message(), replace the switch statement with:
switch (payload.lora_eventID)
{
    case LORA_EVENT_DISABLE_SLEEP_MODE:
        handle_disable_sleep_event();
        break;
        
    case LORA_EVENT_RESUME_SLEEP_MODE:
        handle_resume_sleep_event();
        break;
        
    case LORA_EVENT_SET_CONFIG:
        handle_set_config_event(rx_buffer, total_received);
        break;
        
    case LORA_EVENT_RESET_CONFIG:
        handle_reset_config_event();
        break;
        
    case LORA_EVENT_SEND_LORA_PARAMS:
        ESP_LOGI(TAG, "Received LORA_EVENT_SEND_LORA_PARAMS - not yet implemented");
        // TODO: Implement sending LORA parameters
        break;
        
    case LORA_EVENT_SEND_PROG_PARAMS:
        ESP_LOGI(TAG, "Received LORA_EVENT_SEND_PROG_PARAMS - not yet implemented");
        // TODO: Implement sending program parameters
        break;
        
    default:
        ESP_LOGW(TAG, "Received unknown event ID: 0x%04X", payload.lora_eventID);
        break;
}
```

---

### Step 5: Implement Response Sender Functions

**File:** `main/rainsensor_main.c`

#### 5.1 Create Shared Send Helper Function
```c
/**
 * @brief Send LoRa message with delimiter (shared helper)
 * 
 * @param data Pointer to data to send
 * @param size Size of data in bytes
 * @param messageID Pointer to messageID (will be incremented)
 * @return ESP_OK on success, error code otherwise
 */
static esp_err_t send_lora_with_delimiter(const uint8_t *data, size_t size, uint16_t *messageID)
{
    esp_err_t err = e32_send_message_with_delimiter(data, size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send LoRa message: %s", esp_err_to_name(err));
        return err;
    }
    
    // Increment and store messageID
    increment_and_store_messageId(messageID);
    return ESP_OK;
}
```

#### 5.2 Implement Simple ACK Response Sender
```c
/**
 * @brief Send simple acknowledgment response
 * 
 * Sends ACK for events that don't have specific response data (e.g., sleep mode control).
 * Uses existing lora_payload_t structure with response event ID.
 * Response event ID = Request event ID + 0x1000
 * 
 * @param request_event_id The event ID that was received
 */
static void send_simple_ack(uint16_t request_event_id)
{
    ESP_LOGI(TAG, "Sending ACK for event 0x%04X", request_event_id);
    
    lora_payload_t ack;
    
    // Read messageID from NVS (reuse existing function)
    uint16_t messageID = read_messageId_fromNVS();
    
    // Fill ACK response
    ack.messageID = messageID;
    ack.lora_eventID = request_event_id + 0x1000;  // Response = Request + 0x1000
    ack.elapsed_time_ms = 0;  // Not used for ACK
    ack.pulse_count = 0;      // Not used for ACK
    ack.checksum = lora_payload_checksum(&ack);
    
    ESP_LOGI(TAG, "ACK: messageID=%u, responseEventID=0x%04X", messageID, ack.lora_eventID);
    
    // Send ACK using shared helper
    send_lora_with_delimiter((uint8_t *)&ack, sizeof(ack), &messageID);
}
```

#### 5.3 Implement Configuration Response Sender
```c
/**
 * @brief Send configuration response message
 * 
 * Sends current configuration back to receiver using LORA_EVENT_SET_CONFIG_RESPONSE.
 * This is a separate function from send_lora_message() for clarity.
 */
static void send_config_response(void)
{
    ESP_LOGI(TAG, "Sending configuration response");
    
    lora_config_payload_t response;
    
    // Read messageID from NVS (reuse existing function)
    uint16_t messageID = read_messageId_fromNVS();
    
    // Fill response with current configuration
    response.messageID = messageID;
    response.lora_eventID = LORA_EVENT_SET_CONFIG_RESPONSE;
    response.ulp_pulses_to_wake_up = g_config.ulp_pulses_to_wake_up;
    response.reserved1 = 0;
    response.wakeup_interval_sec = g_config.wakeup_interval_seconds;
    response.shutdown_delay_ms = g_config.shutdown_delay_ms;
    response.lora_receive_delay_ms = g_config.lora_receive_delay_ms;
    response.reserved2 = 0;
    response.checksum = LORA_CALCULATE_CHECKSUM(&response, lora_config_payload_t);
    
    // Log response for debugging
    lora_config_payload_log(&response, TAG);
    
    // Send response using shared helper
    send_lora_with_delimiter((uint8_t *)&response, sizeof(response), &messageID);
}
```

#### 5.4 Update Existing send_lora_message()
```c
// In send_lora_message(), replace the send and increment code with:
send_lora_with_delimiter((uint8_t *)&payload, sizeof(payload), &messageID);
```

---

### Step 6: Update Code to Use Runtime Configuration

**File:** `main/rainsensor_main.c`

#### 6.1 Update initialize_system()
```c
// In initialize_system(), after nvs_flash_init():
// Load configuration from NVS
load_config_from_nvs();
```

#### 6.2 Update init_ulp_program()
```c
// Replace hardcoded values with runtime configuration:
ulp_edge_count_to_wake_up = g_config.ulp_pulses_to_wake_up * ULP_EDGES_PER_PULSE;

// Replace:
// uint16_t increments = calculate_increments_for_interval(wakeup_interval_seconds);
// With:
uint16_t increments = calculate_increments_for_interval((double)g_config.wakeup_interval_seconds);

// Update debug log:
ESP_LOGD(TAG, "Wake-up interval: %u seconds -> Increments: %u", 
         g_config.wakeup_interval_seconds, increments);
```

#### 6.3 Update prepare_for_deep_sleep()
```c
// Replace:
// vTaskDelay(pdMS_TO_TICKS(SHUTDOWN_DELAY_MS));
// With:
vTaskDelay(pdMS_TO_TICKS(g_config.shutdown_delay_ms));
```

#### 6.4 Update receive_lora_message()
```c
// Replace:
// vTaskDelay(pdMS_TO_TICKS(LORA_RECEIVE_DELAY_MS));
// With:
vTaskDelay(pdMS_TO_TICKS(g_config.lora_receive_delay_ms));
```

---

## Summary of Improvements

### What Changed from Previous Plan
1. ✅ **No bit packing** - Clean, readable structure (16 bytes vs 14 bytes)
2. ✅ **Min/Max in communication.h** - Shared with receiver
3. ✅ **Separate send functions** - Clear purpose, shared helper
4. ✅ **Debug logging** - `lora_config_payload_log()` for troubleshooting
5. ✅ **Better error handling** - Detailed logging at each step
6. ✅ **Validation uses constants** - Single source of truth

### Benefits
- **Debuggable** - Clear structure, logging at every step
- **Maintainable** - No complex bit manipulation
- **Extensible** - 2 reserved bytes for future parameters
- **Consistent** - Follows existing code patterns
- **Shared** - Min/Max available to receiver for validation

### Trade-offs
- **Size:** 16 bytes vs 14 bytes (2 byte overhead = 3.4% of 58 byte limit)
- **Benefit:** Much clearer code, easier debugging, less error-prone

---

## Testing Plan

### Test Case 1: Configuration Message Reception
1. Send configuration message with new values
2. Verify rainsensor logs received values
3. Verify rainsensor applies configuration
4. Verify rainsensor sends response with applied values
5. Verify configuration persists after reboot

### Test Case 2: Reset Configuration
1. Send reset configuration message
2. Verify rainsensor resets to default values
3. Verify rainsensor sends response with default values
4. Verify defaults persist after reboot

### Test Case 3: Invalid Checksum
1. Send configuration message with invalid checksum
2. Verify rainsensor logs checksum mismatch
3. Verify rainsensor ignores message
4. Verify configuration remains unchanged

### Test Case 4: Out-of-Range Values
1. Send configuration with values outside valid ranges
2. Verify rainsensor logs warnings for invalid values
3. Verify rainsensor uses defaults for invalid values
4. Verify response contains actual applied values (defaults)

### Test Case 5: Configuration Persistence
1. Set custom configuration
2. Reboot rainsensor
3. Verify custom configuration is loaded from NVS
4. Verify ULP uses custom values

### Test Case 6: Normal Messages Still Work
1. Send normal sensor data message
2. Verify rainsensor processes it correctly
3. Verify configuration messages don't interfere

### Test Case 7: Debug Logging
1. Enable debug logging
2. Send configuration message
3. Verify all steps are logged clearly
4. Verify unpacked values are visible in logs

---

## Memory Considerations

### NVS Storage (in "pulsecnt" namespace)
**Existing:**
- `count`: 4 bytes
- `pulse`: 4 bytes
- `messageID`: 2 bytes

**New:**
- `cfg_pulses`: 1 byte
- `cfg_wakeup`: 2 bytes
- `cfg_shutdown`: 2 bytes
- `cfg_lora_dly`: 2 bytes

**Total: 17 bytes** (10 existing + 7 new)

### Message Sizes
- Normal message ([`lora_payload_t`](include/communication.h:22)): 14 bytes
- Config message (`lora_config_payload_t`): 16 bytes
- Both well within 58 byte LoRa limit

### RAM Usage
- `rainsensor_config_t`: 7 bytes
- Minimal impact

---

## Final Recommendation

This plan provides:
- ✅ Clean, debuggable code
- ✅ Shared constants between sender/receiver
- ✅ Separate, clear functions
- ✅ Comprehensive error handling
- ✅ Detailed logging for troubleshooting
- ✅ Minimal overhead (2 bytes)
- ✅ Follows existing patterns
- ✅ Easy to extend in future

**Ready for implementation in Code mode.**
