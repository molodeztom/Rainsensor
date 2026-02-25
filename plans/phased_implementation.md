# Phased Implementation Plan - Configuration Message System

## Overview
Implement the configuration message system in 6 testable phases, allowing incremental development and validation at each step.

---

## Phase 1: Protocol & Constants

**Goal:** Define the communication protocol foundation

**Files:** `include/communication.h`

**Tasks:**
1. Add configuration parameter limits (min/max values)
2. Add default configuration values
3. Add new event IDs (SET_CONFIG, RESET_CONFIG)
4. Define `lora_config_payload_t` structure
5. Add generic checksum macro
6. Add debug logging helper function

**Deliverables:**
- All constants defined and documented
- Structures defined with clear comments
- Checksum macro tested with example

**Testing:**
- Compile check - no errors
- Verify structure size (should be 16 bytes)
- Manual checksum calculation test

**Code Changes:**
```c
// Configuration parameter limits
#define CONFIG_MIN_ULP_PULSES 1
#define CONFIG_MAX_ULP_PULSES 100
// ... (all limits)

// Default values
#define CONFIG_DEFAULT_ULP_PULSES 4
// ... (all defaults)

// Event IDs
#define LORA_EVENT_SET_CONFIG 0x0005
#define LORA_EVENT_SET_CONFIG_RESPONSE 0x1005
#define LORA_EVENT_RESET_CONFIG 0x0006

// Structure
typedef struct __attribute__((packed)) {
    uint16_t messageID;
    uint16_t lora_eventID;
    uint8_t ulp_pulses_to_wake_up;
    uint8_t reserved1;
    uint16_t wakeup_interval_sec;
    uint16_t shutdown_delay_ms;
    uint16_t lora_receive_delay_ms;
    uint16_t reserved2;
    uint16_t checksum;
} lora_config_payload_t;

// Generic checksum macro
#define LORA_CALCULATE_CHECKSUM(struct_ptr, struct_type) ...

// Debug helper
static inline void lora_config_payload_log(...) ...
```

---

## Phase 2: Parsing & Validation

**Goal:** Implement message parsing and validation without applying changes

**Files:** `main/rainsensor_main.c`

**Tasks:**
1. Create `rainsensor_config_t` runtime structure
2. Initialize global config with defaults
3. Implement `validate_and_apply_config()` function
4. Implement `log_config()` helper
5. Add forward declarations for all handlers

**Deliverables:**
- Runtime config structure defined
- Validation function with range checking
- Logging helper for debugging

**Testing:**
- Test validation with valid values (should pass)
- Test validation with out-of-range values (should clamp/default)
- Test validation with edge cases (min, max, zero)
- Verify logging output is clear

**Code Changes:**
```c
// Runtime config structure
typedef struct {
    uint8_t ulp_pulses_to_wake_up;
    uint16_t wakeup_interval_seconds;
    uint16_t shutdown_delay_ms;
    uint16_t lora_receive_delay_ms;
} rainsensor_config_t;

// Global instance
static rainsensor_config_t g_config = {
    .ulp_pulses_to_wake_up = CONFIG_DEFAULT_ULP_PULSES,
    // ... (all defaults)
};

// Validation function
static bool validate_and_apply_config(rainsensor_config_t *config) {
    // Check all parameters against CONFIG_MIN_* and CONFIG_MAX_*
    // Log warnings for invalid values
    // Replace invalid values with defaults
    // Return true if all valid, false if any corrected
}

// Logging helper
static void log_config(const char *prefix) {
    ESP_LOGI(TAG, "%s: pulses=%u, wakeup=%us, ...", prefix, ...);
}
```

---

## Phase 3: Apply & Response

**Goal:** Implement parameter application and response sending (without persistence)

**Files:** `main/rainsensor_main.c`

**Tasks:**
1. Implement `send_simple_ack()` for control events
2. Implement `send_config_response()` for config events
3. Implement `send_lora_with_delimiter()` shared helper
4. Implement event handler functions:
   - `handle_set_config_event()`
   - `handle_reset_config_event()`
   - `handle_disable_sleep_event()`
   - `handle_resume_sleep_event()`
5. Update switch statement in `receive_lora_message()` to call handlers

**Deliverables:**
- All event handlers implemented
- Response messages sent correctly
- ACK/NACK logic working

**Testing:**
- Send SET_CONFIG message → verify response with applied values
- Send RESET_CONFIG message → verify response with defaults
- Send DISABLE_SLEEP_MODE → verify ACK received
- Send RESUME_SLEEP_MODE → verify ACK received
- Verify response event IDs are correct (request + 0x1000)
- **Note:** Configuration won't persist across reboots yet

**Code Changes:**
```c
// Shared send helper
static esp_err_t send_lora_with_delimiter(...) {
    // Send message
    // Increment messageID
}

// Simple ACK sender
static void send_simple_ack(uint16_t request_event_id) {
    // Create ACK with response event ID = request + 0x1000
    // Send using shared helper
}

// Config response sender
static void send_config_response(void) {
    // Create config response with current g_config values
    // Send using shared helper
}

// Event handlers
static void handle_set_config_event(...) {
    // Parse message
    // Validate checksum
    // Validate parameters
    // Apply to g_config
    // Send response
}

static void handle_reset_config_event(void) {
    // Reset g_config to defaults
    // Send response
}

static void handle_disable_sleep_event(void) {
    // Set skip_deep_sleep = true
    // Send ACK
}

static void handle_resume_sleep_event(void) {
    // Set skip_deep_sleep = false
    // Send ACK
}

// Update switch statement
switch (payload.lora_eventID) {
    case LORA_EVENT_SET_CONFIG:
        handle_set_config_event(rx_buffer, total_received);
        break;
    // ... (all other cases)
}
```

---

## Phase 4: Persistence

**Goal:** Add NVS storage for configuration persistence

**Files:** `main/rainsensor_main.c`

**Tasks:**
1. Implement `load_config_from_nvs()`
2. Implement `save_config_to_nvs()`
3. Update `initialize_system()` to load config at startup
4. Update `handle_set_config_event()` to save after applying
5. Update `reset_config_to_defaults()` to save after resetting

**Deliverables:**
- Configuration persists across reboots
- NVS keys use "cfg_" prefix in "pulsecnt" namespace
- Error handling for NVS operations

**Testing:**
- Set custom configuration
- Reboot ESP32
- Verify configuration loaded from NVS
- Verify defaults used if NVS empty
- Test NVS error handling (simulate failures)

**Code Changes:**
```c
// Load from NVS
static void load_config_from_nvs(void) {
    // Open NVS namespace "pulsecnt"
    // Read cfg_pulses, cfg_wakeup, cfg_shutdown, cfg_lora_dly
    // Use defaults if not found
    // Validate loaded config
    // Log loaded config
}

// Save to NVS
static void save_config_to_nvs(void) {
    // Open NVS namespace "pulsecnt"
    // Write cfg_pulses, cfg_wakeup, cfg_shutdown, cfg_lora_dly
    // Commit changes
    // Log success/failure
}

// Update initialize_system()
static bool initialize_system(void) {
    // ... existing code ...
    if (nvs_flash_init() != ESP_OK) {
        // ... error handling ...
    }
    
    // NEW: Load configuration from NVS
    load_config_from_nvs();
    
    // ... rest of initialization ...
}

// Update handlers to save
static void handle_set_config_event(...) {
    // ... validation and apply ...
    save_config_to_nvs();  // NEW: Save after applying
    send_config_response();
}

static void reset_config_to_defaults(void) {
    // ... reset to defaults ...
    save_config_to_nvs();  // NEW: Save after resetting
    send_config_response();
}
```

---

## Phase 5: Integration

**Goal:** Update existing code to use runtime configuration

**Files:** `main/rainsensor_main.c`

**Tasks:**
1. Update `init_ulp_program()` to use `g_config.ulp_pulses_to_wake_up`
2. Update `init_ulp_program()` to use `g_config.wakeup_interval_seconds`
3. Update `prepare_for_deep_sleep()` to use `g_config.shutdown_delay_ms`
4. Update `receive_lora_message()` to use `g_config.lora_receive_delay_ms`
5. Remove or comment out old hardcoded constants

**Deliverables:**
- All hardcoded values replaced with runtime config
- System uses configuration from NVS
- Configuration changes take effect immediately

**Testing:**
- Set custom configuration
- Verify ULP uses new pulse threshold
- Verify wakeup interval changes
- Verify delays use new values
- Test with defaults
- Test with extreme values (within valid range)

**Code Changes:**
```c
// In init_ulp_program():
// OLD: ulp_edge_count_to_wake_up = ULP_PULSES_TO_WAKE_UP * ULP_EDGES_PER_PULSE;
// NEW:
ulp_edge_count_to_wake_up = g_config.ulp_pulses_to_wake_up * ULP_EDGES_PER_PULSE;

// OLD: uint16_t increments = calculate_increments_for_interval(wakeup_interval_seconds);
// NEW:
uint16_t increments = calculate_increments_for_interval((double)g_config.wakeup_interval_seconds);

// In prepare_for_deep_sleep():
// OLD: vTaskDelay(pdMS_TO_TICKS(SHUTDOWN_DELAY_MS));
// NEW:
vTaskDelay(pdMS_TO_TICKS(g_config.shutdown_delay_ms));

// In receive_lora_message():
// OLD: vTaskDelay(pdMS_TO_TICKS(LORA_RECEIVE_DELAY_MS));
// NEW:
vTaskDelay(pdMS_TO_TICKS(g_config.lora_receive_delay_ms));
```

---

## Phase 6: Testing & Validation

**Goal:** Comprehensive end-to-end testing

**Files:** All

**Tasks:**
1. Test normal operation (no config changes)
2. Test configuration message flow
3. Test reset configuration
4. Test persistence across reboots
5. Test invalid messages (bad checksum, wrong size)
6. Test out-of-range values
7. Test sleep mode control with configuration
8. Performance testing (message timing)

**Test Scenarios:**

### Test 1: Normal Operation
- Boot ESP32
- Verify defaults loaded
- Send sensor data
- Verify normal operation unchanged

### Test 2: Set Configuration
- Send SET_CONFIG with new values
- Verify response contains applied values
- Verify configuration used immediately
- Reboot and verify persistence

### Test 3: Reset Configuration
- Set custom configuration
- Send RESET_CONFIG
- Verify response contains defaults
- Verify defaults applied
- Reboot and verify defaults persist

### Test 4: Invalid Messages
- Send config with bad checksum → verify ignored
- Send config too short → verify ignored
- Send config with out-of-range values → verify clamped/defaulted

### Test 5: Sleep Mode Control
- Send DISABLE_SLEEP_MODE → verify ACK, stays awake
- Trigger ULP wakeup → verify handled
- Send RESUME_SLEEP_MODE → verify ACK, enters sleep

### Test 6: Configuration Limits
- Send config with values at min limits → verify accepted
- Send config with values at max limits → verify accepted
- Send config with values below min → verify defaulted
- Send config with values above max → verify defaulted

### Test 7: Message Timing
- Measure time to process config message
- Measure time to send response
- Verify no timeout issues

### Test 8: Stress Testing
- Send multiple config changes rapidly
- Verify all processed correctly
- Verify NVS not corrupted
- Verify messageID increments correctly

---

## Phase Summary

| Phase | Focus | Can Test Independently | Depends On |
|-------|-------|----------------------|------------|
| 1 | Protocol & Constants | ✅ Yes (compile check) | None |
| 2 | Parsing & Validation | ✅ Yes (unit test validation) | Phase 1 |
| 3 | Apply & Response | ✅ Yes (send/receive test) | Phase 1, 2 |
| 4 | Persistence | ✅ Yes (reboot test) | Phase 1, 2, 3 |
| 5 | Integration | ✅ Yes (runtime behavior) | Phase 1, 2, 3, 4 |
| 6 | Testing | ✅ Yes (full system test) | All phases |

---

## Benefits of Phased Approach

1. **Incremental Testing** - Catch issues early
2. **Easier Debugging** - Isolate problems to specific phase
3. **Safer Development** - Can stop/rollback at any phase
4. **Clear Progress** - Know exactly what's done
5. **Parallel Work** - Receiver can be developed alongside
6. **Documentation** - Each phase is self-contained

---

## Rollback Strategy

If issues found in any phase:
- **Phase 1:** Just remove additions to communication.h
- **Phase 2:** Remove validation functions, keep Phase 1
- **Phase 3:** Remove handlers, keep Phase 1-2
- **Phase 4:** Remove NVS functions, keep Phase 1-3 (config works but doesn't persist)
- **Phase 5:** Revert to hardcoded values, keep Phase 1-4
- **Phase 6:** Fix issues and retest

---

## Implementation Order

### Recommended Sequence:
1. **Phase 1** → Compile & verify
2. **Phase 2** → Test validation logic
3. **Phase 3** → Test message exchange (without persistence)
4. **Phase 4** → Test persistence
5. **Phase 5** → Test runtime behavior
6. **Phase 6** → Full system test

### Alternative (Minimal Viable):
1. **Phase 1** → Protocol defined
2. **Phase 3** → Basic send/receive working
3. **Phase 2** → Add validation later
4. **Phase 4** → Add persistence later
5. **Phase 5** → Integration
6. **Phase 6** → Testing

---

## Success Criteria

### Phase 1 Complete When:
- ✅ Code compiles without errors
- ✅ Structure size is 16 bytes
- ✅ All constants defined
- ✅ Checksum macro works

### Phase 2 Complete When:
- ✅ Validation function handles all cases
- ✅ Out-of-range values corrected
- ✅ Logging shows clear information

### Phase 3 Complete When:
- ✅ Can receive config message
- ✅ Can send config response
- ✅ ACKs sent for control events
- ✅ Response event IDs correct

### Phase 4 Complete When:
- ✅ Configuration persists across reboots
- ✅ Defaults used on first boot
- ✅ NVS errors handled gracefully

### Phase 5 Complete When:
- ✅ ULP uses runtime config
- ✅ Delays use runtime config
- ✅ No hardcoded values remain

### Phase 6 Complete When:
- ✅ All test scenarios pass
- ✅ No regressions in normal operation
- ✅ Performance acceptable

---

## Risk Mitigation

### Phase 1 Risks:
- **Risk:** Structure size exceeds LoRa limit
- **Mitigation:** Verify size = 16 bytes (well within 58 byte limit)

### Phase 2 Risks:
- **Risk:** Validation too strict/loose
- **Mitigation:** Test with boundary values, adjust limits

### Phase 3 Risks:
- **Risk:** Message parsing errors
- **Mitigation:** Extensive logging, checksum validation

### Phase 4 Risks:
- **Risk:** NVS corruption
- **Mitigation:** Use existing namespace, test error handling

### Phase 5 Risks:
- **Risk:** Breaking existing functionality
- **Mitigation:** Test normal operation after each change

### Phase 6 Risks:
- **Risk:** Edge cases not covered
- **Mitigation:** Comprehensive test matrix

---

## Documentation Updates

After each phase, update:
1. **Code comments** - Document new functions
2. **README.md** - Add configuration instructions
3. **Version history** - Update version number
4. **This plan** - Mark phase complete

---

## Next Steps

1. Review this phased plan
2. Confirm approach with user
3. Switch to Code mode
4. Implement Phase 1
5. Test Phase 1
6. Proceed to Phase 2
7. Continue through all phases

**Ready to begin Phase 1 implementation.**
