// e32_900t30d.h
#ifndef E32_900T30D_H
#define E32_900T30D_H

#include <stdint.h>
#include <stddef.h>
#include "esp_log.h"
#include "esp_err.h"

// Initialisierungsfunktion
void e32_init(void);

// Modus setzen
void e32_set_mode(uint8_t m0, uint8_t m1);

// Daten senden

esp_err_t e32_send_data(const uint8_t *data, size_t len);
esp_err_t e32_receive_data(uint8_t *buffer, size_t len, uint32_t timeout_ms);


// Empfangscallback registrieren
typedef void (*e32_receive_callback_t)(const uint8_t *data, size_t len);
void e32_set_receive_callback(e32_receive_callback_t callback);

// Konfigurationsfunktion
void e32_configure(void);

    // Parameter auslesen und anzeigen
 void  e32_read_and_display_parameters();

 void handle_e32_error(esp_err_t err, const char *operation);

#endif // E32_900T30D_H