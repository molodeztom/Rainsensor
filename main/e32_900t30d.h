// e32_900t30d.h
#ifndef E32_900T30D_H
#define E32_900T30D_H

#include <stdint.h>
#include <stddef.h>

// Initialisierungsfunktion
void e32_init(void);

// Modus setzen
void e32_set_mode(uint8_t m0, uint8_t m1);

// Daten senden
void e32_send_data(const uint8_t *data, size_t len);

// Empfangscallback registrieren
typedef void (*e32_receive_callback_t)(const uint8_t *data, size_t len);
void e32_set_receive_callback(e32_receive_callback_t callback);

// Konfigurationsfunktion
void e32_configure(void);

#endif // E32_900T30D_H