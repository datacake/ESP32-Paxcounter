#ifndef _LORAWAN_H
#define _LORAWAN_H

#ifdef HAS_LORA

// LMIC-Arduino LoRaWAN Stack
#include <lmic.h>
#include <hal/hal.h>
#include "loraconf.h"

void onEvent(ev_t ev);
void gen_lora_deveui(uint8_t *pdeveui);
void RevBytes(unsigned char *b, size_t c);
void get_hard_deveui(uint8_t *pdeveui);
void os_getDevKey(u1_t *buf);
void os_getArtEui(u1_t *buf);
void os_getDevEui(u1_t *buf);
void showLoraKeys(void);
void lorawan_loop(void *pvParameters);
void switch_lora(uint8_t sf, uint8_t tx);

#endif

#endif
