#ifndef EVENT_HANDLER_H
#define EVENT_HANDLER_H

#include <stdint.h>

extern void handle_event(uint8_t eventFlag);

extern volatile uint8_t enableACMD23; // ACMD23を有効にするフラグ

#endif // EVENT_HANDLER_H