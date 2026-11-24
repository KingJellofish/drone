#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>

void debug_print(const char *message);
void debug_process(void);
void debug_tx_complete_callback(void);

extern volatile uint32_t debug_dropped_messages;

#endif