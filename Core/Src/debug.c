#include "debug.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

#include <string.h>

#define DEBUG_QUEUE_LENGTH 8
#define DEBUG_MESSAGE_LENGTH 128

// The actual message queue
static char debug_queue[DEBUG_QUEUE_LENGTH][DEBUG_MESSAGE_LENGTH];

// Queue management variables
static uint8_t queue_head = 0;
static uint8_t queue_tail = 0;
static uint8_t queue_count = 0;

// USB transmission state
static volatile uint8_t tx_busy = 0;

// Buffer currently being transmitted
static char tx_buffer[DEBUG_MESSAGE_LENGTH];

// Number of messages discarded because the queue was full
volatile uint32_t debug_dropped_messages = 0;


// Add a message to the queue
void debug_print(const char *message)
{
    if (message == NULL) return;

    // Is the queue full?
    if (queue_count >= DEBUG_QUEUE_LENGTH)
    {
        debug_dropped_messages++;
        return;
    }

    // Get the next available queue slot
    char *slot = debug_queue[queue_head];

    // Copy the message into the queue
    size_t len = strlen(message);

    if (len > DEBUG_MESSAGE_LENGTH - 3)
        len = DEBUG_MESSAGE_LENGTH - 3;

    memcpy(slot, message, len);

    // Append a newline if one isn't already present
    if (len < 2 || slot[len - 2] != '\r' || slot[len - 1] != '\n')
    {
        slot[len++] = '\r';
        slot[len++] = '\n';
    }

    slot[len] = '\0';

    // Advance the head
    queue_head = (queue_head + 1) % DEBUG_QUEUE_LENGTH;

    // Increment the number of queued messages
    queue_count++;
}


// Called repeatedly from the main loop
void debug_process(void)
{
    // Don't transmit if USB is busy or the queue is empty
    if (tx_busy || queue_count == 0)
        return;

    // Copy the oldest queued message into the TX buffer
    strcpy(tx_buffer, debug_queue[queue_tail]);

    uint16_t len = strlen(tx_buffer);

    // Mark transmission as active BEFORE starting it
    tx_busy = 1;

    // Attempt transmission
    uint8_t result = CDC_Transmit_FS(
        (uint8_t *)tx_buffer,
        len
    );

    if (result == USBD_OK)
    {
        // Transmission accepted. Remove message from queue.
        queue_tail = (queue_tail + 1) % DEBUG_QUEUE_LENGTH;
        queue_count--;
    }
    else
    {
        // USB wasn't ready. Keep the message for another attempt.
        tx_busy = 0;
    }
}


// Called when USB finishes transmitting
void debug_tx_complete_callback(void)
{
    tx_busy = 0;
}