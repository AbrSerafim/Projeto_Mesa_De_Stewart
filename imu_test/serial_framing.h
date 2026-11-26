#ifndef SERIAL_FRAMING_H
#define SERIAL_FRAMING_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define RX_FRAGMENT_TIMEOUT_US 100000u

typedef enum { RX_NONE, RX_COMMAND, RX_INVALID } RxResult;

typedef struct {
    char line[192];
    size_t length;
    bool discard;
    uint64_t last_byte;
} SerialFrame;

static inline void serial_frame_expire(SerialFrame *frame, uint64_t now)
{
    if (frame->length && now - frame->last_byte > RX_FRAGMENT_TIMEOUT_US) {
        frame->length = 0;
        frame->discard = true;
    }
}

static inline RxResult serial_frame_feed(SerialFrame *frame, int byte, uint64_t now)
{
    serial_frame_expire(frame, now);
    frame->last_byte = now;
    if (byte == 0x18) {
        frame->length = 0;
        frame->discard = false;
    } else if (byte == '\n') {
        const bool invalid = frame->discard;
        size_t length = frame->length;
        frame->length = 0;
        frame->discard = false;
        if (invalid) {
            return RX_INVALID;
        }
        if (length && frame->line[length - 1] == '\r') {
            --length;
        }
        frame->line[length] = '\0';
        return length ? RX_COMMAND : RX_NONE;
    } else if (!frame->discard) {
        if (frame->length == sizeof frame->line - 1
            || (frame->length && frame->line[frame->length - 1] == '\r')
            || (byte != '\r' && (byte < 32 || byte > 126))) {
            frame->discard = true;
        } else {
            frame->line[frame->length++] = (char)byte;
        }
    }
    return RX_NONE;
}

#endif
