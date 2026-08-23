/* Minimal single-channel SEGGER RTT implementation, just enough for
 * probe-rs's `attach` command to find the control block and stream
 * text back over SWD without needing a UART. */
#include <stdint.h>
#include "rtt.h"

typedef struct
{
    const char *name;
    char *buffer;
    unsigned int size;
    volatile unsigned int wr_off;
    volatile unsigned int rd_off;
    unsigned int flags;
} rtt_channel_t;

typedef struct
{
    char id[16];
    unsigned int max_up;
    unsigned int max_down;
    rtt_channel_t up[1];
    rtt_channel_t down[1];
} rtt_cb_t;

#define RTT_UP_BUF_SIZE 256

static char s_up_buffer[RTT_UP_BUF_SIZE];

__attribute__((used, aligned(4))) rtt_cb_t _SEGGER_RTT = {
    .id = "SEGGER RTT",
    .max_up = 1,
    .max_down = 1,
    .up = {{.name = "Terminal", .buffer = s_up_buffer, .size = RTT_UP_BUF_SIZE, .wr_off = 0, .rd_off = 0, .flags = 0}},
    .down = {{.name = "Terminal", .buffer = 0, .size = 0, .wr_off = 0, .rd_off = 0, .flags = 0}},
};

void rtt_init(void)
{
    _SEGGER_RTT.up[0].wr_off = 0;
    _SEGGER_RTT.up[0].rd_off = 0;
}

void rtt_write(const char *data, unsigned int len)
{
    rtt_channel_t *ch = &_SEGGER_RTT.up[0];
    while (len--)
    {
        unsigned int next = (ch->wr_off + 1U) % ch->size;
        if (next == ch->rd_off)
        {
            break; /* buffer full, drop */
        }
        ch->buffer[ch->wr_off] = *data++;
        ch->wr_off = next;
    }
}

void rtt_puts(const char *s)
{
    unsigned int len = 0;
    const char *p    = s;
    while (*p++)
    {
        len++;
    }
    rtt_write(s, len);
}
