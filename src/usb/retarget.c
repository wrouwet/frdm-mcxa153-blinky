/* Redirects newlib's printf (used by the USB stack's usb_echo()) to our
 * RTT logger, since we have no UART wired up for a debug console. */
#include "rtt.h"

int _write(int file, char *ptr, int len)
{
    (void)file;
    rtt_write(ptr, (unsigned int)len);
    return len;
}
