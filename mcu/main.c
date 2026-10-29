#include <stdint.h>
#include "al_core.h"
#include "al_hwcfg.h"
#include "al_usart_ll.h"

#define UART_BASE USART1_BASE_ADDR
#define UART_RX_EMPTY (1u << AL_USART_STATUS_EMPTY)
#define RX_LINE_MAX 48u

static char rx_line[RX_LINE_MAX];
static unsigned rx_len;
static unsigned rx_discard;

static int parse_uint(const char **p, unsigned *value, char delimiter)
{
    unsigned n = 0;
    unsigned digits = 0;
    while (**p >= '0' && **p <= '9') {
        n = n * 10u + (unsigned)(**p - '0');
        ++*p;
        ++digits;
        if (n > 999u) return 0;
    }
    if (digits == 0 || **p != delimiter) return 0;
    ++*p;
    *value = n;
    return 1;
}

static int parse_digit_result(const char *line,
                              unsigned *x, unsigned *y,
                              unsigned *w, unsigned *h, unsigned *digit)
{
    const char *p = line;
    if (*p++ != 'D' || *p++ != ',') return 0;
    if (!parse_uint(&p, x, ',') || !parse_uint(&p, y, ',') ||
        !parse_uint(&p, w, ',') || !parse_uint(&p, h, ',')) return 0;
    if (*p < '0' || *p > '9' || p[1] != '\0') return 0;
    *digit = (unsigned)(*p - '0');
    return *x < 320u && *y < 180u && *w <= 320u && *h <= 180u;
}

static void consume_byte(char c)
{
    if (c == '\r' || c == '\n') {
        if (!rx_discard && rx_len != 0) {
            unsigned x, y, w, h, digit;
            rx_line[rx_len] = '\0';
            if (parse_digit_result(rx_line, &x, &y, &w, &h, &digit)) {
                al_printf("MCU,D,%03u,%03u,%03u,%03u,%u\r\n", x, y, w, h, digit);
            } else {
                al_printf("MCU,REJECT,%s\r\n", rx_line);
            }
        }
        rx_len = 0;
        rx_discard = 0;
        return;
    }

    if (rx_discard) return;
    if (c < 32 || c > 126 || rx_len >= RX_LINE_MAX - 1u) {
        rx_len = 0;
        rx_discard = 1;
        return;
    }
    rx_line[rx_len++] = c;
}

int main(void)
{
    al_printf("MCU,DIGIT_UART_READY,USART1,115200,8N1\r\n");

    for (;;) {
        while ((AlUsart_ll_GetStatus(UART_BASE) & UART_RX_EMPTY) == 0u) {
            consume_byte((char)AlUsart_ll_RecvData(UART_BASE));
        }
    }
}
