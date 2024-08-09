/**
 * @file usrcmd.c
 * @author CuBeatSystems
 * @author Shinichiro Nakamura
 * @copyright
 * ===============================================================
 * Natural Tiny Shell (NT-Shell) Version 0.3.1
 * ===============================================================
 * Copyright (c) 2010-2016 Shinichiro Nakamura
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include "ntopt.h"
#include "ntlibc.h"

#include "ntshell.h"

#include <stdio.h>
#include <stdlib.h>

#include "main.h"


#define UartHandler (huart2)


#define uart_puts(str) puts(str)
#define uart_putc(str) putc(str)


#define UART_BUF_LENGTH 128
uint8_t uart_rx_buf[UART_BUF_LENGTH] __attribute__ ((aligned (4)));
uint32_t uart_rx_cursor = 0;


typedef int (*USRCMDFUNC)(int argc, char **argv);


static int ntshell_serial_read(char *buf, int cnt, void *extobj);
static int ntshell_serial_write(const char *buf, int cnt, void *extobj);
static int ntshell_callback(const char *text, void *extobj);

static char ntshell_serial_getc_timeout(int timeout_ms);

int usrcmd_execute(const char *text);

static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj);
static int usrcmd_help(int argc, char **argv);
static int usrcmd_info(int argc, char **argv);

typedef struct {
    char *cmd;
    char *desc;
    USRCMDFUNC func;
} cmd_table_t;

static const cmd_table_t cmdlist[] = {
    { "help", "This is a description text string for help command.", usrcmd_help },
    { "info", "This is a description text string for info command.", usrcmd_info },
};



void ntshell_usr_init(ntshell_t *p)
{

	HAL_UART_Receive_DMA(&UartHandler, uart_rx_buf, sizeof(uart_rx_buf));

	void *extobj = 0;
	ntshell_init(p, ntshell_serial_read, ntshell_serial_write, ntshell_callback, extobj);
	ntshell_set_prompt(p, "ntshell>");

}

static int ntshell_serial_read(char *buf, int cnt, void *extobj)
{
	int rx_counter = 0;
	while(rx_counter < cnt)
	{
		int next_idx = UART_BUF_LENGTH - __HAL_DMA_GET_COUNTER(UartHandler.hdmarx);
		int new_data_length = next_idx - uart_rx_cursor;
		if(new_data_length < 0)
		{
			new_data_length += UART_BUF_LENGTH;
		}
		for(int i = 0; i < cnt; i++)
		{
			if(i >= new_data_length)
			{
				break;
			}
			rx_counter++;
			buf[i] = uart_rx_buf[uart_rx_cursor];
			uart_rx_cursor++;
			if(uart_rx_cursor >= UART_BUF_LENGTH)
			{
				uart_rx_cursor = 0;
			}
		}
	}
	return rx_counter;
}

static int ntshell_serial_write(const char *buf, int cnt, void *extobj)
{

	while(HAL_UART_Transmit(&huart2, (uint8_t*)buf, cnt, 1000) != HAL_OK);
	return cnt;
}

static int ntshell_callback(const char *text, void *extobj)
{

#if 0
    /*
     * This is a really simple example codes for the callback function.
     */
    uart_puts("USERINPUT[");
    uart_puts(text);
    uart_puts("]\r\n");
#else
    /*
     * This is a complete example for a real embedded application.
     */
    usrcmd_execute(text);
#endif

    return 0;

}

static char ntshell_serial_getc_timeout(int timeout_ms)
{
	char c = 0;
	uint32_t tickstart = HAL_GetTick();

	while((HAL_GetTick() - tickstart) < timeout_ms)
	{
		int next_idx = UART_BUF_LENGTH - __HAL_DMA_GET_COUNTER(UartHandler.hdmarx);
		int new_data_length = next_idx - uart_rx_cursor;
		if(new_data_length < 0)
		{
			new_data_length += UART_BUF_LENGTH;
		}

		if(new_data_length > 0)
		{
			c = uart_rx_buf[uart_rx_cursor];
			uart_rx_cursor++;
			if(uart_rx_cursor >= UART_BUF_LENGTH)
			{
				uart_rx_cursor = 0;
			}
			break;
		}

	}

	return c;
}


static int checkSuspens()
{
	char c;
	c = ntshell_serial_getc_timeout(1);
	if(c == 0x03)
	{
		puts("\r\n^C\r\n");
		return 1;
	}
	return 0;
}


/*
 * User command
 */



int usrcmd_execute(const char *text)
{
    return ntopt_parse(text, usrcmd_ntopt_callback, 0);
}

static int usrcmd_ntopt_callback(int argc, char **argv, void *extobj)
{
    if (argc == 0) {
        return 0;
    }
    const cmd_table_t *p = &cmdlist[0];
    for (int i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
        if (ntlibc_strcmp((const char *)argv[0], p->cmd) == 0) {
            return p->func(argc, argv);
        }
        p++;
    }
    uart_puts("Unknown command found.\r\n");
    return 0;
}

static int usrcmd_help(int argc, char **argv)
{
    const cmd_table_t *p = &cmdlist[0];
    for (int i = 0; i < sizeof(cmdlist) / sizeof(cmdlist[0]); i++) {
        uart_puts(p->cmd);
        uart_puts("\t:");
        uart_puts(p->desc);
        uart_puts("\r\n");
        p++;
    }
    return 0;
}

static int usrcmd_info(int argc, char **argv)
{
    if (argc != 2) {
        uart_puts("info sys\r\n");
        uart_puts("info ver\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "sys") == 0) {
        uart_puts("NXP LPC824 Monitor\r\n");
        return 0;
    }
    if (ntlibc_strcmp(argv[1], "ver") == 0) {
        uart_puts("Version 0.0.0\r\n");
        return 0;
    }
    uart_puts("Unknown sub command found\r\n");
    return -1;
}










