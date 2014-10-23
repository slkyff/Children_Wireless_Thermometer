/* Copyright (c) 2012 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include "app_uart.h"
#include "simple_uart.h"
#include "app_gpiote.h"
#include "nordic_common.h"

//#include "boards.h"
#include "sc_ht_perpin_def.h"					//modify by slk at 2014.08.19
#include "app_trace.h"

struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;

#ifdef ENABLE_DEBUG_LOG_SUPPORT

void app_trace_init(void)
{
    simple_uart_config(RTS_PIN_NO, TX_PIN_NO, CTS_PIN_NO, RX_PIN_NO, HWFC);
}

void app_trace_deinit(void)
{
	simple_uart_deinit(RTS_PIN_NO, TX_PIN_NO, CTS_PIN_NO, RX_PIN_NO, HWFC);
}

int fputc(int ch, FILE * p_file)
{
    simple_uart_put((uint8_t)ch);
    return ch;
}

void app_trace_dump(uint8_t * p_buffer, uint32_t len)
{
    app_trace_log("\r\n");
    for (uint32_t index = 0; index <  len; index++)
    {
        app_trace_log("0x%02X ", p_buffer[index]);
    }
    app_trace_log("\r\n");
}

void app_trace_dump_var(uint8_t * p_buffer, uint32_t len)
{
	for (uint32_t index = 0; index <  len; index++)
    {
        app_trace_log("%c", p_buffer[index]);
    }
    app_trace_log("\r\n");
}

void itoa(unsigned int value, char *str)
{
    int i,j;
    for(i=1; value > 0; i++,value/=10) //��value[1]��ʼ���value�������ַ������������򣬵����ٷ������

        str[i] = value%10+'0'; //�����ּ���0��ASCIIֵ(��'0')�͵õ������ֵ�ASCIIֵ

    for(j=i-1,i=1; j-i>=1; j--,i++) //�������ַ�������
    {
        str[i] = str[i]^str[j];
        str[j] = str[i]^str[j];
        str[i] = str[i]^str[j];
    }
    if(str[0] != '-') //������Ǹ���������Ҫ�������ַ��±�����һλ������1
    {
        for(i=0; str[i+1]!='\0'; i++)
            str[i] = str[i+1];
        str[i] = '\0';
    }
}

int atoi(const char *str)
{
	int ret = 0,sign = 1;

	for(;*str == ' ' || *str == '\t';str++);
	if(*str == '-') sign = -1;
	if(*str == '-' || *str == '+') str ++;

	while(isdigit(*str))
	{
		ret = ret*10 + *str - '0';
		str++;
	}
	return sign*ret;
}

#endif // ENABLE_DEBUG_LOG_SUPPORT

/**
 *@}
 **/

