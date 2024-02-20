/**
 * @file	httpUtil.c
 * @brief	HTTP Server Utilities
 * @version 1.0
 * @date	2014/07/15
 * @par Revision
 *			2014/07/15 - 1.0 Release
 * @author
 * \n\n @par Copyright (C) 1998 - 2014 WIZnet. All rights reserved.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "httpUtil.h"

extern jsonStringToSend;
extern DATOX_CAMBIAR;
extern DATOY_CAMBIAR;

uint8_t http_get_cgi_handler(uint8_t *uri_name, uint8_t *buf, uint32_t *file_len)
{
	uint8_t ret = HTTP_OK;
	uint16_t len = 0;

	if (predefined_get_cgi_processor(uri_name, buf, &len))
	{
		;
	}
	else if (strcmp((const char *)uri_name, "example.cgi") == 0)
	{
		// To do
		;
	}
	else if (strcmp((const char *)uri_name, "data.cgi") == 0)
	{
		// To do
		len = sprintf(buf, "{\"x\":%d,\"y\":%d}", DATOX_CAMBIAR, DATOY_CAMBIAR);
	}
	else
	{
		// CGI file not found
		ret = HTTP_FAILED;
	}

	if (ret)
		*file_len = len;
	return ret;
}

uint8_t http_post_cgi_handler(uint8_t *uri_name, st_http_request *p_http_request, uint8_t *buf, uint32_t *file_len)
{
	uint8_t ret = HTTP_OK;
	uint16_t len = 0;
	uint8_t val = 0;

	if (predefined_set_cgi_processor(uri_name, p_http_request->URI, buf, &len))
	{
		;
	}
	else if (strcmp((const char *)uri_name, "example.cgi") == 0)
	{
		// To do
		val = 1;
		len = sprintf((char *)buf, "%d", val);
	}
	else
	{
		// CGI file not found
		ret = HTTP_FAILED;
	}

	if (ret)
		*file_len = len;
	return ret;
}
