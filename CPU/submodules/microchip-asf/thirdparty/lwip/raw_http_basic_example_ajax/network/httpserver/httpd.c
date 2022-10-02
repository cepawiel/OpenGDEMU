/**
 * \file
 *
 * \brief Httpd server.
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */

#include <string.h>
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "lwipopts.h"
#include "httpd.h"
#include "fs.h"
#include <port.h>

/** state info for file sending by http */
struct http_state {
	const char *file;
	u32_t left;
	u8_t retries;
};
struct http_state *g_hs;

/** http content type indicated with keys*/
static struct {
	const char *key; 
	const char *content; }http_content_type[] = {
	{"", "Content-type: application/json\r\n\r\n"},
	{"html", "Content-type: text/html\r\n\r\n"},
	{"css", "Content-type: text/css\r\n\r\n"},
	{"js", "Content-type: text/javascript\r\n\r\n"},
	{"png", "Content-type: image/png\r\n\r\n"},
	{"jpg", "Content-type: image/jpeg\r\n\r\n"},
	{"gif", "Content-type: image/gif\r\n\r\n"},
	{"txt", "Content-type: text/plain\r\n\r\n"},
};

/** Handle the incoming client request. */
static char req_string[80];
u8_t tx_buf[CONFIG_AFSK_TX_BUFLEN];

/** http html header*/
static const char http_html_hdr_200[] = "HTTP/1.0 200 OK\r\n";
static const char http_html_hdr_404[] = "HTTP/1.0 404 Not Found\r\n";
static const char http_html_hdr_500[] = "HTTP/1.0 500 Internal Server Error\r\n";

/** http error page html*/
const char http_server_error[] = " \
<!DOCTYPE html PUBLIC \"-//IETF//DTD HTML 2.0//EN\"> \
<html><head><meta http-equiv=\"Content-Type\" content=\"text/html; charset=ISO-8859-1\"> \
<title>500 Internal Server error</title></head><body><h1>500 Internal Server error</h1> \
<p>The internal server error was occur while processing the requested page.</p> \
</body></html>";


/** Decoded message buffer. */
static char decoded_str[80];

/** the TCP protocol control block */
struct tcp_pcb *g_pcb;

/** HTTP buffer to store key value. */
static char key_value[80];

static int cgi_GetStaus(const char *name, char *recv_buf,size_t recv_len);
static int cgi_SetLed(const char *name, char *recv_buf,size_t recv_len);

/**
 * CGI table where we associate one callback to one page.
 */
HttpCGI cgi_table[] = {
	{CGI_MATCH_NAME, "set_led", cgi_SetLed},
	{CGI_MATCH_NAME, "status", cgi_GetStaus},
	{CGI_MATCH_NONE, NULL, NULL}
};

/**
 * \brief Callback called on connection error.
 *
 * \param arg Pointer to structure representing the HTTP state.
 * \param err Error code.
 */
static void http_conn_err(void *arg, err_t err)
{
	struct http_state *hs;

	LWIP_UNUSED_ARG(err);

	hs = arg;
	mem_free(hs);
}

/**
 * \brief Close HTTP connection.
 *
 * \param pcb Pointer to a TCP connection structure.
 * \param hs Pointer to structure representing the HTTP state.
 */
static void http_close_conn(struct tcp_pcb *pcb, struct http_state *hs)
{
	tcp_arg(pcb, NULL);
	tcp_sent(pcb, NULL);
	tcp_recv(pcb, NULL);
	mem_free(hs);
	tcp_close(pcb);
}

/**
 * \brief Send HTTP data.
 *
 * \param pcb Pointer to a TCP connection structure.
 * \param hs Pointer to structure representing the HTTP state.
 */
static void http_send_data(struct tcp_pcb *pcb, struct http_state *hs)
{
	err_t err;
	u32_t len;

	/* We cannot send more data than space available in the send buffer. */
	if (tcp_sndbuf(pcb) < hs->left) {
		len = tcp_sndbuf(pcb);
	} else {
		len = hs->left;
	}

	do {
		/* Use copy flag to avoid using flash as a DMA source (forbidden). */
		err = tcp_write(pcb, hs->file, len, TCP_WRITE_FLAG_COPY);
		if (err == ERR_MEM) {
			len /= 2;
		}
	} while (err == ERR_MEM && len > 1);

	if (err == ERR_OK) {
		hs->file += len;
		hs->left -= len;
	}
}

/**
 * \brief Poll for HTTP data.
 *
 * \param arg Pointer to structure representing the HTTP state.
 * \param pcb Pointer to a TCP connection structure.
 *
 * \return ERR_OK on success, ERR_ABRT otherwise.
 */
static err_t http_poll(void *arg, struct tcp_pcb *pcb)
{
	struct http_state *hs;

	hs = arg;

	if (hs == NULL) {
		tcp_abort(pcb);
		return ERR_ABRT;
	} else {
		if (hs->file == 0) {
			tcp_abort(pcb);
			return ERR_ABRT;
		}

		++hs->retries;
		if (hs->retries == 4) {
			tcp_abort(pcb);
			return ERR_ABRT;
		}

		http_send_data(pcb, hs);
	}

	return ERR_OK;
}

/**
 * \brief Callback to handle data transfered.
 *
 * \param arg Pointer to structure representing the HTTP state.
 * \param pcb Pointer to a TCP connection structure.
 * \param len Unused.
 *
 * \return ERR_OK on success, ERR_ABRT otherwise.
 */
static err_t http_sent(void *arg, struct tcp_pcb *pcb, u16_t len)
{
	struct http_state *hs;

	LWIP_UNUSED_ARG(len);

	hs = arg;

	hs->retries = 0;

	if (hs->left > 0) {
		http_send_data(pcb, hs);
	} else {
		http_close_conn(pcb, hs);
	}

	return ERR_OK;
}

/**
 * \brief Write HTTP data.
 *
 * \param buf Buffer to write.
 * \param len Buffer length.
 */
static void http_write(const char *buf, u32_t len)
{
	g_hs->file = buf;
	g_hs->left = len;

	http_send_data(g_pcb, g_hs);

	/*
	 * Tell TCP that we wish be to informed of buf that has been
	 * successfully sent by a call to the http_sent() callback.
	 */
	tcp_sent(g_pcb, http_sent);
}

/**
 * \brief Send the HTTP header with the appropriated content type.
 *
 * \param content_type Index in the http_content_type table.
 */
static void http_sendOk(int content_type)
{
	/* ASSERT(content_type < HTTP_CONTENT_CNT); */

	http_write(http_html_hdr_200, sizeof(http_html_hdr_200) - 1);
	http_write(http_content_type[content_type].content,
			strlen(http_content_type[content_type].content));
}

/**
 * \brief Tokenize a buffer.
 *
 * \param raw_buf Buffer to parse.
 * \param raw_len Buffer length.
 *
 * \return The number of found tokens.
 */
static int http_tokenizeGetRequest(char *raw_buf, size_t raw_len)
{
	size_t token = 0;

	for (size_t i = 0; (i < raw_len) && raw_buf; i++) {
		if (raw_buf[i] == '&') {
			token++;
			raw_buf[i] = '\0';
		}

		if (raw_buf[i] == '=') {
			raw_buf[i] = '\0';
		}
	}

	return token + 1;
}

/**
 * \brief ASCII to HEX converter.
 *
 * \param first First digit.
 * \param second Second digit.
 *
 * \return String result.
 */
static char http_hexToAscii(char first, char second)
{
	char hex[5];
	char *stop;

	hex[0] = '0';
	hex[1] = 'x';
	hex[2] = first;
	hex[3] = second;
	hex[4] = 0;

	return strtol(hex, &stop, 16);
}

/**
 * \brief Decode URL.
 *
 * \param raw_buf Input buffer.
 * \param raw_len Input buffer length.
 * \param decodec_buf Output decoded buffer.
 * \param len Output buffer length.
 */
static void http_decodeUrl(const char *raw_buf, size_t raw_len,
		char *decodec_buf, size_t len)
{
	char value;

	/* ASSERT(decodec_buf); */
	memset(decodec_buf, 0, len);

	for (size_t i = 0; i < raw_len; i++) {
		if (!len) {
			return;
		}

		if (raw_buf[i] == '%') {
			if (i + 2 < raw_len) {
				/* Convert HEX value after %. */
				value = http_hexToAscii(raw_buf[i + 1], raw_buf[i + 2]);
				if (value) {
					*decodec_buf++ = value;
					len--;
					/* Decoded two digit of hex value, go to next value. */
					i += 2;
					continue;
				}
			}
		}

		/* Manage special case of '+', that should be converted to spaces. */
		*decodec_buf++ = (raw_buf[i] == '+' ? ' ' : raw_buf[i]);
		len--;
	}
}

/**
 * \brief Get key value from tokenized buffer.
 *
 * \param tolenized_buf .
 * \param tolenized_buf_len .
 * \param key .
 * \param value
 * \param len .
 */
static int http_getValue(char *tolenized_buf, size_t tolenized_buf_len,
		const char *key, char *value, size_t len)
{
	if (!tolenized_buf || !key || !value) {
		return -1;
	}

	char *p = tolenized_buf;
	size_t value_len = 0;

	memset(value, 0, len);

	for (size_t i = 0; i < tolenized_buf_len; i++) {
		size_t token_len = strlen(p);
		http_decodeUrl(p, token_len, decoded_str, sizeof(decoded_str));

		if (!strcmp(key, decoded_str)) {
			/* skip key */
			p += token_len + 1;

			http_decodeUrl(p, strlen(p), decoded_str, sizeof(decoded_str));
			value_len = strlen(decoded_str);

			if (value_len >= len) {
				return -1;
			}

			strcpy(value, decoded_str);
			return value_len;
		}

		/* jump to next pair */
		p += token_len + 1;
	}

	return -1;
}

/**
 * \brief Send HTTP internal server error code.
 *
 * \param content_type HTTP content type.
 */
static void http_sendInternalErr(int content_type)
{
	/*	ASSERT(content_type < HTTP_CONTENT_CNT); */

	http_write(http_html_hdr_500, sizeof(http_html_hdr_500) - 1);
	http_write(http_content_type[content_type].content,
			strlen(http_content_type[content_type].content));
}

/**
 * \brief Get requested page name.
 *
 * \param recv_buf Receive buffer.
 * \param recv_len Receive buffer length.
 * \param page_name Output buffer to store requestd page name.
 * \param len Output buffer length.
 */
static void http_getPageName(const char *recv_buf, size_t recv_len,
		char *page_name, size_t len)
{
	int i = 0;
	int str_ok = 0;
	const char *p = recv_buf;
	if (p && (recv_len > sizeof("GET /"))) {
		if (*p++ == 'G' && *p++ == 'E' && *p++ == 'T') {
			str_ok = 1;
			/* skip the space and "/" */
			p += 2;
		}
	}

	if (str_ok) {
		while ((size_t)i < recv_len) {
			char ch = *(p++);
			if (ch == ' ' || ch == '\t' || ch == '\n') {
				break;
			}

			if ((size_t)i == len - 1) {
				break;
			}

			page_name[i++] = ch;
		}
	}

	page_name[i] = '\0';
}

/**
 * \brief Get filename extension type.
 *
 * \param name String containing a filename.
 *
 * \return Extension type on success, NULL otherwise.
 */
static const char *get_ext(const char *name)
{
	const char *ext = strstr(name, ".");
	if (ext && (ext + 1)) {
		return (ext + 1);
	}

	return NULL;
}

/**
 * \brief Match content type based on filename extension.
 *
 * \param name String containing a filename.
 *
 * \return Content type index on success, 0 otherwise.
 */
static int http_searchContentType(const char *name)
{
	if (!name) {
		return 0;
	}

	const char *ext = get_ext(name);

	if (!ext) {
		return 0;
	}

	if (!strcmp(ext, "ico")) {
		return HTTP_CONTENT_JPEG;
	}

	for (int i = 0; i < HTTP_CONTENT_CNT; i++) {
		if (!strcmp(ext, http_content_type[i].key)) {
			return i;
		}
	}

	return 0;
}

/**
 * \brief Send a JSON string representing the board status.
 * \param name String containing the status request.
 * \param recv_buf Receive buffer.
 * \param recv_len Receive buffer length.
 *
 * \return -1 failed,0:success.
 */
static int cgi_GetStaus(const char *name, char *recv_buf,size_t recv_len)
{
	UNUSED(recv_buf);
	UNUSED(recv_len);
	UNUSED(name);
	
	U16 len=0;
	
	memset(tx_buf,'\0',sizeof(tx_buf));
	
	len = 0;
	len += sprintf((char *)tx_buf+len,"{\"leds\":{ \"0\":\"%d\"} }",!port_pin_get_output_level(LED_0_PIN));
	http_sendOk(HTTP_CONTENT_JSON);
	http_write((char const *)tx_buf, len);
	

	return 0;
}

/**
 * \brief Set the led status and send back the new status.
  * \param name String containing the status request.
 * \param recv_buf Receive buffer.
 * \param recv_len Receive buffer length.
 *
  * \return -1 failed,0:success.
 */
static int cgi_SetLed(const char *name, char *recv_buf,size_t recv_len)
{
	UNUSED(recv_buf);
	UNUSED(recv_len);
	UNUSED(name);

	char *query_str = strstr(name, "?") + 1;
	size_t query_str_len = strlen(query_str);
	int led_id;
	int led_cmd;

	http_tokenizeGetRequest(query_str, query_str_len);

	if (http_getValue(query_str, query_str_len, CGI_LED_ID_KEY, key_value,
			sizeof(key_value)) < 0) {
		goto error;
	}

	led_id = atoi(key_value);

	if (http_getValue(query_str, query_str_len, CGI_LED_CMD_KEY, key_value,
			sizeof(key_value)) < 0) {
		goto error;
	}

	led_cmd = atoi(key_value);

	if (led_id != 0) {
		goto error;
	}
	port_pin_set_output_level(LED_0_PIN,!led_cmd);
	
	sprintf((char *)tx_buf, "{\"n\":%d, \"set\":,%d}", led_id, led_cmd);

	http_sendOk(HTTP_CONTENT_JSON);
	http_write((const char *)tx_buf, strlen((char *)tx_buf));
	return 0;

error:
	http_sendInternalErr(HTTP_CONTENT_JSON);
	return 0;
}

/**
 * \brief CGI request search engine.
 *
 * \param name CGI request name.
 * \param table CGI handler table.
 *
 * \return A valid function handler for the specified CGI request, NULL
 * otherwise.
 */
static http_handler_t cgi_search(const char *name, HttpCGI *table)
{
	if (!table) {
		return NULL;
	}

	int i = 0;
	const char *ext = get_ext(name);

	while (table[i].name) {
		if (ext && table[i].type == CGI_MATCH_EXT) {
			if (!strcmp(table[i].name, ext)) {
				break;
			}
		} else if (table[i].type == CGI_MATCH_NAME) {
			if (strstr(name, table[i].name) != NULL) {
				break;
			}
		} else { /* (table[i].type == CGI_MATCH_WORD) */
			if (!strcmp(table[i].name, name)) {
				break;
			}
		}

		i++;
	}

	return table[i].handler;
}

/**
 * \brief Core HTTP server receive function. Handle the request and process it.
 *
 * \param arg Pointer to structure representing the HTTP state.
 * \param pcb Pointer to a TCP connection structure.
 * \param p Incoming request.
 * \param err Connection status.
 *
 * \return ERR_OK.
 */
static err_t http_recv(void *arg, struct tcp_pcb *pcb, struct pbuf *p, err_t err)
{
	char *buf;
	u16_t buflen;
	struct fs_file file;
	struct http_state *hs;
	http_handler_t cgi;

	hs = arg;

	if (err == ERR_OK && p != NULL) {
		/* Inform TCP that we have taken the buf. */
		tcp_recved(pcb, p->tot_len);

		if (hs->file == NULL) {
			g_hs = hs;
			g_pcb = pcb;

			buf = p->payload;
			buflen = p->len;
			memset(req_string, 0, sizeof(req_string));
			http_getPageName(buf, buflen, req_string,
					sizeof(req_string));

			if (req_string[0] == '\0') {
				strcpy(req_string, HTTP_DEFAULT_PAGE);
			}
			
			cgi = cgi_search(req_string, cgi_table);
			if (cgi) {
				if (cgi(req_string, buf, buflen) < 0) {
					http_sendInternalErr(HTTP_CONTENT_HTML);
					http_write(http_server_error,
							sizeof(http_server_error) - 1);
				}
			} else {
				if (fs_open(req_string, &file) == 0) {
					http_write(http_html_hdr_404,
							sizeof(http_html_hdr_404) - 1);
				} else {
					/* Send the HTML header. */
					int type = http_searchContentType(
							req_string);
					http_sendOk(type);

					/* Send the HTML content. */
					http_write(file.data, file.len);
				}
			}
		}
	}

	pbuf_free(p);

	if (err == ERR_OK && p == NULL) {
		http_close_conn(pcb, hs);
	}

	return ERR_OK;
}



/**
 * \brief Accept incoming HTTP connection requests.
 *
 * \param arg Pointer to structure representing the HTTP state.
 * \param pcb Pointer to a TCP connection structure.
 * \param err Connection status.
 *
 * \return ERR_OK on success.
 */
static err_t http_accept(void *arg, struct tcp_pcb *pcb, err_t err)
{
	struct http_state *hs;

	LWIP_UNUSED_ARG(arg);
	LWIP_UNUSED_ARG(err);

	tcp_setprio(pcb, TCP_PRIO_MIN);

	/* Allocate memory for the structure that holds the state of the
	connection. */
	hs = (struct http_state *)mem_malloc(sizeof(struct http_state));

	if (hs == NULL) {
		//printf("http_accept: Out of memory\n");
		return ERR_MEM;
	}

	/* Initialize the structure. */
	hs->file = NULL;
	hs->left = 0;
	hs->retries = 0;

	/* Tell TCP that this is the structure we wish to be passed for our
	callbacks. */
	tcp_arg(pcb, hs);

	/* Tell TCP that we wish to be informed of incoming data by a call
	to the http_recv() function. */
	tcp_recv(pcb, http_recv);

	tcp_err(pcb, http_conn_err);

	tcp_poll(pcb, http_poll, 4);
	return ERR_OK;
}

/**
 * \brief HTTP server init.
 */
void httpd_init(void)
{
	struct tcp_pcb *pcb;

	pcb = tcp_new();
	tcp_bind(pcb, IP_ADDR_ANY, 80);
	pcb = tcp_listen(pcb);
	tcp_accept(pcb, http_accept);
}
