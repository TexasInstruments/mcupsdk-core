/**
 * @file
 * LWIP HTTP server implementation
 */

/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *         Simon Goldschmidt
 *
 */

/**
 * @defgroup httpd HTTP server
 * @ingroup apps
 *
 * This httpd supports for a
 * rudimentary server-side-include facility which will replace tags of the form
 * <!--#tag--> in any file whose extension is .shtml, .shtm or .ssi with
 * strings provided by an include handler whose pointer is provided to the
 * module via function http_set_ssi_handler().
 * Additionally, a simple common
 * gateway interface (CGI) handling mechanism has been added to allow clients
 * to hook functions to particular request URIs.
 *
 * To enable SSI support, define label LWIP_HTTPD_SSI in lwipopts.h.
 * To enable CGI support, define label LWIP_HTTPD_CGI in lwipopts.h.
 *
 * By default, the server assumes that HTTP headers are already present in
 * each file stored in the file system.  By defining LWIP_HTTPD_DYNAMIC_HEADERS in
 * lwipopts.h, this behavior can be changed such that the server inserts the
 * headers automatically based on the extension of the file being served.  If
 * this mode is used, be careful to ensure that the file system image used
 * does not already contain the header information.
 *
 * File system images without headers can be created using the makefsfile
 * tool with the -h command line option.
 *
 *
 * Notes about valid SSI tags
 * --------------------------
 *
 * The following assumptions are made about tags used in SSI markers:
 *
 * 1. No tag may contain '-' or whitespace characters within the tag name.
 * 2. Whitespace is allowed between the tag leadin "<!--#" and the start of
 *    the tag name and between the tag name and the leadout string "-->".
 * 3. The maximum tag name length is LWIP_HTTPD_MAX_TAG_NAME_LEN, currently 8 characters.
 *
 * Notes on CGI usage
 * ------------------
 *
 * The simple CGI support offered here works with GET method requests only
 * and can handle up to 16 parameters encoded into the URI. The handler
 * function may not write directly to the HTTP output but must return a
 * filename that the HTTP server will send to the browser as a response to
 * the incoming CGI request.
 *
 *
 *
 * The list of supported file types is quite short, so if makefsdata complains
 * about an unknown extension, make sure to add it (and its doctype) to
 * the 'g_psHTTPHeaders' list.
 */
#include "lwip/init.h"
#include "lwip/apps/httpd.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/apps/fs.h"
#include "httpd_structs.h"
#include "lwip/def.h"

#include "lwip/altcp.h"
#include "lwip/altcp_tcp.h"
#if HTTPD_ENABLE_HTTPS
#include "lwip/altcp_tls.h"
#endif
#ifdef LWIP_HOOK_FILENAME
#include LWIP_HOOK_FILENAME
#endif

#include <string.h> /* memset */
#include <stdlib.h> /* atoi */
#include <stdio.h>


/** Minimum length for a valid HTTP/0.9 request: "GET /\r\n" -> 7 bytes */
#define MIN_REQ_LEN   7

#define CRLF "\r\n"


#define HTTP_IS_DYNAMIC_FILE(hs) 0

/* This defines checks whether tcp_write has to copy data or not */

#ifndef HTTP_IS_DATA_VOLATILE
/** tcp_write does not have to copy data when sent from rom-file-system directly */
#define HTTP_IS_DATA_VOLATILE(hs)       (HTTP_IS_DYNAMIC_FILE(hs) ? TCP_WRITE_FLAG_COPY : 0)
#endif
/** Default: dynamic headers are sent from ROM (non-dynamic headers are handled like file data) */
#ifndef HTTP_IS_HDR_VOLATILE
#define HTTP_IS_HDR_VOLATILE(hs, ptr)   0
#endif

/* Return values for http_send_*() */
#define HTTP_DATA_TO_SEND_FREED    3
#define HTTP_DATA_TO_SEND_BREAK    2
#define HTTP_DATA_TO_SEND_CONTINUE 1
#define HTTP_NO_DATA_TO_SEND       0

typedef struct {
  const char *name;
  u8_t shtml;
} default_filename;

#if LWIP_HTTPD_SUPPORT_REQUESTLIST
/** HTTP request is copied here from pbufs for simple parsing */
static char httpd_req_buf[LWIP_HTTPD_MAX_REQ_LENGTH + 1];
#endif /* LWIP_HTTPD_SUPPORT_REQUESTLIST */


static const default_filename httpd_default_filenames[] = {
  {"/index.shtml", 1 },
  {"/index.ssi",   1 },
  {"/index.shtm",  1 },
  {"/index.html",  0 },
  {"/index.htm",   0 }
};

#define NUM_DEFAULT_FILENAMES LWIP_ARRAYSIZE(httpd_default_filenames)

#ifndef LWIP_HTTPD_URI_BUF_LEN
#define LWIP_HTTPD_URI_BUF_LEN LWIP_HTTPD_MAX_REQUEST_URI_LEN
#endif
static char http_uri_buf[LWIP_HTTPD_URI_BUF_LEN + 1];

#if LWIP_HTTPD_DYNAMIC_HEADERS
/* The number of individual strings that comprise the headers sent before each
 * requested file.
 */
#define NUM_FILE_HDR_STRINGS 5
#define HDR_STRINGS_IDX_HTTP_STATUS           0 /* e.g. "HTTP/1.0 200 OK\r\n" */
#define HDR_STRINGS_IDX_SERVER_NAME           1 /* e.g. "Server: "HTTPD_SERVER_AGENT"\r\n" */
#define HDR_STRINGS_IDX_CONTENT_LEN_KEEPALIVE 2 /* e.g. "Content-Length: xy\r\n" and/or "Connection: keep-alive\r\n" */
#define HDR_STRINGS_IDX_CONTENT_LEN_NR        3 /* the byte count, when content-length is used */
#define HDR_STRINGS_IDX_CONTENT_TYPE          4 /* the content type (or default answer content type including default document) */

/* The dynamically generated Content-Length buffer needs space for CRLF + NULL */
#define LWIP_HTTPD_MAX_CONTENT_LEN_OFFSET 3
#ifndef LWIP_HTTPD_MAX_CONTENT_LEN_SIZE
/* The dynamically generated Content-Length buffer shall be able to work with
   ~953 MB (9 digits) */
#define LWIP_HTTPD_MAX_CONTENT_LEN_SIZE   (9 + LWIP_HTTPD_MAX_CONTENT_LEN_OFFSET)
#endif
#endif /* LWIP_HTTPD_DYNAMIC_HEADERS */


struct http_state {
  struct fs_file file_handle;
  struct fs_file *handle;
  const char *file;       /* Pointer to first unsent byte in buf. */

  struct altcp_pcb *pcb;
#if LWIP_HTTPD_SUPPORT_REQUESTLIST
  struct pbuf *req;
#endif /* LWIP_HTTPD_SUPPORT_REQUESTLIST */

  u32_t left;       /* Number of unsent bytes in buf. */
  u8_t retries;

#if LWIP_HTTPD_DYNAMIC_HEADERS
  const char *hdrs[NUM_FILE_HDR_STRINGS]; /* HTTP headers to be sent. */
  char hdr_content_len[LWIP_HTTPD_MAX_CONTENT_LEN_SIZE];
  u16_t hdr_pos;     /* The position of the first unsent header byte in the
                        current string */
  u16_t hdr_index;   /* The index of the hdr string currently being sent. */
#endif /* LWIP_HTTPD_DYNAMIC_HEADERS */

};

#define HTTP_ALLOC_HTTP_STATE() (struct http_state *)mem_malloc(sizeof(struct http_state))
#define HTTP_FREE_HTTP_STATE(x) mem_free(x)

static err_t http_close_conn(struct altcp_pcb *pcb, struct http_state *hs);
static err_t http_close_or_abort_conn(struct altcp_pcb *pcb, struct http_state *hs, u8_t abort_conn);
static err_t http_find_file(struct http_state *hs, const char *uri, int is_09);
static err_t http_init_file(struct http_state *hs, struct fs_file *file, int is_09, const char *uri, u8_t tag_check, char *params);
static err_t http_poll(void *arg, struct altcp_pcb *pcb);
static u8_t http_check_eof(struct altcp_pcb *pcb, struct http_state *hs);



#define http_add_connection(hs)
#define http_remove_connection(hs)

/** Initialize a struct http_state.
 */
static void
http_state_init(struct http_state *hs)
{
  /* Initialize the structure. */
  memset(hs, 0, sizeof(struct http_state));
#if LWIP_HTTPD_DYNAMIC_HEADERS
  /* Indicate that the headers are not yet valid */
  hs->hdr_index = NUM_FILE_HDR_STRINGS;
#endif /* LWIP_HTTPD_DYNAMIC_HEADERS */
}

/** Allocate a struct http_state. */
static struct http_state* http_state_alloc(void)
{
    struct http_state *ret = HTTP_ALLOC_HTTP_STATE();
    if (ret != NULL)
    {
        http_state_init(ret);
        http_add_connection(ret);
    }
    return ret;
}

/** Free a struct http_state.
 * Also frees the file data if dynamic.
 */
static void
http_state_eof(struct http_state *hs)
{
  if (hs->handle) {
    fs_close(hs->handle);
    hs->handle = NULL;
  }

#if LWIP_HTTPD_SUPPORT_REQUESTLIST
  if (hs->req) {
    pbuf_free(hs->req);
    hs->req = NULL;
  }
#endif /* LWIP_HTTPD_SUPPORT_REQUESTLIST */
}

/** Free a struct http_state.
 * Also frees the file data if dynamic.
 */
static void
http_state_free(struct http_state *hs)
{
  if (hs != NULL) {
    http_state_eof(hs);
    http_remove_connection(hs);
    HTTP_FREE_HTTP_STATE(hs);
  }
}

/** Call tcp_write() in a loop trying smaller and smaller length
 *
 * @param pcb altcp_pcb to send
 * @param ptr Data to send
 * @param length Length of data to send (in/out: on return, contains the
 *        amount of data sent)
 * @param apiflags directly passed to tcp_write
 * @return the return value of tcp_write
 */
static err_t
http_write(struct altcp_pcb *pcb, const void *ptr, u16_t *length, u8_t apiflags)
{
  u16_t len, max_len;
  err_t err;
  LWIP_ASSERT("length != NULL", length != NULL);
  len = *length;
  if (len == 0) {
    return ERR_OK;
  }
  /* We cannot send more data than space available in the send buffer. */
  max_len = altcp_sndbuf(pcb);
  if (max_len < len) {
    len = max_len;
  }
#ifdef HTTPD_MAX_WRITE_LEN
  /* Additional limitation: e.g. don't enqueue more than 2*mss at once */
  max_len = HTTPD_MAX_WRITE_LEN(pcb);
  if (len > max_len) {
    len = max_len;
  }
#endif /* HTTPD_MAX_WRITE_LEN */
  do {
    LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("Trying to send %d bytes\n", len));
    err = altcp_write(pcb, ptr, len, apiflags);
    if (err == ERR_MEM) {
      if ((altcp_sndbuf(pcb) == 0) ||
          (altcp_sndqueuelen(pcb) >= TCP_SND_QUEUELEN)) {
        /* no need to try smaller sizes */
        len = 1;
      } else {
        len /= 2;
      }
      LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE,
                  ("Send failed, trying less (%d bytes)\n", len));
    }
  } while ((err == ERR_MEM) && (len > 1));

  if (err == ERR_OK) {
    LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("Sent %d bytes\n", len));
    *length = len;
  } else {
    LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("Send failed with err %d (\"%s\")\n", err, lwip_strerr(err)));
    *length = 0;
  }

  return err;
}

/**
 * The connection shall be actively closed (using RST to close from fault states).
 * Reset the sent- and recv-callbacks.
 *
 * @param pcb the tcp pcb to reset callbacks
 * @param hs connection state to free
 */
static err_t
http_close_or_abort_conn(struct altcp_pcb *pcb, struct http_state *hs, u8_t abort_conn)
{
  err_t err;
  LWIP_DEBUGF(HTTPD_DEBUG, ("Closing connection %p\n", (void *)pcb));

  altcp_arg(pcb, NULL);
  altcp_recv(pcb, NULL);
  altcp_err(pcb, NULL);
  altcp_poll(pcb, NULL, 0);
  altcp_sent(pcb, NULL);
  if (hs != NULL) {
    http_state_free(hs);
  }

  if (abort_conn) {
    altcp_abort(pcb);
    return ERR_OK;
  }
  err = altcp_close(pcb);
  if (err != ERR_OK) {
    LWIP_DEBUGF(HTTPD_DEBUG, ("Error %d closing %p\n", err, (void *)pcb));
    /* error closing, try again later in poll */
    altcp_poll(pcb, http_poll, HTTPD_POLL_INTERVAL);
  }
  return err;
}

/**
 * The connection shall be actively closed.
 * Reset the sent- and recv-callbacks.
 *
 * @param pcb the tcp pcb to reset callbacks
 * @param hs connection state to free
 */
static err_t
http_close_conn(struct altcp_pcb *pcb, struct http_state *hs)
{
  return http_close_or_abort_conn(pcb, hs, 0);
}

/** End of file: either close the connection (Connection: close) or
 * close the file (Connection: keep-alive)
 */
static void
http_eof(struct altcp_pcb *pcb, struct http_state *hs)
{
  /* HTTP/1.1 persistent connection? (Not supported for SSI) */
  {
    http_close_conn(pcb, hs);
  }
}



#if LWIP_HTTPD_DYNAMIC_HEADERS
/**
 * Generate the relevant HTTP headers for the given filename and write
 * them into the supplied buffer.
 */
static void
get_http_headers(struct http_state *hs, const char *uri)
{
  size_t content_type;
  char *tmp;
  char *ext;
  char *vars;

  /* In all cases, the second header we send is the server identification
     so set it here. */
  hs->hdrs[HDR_STRINGS_IDX_SERVER_NAME] = g_psHTTPHeaderStrings[HTTP_HDR_SERVER];
  hs->hdrs[HDR_STRINGS_IDX_CONTENT_LEN_KEEPALIVE] = NULL;
  hs->hdrs[HDR_STRINGS_IDX_CONTENT_LEN_NR] = NULL;

  /* Is this a normal file or the special case we use to send back the
     default "404: Page not found" response? */
  if (uri == NULL) {
    hs->hdrs[HDR_STRINGS_IDX_HTTP_STATUS] = g_psHTTPHeaderStrings[HTTP_HDR_NOT_FOUND];
    {
      hs->hdrs[HDR_STRINGS_IDX_CONTENT_TYPE] = g_psHTTPHeaderStrings[DEFAULT_404_HTML];
    }

    /* Set up to send the first header string. */
    hs->hdr_index = 0;
    hs->hdr_pos = 0;
    return;
  }
  /* We are dealing with a particular filename. Look for one other
      special case.  We assume that any filename with "404" in it must be
      indicative of a 404 server error whereas all other files require
      the 200 OK header. */
  if (strstr(uri, "404")) {
    hs->hdrs[HDR_STRINGS_IDX_HTTP_STATUS] = g_psHTTPHeaderStrings[HTTP_HDR_NOT_FOUND];
  } else if (strstr(uri, "400")) {
    hs->hdrs[HDR_STRINGS_IDX_HTTP_STATUS] = g_psHTTPHeaderStrings[HTTP_HDR_BAD_REQUEST];
  } else if (strstr(uri, "501")) {
    hs->hdrs[HDR_STRINGS_IDX_HTTP_STATUS] = g_psHTTPHeaderStrings[HTTP_HDR_NOT_IMPL];
  } else {
    hs->hdrs[HDR_STRINGS_IDX_HTTP_STATUS] = g_psHTTPHeaderStrings[HTTP_HDR_OK];
  }

  /* Determine if the URI has any variables and, if so, temporarily remove
      them. */
  vars = strchr(uri, '?');
  if (vars) {
    *vars = '\0';
  }

  /* Get a pointer to the file extension.  We find this by looking for the
      last occurrence of "." in the filename passed. */
  ext = NULL;
  tmp = strchr(uri, '.');
  while (tmp) {
    ext = tmp + 1;
    tmp = strchr(ext, '.');
  }
  if (ext != NULL) {
    /* Now determine the content type and add the relevant header for that. */
    for (content_type = 0; content_type < NUM_HTTP_HEADERS; content_type++) {
      /* Have we found a matching extension? */
      if (!lwip_stricmp(g_psHTTPHeaders[content_type].extension, ext)) {
        break;
      }
    }
  } else {
    content_type = NUM_HTTP_HEADERS;
  }

  /* Reinstate the parameter marker if there was one in the original URI. */
  if (vars) {
    *vars = '?';
  }

#if LWIP_HTTPD_OMIT_HEADER_FOR_EXTENSIONLESS_URI
  /* Does the URL passed have any file extension?  If not, we assume it
     is a special-case URL used for control state notification and we do
     not send any HTTP headers with the response. */
  if (!ext) {
    /* Force the header index to a value indicating that all headers
       have already been sent. */
    hs->hdr_index = NUM_FILE_HDR_STRINGS;
    return;
  }
#endif /* LWIP_HTTPD_OMIT_HEADER_FOR_EXTENSIONLESS_URI */
  /* Did we find a matching extension? */
  if (content_type < NUM_HTTP_HEADERS) {
    /* yes, store it */
    hs->hdrs[HDR_STRINGS_IDX_CONTENT_TYPE] = g_psHTTPHeaders[content_type].content_type;
  } else if (!ext) {
    /* no, no extension found -> use binary transfer to prevent the browser adding '.txt' on save */
    hs->hdrs[HDR_STRINGS_IDX_CONTENT_TYPE] = HTTP_HDR_APP;
  } else {
    /* No - use the default, plain text file type. */
    hs->hdrs[HDR_STRINGS_IDX_CONTENT_TYPE] = HTTP_HDR_DEFAULT_TYPE;
  }
  /* Set up to send the first header string. */
  hs->hdr_index = 0;
  hs->hdr_pos = 0;
}

/* Add content-length header? */
static void
get_http_content_length(struct http_state *hs)
{
  u8_t add_content_len = 0;

  LWIP_ASSERT("already been here?", hs->hdrs[HDR_STRINGS_IDX_CONTENT_LEN_KEEPALIVE] == NULL);

  add_content_len = 0;

  {
    if ((hs->handle != NULL) && (hs->handle->flags & FS_FILE_FLAGS_HEADER_PERSISTENT)) {
      add_content_len = 1;
    }
  }
  if (add_content_len) {
    size_t len;
    lwip_itoa(hs->hdr_content_len, (size_t)LWIP_HTTPD_MAX_CONTENT_LEN_SIZE,
              hs->handle->len);
    len = strlen(hs->hdr_content_len);
    if (len <= LWIP_HTTPD_MAX_CONTENT_LEN_SIZE - LWIP_HTTPD_MAX_CONTENT_LEN_OFFSET) {
      SMEMCPY(&hs->hdr_content_len[len], CRLF, 3);
      hs->hdrs[HDR_STRINGS_IDX_CONTENT_LEN_NR] = hs->hdr_content_len;
    } else {
      add_content_len = 0;
    }
  }
  if (add_content_len) {
    hs->hdrs[HDR_STRINGS_IDX_CONTENT_LEN_KEEPALIVE] = g_psHTTPHeaderStrings[HTTP_HDR_CONTENT_LENGTH];
  }
}

/** Sub-function of http_send(): send dynamic headers
 *
 * @returns: - HTTP_NO_DATA_TO_SEND: no new data has been enqueued
 *           - HTTP_DATA_TO_SEND_CONTINUE: continue with sending HTTP body
 *           - HTTP_DATA_TO_SEND_BREAK: data has been enqueued, headers pending,
 *                                      so don't send HTTP body yet
 *           - HTTP_DATA_TO_SEND_FREED: http_state and pcb are already freed
 */
static u8_t
http_send_headers(struct altcp_pcb *pcb, struct http_state *hs)
{
  err_t err;
  u16_t len;
  u8_t data_to_send = HTTP_NO_DATA_TO_SEND;
  u16_t hdrlen, sendlen;

  if (hs->hdrs[HDR_STRINGS_IDX_CONTENT_LEN_KEEPALIVE] == NULL) {
    /* set up "content-length" and "connection:" headers */
    get_http_content_length(hs);
  }

  /* How much data can we send? */
  len = altcp_sndbuf(pcb);
  sendlen = len;

  while (len && (hs->hdr_index < NUM_FILE_HDR_STRINGS) && sendlen) {
    const void *ptr;
    u16_t old_sendlen;
    u8_t apiflags;
    /* How much do we have to send from the current header? */
    hdrlen = (u16_t)strlen(hs->hdrs[hs->hdr_index]);

    /* How much of this can we send? */
    sendlen = (len < (hdrlen - hs->hdr_pos)) ? len : (hdrlen - hs->hdr_pos);

    /* Send this amount of data or as much as we can given memory
     * constraints. */
    ptr = (const void *)(hs->hdrs[hs->hdr_index] + hs->hdr_pos);
    old_sendlen = sendlen;
    apiflags = HTTP_IS_HDR_VOLATILE(hs, ptr);
    if (hs->hdr_index == HDR_STRINGS_IDX_CONTENT_LEN_NR) {
      /* content-length is always volatile */
      apiflags |= TCP_WRITE_FLAG_COPY;
    }
    if (hs->hdr_index < NUM_FILE_HDR_STRINGS - 1) {
      apiflags |= TCP_WRITE_FLAG_MORE;
    }
    err = http_write(pcb, ptr, &sendlen, apiflags);
    if ((err == ERR_OK) && (old_sendlen != sendlen)) {
      /* Remember that we added some more data to be transmitted. */
      data_to_send = HTTP_DATA_TO_SEND_CONTINUE;
    } else if (err != ERR_OK) {
      /* special case: http_write does not try to send 1 byte */
      sendlen = 0;
    }

    /* Fix up the header position for the next time round. */
    hs->hdr_pos += sendlen;
    len -= sendlen;

    /* Have we finished sending this string? */
    if (hs->hdr_pos == hdrlen) {
      /* Yes - move on to the next one */
      hs->hdr_index++;
      /* skip headers that are NULL (not all headers are required) */
      while ((hs->hdr_index < NUM_FILE_HDR_STRINGS) &&
             (hs->hdrs[hs->hdr_index] == NULL)) {
        hs->hdr_index++;
      }
      hs->hdr_pos = 0;
    }
  }

  if ((hs->hdr_index >= NUM_FILE_HDR_STRINGS) && (hs->file == NULL)) {
    /* When we are at the end of the headers, check for data to send
     * instead of waiting for ACK from remote side to continue
     * (which would happen when sending files from async read). */
    if (http_check_eof(pcb, hs)) {
      data_to_send = HTTP_DATA_TO_SEND_BREAK;
    } else {
      /* At this point, for non-keepalive connections, hs is deallocated an
         pcb is closed. */
      return HTTP_DATA_TO_SEND_FREED;
    }
  }
  /* If we get here and there are still header bytes to send, we send
   * the header information we just wrote immediately. If there are no
   * more headers to send, but we do have file data to send, drop through
   * to try to send some file data too. */
  if ((hs->hdr_index < NUM_FILE_HDR_STRINGS) || !hs->file) {
    LWIP_DEBUGF(HTTPD_DEBUG, ("tcp_output\n"));
    return HTTP_DATA_TO_SEND_BREAK;
  }
  return data_to_send;
}
#endif /* LWIP_HTTPD_DYNAMIC_HEADERS */

/** Sub-function of http_send(): end-of-file (or block) is reached,
 * either close the file or read the next block (if supported).
 *
 * @returns: 0 if the file is finished or no data has been read
 *           1 if the file is not finished and data has been read
 */
static u8_t
http_check_eof(struct altcp_pcb *pcb, struct http_state *hs)
{
  int bytes_left;

  /* Do we have a valid file handle? */
  if (hs->handle == NULL) {
    /* No - close the connection. */
    http_eof(pcb, hs);
    return 0;
  }
  bytes_left = fs_bytes_left(hs->handle);
  if (bytes_left <= 0) {
    /* We reached the end of the file so this request is done. */
    LWIP_DEBUGF(HTTPD_DEBUG, ("End of file.\n"));
    http_eof(pcb, hs);
    return 0;
  }
  LWIP_ASSERT("SSI and DYNAMIC_HEADERS turned off but eof not reached", 0);
  return 1;
}

/** Sub-function of http_send(): This is the normal send-routine for non-ssi files
 *
 * @returns: - 1: data has been written (so call tcp_ouput)
 *           - 0: no data has been written (no need to call tcp_output)
 */
static u8_t
http_send_data_nonssi(struct altcp_pcb *pcb, struct http_state *hs)
{
  err_t err;
  u16_t len;
  u8_t data_to_send = 0;

  /* We are not processing an SHTML file so no tag checking is necessary.
   * Just send the data as we received it from the file. */
  len = (u16_t)LWIP_MIN(hs->left, 0xffff);

  err = http_write(pcb, hs->file, &len, HTTP_IS_DATA_VOLATILE(hs));
  if (err == ERR_OK) {
    data_to_send = 1;
    hs->file += len;
    hs->left -= len;
  }

  return data_to_send;
}


/**
 * Try to send more data on this pcb.
 *
 * @param pcb the pcb to send data
 * @param hs connection state
 */
static u8_t
http_send(struct altcp_pcb *pcb, struct http_state *hs)
{
  u8_t data_to_send = HTTP_NO_DATA_TO_SEND;

  LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("http_send: pcb=%p hs=%p left=%d\n", (void *)pcb,
              (void *)hs, hs != NULL ? (int)hs->left : 0));

  /* If we were passed a NULL state structure pointer, ignore the call. */
  if (hs == NULL) {
    return 0;
  }


#if LWIP_HTTPD_DYNAMIC_HEADERS
  /* Do we have any more header data to send for this file? */
  if (hs->hdr_index < NUM_FILE_HDR_STRINGS) {
    data_to_send = http_send_headers(pcb, hs);
    if ((data_to_send == HTTP_DATA_TO_SEND_FREED) ||
        ((data_to_send != HTTP_DATA_TO_SEND_CONTINUE) &&
         (hs->hdr_index < NUM_FILE_HDR_STRINGS))) {
      return data_to_send;
    }
  }
#endif /* LWIP_HTTPD_DYNAMIC_HEADERS */

  /* Have we run out of file data to send? If so, we need to read the next
   * block from the file. */
  if (hs->left == 0) {
    if (!http_check_eof(pcb, hs)) {
      return 0;
    }
  }

  data_to_send = http_send_data_nonssi(pcb, hs);

  if ((hs->left == 0) && (fs_bytes_left(hs->handle) <= 0)) {
    /* We reached the end of the file so this request is done.
     * This adds the FIN flag right into the last data segment. */
    LWIP_DEBUGF(HTTPD_DEBUG, ("End of file.\n"));
    http_eof(pcb, hs);
    return 0;
  }
  LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("send_data end.\n"));
  return data_to_send;
}

#define http_find_error_file(hs, error_nr) ERR_ARG

/**
 * Get the file struct for a 404 error page.
 * Tries some file names and returns NULL if none found.
 *
 * @param uri pointer that receives the actual file name URI
 * @return file struct for the error page or NULL no matching file was found
 */
static struct fs_file *
http_get_404_file(struct http_state *hs, const char **uri)
{
  err_t err;

  *uri = "/404.html";
  err = fs_open(&hs->file_handle, *uri);
  if (err != ERR_OK) {
    /* 404.html doesn't exist. Try 404.htm instead. */
    *uri = "/404.htm";
    err = fs_open(&hs->file_handle, *uri);
    if (err != ERR_OK) {
      /* 404.htm doesn't exist either. Try 404.shtml instead. */
      *uri = "/404.shtml";
      err = fs_open(&hs->file_handle, *uri);
      if (err != ERR_OK) {
        /* 404.htm doesn't exist either. Indicate to the caller that it should
         * send back a default 404 page.
         */
        *uri = NULL;
        return NULL;
      }
    }
  }

  return &hs->file_handle;
}




/**
 * When data has been received in the correct state, try to parse it
 * as a HTTP request.
 *
 * @param inp the received pbuf
 * @param hs the connection state
 * @param pcb the altcp_pcb which received this packet
 * @return ERR_OK if request was OK and hs has been initialized correctly
 *         ERR_INPROGRESS if request was OK so far but not fully received
 *         another err_t otherwise
 */
static err_t
http_parse_request(struct pbuf *inp, struct http_state *hs, struct altcp_pcb *pcb)
{
  char *data;
  char *crlf;
  u16_t data_len;
  struct pbuf *p = inp;
#if LWIP_HTTPD_SUPPORT_REQUESTLIST
  u16_t clen;
#endif /* LWIP_HTTPD_SUPPORT_REQUESTLIST */


  LWIP_UNUSED_ARG(pcb); /* only used for post */
  LWIP_ASSERT("p != NULL", p != NULL);
  LWIP_ASSERT("hs != NULL", hs != NULL);

  if ((hs->handle != NULL) || (hs->file != NULL)) {
    LWIP_DEBUGF(HTTPD_DEBUG, ("Received data while sending a file\n"));
    /* already sending a file */
    /* @todo: abort? */
    return ERR_USE;
  }

#if LWIP_HTTPD_SUPPORT_REQUESTLIST

  LWIP_DEBUGF(HTTPD_DEBUG, ("Received %"U16_F" bytes\n", p->tot_len));

  /* first check allowed characters in this pbuf? */

  /* enqueue the pbuf */
  if (hs->req == NULL) {
    LWIP_DEBUGF(HTTPD_DEBUG, ("First pbuf\n"));
    hs->req = p;
  } else {
    LWIP_DEBUGF(HTTPD_DEBUG, ("pbuf enqueued\n"));
    pbuf_cat(hs->req, p);
  }
  /* increase pbuf ref counter as it is freed when we return but we want to
     keep it on the req list */
  pbuf_ref(p);

  if (hs->req->next != NULL) {
    data_len = LWIP_MIN(hs->req->tot_len, LWIP_HTTPD_MAX_REQ_LENGTH);
    pbuf_copy_partial(hs->req, httpd_req_buf, data_len, 0);
    data = httpd_req_buf;
  } else
#endif /* LWIP_HTTPD_SUPPORT_REQUESTLIST */
  {
    data = (char *)p->payload;
    data_len = p->len;
    if (p->len != p->tot_len) {
      LWIP_DEBUGF(HTTPD_DEBUG, ("Warning: incomplete header due to chained pbufs\n"));
    }
  }

  /* received enough data for minimal request? */
  if (data_len >= MIN_REQ_LEN) {
    /* wait for CRLF before parsing anything */
    crlf = lwip_strnstr(data, CRLF, data_len);
    if (crlf != NULL) {

      int is_09 = 0;
      char *sp1, *sp2;
      u16_t left_len, uri_len;
      LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("CRLF received, parsing request\n"));
      /* parse method */
      if (!strncmp(data, "GET ", 4)) {
        sp1 = data + 3;
        /* received GET request */
        LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("Received GET request\"\n"));
      } else {
        /* null-terminate the METHOD (pbuf is freed anyway wen returning) */
        data[4] = 0;
        /* unsupported method! */
        LWIP_DEBUGF(HTTPD_DEBUG, ("Unsupported request method (not implemented): \"%s\"\n",
                                  data));
        return http_find_error_file(hs, 501);
      }
      /* if we come here, method is OK, parse URI */
      left_len = (u16_t)(data_len - ((sp1 + 1) - data));
      sp2 = lwip_strnstr(sp1 + 1, " ", left_len);

      uri_len = (u16_t)(sp2 - (sp1 + 1));
      if ((sp2 != 0) && (sp2 > sp1)) {
        /* wait for CRLFCRLF (indicating end of HTTP headers) before parsing anything */
        if (lwip_strnstr(data, CRLF CRLF, data_len) != NULL) {
          char *uri = sp1 + 1;
          /* null-terminate the METHOD (pbuf is freed anyway wen returning) */
          *sp1 = 0;
          uri[uri_len] = 0;
          LWIP_DEBUGF(HTTPD_DEBUG, ("Received \"%s\" request for URI: \"%s\"\n",
                                    data, uri));
          {
            return http_find_file(hs, uri, is_09);
          }
        }
      } else {
        LWIP_DEBUGF(HTTPD_DEBUG, ("invalid URI\n"));
      }
    }
  }

#if LWIP_HTTPD_SUPPORT_REQUESTLIST
  clen = pbuf_clen(hs->req);
  if ((hs->req->tot_len <= LWIP_HTTPD_REQ_BUFSIZE) &&
      (clen <= LWIP_HTTPD_REQ_QUEUELEN)) {
    /* request not fully received (too short or CRLF is missing) */
    return ERR_INPROGRESS;
  } else
#endif /* LWIP_HTTPD_SUPPORT_REQUESTLIST */
  {
    LWIP_DEBUGF(HTTPD_DEBUG, ("bad request\n"));
    /* could not parse request */
    return http_find_error_file(hs, 400);
  }
}

/** Try to find the file specified by uri and, if found, initialize hs
 * accordingly.
 *
 * @param hs the connection state
 * @param uri the HTTP header URI
 * @param is_09 1 if the request is HTTP/0.9 (no HTTP headers in response)
 * @return ERR_OK if file was found and hs has been initialized correctly
 *         another err_t otherwise
 */
static err_t
http_find_file(struct http_state *hs, const char *uri, int is_09)
{

    size_t loop;
    struct fs_file *file = NULL;
    char *params = NULL;
    err_t err;
  #if LWIP_HTTPD_CGI
    int i;
  #endif /* LWIP_HTTPD_CGI */
  #if !LWIP_HTTPD_SSI
    const
  #endif /* !LWIP_HTTPD_SSI */
    /* By default, assume we will not be processing server-side-includes tags */
    u8_t tag_check = 0;

    /* Have we been asked for the default file (in root or a directory) ? */
  #if LWIP_HTTPD_MAX_REQUEST_URI_LEN
    size_t uri_len = strlen(uri);
    if ((uri_len > 0) && (uri[uri_len - 1] == '/') &&
        ((uri != http_uri_buf) || (uri_len == 1))) {
      size_t copy_len = LWIP_MIN(sizeof(http_uri_buf) - 1, uri_len - 1);
      if (copy_len > 0) {
        MEMCPY(http_uri_buf, uri, copy_len);
        http_uri_buf[copy_len] = 0;
      }
  #else /* LWIP_HTTPD_MAX_REQUEST_URI_LEN */
    if ((uri[0] == '/') &&  (uri[1] == 0)) {
  #endif /* LWIP_HTTPD_MAX_REQUEST_URI_LEN */
      /* Try each of the configured default filenames until we find one
         that exists. */
      for (loop = 0; loop < NUM_DEFAULT_FILENAMES; loop++) {
        const char *file_name;
  #if LWIP_HTTPD_MAX_REQUEST_URI_LEN
        if (copy_len > 0) {
          size_t len_left = sizeof(http_uri_buf) - copy_len - 1;
          if (len_left > 0) {
            size_t name_len = strlen(httpd_default_filenames[loop].name);
            size_t name_copy_len = LWIP_MIN(len_left, name_len);
            MEMCPY(&http_uri_buf[copy_len], httpd_default_filenames[loop].name, name_copy_len);
            http_uri_buf[copy_len + name_copy_len] = 0;
          }
          file_name = http_uri_buf;
        } else
  #endif /* LWIP_HTTPD_MAX_REQUEST_URI_LEN */
        {
          file_name = httpd_default_filenames[loop].name;
        }
        LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("Looking for %s...\n", file_name));
        err = fs_open(&hs->file_handle, file_name);
        if (err == ERR_OK) {
          uri = file_name;
          file = &hs->file_handle;
          LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("Opened.\n"));
  #if LWIP_HTTPD_SSI
          tag_check = httpd_default_filenames[loop].shtml;
  #endif /* LWIP_HTTPD_SSI */
          break;
        }
      }
    }
    if (file == NULL) {
      /* No - we've been asked for a specific file. */
      /* First, isolate the base URI (without any parameters) */
      params = (char *)strchr(uri, '?');
      if (params != NULL) {
        /* URI contains parameters. NULL-terminate the base URI */
        *params = '\0';
        params++;
      }
  #if LWIP_HTTPD_CGI
      http_cgi_paramcount = -1;
      /* Does the base URI we have isolated correspond to a CGI handler? */
      if (httpd_num_cgis && httpd_cgis) {
        for (i = 0; i < httpd_num_cgis; i++) {
          if (strcmp(uri, httpd_cgis[i].pcCGIName) == 0) {
            /*
             * We found a CGI that handles this URI so extract the
             * parameters and call the handler.
             */
            http_cgi_paramcount = extract_uri_parameters(hs, params);
            uri = httpd_cgis[i].pfnCGIHandler(i, http_cgi_paramcount, hs->params,
                                           hs->param_vals);
            break;
          }
        }
      }
  #endif /* LWIP_HTTPD_CGI */

      LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("Opening %s\n", uri));

      err = fs_open(&hs->file_handle, uri);
      if (err == ERR_OK) {
        file = &hs->file_handle;
      } else {
        file = http_get_404_file(hs, &uri);
      }
  #if LWIP_HTTPD_SSI
      if (file != NULL) {
        if (file->flags & FS_FILE_FLAGS_SSI) {
          tag_check = 1;
        } else {
  #if LWIP_HTTPD_SSI_BY_FILE_EXTENSION
          tag_check = http_uri_is_ssi(file, uri);
  #endif /* LWIP_HTTPD_SSI_BY_FILE_EXTENSION */
        }
      }
  #endif /* LWIP_HTTPD_SSI */
    }
    if (file == NULL) {
      /* None of the default filenames exist so send back a 404 page */
      file = http_get_404_file(hs, &uri);
    }
    return http_init_file(hs, file, is_09, uri, tag_check, params);


}

/** Initialize a http connection with a file to send (if found).
 * Called by http_find_file and http_find_error_file.
 *
 * @param hs http connection state
 * @param file file structure to send (or NULL if not found)
 * @param is_09 1 if the request is HTTP/0.9 (no HTTP headers in response)
 * @param uri the HTTP header URI
 * @param tag_check enable SSI tag checking
 * @param params != NULL if URI has parameters (separated by '?')
 * @return ERR_OK if file was found and hs has been initialized correctly
 *         another err_t otherwise
 */
static err_t
http_init_file(struct http_state *hs, struct fs_file *file, int is_09, const char *uri,
               u8_t tag_check, char *params)
{

#if !LWIP_HTTPD_SUPPORT_V09
  LWIP_UNUSED_ARG(is_09);
#endif
  if (file != NULL) {
    /* file opened, initialise struct http_state */
#if !LWIP_HTTPD_DYNAMIC_FILE_READ
    /* If dynamic read is disabled, file data must be in one piece and available now */
    LWIP_ASSERT("file->data != NULL", file->data != NULL);
#endif

#if LWIP_HTTPD_SSI
    if (tag_check) {
      struct http_ssi_state *ssi = http_ssi_state_alloc();
      if (ssi != NULL) {
        ssi->tag_index = 0;
        ssi->tag_state = TAG_NONE;
        ssi->parsed = file->data;
        ssi->parse_left = file->len;
        ssi->tag_end = file->data;
        hs->ssi = ssi;
      }
    }
#else /* LWIP_HTTPD_SSI */
    LWIP_UNUSED_ARG(tag_check);
#endif /* LWIP_HTTPD_SSI */
    hs->handle = file;
#if LWIP_HTTPD_CGI_SSI
    if (params != NULL) {
      /* URI contains parameters, call generic CGI handler */
      int count;
#if LWIP_HTTPD_CGI
      if (http_cgi_paramcount >= 0) {
        count = http_cgi_paramcount;
      } else
#endif
      {
        count = extract_uri_parameters(hs, params);
      }
      httpd_cgi_handler(file, uri, count, http_cgi_params, http_cgi_param_vals
#if defined(LWIP_HTTPD_FILE_STATE) && LWIP_HTTPD_FILE_STATE
                        , file->state
#endif /* LWIP_HTTPD_FILE_STATE */
                       );
    }
#else /* LWIP_HTTPD_CGI_SSI */
    LWIP_UNUSED_ARG(params);
#endif /* LWIP_HTTPD_CGI_SSI */
    hs->file = file->data;
    LWIP_ASSERT("File length must be positive!", (file->len >= 0));
#if LWIP_HTTPD_CUSTOM_FILES
    if (file->is_custom_file && (file->data == NULL)) {
      /* custom file, need to read data first (via fs_read_custom) */
      hs->left = 0;
    } else
#endif /* LWIP_HTTPD_CUSTOM_FILES */
    {
      hs->left = (u32_t)file->len;
    }
    hs->retries = 0;
#if LWIP_HTTPD_TIMING
    hs->time_started = sys_now();
#endif /* LWIP_HTTPD_TIMING */
#if !LWIP_HTTPD_DYNAMIC_HEADERS
    LWIP_ASSERT("HTTP headers not included in file system",
                (hs->handle->flags & FS_FILE_FLAGS_HEADER_INCLUDED) != 0);
#endif /* !LWIP_HTTPD_DYNAMIC_HEADERS */
#if LWIP_HTTPD_SUPPORT_V09
    if (is_09 && ((hs->handle->flags & FS_FILE_FLAGS_HEADER_INCLUDED) != 0)) {
      /* HTTP/0.9 responses are sent without HTTP header,
         search for the end of the header. */
      char *file_start = lwip_strnstr(hs->file, CRLF CRLF, hs->left);
      if (file_start != NULL) {
        int diff = file_start + 4 - hs->file;
        hs->file += diff;
        hs->left -= (u32_t)diff;
      }
    }
#endif /* LWIP_HTTPD_SUPPORT_V09*/
  } else {
    hs->handle = NULL;
    hs->file = NULL;
    hs->left = 0;
    hs->retries = 0;
  }
#if LWIP_HTTPD_DYNAMIC_HEADERS
  /* Determine the HTTP headers to send based on the file extension of
   * the requested URI. */
  if ((hs->handle == NULL) || ((hs->handle->flags & FS_FILE_FLAGS_HEADER_INCLUDED) == 0)) {
    get_http_headers(hs, uri);
  }
#else /* LWIP_HTTPD_DYNAMIC_HEADERS */
  LWIP_UNUSED_ARG(uri);
#endif /* LWIP_HTTPD_DYNAMIC_HEADERS */
#if LWIP_HTTPD_SUPPORT_11_KEEPALIVE
  if (hs->keepalive) {
#if LWIP_HTTPD_SSI
    if (hs->ssi != NULL) {
      hs->keepalive = 0;
    } else
#endif /* LWIP_HTTPD_SSI */
    {
      if ((hs->handle != NULL) &&
          ((hs->handle->flags & (FS_FILE_FLAGS_HEADER_INCLUDED | FS_FILE_FLAGS_HEADER_PERSISTENT)) == FS_FILE_FLAGS_HEADER_INCLUDED)) {
        hs->keepalive = 0;
      }
    }
  }
#endif /* LWIP_HTTPD_SUPPORT_11_KEEPALIVE */
  return ERR_OK;

}

/**
 * The pcb had an error and is already deallocated.
 * The argument might still be valid (if != NULL).
 */
static void http_err(void *arg, err_t err)
{
    struct http_state *hs = (struct http_state*) arg;
    LWIP_UNUSED_ARG(err);

    LWIP_DEBUGF(HTTPD_DEBUG, ("http_err: %s", lwip_strerr(err)));

    if (hs != NULL)
    {
        http_state_free(hs);
    }
}

/**
 * Data has been sent and acknowledged by the remote host.
 * This means that more data can be sent.
 */
static err_t http_sent(void *arg, struct altcp_pcb *pcb, u16_t len)
{
    struct http_state *hs = (struct http_state*) arg;

    LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("http_sent %p\n", (void *)pcb));

    LWIP_UNUSED_ARG(len);

    if (hs == NULL)
    {
        return ERR_OK;
    }

    hs->retries = 0;

    http_send(pcb, hs);

    return ERR_OK;
}

/**
 * The poll function is called every 2nd second.
 * If there has been no data sent (which resets the retries) in 8 seconds, close.
 * If the last portion of a file has not been sent in 2 seconds, close.
 *
 * This could be increased, but we don't want to waste resources for bad connections.
 */
static err_t http_poll(void *arg, struct altcp_pcb *pcb)
{
    struct http_state *hs = (struct http_state*) arg;
    LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("http_poll: pcb=%p hs=%p pcb_state=%s\n",
                    (void *)pcb, (void *)hs, tcp_debug_state_str(altcp_dbg_get_tcp_state(pcb))));

    if (hs == NULL)
    {
        err_t closed;
        /* arg is null, close. */
        LWIP_DEBUGF(HTTPD_DEBUG, ("http_poll: arg is NULL, close\n"));
        closed = http_close_conn(pcb, NULL);
        LWIP_UNUSED_ARG(closed);
        return ERR_OK;
    }
    else
    {
        hs->retries++;
        if (hs->retries == HTTPD_MAX_RETRIES)
        {
            LWIP_DEBUGF(HTTPD_DEBUG, ("http_poll: too many retries, close\n"));
            http_close_conn(pcb, hs);
            return ERR_OK;
        }

        /* If this connection has a file open, try to send some more data. If
         * it has not yet received a GET request, don't do this since it will
         * cause the connection to close immediately. */
        if (hs->handle)
        {
            LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("http_poll: try to send more data\n"));
            if (http_send(pcb, hs))
            {
                /* If we wrote anything to be sent, go ahead and send it now. */
                LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("tcp_output\n"));
                altcp_output(pcb);
            }
        }
    }

    return ERR_OK;
}

/**
 * Data has been received on this pcb.
 * For HTTP 1.0, this should normally only happen once (if the request fits in one packet).
 */
static err_t http_recv(void *arg, struct altcp_pcb *pcb, struct pbuf *p, err_t err)
{
    struct http_state *hs = (struct http_state*) arg;
    LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("http_recv: pcb=%p pbuf=%p err=%s\n", (void *)pcb,
                    (void *)p, lwip_strerr(err)));

    if ((err != ERR_OK) || (p == NULL) || (hs == NULL))
    {
        /* error or closed by other side? */
        if (p != NULL)
        {
            /* Inform TCP that we have taken the data. */
            altcp_recved(pcb, p->tot_len);
            pbuf_free(p);
        }
        if (hs == NULL)
        {
            /* this should not happen, only to be robust */
            LWIP_DEBUGF(HTTPD_DEBUG, ("Error, http_recv: hs is NULL, close\n"));
        }
        http_close_conn(pcb, hs);
        return ERR_OK;
    }

    /* Inform TCP that we have taken the data. */
    altcp_recved(pcb, p->tot_len);

    if (hs->handle == NULL)
    {
        err_t parsed = http_parse_request(p, hs, pcb);
        LWIP_ASSERT("http_parse_request: unexpected return value",
                parsed == ERR_OK || parsed == ERR_INPROGRESS
                        || parsed == ERR_ARG || parsed == ERR_USE);
#if LWIP_HTTPD_SUPPORT_REQUESTLIST
        if (parsed != ERR_INPROGRESS)
        {
            /* request fully parsed or error */
            if (hs->req != NULL)
            {
                pbuf_free(hs->req);
                hs->req = NULL;
            }
        }
#endif /* LWIP_HTTPD_SUPPORT_REQUESTLIST */
        pbuf_free(p);
        if (parsed == ERR_OK)
        {
            {
                LWIP_DEBUGF(HTTPD_DEBUG | LWIP_DBG_TRACE, ("http_recv: data %p len %"S32_F"\n", (const void *)hs->file, hs->left));
                http_send(pcb, hs);
            }
        }
        else if (parsed == ERR_ARG)
        {
            /* @todo: close on ERR_USE? */
            http_close_conn(pcb, hs);
        }
    }
    else
    {
        LWIP_DEBUGF(HTTPD_DEBUG, ("http_recv: already sending data\n"));
        /* already sending but still receiving data, we might want to RST here? */
        pbuf_free(p);
    }

    return ERR_OK;
}

/**
 * A new incoming connection has been accepted.
 */
static err_t http_accept(void *arg, struct altcp_pcb *pcb, err_t err)
{
    struct http_state *hs;
    LWIP_UNUSED_ARG(err);
    LWIP_UNUSED_ARG(arg); LWIP_DEBUGF(HTTPD_DEBUG, ("http_accept %p / %p\n", (void *)pcb, arg));

    if ((err != ERR_OK) || (pcb == NULL))
    {
        return ERR_VAL;
    }

    /* Set priority */
    altcp_setprio(pcb, HTTPD_TCP_PRIO);

    /* Allocate memory for the structure that holds the state of the
     connection - initialized by that function. */
    hs = http_state_alloc();
    if (hs == NULL)
    {
        LWIP_DEBUGF(HTTPD_DEBUG, ("http_accept: Out of memory, RST\n"));
        return ERR_MEM;
    }
    hs->pcb = pcb;

    /* Tell TCP that this is the structure we wish to be passed for our
     callbacks. */
    altcp_arg(pcb, hs);

    /* Set up the various callback functions */
    altcp_recv(pcb, http_recv);
    altcp_err(pcb, http_err);
    altcp_poll(pcb, http_poll, HTTPD_POLL_INTERVAL);
    altcp_sent(pcb, http_sent);

    return ERR_OK;
}

static void httpd_init_pcb(struct altcp_pcb *pcb, u16_t port)
{

    if (pcb)
    {
        altcp_setprio(pcb, HTTPD_TCP_PRIO);
        /* set SOF_REUSEADDR here to explicitly bind httpd to multiple interfaces */
        err_t err = altcp_bind(pcb, IP4_ADDR_ANY, port);
        LWIP_UNUSED_ARG(err); /* in case of LWIP_NOASSERT */
        LWIP_ASSERT("httpd_init: tcp_bind failed", err == ERR_OK);
        pcb = altcp_listen(pcb);
        LWIP_ASSERT("httpd_init: tcp_listen failed", pcb != NULL);
        altcp_accept(pcb, http_accept);
    }
}

/**
 * @ingroup httpd
 * Initialize the httpd: set up a listening PCB and bind it to the defined port
 */
void httpd_init(void)
{
    struct altcp_pcb *pcb;

    LWIP_DEBUGF(HTTPD_DEBUG, ("httpd_init\n"));

    pcb = altcp_tcp_new_ip_type(IPADDR_TYPE_V4);
    LWIP_ASSERT("httpd_init: tcp_new failed", pcb != NULL);
    httpd_init_pcb(pcb, HTTPD_SERVER_PORT);
}

#ifndef NO_SYS
#error "NO_SYS undefined"
#endif

#if !NO_SYS
#error "NO_SYS is 0"
#endif
