/*!
 *  \example appWebServer.c
 *
 *  \brief
 *  Application Web Server task
 *
 *  \author
 *  KUNBUS GmbH
 *
 *  \date
 *  2022-05-19
 *
 *  \copyright
 *  Copyright (c) 2022, KUNBUS GmbH<br /><br />
 *  All rights reserved.<br />
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:<br />
 *  <ol>
 *  <li>Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.</li>
 *  <li>Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.</li>
 *  <li>Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.</li>
 *  </ol>
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "kernel/dpl/TaskP.h"
#include "kernel/dpl/SystemP.h"
#include "osal.h"
#include "osal_error.h"
#include "appWebServer.h"
#include "appWebServerData.h"
#include "lwip/sockets.h"
#include "lwip/errno.h"

#include "CMN_CPU_api.h"

/*!
 *  \brief Application Webserver task's stack size.
 */
#define APP_WEBSRV_TASK_STACK_SIZE     4096U

/*!
 *  \brief Application Webserver task's stack.
 */
static uint8_t 
APP_aWebSrvTaskStack_g[APP_WEBSRV_TASK_STACK_SIZE] __attribute__((aligned(32), section(".threadstack"))) = {0};

/*!
 *  \brief Application Webserver task's object.
 */
static TaskP_Object APP_WebSrvTaskObj_g = { 0 };

/*!
*  Function: APP_WebSrvStrError
*
*  \brief
*  Helper function to map errno to a string.
*
*  \details
*  Helper function to map errno to a string. Only the relevant 
*  errno for sockets are mapped.
*
*  \param[inout]     err_p         errno set by the lwip stack.
*
*  \return           result of the operation as character string.
*
*/
static char *APP_WebSrvStrError(int err_p)
{
    switch (err_p)
    {
    case 0:  return ("No error");
    case EACCES:  return ("Permission denied");
    case EPERM:  return ("Permission denied");
    case EADDRINUSE:  return ("Local Address already in use");
    case EADDRNOTAVAIL:  return ("Address not available");
    case EAFNOSUPPORT:  return ("Incorrect address family address");
    case EAGAIN:  return ("Operation would block, Try Again");
    case EALREADY:  return ("Previous operation still in progress");
    case EBADF:  return ("Not a valid file descriptor");
    case ECONNREFUSED:  return ("Connection refused");
    case EFAULT:  return ("Socket structure address is not valid");
    case EINPROGRESS:  return ("Previous operation is now in progress");
    case EINTR:  return ("Operation interrupted by signal");
    case EISCONN:  return ("Socket already in use");
    case ENETUNREACH:  return ("Network is unreachable");
    case ENOTSOCK:  return ("Given file descriptor is not a socket");
    case EPROTOTYPE:  return ("Socket does not support the comms protocol");
    case ETIMEDOUT:  return ("Timeout while attempting connection");
    case ELOOP:  return ("Too many symbolic links were encountered");
    case ENAMETOOLONG:  return ("File name too long");
    case ENOENT:  return ("Socket pathname does not exist");
    case ENOMEM:  return ("Insufficient kernel memory");
    case ENOTDIR:  return ("Path prefix is not a directory");
    case EROFS:  return ("Socket inode in a read-only filesystem");
    case EOPNOTSUPP:  return ("Operation not supported");
    case EPROTO:  return ("Socket does not support the comms protocol");
    case ECONNABORTED:  return ("Connection aborted");
    case ECONNRESET:  return ("Connection reset by peer");
    case ENOTCONN:  return ("Transport endpoint not connected");
    }

    return ("Unknown error");
}


/*!
*  Function: APP_WebSrvIsFatalErr
*
*  \brief
*  Helper function to check if errno is fatal.
*
*  \details
*  Helper function to check if errno is fatal.
*  Non-Fatal are : 
*  EAGAIN
*  EWOULDBLOCK
*  ECONNABORTED
*  EINTR
*  ECONNRESET
* 
*  \param[inout]     errNo_p    errno set by the lwip stack.
*
*  \return           result of the operation as bool
*  \retval           true          Error is fatal
*  \retval           false         Error is not fatal
*
*/
static bool APP_WebSrvIsFatalErr(int errNo_p)
{
    bool isFatal = true;
    if ((errno == EAGAIN) ||
        (errno == EWOULDBLOCK) ||
        (errno == ECONNABORTED) ||
        (errno == EINTR) ||
        (errno == ECONNRESET))
    {
        isFatal = false;
    }
    return isFatal;
}

/*!
*  Function: APP_WebSrvInit
*
*  \brief
*  Helper function to initialize the http socket.
*
*  \details
*  Helper function to initialize the http socket. 
*  If an operation fails the reason for the failure is read through errno.
*
*
*  \param[inout]     pSockFd_p   Pointer to location where the socket file descriptor will be stored.
*
*  \return           result of the operation as int
*  \retval            0          Operation succeeded
*  \retval           -1          Operation failed
*
*/
static int APP_WebSrvInit(int *pSockFd_p)
{
    int sock = -1;
    int ret = -1;
    struct sockaddr_in addr = { 0 };
    addr.sin_family = AF_INET;
    addr.sin_port = PP_HTONS(80);
    addr.sin_addr.s_addr = PP_HTONL(INADDR_ANY);

    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0)
    {
        DebugP_log("Failed to open socket : %d : %s \r\n", -errno, 
                   APP_WebSrvStrError(errno));
    }
    else
    {
        ret = bind(sock, (struct sockaddr*)&addr, sizeof(addr));

        if (ret == 0)
        {
            ret = listen(sock, 3);
            if (ret == 0)
            {
                *pSockFd_p = sock;
            }
        }

        if (ret < 0)
        {
            DebugP_log("Failed to bind / listen to socket : %d : %s \r\n", 
                       -errno, APP_WebSrvStrError(errno));
        }
    }

    return ret;
}

/*!
*  Function: APP_WebSrvProcessGetAndRespond
*
*  \brief
*  Function to process the received http GET request and respond to it.
*
*  \details
*  Function to process the received http GET request and respond to it.
*  At the moment it only responds to the following get requests :
*  "/"
*  "/main.html"
*  "/main.js"
*  "/favicon.ico
*  "/cpuLoad"
*  A 404 error will be returned for any other request.
*   
*  \param[in]        clientFd_p   Client file descriptor for sending response.
*  \param[in]        pBuf_p       Pointer to received data (with "GET " request removed).
*
*  \return           result of the operation as int
*  \retval            > 0          Operation succeeded
*  \retval            < 0          Operation failed
*
*/
static int APP_WebSrvProcessGetAndRespond(int clientFd_p, const char *const pBuf_p)
{
    int ret = -1;

    CMN_CPU_API_SData_t* data = CMN_CPU_API_getData();

    if ((strncmp(&pBuf_p[0], "/ HTTP/1.1\r\n", 12) == 0) ||
        (strncmp(&pBuf_p[0], "/main.html HTTP/1.1\r\n", 21) == 0))
    {
        // get request for main.html
        ret = send(clientFd_p, response_200_content_html, strlen(response_200_content_html), 0);
            
        if (ret > 0)
        {
            ret = send(clientFd_p, main_html, strlen(main_html), 0);
        }
    }
    else if ((strncmp(&pBuf_p[0], "/main.js HTTP/1.1\r\n", 19) == 0))
    {
        // get request for main.js
        ret = send(clientFd_p, response_200_content_js, strlen(response_200_content_js), 0);

        if (ret > 0)
        {
            ret = send(clientFd_p, main_js, strlen(main_js), 0);
        }
    }
    else if ((strncmp(&pBuf_p[0], "/favicon.ico HTTP/1.1\r\n", 23) == 0))
    {
        // get request for favicon.ico
        ret = send(clientFd_p, response_200_content_image, strlen(response_200_content_image), 0);

        if (ret > 0)
        {
            ret = send(clientFd_p, favicon_ico, sizeof(favicon_ico), 0);
        }
    }
    else if ((strncmp(&pBuf_p[0], "/cpuLoad", 8) == 0))
    {
        // get request for cpuLoad
        static char cpuLoad[1200] = "";
        int snprintfRes = 0;
        uint32_t lineValue = 0;
        uint32_t textSize = 0;
        uint32_t idIdle = -1;
        CMN_CPU_API_SLoad_t idleLoad;


        memset(&cpuLoad[0], 0, sizeof(cpuLoad));

        for (int i = 0; i < data->tasksNum + 1; i++)
        {
            if (strncmp(data->tasks[i].name, "IDLE", 32) == 0)
            {
                idleLoad.cpuLoad    = data->tasks[i].cpuLoad;
                idleLoad.exists     = data->tasks[i].exists;
                idleLoad.taskHandle = data->tasks[i].taskHandle;

                memcpy(idleLoad.name, data->tasks[i].name, 32);

                idIdle = i;
                break;
            }
        }

        if (idIdle != -1)
        {
            data->tasks[idIdle].cpuLoad    = data->tasks[0].cpuLoad;
            data->tasks[idIdle].exists     = data->tasks[0].exists;
            data->tasks[idIdle].taskHandle = data->tasks[0].taskHandle;

            memcpy(data->tasks[idIdle].name, data->tasks[0].name, 32);

            data->tasks[0].cpuLoad    = idleLoad.cpuLoad;
            data->tasks[0].exists     = idleLoad.exists;
            data->tasks[0].taskHandle = idleLoad.taskHandle;

            memcpy(data->tasks[0].name, idleLoad.name, 32);
        }

        for (int i = 0; i < data->tasksNum + 1; i++)
        {
            if (i == 0)
            {
                snprintfRes = snprintf(&cpuLoad[textSize], 33, "%s", data->cpu.name);
                textSize += snprintfRes;
                textSize++;

                snprintfRes = snprintf(&cpuLoad[textSize], 2, "%s", ",");
                textSize += snprintfRes;

                lineValue   = data->cpu.cpuLoad;
                snprintfRes = snprintf(&cpuLoad[textSize], 8, "%2d.%2d %%", \
                                       lineValue / 100, lineValue % 100);
                textSize += snprintfRes;
                textSize++;

                snprintfRes = snprintf(&cpuLoad[textSize], 2, "%s", ",");
                textSize += snprintfRes;
                textSize++;
            }
            else
            {
                snprintfRes = snprintf(&cpuLoad[textSize], 33, "%s", data->tasks[i - 1].name);
                textSize += snprintfRes;
                textSize++;

                snprintfRes = snprintf(&cpuLoad[textSize], 2, "%s", ",");
                textSize += snprintfRes;

                lineValue   = data->tasks[i-1].cpuLoad;
                snprintfRes = snprintf(&cpuLoad[textSize], 8, "%2d.%2d %%", \
                                       lineValue / 100, lineValue % 100);
                textSize += snprintfRes;
                textSize++;

                snprintfRes = snprintf(&cpuLoad[textSize], 2, "%s", ",");
                textSize += snprintfRes;
                textSize++;
            }
        }

        if (textSize > 0)
        {
            ret = send(clientFd_p, cpuLoad, textSize, 0);
        }
    }
    else
    {
        // Unknown get request
        ret = send(clientFd_p, response_404, strlen(response_404), 0);
    }
    return ret;
}

/*!
*  Function: APP_WebSrvTask
*
*  \brief
*  Webserver task function.
*
*  \details
*  Webserver task function. It waits for bytes to arrive in a blocking manner.
*  If an operation fails the reason for the failure is read through errno.
*  If the error is other than : EAGAIN, EWOULDBLOCK, ECONNABORTED, EINTR, 
*  ECONNRESET and ENOTCONN (only for shutdown), then the task is stopped.
*
*  \param[in]        pArgs_p   Not used.
*
*  \return           None
*
*/
static void APP_WebSrvTask(void* pArgs_p)
{
    int socketFd = -1;
    int ret = APP_WebSrvInit(&socketFd);

    if ((ret == 0) && (socketFd >= 0))
    {
        while (1)
        {
            int clientFd = accept(socketFd, NULL, NULL);
            static char aBuf[1500] = { 0 };
            memset(aBuf, 0, sizeof(aBuf));
            if (clientFd < 0)
            {
                if (APP_WebSrvIsFatalErr(errno))
                {
                    // If error is fatal log it and stop webserver task.
                    DebugP_log("Failed to accept connection : %d : %s \r\n", -errno,
                               APP_WebSrvStrError(errno));
                    break;
                }
                continue;
            }

            ret = recv(clientFd, aBuf, sizeof(aBuf), 0);
            if (ret < 0)
            {
                if (APP_WebSrvIsFatalErr(errno))
                {
                    // If error is fatal log it and stop webserver task.
                    DebugP_log("Failed to receive bytes on connection : %d : %s \r\n", -errno,
                               APP_WebSrvStrError(errno));
                    break;
                }
            }
            else if (ret == 0)
            {
                DebugP_log("Connection closed ! \r\n");
            }
            else
            {   
                if (strncmp(&aBuf[0], "GET ", 4) == 0)
                {
                    // We received some bytes and it is a get request, 
                    // call processing function.
                    ret = APP_WebSrvProcessGetAndRespond(clientFd, &aBuf[4]);
                    if (ret <= 0)
                    {
                        if (APP_WebSrvIsFatalErr(errno))
                        {
                            DebugP_log("Failed to process request sent : %d : %s \r\n", -errno,
                                       APP_WebSrvStrError(errno));
                            break;
                        }
                    }
                }
            }

            ret = shutdown(clientFd, SHUT_RDWR);
            if (ret != 0)
            {
                if ((APP_WebSrvIsFatalErr(errno)) &&
                    (errno != ENOTCONN))
                {
                    DebugP_log("Failed to shutdown connection : %d : %s \r\n", -errno,
                               APP_WebSrvStrError(errno));
                    break;
                }
            }

            ret = close(clientFd);
            if (ret != 0)
            {
                if (APP_WebSrvIsFatalErr(errno))
                {
                    DebugP_log("Failed to close connection : %d : %s \r\n", -errno,
                               APP_WebSrvStrError(errno));
                    break;
                }
            }

            OSAL_SCHED_sleep(APP_WEBSRV_TASK_TICK_MS);
        }

        ret = close(socketFd);
        if (ret != 0)
        {
            DebugP_log("Failed to close socket : %d : %s \r\n", -errno, 
                       APP_WebSrvStrError(errno));
        }
    }

    TaskP_exit();
}

/*!
 *  Function: APP_startWebServerTask
 *
 *  \brief
 *  Function to start the Webserver task.
 *
 *  \details
 *  Function to start the Webserver task.
 *
 *  \param[in]     None
 *
 *  \return        Result of the operation as boolean.
 *  \retval        true        Operation completed successfully.
 *  \retval        false       Operation could not be completed successfully.
 */
bool APP_startWebServerTask(APP_WEBSRV_SParams_t* pParams_p)
{   
    bool result = true;
    int32_t taskErr = SystemP_SUCCESS;
    TaskP_Params webserverTaskParam = { 0 };

    TaskP_Params_init (&webserverTaskParam);

    webserverTaskParam.name        = "webserver_task";
    webserverTaskParam.stackSize   = sizeof(APP_aWebSrvTaskStack_g);
    webserverTaskParam.stack       = (uint8_t*)APP_aWebSrvTaskStack_g;
    webserverTaskParam.priority    = pParams_p->taskPrio;
    webserverTaskParam.taskMain    = (TaskP_FxnMain)APP_WebSrvTask;
    webserverTaskParam.args        = pParams_p;

    taskErr = TaskP_construct(&APP_WebSrvTaskObj_g, &webserverTaskParam);
    if (taskErr != SystemP_SUCCESS)
    {
        OSAL_printf("[APP] ERROR: Failed to create task %s (%ld)\r\n", 
                    webserverTaskParam.name, taskErr);
        TaskP_destruct(&APP_WebSrvTaskObj_g);
        result = false;
    }
    return result;
}
