/**
 * NOTE:
 *
 * HAL_TCP_xxx API reference implementation: wrappers/os/ubuntu/HAL_TCP_linux.c
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
 
#include <pthread.h>
#include <unistd.h>
#include <sys/time.h>
#include <errno.h>
#include <assert.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <time.h>
#include <signal.h>
 
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
 
#include "esp_wifi.h"
#include "esp_timer.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "sdkconfig.h"
 
#include "infra_types.h"
#include "infra_defs.h"
//#include "infra_compat.h"
#include "wrappers_defs.h"
#include "stdarg.h"
 
#include "lwip/netdb.h"

char _product_key[IOTX_PRODUCT_KEY_LEN + 1]       = "a1teJOs9bNj";
char _product_secret[IOTX_PRODUCT_SECRET_LEN + 1] = "hAb28jYss9xwAyA9";
char _device_name[IOTX_DEVICE_NAME_LEN + 1]       = "Light001";
char _device_secret[IOTX_DEVICE_SECRET_LEN + 1]   = "sYs9LEWGoGB9dYdKhXk5oQTv461XVBC3";
 
static const char* TAG = "in";
 
#define hal_emerg(...)      ESP_LOGE(TAG, __VA_ARGS__)
#define hal_crit(...)       ESP_LOGE(TAG, __VA_ARGS__)
#define hal_err(...)        ESP_LOGE(TAG, __VA_ARGS__)
#define hal_warning(...)    ESP_LOGW(TAG, __VA_ARGS__)
#define hal_info(...)       ESP_LOGI(TAG, __VA_ARGS__)
#define hal_debug(...)      ESP_LOGD(TAG, __VA_ARGS__)
 
#define HAL_TCP_CONNECT_TIMEOUT 10 * 1000000
 
 
static uint64_t _linux_get_time_ms(void)
{
    struct timeval tv = { 0 };
    uint64_t time_ms;
 
    gettimeofday(&tv, NULL);
 
    time_ms = tv.tv_sec * 1000 + tv.tv_usec / 1000;
 
    return time_ms;
}
 
static uint64_t _linux_time_left(uint64_t t_end, uint64_t t_now)
{
    uint64_t t_left;
 
    if (t_end > t_now) {
        t_left = t_end - t_now;
    } else {
        t_left = 0;
    }
 
    return t_left;
}


/**
 * @brief Deallocate memory block
 *
 * @param[in] ptr @n Pointer to a memory block previously allocated with platform_malloc.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_Free(void *ptr)
{
	free(ptr);	//vPortFree(ptr);
	return;
}


/**
 * @brief Get device name from user's system persistent storage
 *
 * @param [ou] device_name: array to store device name, max length is IOTX_DEVICE_NAME_LEN
 * @return the actual length of device name
 */
int HAL_GetDeviceName(char device_name[IOTX_DEVICE_NAME_LEN + 1])
{
    int len = strlen(_device_name);
    memset(device_name, 0x0, IOTX_DEVICE_NAME_LEN + 1);

    strncpy(device_name, _device_name, len);

    return strlen(device_name);
}


/**
 * @brief Get device secret from user's system persistent storage
 *
 * @param [ou] device_secret: array to store device secret, max length is IOTX_DEVICE_SECRET_LEN
 * @return the actual length of device secret
 */
int HAL_GetDeviceSecret(char device_secret[IOTX_DEVICE_SECRET_LEN + 1])
{
    int len = strlen(_device_secret);
    memset(device_secret, 0x0, IOTX_DEVICE_SECRET_LEN + 1);

    strncpy(device_secret, _device_secret, len);

    return len;
}


/**
 * @brief Get firmware version
 *
 * @param [ou] version: array to store firmware version, max length is IOTX_FIRMWARE_VER_LEN
 * @return the actual length of firmware version
 */
int HAL_GetFirmwareVersion(char *version)
{
    char *ver = "app-1.0.0-20180101.1000";
    int len = strlen(ver);
    memset(version, 0x0, IOTX_FIRMWARE_VER_LEN);
    strncpy(version, ver, IOTX_FIRMWARE_VER_LEN);
    version[len] = '\0';
    return strlen(version);
}


/**
 * @brief Get product key from user's system persistent storage
 *
 * @param [ou] product_key: array to store product key, max length is IOTX_PRODUCT_KEY_LEN
 * @return  the actual length of product key
 */
int HAL_GetProductKey(char product_key[IOTX_PRODUCT_KEY_LEN + 1])
{
    int len = strlen(_product_key);
    memset(product_key, 0x0, IOTX_PRODUCT_KEY_LEN + 1);

    strncpy(product_key, _product_key, len);

    return len;
}


int HAL_GetProductSecret(char product_secret[IOTX_PRODUCT_SECRET_LEN + 1])
{
    int len = strlen(_product_secret);
    memset(product_secret, 0x0, IOTX_PRODUCT_SECRET_LEN + 1);

    strncpy(product_secret, _product_secret, len);

    return len;
}


/**
 * @brief Allocates a block of size bytes of memory, returning a pointer to the beginning of the block.
 *
 * @param [in] size @n specify block size in bytes.
 * @return A pointer to the beginning of the block.
 * @see None.
 * @note Block value is indeterminate.
 */
void *HAL_Malloc(uint32_t size)
{
	return malloc(size);	//return pvPortMalloc(size);
}


void *HAL_MutexCreate(void)
{
    int err_num;
    pthread_mutex_t *mutex = (pthread_mutex_t *)HAL_Malloc(sizeof(pthread_mutex_t));
    if (NULL == mutex) {
        return NULL;
    }

    if (0 != (err_num = pthread_mutex_init(mutex, NULL))) {
        hal_err("create mutex failed");
        HAL_Free(mutex);
        return NULL;
    }

    return mutex;
}

void HAL_MutexDestroy(_IN_ void *mutex)
{
    int err_num;

    if (!mutex) {
        hal_warning("mutex want to destroy is NULL!");
        return;
    }
    if (0 != (err_num = pthread_mutex_destroy((pthread_mutex_t *)mutex))) {
        hal_err("destroy mutex failed");
    }

    HAL_Free(mutex);
}

void HAL_MutexLock(_IN_ void *mutex)
{
    int err_num;
    if (0 != (err_num = pthread_mutex_lock((pthread_mutex_t *)mutex))) {
        hal_err("lock mutex failed: - '%s' (%d)", strerror(err_num), err_num);
    }
}

void HAL_MutexUnlock(_IN_ void *mutex)
{
    int err_num;
    if (0 != (err_num = pthread_mutex_unlock((pthread_mutex_t *)mutex))) {
        hal_err("unlock mutex failed - '%s' (%d)", strerror(err_num), err_num);
    }
}


/**
 * @brief Writes formatted data to stream.
 *
 * @param [in] fmt: @n String that contains the text to be written, it can optionally contain embedded format specifiers
     that specifies how subsequent arguments are converted for output.
 * @param [in] ...: @n the variable argument list, for formatted and inserted in the resulting string replacing their respective specifiers.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_Printf(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    fflush(stdout);

	return;
}


uint32_t random(void)
{
    return esp_random();
}

uint32_t HAL_Random(uint32_t region)
{
	return (region > 0) ? (random() % region) : 0;
}


/**
 * @brief Sleep thread itself.
 *
 * @param [in] ms @n the time interval for which execution is to be suspended, in milliseconds.
 * @return None.
 * @see None.
 * @note None.
 */
void HAL_SleepMs(uint32_t ms)
{
	usleep(1000 * ms);
	return;
}


/**
 * @brief Writes formatted data to string.
 *
 * @param [out] str: @n String that holds written text.
 * @param [in] len: @n Maximum length of character will be written
 * @param [in] fmt: @n Format that contains the text to be written, it can optionally contain embedded format specifiers
     that specifies how subsequent arguments are converted for output.
 * @param [in] ...: @n the variable argument list, for formatted and inserted in the resulting string replacing their respective specifiers.
 * @return bytes of character successfully written into string.
 * @see None.
 * @note None.
 */
int HAL_Snprintf(char *str, const int len, const char *fmt, ...)
{
    va_list args;
    int rc;

    va_start(args, fmt);
    rc = vsnprintf(str, len, fmt, args);
    va_end(args);

    return rc;
}


void HAL_Srandom(uint32_t seed)
{
    // espressif does not need a seed for esp_random()
#if 0
    srandom(seed);
#endif
	return;
}


/**
 * @brief Destroy the specific TCP connection.
 *
 * @param [in] fd: @n Specify the TCP connection by handle.
 *
 * @return The result of destroy TCP connection.
 * @retval < 0 : Fail.
 * @retval   0 : Success.
 */
int HAL_TCP_Destroy(uintptr_t fd)
{
    int rc;
 
    /* Shutdown both send and receive operations. */
    rc = shutdown((int) fd, 2);
    if (0 != rc) {
        hal_err("shutdown error");
        return -1;
    }
 
    rc = close((int) fd);
    if (0 != rc) {
        hal_err("closesocket error");
        return -1;
    }
 
    return 0;
}

/**
 * @brief Establish a TCP connection.
 *
 * @param [in] host: @n Specify the hostname(IP) of the TCP server
 * @param [in] port: @n Specify the TCP port of TCP server
 *
 * @return The handle of TCP connection.
   @retval   0 : Fail.
   @retval > 0 : Success, the value is handle of this TCP connection.
 */
    uintptr_t HAL_TCP_Establish(const char *host, uint16_t port)
{
     struct addrinfo hints;
    struct addrinfo *addrInfoList = NULL;
    struct addrinfo *cur = NULL;
    int fd = 0;
    int rc = 0;
    char service[6];
 
    memset(&hints, 0, sizeof(hints));
 
    hal_info("establish tcp connection with server(host='%s', port=[%u])", host, port);
 
    hints.ai_family = AF_INET; /* only IPv4 */
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    sprintf(service, "%u", port);
 
    if ((rc = getaddrinfo(host, service, &hints, &addrInfoList)) != 0) {
        hal_err("getaddrinfo error(%d), host = '%s', port = [%d]", rc, host, port);
        return -1;
    }
 
    for (cur = addrInfoList; cur != NULL; cur = cur->ai_next) {
        if (cur->ai_family != AF_INET) {
            hal_err("socket type error");
            rc = -1;
            continue;
        }
 
        fd = socket(cur->ai_family, cur->ai_socktype, cur->ai_protocol);
        if (fd < 0) {
            hal_err("create socket error");
            rc = -1;
            continue;
        }
 
        if (connect(fd, cur->ai_addr, cur->ai_addrlen) == 0) {
            rc = fd;
            break;
        }
 
        close(fd);
        hal_err("connect error");
        rc = -1;
    }
 
    if (-1 == rc) {
        hal_err("fail to establish tcp");
    } else {
        hal_info("success to establish tcp, fd=%d", rc);
    }
    freeaddrinfo(addrInfoList);
 
    return (uintptr_t)rc;
 
 
}
 
 
/**
 * @brief Read data from the specific TCP connection with timeout parameter.
 *        The API will return immediately if 'len' be received from the specific TCP connection.
 *
 * @param [in] fd @n A descriptor identifying a TCP connection.
 * @param [out] buf @n A pointer to a buffer to receive incoming data.
 * @param [out] len @n The length, in bytes, of the data pointed to by the 'buf' parameter.
 * @param [in] timeout_ms @n Specify the timeout value in millisecond. In other words, the API block 'timeout_ms' millisecond maximumly.
 *
 * @retval       -2 : TCP connection error occur.
 * @retval       -1 : TCP connection be closed by remote server.
 * @retval        0 : No any data be received in 'timeout_ms' timeout period.
 * @retval (0, len] : The total number of bytes be received in 'timeout_ms' timeout period.
 
 * @see None.
 */
    int32_t HAL_TCP_Read(uintptr_t fd, char *buf, uint32_t len, uint32_t timeout_ms)
{
    int ret, err_code;
    uint32_t len_recv;
    uint64_t t_end, t_left;
    fd_set sets;
    struct timeval timeout;
 
    t_end = _linux_get_time_ms() + timeout_ms;
    len_recv = 0;
    err_code = 0;
 
    do {
        t_left = _linux_time_left(t_end, _linux_get_time_ms());
        if (0 == t_left) {
            break;
        }
        FD_ZERO(&sets);
        FD_SET(fd, &sets);
 
        timeout.tv_sec = t_left / 1000;
        timeout.tv_usec = (t_left % 1000) * 1000;
 
        ret = select(fd + 1, &sets, NULL, NULL, &timeout);
        if (ret > 0) {
            ret = recv(fd, buf + len_recv, len - len_recv, 0);
            if (ret > 0) {
                len_recv += ret;
            } else if (0 == ret) {
                hal_err("connection is closed");
                err_code = -1;
                break;
            } else {
                if (EINTR == errno) {
                    hal_err("EINTR be caught");
                    continue;
                }
                hal_err("recv fail");
                err_code = -2;
                break;
            }
        } else if (0 == ret) {
            break;
        } else {
            hal_err("select-recv fail");
            err_code = -2;
            break;
        }
    } while ((len_recv < len));
 
    /* priority to return data bytes if any data be received from TCP connection. */
    /* It will get error code on next calling */
    return (0 != len_recv) ? len_recv : err_code;
}
 
 
/**
 * @brief Write data into the specific TCP connection.
 *        The API will return immediately if 'len' be written into the specific TCP connection.
 *
 * @param [in] fd @n A descriptor identifying a connection.
 * @param [in] buf @n A pointer to a buffer containing the data to be transmitted.
 * @param [in] len @n The length, in bytes, of the data pointed to by the 'buf' parameter.
 * @param [in] timeout_ms @n Specify the timeout value in millisecond. In other words, the API block 'timeout_ms' millisecond maximumly.
 *
 * @retval      < 0 : TCP connection error occur..
 * @retval        0 : No any data be write into the TCP connection in 'timeout_ms' timeout period.
 * @retval (0, len] : The total number of bytes be written in 'timeout_ms' timeout period.
 
 * @see None.
 */
    int32_t HAL_TCP_Write(uintptr_t fd, const char *buf, uint32_t len, uint32_t timeout_ms)
{
    int ret;
    uint32_t len_sent;
    uint64_t t_end, t_left;
    fd_set sets;
 
    t_end = _linux_get_time_ms() + timeout_ms;
    len_sent = 0;
    ret = 1; /* send one time if timeout_ms is value 0 */
 
    do {
        t_left = _linux_time_left(t_end, _linux_get_time_ms());
 
        if (0 != t_left) {
            struct timeval timeout;
 
            FD_ZERO(&sets);
            FD_SET(fd, &sets);
 
            timeout.tv_sec = t_left / 1000;
            timeout.tv_usec = (t_left % 1000) * 1000;
 
            ret = select(fd + 1, NULL, &sets, NULL, &timeout);
            if (ret > 0) {
                if (0 == FD_ISSET(fd, &sets)) {
                    hal_err("Should NOT arrive");
                    /* If timeout in next loop, it will not sent any data */
                    ret = 0;
                    continue;
                }
            } else if (0 == ret) {
                hal_err("select-write timeout %d", (int)fd);
                break;
            } else {
                if (EINTR == errno) {
                    hal_err("EINTR be caught");
                    continue;
                }
 
                hal_err("select-write fail");
                break;
            }
        }
 
        if (ret > 0) {
            ret = send(fd, buf + len_sent, len - len_sent, 0);
            if (ret > 0) {
                len_sent += ret;
            } else if (0 == ret) {
                hal_err("No data be sent");
            } else {
                if (EINTR == errno) {
                    hal_err("EINTR be caught");
                    continue;
                }
 
                hal_err("send fail");
                break;
            }
        }
    } while ((len_sent < len) && (_linux_time_left(t_end, _linux_get_time_ms()) > 0));
 
    return len_sent;
}


/**
 * @brief Retrieves the number of milliseconds that have elapsed since the system was boot.
 *
 * @return the number of milliseconds.
 * @see None.
 * @note None.
 */
uint64_t HAL_UptimeMs(void)
{
    uint64_t            time_ms;
    struct timespec     ts;

    clock_gettime(CLOCK_MONOTONIC, &ts);
    time_ms = ((uint64_t)ts.tv_sec * (uint64_t)1000) + (ts.tv_nsec / 1000 / 1000);

    return time_ms;
}


int HAL_Vsnprintf(char *str, const int len, const char *format, va_list ap)
{
	return vsnprintf(str, len, format, ap);
	//return (int)1;
}


