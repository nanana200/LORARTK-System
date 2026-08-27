#include "rtk_bridge.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

#define UM982_DMA_SIZE       256U
#define UM982_RX_RING_SIZE  2048U
#define UM982_TX_RING_SIZE  4096U
#define DEBUG_TX_RING_SIZE  8192U
#define NMEA_LINE_SIZE       384U

typedef struct
{
    uint8_t *data;
    uint16_t size;
    volatile uint16_t head;
    volatile uint16_t tail;
} ByteRing;

static uint8_t um982_dma[UM982_DMA_SIZE];
static uint8_t um982_rx_storage[UM982_RX_RING_SIZE];
static uint8_t um982_tx_storage[UM982_TX_RING_SIZE];
static uint8_t debug_tx_storage[DEBUG_TX_RING_SIZE];

static ByteRing um982_rx_ring = {um982_rx_storage, UM982_RX_RING_SIZE, 0U, 0U};
static ByteRing um982_tx_ring = {um982_tx_storage, UM982_TX_RING_SIZE, 0U, 0U};
static ByteRing debug_tx_ring = {debug_tx_storage, DEBUG_TX_RING_SIZE, 0U, 0U};

static volatile bool um982_tx_busy;
static volatile bool debug_tx_busy;
static volatile bool um982_rx_restart;
static volatile uint16_t um982_tx_active_length;
static volatile uint16_t debug_tx_active_length;

static uint32_t forwarded_frames;
static uint32_t forwarded_bytes;
static uint32_t dropped_frames;
static uint32_t um982_received_bytes;
static uint32_t um982_rx_overflow_bytes;
static uint32_t debug_dropped_messages;

static char nmea_line[NMEA_LINE_SIZE];
static uint16_t nmea_line_length;

static uint16_t Ring_Used(const ByteRing *ring)
{
    if (ring->head >= ring->tail)
    {
        return (uint16_t)(ring->head - ring->tail);
    }
    return (uint16_t)(ring->size - ring->tail + ring->head);
}

static uint16_t Ring_Free(const ByteRing *ring)
{
    return (uint16_t)(ring->size - 1U - Ring_Used(ring));
}

static uint16_t Ring_Write(ByteRing *ring, const uint8_t *data, uint16_t length)
{
    uint16_t written = 0U;

    while ((written < length) && (Ring_Free(ring) != 0U))
    {
        ring->data[ring->head] = data[written++];
        ring->head = (uint16_t)((ring->head + 1U) % ring->size);
    }
    return written;
}

static bool Ring_WriteExact(ByteRing *ring, const uint8_t *data, uint16_t length)
{
    if (Ring_Free(ring) < length)
    {
        return false;
    }
    return Ring_Write(ring, data, length) == length;
}

static bool Ring_ReadByte(ByteRing *ring, uint8_t *value)
{
    if (ring->tail == ring->head)
    {
        return false;
    }

    *value = ring->data[ring->tail];
    ring->tail = (uint16_t)((ring->tail + 1U) % ring->size);
    return true;
}

static uint16_t Ring_ContiguousUsed(const ByteRing *ring)
{
    if (ring->head >= ring->tail)
    {
        return (uint16_t)(ring->head - ring->tail);
    }
    return (uint16_t)(ring->size - ring->tail);
}

static void UM982_StartReceive(void)
{
    if (HAL_UARTEx_ReceiveToIdle_DMA(&huart2, um982_dma, sizeof(um982_dma)) == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
        um982_rx_restart = false;
    }
    else
    {
        um982_rx_restart = true;
    }
}

static void StartUM982Transmit(void)
{
    uint16_t length;

    if (um982_tx_busy)
    {
        return;
    }

    length = Ring_ContiguousUsed(&um982_tx_ring);
    if (length == 0U)
    {
        return;
    }

    um982_tx_active_length = length;
    um982_tx_busy = true;
    if (HAL_UART_Transmit_IT(&huart2, &um982_tx_ring.data[um982_tx_ring.tail], length) != HAL_OK)
    {
        um982_tx_busy = false;
        um982_tx_active_length = 0U;
    }
}

static void StartDebugTransmit(void)
{
    uint16_t length;

    if (debug_tx_busy)
    {
        return;
    }

    length = Ring_ContiguousUsed(&debug_tx_ring);
    if (length == 0U)
    {
        return;
    }

    debug_tx_active_length = length;
    debug_tx_busy = true;
    if (HAL_UART_Transmit_IT(&huart3, &debug_tx_ring.data[debug_tx_ring.tail], length) != HAL_OK)
    {
        debug_tx_busy = false;
        debug_tx_active_length = 0U;
    }
}

void Debug_Log(const char *fmt, ...)
{
    char buffer[384];
    int length;
    va_list args;

    va_start(args, fmt);
    length = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (length <= 0)
    {
        return;
    }
    if ((size_t)length >= sizeof(buffer))
    {
        length = (int)sizeof(buffer) - 1;
    }

    if (!Ring_WriteExact(&debug_tx_ring, (const uint8_t *)buffer, (uint16_t)length))
    {
        debug_dropped_messages++;
    }
    StartDebugTransmit();
}

static const char *GGA_StatusName(uint8_t quality)
{
    switch (quality)
    {
        case 0U: return "INVALID";
        case 1U: return "GNSS FIX";
        case 2U: return "DGNSS/DGPS";
        case 4U: return "RTK FIXED";
        case 5U: return "RTK FLOAT";
        case 6U: return "ESTIMATED";
        default: return "OTHER";
    }
}

static bool ParseUnsigned(const char *text, uint32_t *value)
{
    uint32_t result = 0U;
    bool found = false;

    while ((*text >= '0') && (*text <= '9'))
    {
        found = true;
        result = (result * 10U) + (uint32_t)(*text - '0');
        text++;
    }
    *value = result;
    return found;
}

/* NMEA ddmm.mmmm / dddmm.mmmm 좌표를 부동소수점 없이 1e-8 degree로 변환한다. */
static bool CoordinateToE8(const char *text, char hemisphere, int32_t *result)
{
    const char *dot = strchr(text, '.');
    size_t whole_length = dot ? (size_t)(dot - text) : strlen(text);
    uint32_t degrees = 0U;
    uint32_t minutes_whole = 0U;
    uint32_t minutes_fraction = 0U;
    uint32_t fraction_scale = 1U;
    uint32_t minutes_e6;
    int64_t coordinate;
    size_t i;

    if (whole_length < 3U)
    {
        return false;
    }

    for (i = 0U; i < whole_length - 2U; i++)
    {
        if ((text[i] < '0') || (text[i] > '9')) return false;
        degrees = degrees * 10U + (uint32_t)(text[i] - '0');
    }
    if ((text[whole_length - 2U] < '0') || (text[whole_length - 2U] > '9') ||
        (text[whole_length - 1U] < '0') || (text[whole_length - 1U] > '9'))
    {
        return false;
    }
    minutes_whole = (uint32_t)(text[whole_length - 2U] - '0') * 10U +
                    (uint32_t)(text[whole_length - 1U] - '0');

    if (dot != NULL)
    {
        const char *p = dot + 1;
        while ((*p >= '0') && (*p <= '9') && (fraction_scale < 1000000U))
        {
            minutes_fraction = minutes_fraction * 10U + (uint32_t)(*p - '0');
            fraction_scale *= 10U;
            p++;
        }
    }
    minutes_fraction *= (1000000U / fraction_scale);
    minutes_e6 = minutes_whole * 1000000U + minutes_fraction;
    coordinate = (int64_t)degrees * 100000000LL + ((int64_t)minutes_e6 * 100LL) / 60LL;

    if ((hemisphere == 'S') || (hemisphere == 'W'))
    {
        coordinate = -coordinate;
    }
    *result = (int32_t)coordinate;
    return true;
}

static void FormatE8(int32_t value, char *buffer, size_t size)
{
    uint32_t magnitude = (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
    (void)snprintf(buffer, size, "%s%lu.%08lu",
                   (value < 0) ? "-" : "",
                   (unsigned long)(magnitude / 100000000U),
                   (unsigned long)(magnitude % 100000000U));
}

static void ParseGGALine(char *line)
{
    char *fields[16] = {0};
    uint8_t field_count = 0U;
    char *cursor = line;
    uint32_t quality_value = 0U;
    int32_t latitude_e8 = 0;
    int32_t longitude_e8 = 0;
    char latitude[24] = "N/A";
    char longitude[24] = "N/A";

    while ((field_count < 16U) && (cursor != NULL))
    {
        char *comma;
        fields[field_count++] = cursor;
        comma = strchr(cursor, ',');
        if (comma == NULL)
        {
            break;
        }
        *comma = '\0';
        cursor = comma + 1;
    }

    if (field_count < 10U)
    {
        Debug_Log("[UM982] GGA parse error: too few fields (%u)\r\n", field_count);
        return;
    }

    (void)ParseUnsigned(fields[6], &quality_value);
    if ((fields[2][0] != '\0') && (fields[3][0] != '\0') &&
        CoordinateToE8(fields[2], fields[3][0], &latitude_e8))
    {
        FormatE8(latitude_e8, latitude, sizeof(latitude));
    }
    if ((fields[4][0] != '\0') && (fields[5][0] != '\0') &&
        CoordinateToE8(fields[4], fields[5][0], &longitude_e8))
    {
        FormatE8(longitude_e8, longitude, sizeof(longitude));
    }

    Debug_Log("[GGA] UTC=%s Lat=%s Lon=%s Alt=%s m Quality=%lu (%s) Sats=%s HDOP=%s DiffAge=%s s\r\n",
              fields[1][0] ? fields[1] : "N/A",
              latitude,
              longitude,
              fields[9][0] ? fields[9] : "N/A",
              (unsigned long)quality_value,
              GGA_StatusName((uint8_t)quality_value),
              fields[7][0] ? fields[7] : "N/A",
              fields[8][0] ? fields[8] : "N/A",
              (field_count > 13U && fields[13][0]) ? fields[13] : "N/A");
}

static void HandleNMEALine(char *line)
{
    char raw[NMEA_LINE_SIZE];

    if ((strncmp(line, "$GNGGA,", 7U) != 0) && (strncmp(line, "$GPGGA,", 7U) != 0))
    {
        return;
    }

    (void)snprintf(raw, sizeof(raw), "%s", line);
    Debug_Log("[UM982 RAW] %s\r\n", raw);
    ParseGGALine(line);
}

static void ProcessUM982Input(void)
{
    uint8_t byte;

    while (Ring_ReadByte(&um982_rx_ring, &byte))
    {
        if (byte == '\n')
        {
            if (nmea_line_length != 0U)
            {
                if ((nmea_line_length > 0U) && (nmea_line[nmea_line_length - 1U] == '\r'))
                {
                    nmea_line_length--;
                }
                nmea_line[nmea_line_length] = '\0';
                HandleNMEALine(nmea_line);
                nmea_line_length = 0U;
            }
        }
        else if (nmea_line_length < (NMEA_LINE_SIZE - 1U))
        {
            nmea_line[nmea_line_length++] = (char)byte;
        }
        else
        {
            nmea_line_length = 0U;
        }
    }
}

void RTK_Bridge_Init(void)
{
    um982_tx_busy = false;
    debug_tx_busy = false;
    um982_rx_restart = false;
    nmea_line_length = 0U;
    UM982_StartReceive();
}

void RTK_Bridge_Process(void)
{
    if (um982_rx_restart)
    {
        (void)HAL_UART_AbortReceive(&huart2);
        UM982_StartReceive();
    }

    ProcessUM982Input();
    StartUM982Transmit();
    StartDebugTransmit();
}

bool RTK_Bridge_ForwardRTCM(const uint8_t *frame, uint16_t length)
{
    if (!Ring_WriteExact(&um982_tx_ring, frame, length))
    {
        dropped_frames++;
        return false;
    }

    forwarded_frames++;
    forwarded_bytes += length;
    StartUM982Transmit();
    return true;
}

uint32_t RTK_Bridge_GetForwardedFrames(void) { return forwarded_frames; }
uint32_t RTK_Bridge_GetForwardedBytes(void) { return forwarded_bytes; }
uint32_t RTK_Bridge_GetDroppedFrames(void) { return dropped_frames; }
uint32_t RTK_Bridge_GetUM982Bytes(void) { return um982_received_bytes; }

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART2)
    {
        uint16_t written = Ring_Write(&um982_rx_ring, um982_dma, size);
        um982_received_bytes += written;
        um982_rx_overflow_bytes += (uint32_t)(size - written);
        UM982_StartReceive();
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        um982_tx_ring.tail = (uint16_t)((um982_tx_ring.tail + um982_tx_active_length) % um982_tx_ring.size);
        um982_tx_active_length = 0U;
        um982_tx_busy = false;
        StartUM982Transmit();
    }
    else if (huart->Instance == USART3)
    {
        debug_tx_ring.tail = (uint16_t)((debug_tx_ring.tail + debug_tx_active_length) % debug_tx_ring.size);
        debug_tx_active_length = 0U;
        debug_tx_busy = false;
        StartDebugTransmit();
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        um982_rx_restart = true;
    }
    else if (huart->Instance == USART3)
    {
        debug_tx_busy = false;
        debug_tx_active_length = 0U;
    }
}
