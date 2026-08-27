#include "lorartk.h"

#include "lora.h"
#include "main.h"
#include "rtk_bridge.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

extern volatile uint8_t lora_rx_flag;

#define TTGO_PACKET_SINGLE       0xA1U
#define TTGO_PACKET_FRAGMENT     0xA2U
#define TTGO_SINGLE_HEADER_SIZE  2U
#define TTGO_FRAGMENT_HEADER_SIZE 4U
#define RTCM_MAX_FRAME_SIZE      1029U
#define FRAGMENT_TIMEOUT_MS      1500U

typedef struct
{
    bool active;
    uint8_t sequence;
    uint8_t fragment_count;
    uint8_t next_fragment;
    uint16_t length;
    uint32_t last_fragment_tick;
    uint8_t frame[RTCM_MAX_FRAME_SIZE];
} ReassemblyState;

typedef struct
{
    uint32_t lora_packets;
    uint32_t lora_bytes;
    uint32_t lora_phy_crc_errors;
    uint32_t completed_frames;
    uint32_t completed_bytes;
    uint32_t rtcm_crc_errors;
    uint32_t reassembly_errors;
    uint32_t fragment_timeouts;
    uint32_t sequence_misses;
    uint32_t duplicate_packets;
    uint32_t unsupported_packets;
    uint32_t rtcm_1005;
    uint32_t rtcm_1006;
    uint32_t rtcm_1077;
    uint32_t rtcm_1087;
    uint32_t rtcm_1097;
    uint32_t rtcm_1127;
    uint32_t rtcm_1230;
    uint32_t rtcm_other;
} Statistics;

static ReassemblyState reassembly;
static Statistics total;
static Statistics interval;
static bool sequence_initialized;
static uint8_t expected_sequence;
static uint32_t statistics_tick;
static uint32_t totals_tick;
static uint32_t last_forwarded_frames;
static uint32_t last_forwarded_bytes;
static uint32_t last_uart_drops;

static uint32_t CRC24Q(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0U;
    uint16_t i;
    uint8_t bit;

    for (i = 0U; i < length; i++)
    {
        crc ^= (uint32_t)data[i] << 16;
        for (bit = 0U; bit < 8U; bit++)
        {
            crc <<= 1;
            if ((crc & 0x1000000U) != 0U)
            {
                crc ^= 0x1864CFBU;
            }
        }
    }
    return crc & 0xFFFFFFU;
}

static void ResetReassembly(void)
{
    reassembly.active = false;
    reassembly.length = 0U;
    reassembly.next_fragment = 0U;
    reassembly.fragment_count = 0U;
}

/* 프레임 단위 8-bit sequence. 반환 false면 직전 프레임의 중복으로 판단한다. */
static bool AcceptFrameSequence(uint8_t sequence)
{
    if (!sequence_initialized)
    {
        sequence_initialized = true;
        expected_sequence = sequence;
    }

    Debug_Log("[SEQ] RX=%u Expected=%u\r\n", sequence, expected_sequence);
    if (sequence == expected_sequence)
    {
        expected_sequence = (uint8_t)(expected_sequence + 1U);
        return true;
    }

    if (sequence == (uint8_t)(expected_sequence - 1U))
    {
        total.duplicate_packets++;
        interval.duplicate_packets++;
        Debug_Log("[SEQ] Duplicate frame ignored: %u\r\n", sequence);
        return false;
    }

    total.sequence_misses++;
    interval.sequence_misses++;
    Debug_Log("[SEQ] Miss/out-of-order: RX=%u Expected=%u\r\n", sequence, expected_sequence);
    expected_sequence = (uint8_t)(sequence + 1U);
    return true;
}

static void CountRTCMType(uint16_t type)
{
    uint32_t *total_counter = &total.rtcm_other;
    uint32_t *interval_counter = &interval.rtcm_other;

    switch (type)
    {
        case 1005U: total_counter = &total.rtcm_1005; interval_counter = &interval.rtcm_1005; break;
        case 1006U: total_counter = &total.rtcm_1006; interval_counter = &interval.rtcm_1006; break;
        case 1077U: total_counter = &total.rtcm_1077; interval_counter = &interval.rtcm_1077; break;
        case 1087U: total_counter = &total.rtcm_1087; interval_counter = &interval.rtcm_1087; break;
        case 1097U: total_counter = &total.rtcm_1097; interval_counter = &interval.rtcm_1097; break;
        case 1127U: total_counter = &total.rtcm_1127; interval_counter = &interval.rtcm_1127; break;
        case 1230U: total_counter = &total.rtcm_1230; interval_counter = &interval.rtcm_1230; break;
        default: break;
    }
    (*total_counter)++;
    (*interval_counter)++;
}

static void ValidateAndForwardRTCM(const uint8_t *frame, uint16_t length,
                                   uint8_t sequence, uint8_t fragment_count)
{
    uint16_t payload_length;
    uint16_t expected_length;
    uint16_t message_type = 0U;
    uint32_t received_crc;
    uint32_t calculated_crc;

    total.completed_frames++;
    interval.completed_frames++;
    total.completed_bytes += length;
    interval.completed_bytes += length;

    if (length >= 5U)
    {
        message_type = (uint16_t)(((uint16_t)frame[3] << 4) | (frame[4] >> 4));
    }

    if ((length < 8U) || (frame[0] != 0xD3U) || ((frame[1] & 0xFCU) != 0U))
    {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        Debug_Log("[RTCM] Invalid header/preamble, length=%u\r\n", length);
        lora_rtcm_debug_result(sequence, message_type, length, 0U,
                               fragment_count, 0U, "FORMAT_ERROR");
        return;
    }

    payload_length = (uint16_t)(((uint16_t)(frame[1] & 0x03U) << 8) | frame[2]);
    expected_length = (uint16_t)(payload_length + 6U);
    if (length != expected_length)
    {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        Debug_Log("[RTCM] Length mismatch: received=%u header=%u\r\n", length, expected_length);
        lora_rtcm_debug_result(sequence, message_type, length, 0U,
                               fragment_count, 0U, "LENGTH_ERROR");
        return;
    }

    received_crc = ((uint32_t)frame[length - 3U] << 16) |
                   ((uint32_t)frame[length - 2U] << 8) |
                   frame[length - 1U];
    calculated_crc = CRC24Q(frame, (uint16_t)(length - 3U));
    if (received_crc != calculated_crc)
    {
        total.rtcm_crc_errors++;
        interval.rtcm_crc_errors++;
        Debug_Log("[RTCM] Type=%u Length=%u CRC=ERROR rx=%06lX calc=%06lX\r\n",
                  message_type, length, (unsigned long)received_crc, (unsigned long)calculated_crc);
        lora_rtcm_debug_result(sequence, message_type, length, 0U,
                               fragment_count, 0U, "CRC_ERROR");
        return;
    }

    CountRTCMType(message_type);
    Debug_Log("[RTCM] Type=%u Frame Length=%u CRC=OK\r\n", message_type, length);
    if (RTK_Bridge_ForwardRTCM(frame, length))
    {
        Debug_Log("[UM982 TX] Queued RTCM type=%u, %u bytes\r\n", message_type, length);
        lora_rtcm_debug_result(sequence, message_type, length, 1U,
                               fragment_count, 1U, "OK");
    }
    else
    {
        Debug_Log("[UM982 TX] DROP: UART queue full, type=%u, %u bytes\r\n", message_type, length);
        lora_rtcm_debug_result(sequence, message_type, length, 1U,
                               fragment_count, 0U, "UART_DROP");
    }
}

static void ProcessSinglePacket(const uint8_t *packet, uint8_t length)
{
    uint8_t sequence;

    if (length <= TTGO_SINGLE_HEADER_SIZE)
    {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        return;
    }
    sequence = packet[1];
    if (!AcceptFrameSequence(sequence))
    {
        return;
    }

    if (reassembly.active)
    {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        Debug_Log("[REASM] Incomplete sequence %u discarded by single frame %u\r\n",
                  reassembly.sequence, sequence);
        ResetReassembly();
    }

    Debug_Log("[LORA] Single sequence=%u payload=%u\r\n", sequence,
              (uint16_t)(length - TTGO_SINGLE_HEADER_SIZE));
    ValidateAndForwardRTCM(&packet[TTGO_SINGLE_HEADER_SIZE],
                           (uint16_t)(length - TTGO_SINGLE_HEADER_SIZE),
                           sequence, 1U);
}

static void ProcessFragmentPacket(const uint8_t *packet, uint8_t length)
{
    uint8_t sequence;
    uint8_t index;
    uint8_t count;
    uint16_t data_length;

    if (length <= TTGO_FRAGMENT_HEADER_SIZE)
    {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        return;
    }

    sequence = packet[1];
    index = packet[2];
    count = packet[3];
    data_length = (uint16_t)(length - TTGO_FRAGMENT_HEADER_SIZE);
    Debug_Log("[FRAG] Sequence=%u Index=%u/%u Data=%u\r\n", sequence, index, count, data_length);

    if ((count < 2U) || (index >= count))
    {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        Debug_Log("[FRAG] Invalid header\r\n");
        return;
    }

    if (index == 0U)
    {
        if (reassembly.active)
        {
            if ((reassembly.sequence == sequence) && (reassembly.next_fragment > 0U))
            {
                total.duplicate_packets++;
                interval.duplicate_packets++;
                Debug_Log("[FRAG] Duplicate first fragment ignored\r\n");
                return;
            }
            total.reassembly_errors++;
            interval.reassembly_errors++;
            Debug_Log("[REASM] Previous sequence %u discarded\r\n", reassembly.sequence);
            ResetReassembly();
        }

        if (!AcceptFrameSequence(sequence))
        {
            return;
        }

        reassembly.active = true;
        reassembly.sequence = sequence;
        reassembly.fragment_count = count;
        reassembly.next_fragment = 0U;
        reassembly.length = 0U;
    }
    else if (!reassembly.active || (reassembly.sequence != sequence))
    {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        Debug_Log("[FRAG] No matching active frame for sequence=%u index=%u\r\n", sequence, index);
        return;
    }

    if (count != reassembly.fragment_count)
    {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        Debug_Log("[FRAG] Fragment-count changed (%u -> %u)\r\n", reassembly.fragment_count, count);
        ResetReassembly();
        return;
    }
    if (index < reassembly.next_fragment)
    {
        total.duplicate_packets++;
        interval.duplicate_packets++;
        Debug_Log("[FRAG] Duplicate fragment ignored: %u\r\n", index);
        return;
    }
    if (index != reassembly.next_fragment)
    {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        Debug_Log("[FRAG] Missing/out-of-order fragment: RX=%u Expected=%u\r\n",
                  index, reassembly.next_fragment);
        ResetReassembly();
        return;
    }
    if ((uint32_t)reassembly.length + data_length > RTCM_MAX_FRAME_SIZE)
    {
        total.reassembly_errors++;
        interval.reassembly_errors++;
        Debug_Log("[FRAG] Reassembly buffer overflow\r\n");
        ResetReassembly();
        return;
    }

    memcpy(&reassembly.frame[reassembly.length], &packet[TTGO_FRAGMENT_HEADER_SIZE], data_length);
    reassembly.length = (uint16_t)(reassembly.length + data_length);
    reassembly.next_fragment++;
    reassembly.last_fragment_tick = HAL_GetTick();

    if (reassembly.next_fragment == reassembly.fragment_count)
    {
        Debug_Log("[REASM] Complete sequence=%u fragments=%u length=%u\r\n",
                  reassembly.sequence, reassembly.fragment_count, reassembly.length);
        ValidateAndForwardRTCM(reassembly.frame, reassembly.length,
                               reassembly.sequence, reassembly.fragment_count);
        ResetReassembly();
    }
}

static void ProcessLoRaPacket(void)
{
    uint8_t packet[255];
    uint8_t length = 0U;
    uint8_t irq = 0U;
    int result;

    result = lora_receive_packet(packet, &length, &irq);

    /* FIFO를 읽은 직후 다음 패킷을 놓치지 않도록 먼저 RX Continuous로 복귀한다. */
    rx_set();

    if (result < 0)
    {
        rssi_graph_packet(0U, NULL, 0U);
        total.lora_phy_crc_errors++;
        interval.lora_phy_crc_errors++;
        Debug_Log("[LORA] Payload CRC error, IRQ=0x%02X\r\n", irq);
        return;
    }
    if (result == 0)
    {
        Debug_Log("[LORA] DIO0 without RxDone, IRQ=0x%02X\r\n", irq);
        return;
    }

    rssi_graph_packet(1U, packet, length);

    total.lora_packets++;
    interval.lora_packets++;
    total.lora_bytes += length;
    interval.lora_bytes += length;
    Debug_Log("[LORA] RX packet length=%u RSSI=%d dBm SNRx100=%d\r\n",
              length, lora_packet_rssi_dbm(), lora_packet_snr_x100());

    if (length == 0U)
    {
        total.unsupported_packets++;
        interval.unsupported_packets++;
        Debug_Log("[LORA] Empty packet ignored\r\n");
    }
    else if (packet[0] == TTGO_PACKET_SINGLE)
    {
        ProcessSinglePacket(packet, length);
    }
    else if (packet[0] == TTGO_PACKET_FRAGMENT)
    {
        ProcessFragmentPacket(packet, length);
    }
    else
    {
        total.unsupported_packets++;
        interval.unsupported_packets++;
        Debug_Log("[LORA] Unsupported packet type=0x%02X\r\n", packet[0]);
    }
}

static void PrintOneSecondStatistics(void)
{
    uint32_t forwarded_frames = RTK_Bridge_GetForwardedFrames();
    uint32_t forwarded_bytes = RTK_Bridge_GetForwardedBytes();
    uint32_t uart_drops = RTK_Bridge_GetDroppedFrames();

    Debug_Log("[1s] LoRa=%lu pkt/%lu B RTCM=%lu frame/%lu B FWD=%lu/%lu B CRCerr=%lu PHYerr=%lu ReasmErr=%lu SeqMiss=%lu Dup=%lu UARTdrop=%lu\r\n",
              (unsigned long)interval.lora_packets,
              (unsigned long)interval.lora_bytes,
              (unsigned long)interval.completed_frames,
              (unsigned long)interval.completed_bytes,
              (unsigned long)(forwarded_frames - last_forwarded_frames),
              (unsigned long)(forwarded_bytes - last_forwarded_bytes),
              (unsigned long)interval.rtcm_crc_errors,
              (unsigned long)interval.lora_phy_crc_errors,
              (unsigned long)interval.reassembly_errors,
              (unsigned long)interval.sequence_misses,
              (unsigned long)interval.duplicate_packets,
              (unsigned long)(uart_drops - last_uart_drops));
    last_forwarded_frames = forwarded_frames;
    last_forwarded_bytes = forwarded_bytes;
    last_uart_drops = uart_drops;
    memset(&interval, 0, sizeof(interval));
}

static void PrintTenSecondTotals(void)
{
    Debug_Log("[TOTAL] LoRa=%lu pkt/%lu B RTCM=%lu frame/%lu B Forwarded=%lu frame/%lu B UM982_RX=%lu B CRCerr=%lu Timeout=%lu SeqMiss=%lu Unsupported=%lu\r\n",
              (unsigned long)total.lora_packets,
              (unsigned long)total.lora_bytes,
              (unsigned long)total.completed_frames,
              (unsigned long)total.completed_bytes,
              (unsigned long)RTK_Bridge_GetForwardedFrames(),
              (unsigned long)RTK_Bridge_GetForwardedBytes(),
              (unsigned long)RTK_Bridge_GetUM982Bytes(),
              (unsigned long)total.rtcm_crc_errors,
              (unsigned long)total.fragment_timeouts,
              (unsigned long)total.sequence_misses,
              (unsigned long)total.unsupported_packets);
    Debug_Log("[TYPE] 1005=%lu 1006=%lu 1077=%lu 1087=%lu 1097=%lu 1127=%lu 1230=%lu other=%lu\r\n",
              (unsigned long)total.rtcm_1005, (unsigned long)total.rtcm_1006,
              (unsigned long)total.rtcm_1077, (unsigned long)total.rtcm_1087,
              (unsigned long)total.rtcm_1097, (unsigned long)total.rtcm_1127,
              (unsigned long)total.rtcm_1230, (unsigned long)total.rtcm_other);
}

void LORARTK_Init(void)
{
    memset(&reassembly, 0, sizeof(reassembly));
    memset(&total, 0, sizeof(total));
    memset(&interval, 0, sizeof(interval));
    sequence_initialized = false;
    last_forwarded_frames = 0U;
    last_forwarded_bytes = 0U;
    last_uart_drops = 0U;

    RTK_Bridge_Init();
    HAL_Delay(20U);
    lora_setup();
    lora_freq();
    packet_set();
    fifo_set();
    rx_set();

    statistics_tick = HAL_GetTick();
    totals_tick = statistics_tick;
    Debug_Log("\r\n=== LORARTK STM32 Drone Receiver ===\r\n");
    Debug_Log("[RADIO] SX1276 version=0x%02X, 922.1 MHz, SF7, BW125, CR4/5, Explicit, CRC ON\r\n",
              lora_read(0x42));
    Debug_Log("[RADIO] DIO0=RxDone, RX Continuous, FIFO RX base=0x00\r\n");
    Debug_Log("[PACKET] A1=[type,seq,RTCM], A2=[type,seq,index,count,data]\r\n");
    Debug_Log("[UART] UM982 USART2 PD5/PD6 115200 8N1, Debug USART3 PD8/PD9 115200 8N1\r\n");
}

void LORARTK_Process(void)
{
    uint32_t now;

    RTK_Bridge_Process();

    if (lora_rx_flag != 0U)
    {
        lora_rx_flag = 0U;
        ProcessLoRaPacket();
    }

    uart_send_noise_rssi();

    now = HAL_GetTick();
    if (reassembly.active && ((uint32_t)(now - reassembly.last_fragment_tick) >= FRAGMENT_TIMEOUT_MS))
    {
        total.fragment_timeouts++;
        interval.fragment_timeouts++;
        total.reassembly_errors++;
        interval.reassembly_errors++;
        Debug_Log("[REASM] Timeout sequence=%u received=%u/%u length=%u\r\n",
                  reassembly.sequence, reassembly.next_fragment,
                  reassembly.fragment_count, reassembly.length);
        lora_rtcm_debug_result(reassembly.sequence, 0U, reassembly.length, 0U,
                               reassembly.fragment_count, 0U, "TIMEOUT");
        ResetReassembly();
    }

    if ((uint32_t)(now - statistics_tick) >= 1000U)
    {
        statistics_tick += 1000U;
        PrintOneSecondStatistics();
    }
    if ((uint32_t)(now - totals_tick) >= 10000U)
    {
        totals_tick += 10000U;
        PrintTenSecondTotals();
    }

    RTK_Bridge_Process();
}
