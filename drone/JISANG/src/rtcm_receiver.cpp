#include "rtcm_receiver.h"

#include <math.h>
#include <string.h>

#include "config.h"
#include "lora_radio.h"

namespace {
constexpr uint8_t TTGO_PACKET_SINGLE = 0xA1;
constexpr uint8_t TTGO_PACKET_FRAGMENT = 0xA2;
constexpr uint8_t TTGO_PACKET_DOWNLINK_END = 0xA3;
constexpr uint8_t GROUND_STATION_ID = 0xFF;
constexpr uint8_t WAYPOINT_MESSAGE = 0xF2;
constexpr uint8_t MAX_WAYPOINTS = 15;
constexpr uint8_t TTGO_SINGLE_HEADER_SIZE = 2;
constexpr uint8_t TTGO_FRAGMENT_HEADER_SIZE = 4;
constexpr uint8_t DRONE_PACKET_ID = 0xFE;
constexpr uint8_t DRONE_GPS_MESSAGE = 0xF3;
constexpr uint8_t DRONE_GPS_POSITION_COUNT = 1;
constexpr uint8_t DRONE_GPS_PACKET_SIZE = 14;
constexpr uint32_t GPS_RESPONSE_GUARD_MS = 30;
constexpr uint16_t RTCM_MAX_FRAME_SIZE = 1029;
constexpr uint16_t NMEA_LINE_SIZE = 384;

struct ReassemblyState {
  bool active = false;
  uint8_t sequence = 0;
  uint8_t fragment_count = 0;
  uint8_t next_fragment = 0;
  uint16_t length = 0;
  uint32_t started_ms = 0;
  uint32_t last_fragment_ms = 0;
  uint8_t frame[RTCM_MAX_FRAME_SIZE]{};
};

struct Statistics {
  uint32_t lora_packets = 0;
  uint32_t lora_bytes = 0;
  uint32_t lora_phy_crc_errors = 0;
  uint32_t completed_frames = 0;
  uint32_t completed_bytes = 0;
  uint32_t rtcm_crc_errors = 0;
  uint32_t reassembly_errors = 0;
  uint32_t fragment_timeouts = 0;
  uint32_t sequence_misses = 0;
  uint32_t duplicate_packets = 0;
  uint32_t unsupported_packets = 0;
  uint32_t ground_filter_drops = 0;
};

struct LatestGga {
  bool valid = false;
  double latitude = NAN;
  double longitude = NAN;
  uint8_t quality = 0;
  float differential_age = NAN;
  uint32_t received_ms = 0;
};

struct LatestWaypointCommand {
  bool valid = false;
  uint8_t count = 0;
  int32_t latitude[MAX_WAYPOINTS]{};
  int32_t longitude[MAX_WAYPOINTS]{};
  uint32_t received_ms = 0;
};

HardwareSerial *um982 = nullptr;
volatile bool radio_irq_pending = false;
ReassemblyState reassembly;
Statistics total;
Statistics interval_stats;
bool sequence_initialized = false;
uint8_t expected_sequence = 0;
uint32_t statistics_ms = 0;
uint32_t totals_ms = 0;
uint32_t forwarded_frames = 0;
uint32_t forwarded_bytes = 0;
uint32_t uart_drops = 0;
uint32_t um982_received_bytes = 0;
uint32_t last_um982_received_bytes = 0;
uint32_t um982_lines = 0;
uint32_t gga_lines = 0;
uint32_t last_um982_lines = 0;
uint32_t last_gga_lines = 0;
uint32_t last_gga_ms = 0;
uint32_t nmea_overflows = 0;
uint32_t last_forwarded_frames = 0;
uint32_t last_forwarded_bytes = 0;
uint32_t last_uart_drops = 0;
char nmea_line[NMEA_LINE_SIZE]{};
uint16_t nmea_length = 0;
LatestGga latest_gga;
LatestWaypointCommand latest_waypoints;
uint8_t gps_sequence = 0;
uint8_t latest_detected_flag = 0;
uint8_t latest_person_count = 0;

void WriteInt32LE(uint8_t *destination, int32_t value) {
  const uint32_t raw = static_cast<uint32_t>(value);
  destination[0] = static_cast<uint8_t>(raw & 0xFFU);
  destination[1] = static_cast<uint8_t>((raw >> 8) & 0xFFU);
  destination[2] = static_cast<uint8_t>((raw >> 16) & 0xFFU);
  destination[3] = static_cast<uint8_t>((raw >> 24) & 0xFFU);
}

int32_t ReadInt32LE(const uint8_t *source) {
  const uint32_t raw = static_cast<uint32_t>(source[0]) |
                       (static_cast<uint32_t>(source[1]) << 8) |
                       (static_cast<uint32_t>(source[2]) << 16) |
                       (static_cast<uint32_t>(source[3]) << 24);
  if (raw >= 0x80000000U) {
    return static_cast<int32_t>(static_cast<int64_t>(raw) - 0x100000000LL);
  }
  return static_cast<int32_t>(raw);
}

uint32_t CRC24Q(const uint8_t *data, uint16_t length) {
  uint32_t crc = 0;
  for (uint16_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint32_t>(data[i]) << 16;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc <<= 1;
      if ((crc & 0x1000000U) != 0U) {
        crc ^= 0x1864CFBU;
      }
    }
  }
  return crc & 0xFFFFFFU;
}

void ResetReassembly() {
  reassembly.active = false;
  reassembly.length = 0;
  reassembly.next_fragment = 0;
  reassembly.fragment_count = 0;
  reassembly.started_ms = 0;
  reassembly.last_fragment_ms = 0;
}

bool ReassemblyTimedOut(uint32_t now) {
  return reassembly.active &&
         static_cast<uint32_t>(now - reassembly.last_fragment_ms) >=
             FRAGMENT_TIMEOUT_MS;
}

void DiscardTimedOutReassembly(uint32_t now) {
  ++total.fragment_timeouts;
  ++total.reassembly_errors;
  ++interval_stats.reassembly_errors;
  Serial.printf(
      "[REASM] Timeout sequence=%u received=%u/%u length=%u elapsed=%lu ms\r\n",
      reassembly.sequence, reassembly.next_fragment,
      reassembly.fragment_count, reassembly.length,
      static_cast<unsigned long>(now - reassembly.started_ms));
  lora_rtcm_debug_result(reassembly.sequence, 0, reassembly.length, 0,
                         reassembly.fragment_count, 0, "TIMEOUT");
  ResetReassembly();
}

bool AcceptFrameSequence(uint8_t sequence) {
  if (!sequence_initialized) {
    sequence_initialized = true;
    expected_sequence = sequence;
  }

  Serial.printf("[SEQ] RX=%u Expected=%u\r\n", sequence, expected_sequence);
  if (sequence == expected_sequence) {
    expected_sequence = static_cast<uint8_t>(expected_sequence + 1U);
    return true;
  }

  if (sequence == static_cast<uint8_t>(expected_sequence - 1U)) {
    ++total.duplicate_packets;
    ++interval_stats.duplicate_packets;
    Serial.printf("[SEQ] Duplicate frame ignored: %u\r\n", sequence);
    return false;
  }

  ++total.sequence_misses;
  ++interval_stats.sequence_misses;
  Serial.printf("[SEQ] Miss/out-of-order: RX=%u Expected=%u\r\n", sequence,
                expected_sequence);
  expected_sequence = static_cast<uint8_t>(sequence + 1U);
  return true;
}

bool ForwardRTCMToUM982(const uint8_t *frame, uint16_t length) {
  if (um982 == nullptr) {
    ++uart_drops;
    return false;
  }

  // TX buffer를 4096 bytes로 설정했으므로 정상 상태에서는 한 RTCM frame이
  // 통째로 들어간다. 반환 길이가 다르면 성공으로 보고하지 않는다.
  const size_t written = um982->write(frame, length);
  if (written != length) {
    ++uart_drops;
    return false;
  }
  ++forwarded_frames;
  forwarded_bytes += length;
  return true;
}

void ValidateAndForwardRTCM(const uint8_t *frame, uint16_t length,
                            uint8_t sequence, uint8_t fragment_count) {
  ++total.completed_frames;
  ++interval_stats.completed_frames;
  total.completed_bytes += length;
  interval_stats.completed_bytes += length;

  uint16_t message_type = 0;
  if (length >= 5) {
    message_type = static_cast<uint16_t>(
        (static_cast<uint16_t>(frame[3]) << 4) | (frame[4] >> 4));
  }

  if (length < 8 || frame[0] != 0xD3 || (frame[1] & 0xFCU) != 0U) {
    ++total.reassembly_errors;
    ++interval_stats.reassembly_errors;
    Serial.printf("[RTCM] Invalid header/preamble, length=%u\r\n", length);
    lora_rtcm_debug_result(sequence, message_type, length, 0, fragment_count,
                           0, "FORMAT_ERROR");
    return;
  }

  const uint16_t payload_length = static_cast<uint16_t>(
      (static_cast<uint16_t>(frame[1] & 0x03U) << 8) | frame[2]);
  const uint16_t expected_length = static_cast<uint16_t>(payload_length + 6U);
  if (length != expected_length) {
    ++total.reassembly_errors;
    ++interval_stats.reassembly_errors;
    Serial.printf("[RTCM] Length mismatch: received=%u header=%u\r\n", length,
                  expected_length);
    lora_rtcm_debug_result(sequence, message_type, length, 0, fragment_count,
                           0, "LENGTH_ERROR");
    return;
  }

  const uint32_t received_crc =
      (static_cast<uint32_t>(frame[length - 3]) << 16) |
      (static_cast<uint32_t>(frame[length - 2]) << 8) | frame[length - 1];
  const uint32_t calculated_crc = CRC24Q(frame, length - 3);
  if (received_crc != calculated_crc) {
    ++total.rtcm_crc_errors;
    ++interval_stats.rtcm_crc_errors;
    Serial.printf("[RTCM] Type=%u Length=%u CRC=ERROR rx=%06lX calc=%06lX\r\n",
                  message_type, length,
                  static_cast<unsigned long>(received_crc),
                  static_cast<unsigned long>(calculated_crc));
    lora_rtcm_debug_result(sequence, message_type, length, 0, fragment_count,
                           0, "CRC_ERROR");
    return;
  }

  Serial.printf("[RTCM] Type=%u Frame Length=%u CRC=OK\r\n", message_type,
                length);
  if (ForwardRTCMToUM982(frame, length)) {
    Serial.printf("[UM982 TX] Queued RTCM type=%u, %u bytes\r\n", message_type,
                  length);
    lora_rtcm_debug_result(sequence, message_type, length, 1, fragment_count,
                           1, "OK");
  } else {
    Serial.printf("[UM982 TX] DROP/PARTIAL: type=%u, %u bytes\r\n",
                  message_type, length);
    lora_rtcm_debug_result(sequence, message_type, length, 1, fragment_count,
                           0, "UART_DROP");
  }
}

void ProcessSinglePacket(const uint8_t *packet, uint8_t length) {
  if (length <= TTGO_SINGLE_HEADER_SIZE) {
    ++total.reassembly_errors;
    ++interval_stats.reassembly_errors;
    return;
  }

  const uint8_t sequence = packet[1];
  if (!AcceptFrameSequence(sequence)) {
    return;
  }
  if (reassembly.active) {
    ++total.reassembly_errors;
    ++interval_stats.reassembly_errors;
    Serial.printf("[REASM] Incomplete sequence %u discarded by single %u\r\n",
                  reassembly.sequence, sequence);
    ResetReassembly();
  }

  Serial.printf("[LORA] Single sequence=%u payload=%u\r\n", sequence,
                length - TTGO_SINGLE_HEADER_SIZE);
  ValidateAndForwardRTCM(packet + TTGO_SINGLE_HEADER_SIZE,
                         length - TTGO_SINGLE_HEADER_SIZE, sequence, 1);
}

void ProcessFragmentPacket(const uint8_t *packet, uint8_t length) {
  if (length <= TTGO_FRAGMENT_HEADER_SIZE) {
    ++total.reassembly_errors;
    ++interval_stats.reassembly_errors;
    return;
  }

  const uint8_t sequence = packet[1];
  const uint8_t index = packet[2];
  const uint8_t count = packet[3];
  const uint16_t data_length = length - TTGO_FRAGMENT_HEADER_SIZE;
  Serial.printf("[FRAG] Sequence=%u Index=%u/%u Data=%u\r\n", sequence,
                index, count, data_length);

  if (count < 2 || index >= count) {
    ++total.reassembly_errors;
    ++interval_stats.reassembly_errors;
    Serial.print("[FRAG] Invalid header\r\n");
    return;
  }

  if (index == 0) {
    if (reassembly.active) {
      if (reassembly.sequence == sequence && reassembly.next_fragment > 0) {
        ++total.duplicate_packets;
        ++interval_stats.duplicate_packets;
        Serial.print("[FRAG] Duplicate first fragment ignored\r\n");
        return;
      }

      // 아직 timeout 전인 이전 sequence를 새 sequence가 왔다는 이유만으로
      // 폐기하지 않는다. 단일 reassembly buffer이므로 새 첫 fragment를 거절하고
      // 기존 sequence의 다음 fragment를 계속 기다린다.
      const uint32_t current_ms = millis();
      const uint32_t wait_ms =
          static_cast<uint32_t>(current_ms - reassembly.last_fragment_ms);
      if (!ReassemblyTimedOut(current_ms)) {
        ++total.reassembly_errors;
        ++interval_stats.reassembly_errors;
        Serial.printf(
            "[REASM] Busy sequence=%u (%u/%u, wait=%lu ms); "
            "new sequence=%u first fragment ignored\r\n",
            reassembly.sequence, reassembly.next_fragment,
            reassembly.fragment_count, static_cast<unsigned long>(wait_ms),
            sequence);
        return;
      }

      DiscardTimedOutReassembly(current_ms);
    }
    if (!AcceptFrameSequence(sequence)) {
      return;
    }
    reassembly.active = true;
    reassembly.sequence = sequence;
    reassembly.fragment_count = count;
    reassembly.next_fragment = 0;
    reassembly.length = 0;
    reassembly.started_ms = millis();
    reassembly.last_fragment_ms = reassembly.started_ms;
    Serial.printf("[REASM] Start sequence=%u fragments=%u timeout=%lu ms\r\n",
                  sequence, count,
                  static_cast<unsigned long>(FRAGMENT_TIMEOUT_MS));
  } else if (!reassembly.active || reassembly.sequence != sequence) {
    ++total.reassembly_errors;
    ++interval_stats.reassembly_errors;
    Serial.printf("[FRAG] No active frame for seq=%u index=%u\r\n", sequence,
                  index);
    return;
  }

  if (count != reassembly.fragment_count) {
    ++total.reassembly_errors;
    ++interval_stats.reassembly_errors;
    Serial.printf("[FRAG] Fragment-count changed (%u -> %u)\r\n",
                  reassembly.fragment_count, count);
    ResetReassembly();
    return;
  }
  if (index < reassembly.next_fragment) {
    ++total.duplicate_packets;
    ++interval_stats.duplicate_packets;
    Serial.printf("[FRAG] Duplicate fragment ignored: %u\r\n", index);
    return;
  }
  if (index != reassembly.next_fragment) {
    ++total.reassembly_errors;
    ++interval_stats.reassembly_errors;
    Serial.printf("[FRAG] Missing/out-of-order: RX=%u Expected=%u\r\n", index,
                  reassembly.next_fragment);
    ResetReassembly();
    return;
  }
  if (static_cast<uint32_t>(reassembly.length) + data_length >
      RTCM_MAX_FRAME_SIZE) {
    ++total.reassembly_errors;
    ++interval_stats.reassembly_errors;
    Serial.print("[FRAG] Reassembly buffer overflow\r\n");
    ResetReassembly();
    return;
  }

  memcpy(reassembly.frame + reassembly.length,
         packet + TTGO_FRAGMENT_HEADER_SIZE, data_length);
  reassembly.length += data_length;
  ++reassembly.next_fragment;
  reassembly.last_fragment_ms = millis();

  if (reassembly.next_fragment == reassembly.fragment_count) {
    Serial.printf("[REASM] Complete sequence=%u fragments=%u length=%u\r\n",
                  reassembly.sequence, reassembly.fragment_count,
                  reassembly.length);
    ValidateAndForwardRTCM(reassembly.frame, reassembly.length,
                           reassembly.sequence, reassembly.fragment_count);
    ResetReassembly();
  }
}

void SendLatestGgaToGround() {
  Serial.print("[DOWNLINK_END] received\r\n");

  if (!latest_gga.valid) {
    Serial.print("[GPS TX] skipped: no valid GGA stored\r\n");
    rx_set();
    Serial.print("[GPS TX] skipped -> RX\r\n");
    return;
  }

  const int32_t latitude_i =
      static_cast<int32_t>(llround(latest_gga.latitude * 1.0e7));
  const int32_t longitude_i =
      static_cast<int32_t>(llround(latest_gga.longitude * 1.0e7));
  uint8_t packet[DRONE_GPS_PACKET_SIZE]{};
  packet[0] = DRONE_PACKET_ID;
  packet[1] = gps_sequence++;
  packet[2] = DRONE_GPS_MESSAGE;
  packet[3] = DRONE_GPS_POSITION_COUNT;
  WriteInt32LE(packet + 4, latitude_i);
  WriteInt32LE(packet + 8, longitude_i);
  packet[12] = latest_detected_flag;
  packet[13] = latest_person_count;

  const uint32_t sample_age_ms =
      static_cast<uint32_t>(millis() - latest_gga.received_ms);
  if (isfinite(latest_gga.differential_age)) {
    Serial.printf(
        "[GPS TX] lat=%.8f, lon=%.8f, quality=%u, age=%.2f s, "
        "sample_age=%lu ms, det=%u, count=%u\r\n",
        latest_gga.latitude, latest_gga.longitude, latest_gga.quality,
        latest_gga.differential_age,
        static_cast<unsigned long>(sample_age_ms), latest_detected_flag,
        latest_person_count);
  } else {
    Serial.printf(
        "[GPS TX] lat=%.8f, lon=%.8f, quality=%u, age=nan, "
        "sample_age=%lu ms, det=%u, count=%u\r\n",
        latest_gga.latitude, latest_gga.longitude, latest_gga.quality,
        static_cast<unsigned long>(sample_age_ms), latest_detected_flag,
        latest_person_count);
  }

  // 지상 TTGO가 A3 TxDone 뒤 RX Continuous로 전환할 시간을 보장한다.
  delay(GPS_RESPONSE_GUARD_MS);
  radio_irq_pending = false;
  const bool sent = lora_send_packet(packet, sizeof(packet));
  radio_irq_pending = false;  // 자체 TxDone DIO0 edge를 다음 RxDone으로 오인하지 않는다.
  rx_set();
  Serial.printf("[GPS TX] %s -> RX\r\n", sent ? "done" : "failed");
}

void ProcessWaypointPacket(const uint8_t *packet, uint8_t length) {
  const uint8_t count = packet[2];
  const uint16_t expected_length = static_cast<uint16_t>(3U + count * 8U);
  if (count == 0 || count > MAX_WAYPOINTS || length != expected_length) {
    Serial.printf(
        "[WAYPOINT RX] rejected: count=%u length=%u expected=%u\r\n",
        count, length, expected_length);
    return;
  }

  LatestWaypointCommand parsed;
  parsed.valid = true;
  parsed.count = count;
  parsed.received_ms = millis();
  for (uint8_t index = 0; index < count; ++index) {
    const uint16_t offset = static_cast<uint16_t>(3U + index * 8U);
    parsed.latitude[index] = ReadInt32LE(packet + offset);
    parsed.longitude[index] = ReadInt32LE(packet + offset + 4U);
  }
  latest_waypoints = parsed;

  Serial.printf("[WAYPOINT RX] parsed/stored count=%u length=%u\r\n",
                count, length);
  // MATLAB 검증 형식: W,tick,count,lat1,lon1,lat2,lon2,...
  Serial.printf("W,%lu,%u", static_cast<unsigned long>(parsed.received_ms),
                count);
  for (uint8_t index = 0; index < count; ++index) {
    Serial.printf(",%.7f,%.7f", parsed.latitude[index] / 1.0e7,
                  parsed.longitude[index] / 1.0e7);
  }
  Serial.print("\r\n");
}

// 현재 지상 RTKTTGO의 실제 무선 packet에는 lora_drone의 ID_GROUND(0xFF)가
// 실리지 않는다. 따라서 존재하지 않는 ID byte를 검사해 호환성을 깨지 않고,
// 지상국 전용 A1/A2/A3 및 FF/F2 Waypoint 형식과 fragment 문맥을 검사한다.
// 완성 뒤에는 CRC24Q까지 다시 검사하므로 다른 용도의 LoRa packet은 UM982로
// 절대 전달되지 않는다.
bool IsOurGroundStationPacket(const uint8_t *packet, uint8_t length) {
  if (packet == nullptr || length == 0) {
    return false;
  }

  if (packet[0] == TTGO_PACKET_SINGLE) {
    return length >= 5 && packet[2] == 0xD3 && (packet[3] & 0xFCU) == 0U;
  }

  if (packet[0] == TTGO_PACKET_DOWNLINK_END) {
    return length == 1;
  }

  if (packet[0] == GROUND_STATION_ID) {
    if (length < 3 || packet[1] != WAYPOINT_MESSAGE) {
      return false;
    }
    const uint8_t count = packet[2];
    return count >= 1 && count <= MAX_WAYPOINTS &&
           length == static_cast<uint16_t>(3U + count * 8U);
  }

  if (packet[0] != TTGO_PACKET_FRAGMENT ||
      length <= TTGO_FRAGMENT_HEADER_SIZE) {
    return false;
  }

  const uint8_t sequence = packet[1];
  const uint8_t index = packet[2];
  const uint8_t count = packet[3];
  if (count < 2 || index >= count) {
    return false;
  }

  if (index == 0) {
    return length >= 7 && packet[4] == 0xD3 && (packet[5] & 0xFCU) == 0U;
  }

  // 1번 이후 fragment는 올바른 첫 fragment로 시작된 동일 frame만 통과.
  return reassembly.active && reassembly.sequence == sequence &&
         reassembly.fragment_count == count &&
         reassembly.next_fragment == index;
}

void ProcessLoRaPacket() {
  uint8_t packet[255]{};
  uint8_t length = 0;
  uint8_t irq = 0;
  const int result = lora_receive_packet(packet, &length, &irq);

  // FIFO 복사 직후 RX Continuous로 먼저 복귀시킨다.
  rx_set();

  if (result < 0) {
    rssi_graph_packet(0, nullptr, 0);
    ++total.lora_phy_crc_errors;
    ++interval_stats.lora_phy_crc_errors;
    Serial.printf("[LORA] Payload CRC error, IRQ=0x%02X\r\n", irq);
    return;
  }
  if (result == 0) {
    return;
  }

  rssi_graph_packet(1, packet, length);
  ++total.lora_packets;
  ++interval_stats.lora_packets;
  total.lora_bytes += length;
  interval_stats.lora_bytes += length;
  Serial.printf("[LORA] RX length=%u RSSI=%d dBm SNRx100=%d\r\n", length,
                lora_packet_rssi_dbm(), lora_packet_snr_x100());

  if (!IsOurGroundStationPacket(packet, length)) {
    ++total.ground_filter_drops;
    ++interval_stats.ground_filter_drops;
    Serial.printf(
        "[FILTER] DROP: not our ground RTCM packet, marker=0x%02X len=%u\r\n",
        length > 0 ? packet[0] : 0, length);
    return;
  }

  Serial.printf("[FILTER] PASS: ground marker=0x%02X\r\n", packet[0]);
  if (packet[0] == TTGO_PACKET_SINGLE) {
    ProcessSinglePacket(packet, length);
  } else if (packet[0] == TTGO_PACKET_FRAGMENT) {
    ProcessFragmentPacket(packet, length);
  } else if (packet[0] == TTGO_PACKET_DOWNLINK_END) {
    SendLatestGgaToGround();
  } else if (packet[0] == GROUND_STATION_ID &&
             packet[1] == WAYPOINT_MESSAGE) {
    ProcessWaypointPacket(packet, length);
  } else {
    ++total.unsupported_packets;
    ++interval_stats.unsupported_packets;
    Serial.printf("[LORA] Unsupported packet type=0x%02X\r\n", packet[0]);
  }
}

const char *GGAStatusName(uint8_t quality) {
  switch (quality) {
    case 0: return "INVALID";
    case 1: return "GNSS FIX";
    case 2: return "DGNSS/DGPS";
    case 4: return "RTK FIXED";
    case 5: return "RTK FLOAT";
    case 6: return "ESTIMATED";
    default: return "OTHER";
  }
}

double NmeaCoordinateToDegrees(const char *text, char hemisphere) {
  if (text == nullptr || *text == '\0') {
    return NAN;
  }
  const double raw = strtod(text, nullptr);
  const double degrees = floor(raw / 100.0);
  const double minutes = raw - degrees * 100.0;
  double result = degrees + minutes / 60.0;
  if (hemisphere == 'S' || hemisphere == 'W') {
    result = -result;
  }
  return result;
}

void HandleGGALine(char *line) {
  if (strncmp(line, "$GNGGA,", 7) != 0 &&
      strncmp(line, "$GPGGA,", 7) != 0) {
    return;
  }

  char raw[NMEA_LINE_SIZE];
  strlcpy(raw, line, sizeof(raw));
  if (UM982_PRINT_RAW_GGA) {
    Serial.printf("[UM982 RAW] %s\r\n", raw);
  }

  char *fields[16]{};
  uint8_t count = 0;
  char *cursor = line;
  while (count < 16 && cursor != nullptr) {
    fields[count++] = cursor;
    char *comma = strchr(cursor, ',');
    if (comma == nullptr) {
      break;
    }
    *comma = '\0';
    cursor = comma + 1;
  }
  if (count < 10) {
    Serial.printf("[UM982] GGA parse error: fields=%u\r\n", count);
    return;
  }

  const uint8_t quality = static_cast<uint8_t>(atoi(fields[6]));
  const double latitude = NmeaCoordinateToDegrees(
      fields[2], fields[3][0] == '\0' ? 'N' : fields[3][0]);
  const double longitude = NmeaCoordinateToDegrees(
      fields[4], fields[5][0] == '\0' ? 'E' : fields[5][0]);
  const char *diff_age =
      (count > 13 && fields[13][0] != '\0') ? fields[13] : "N/A";

  ++gga_lines;
  last_gga_ms = millis();

  // UM982의 10 Hz GGA가 들어올 때마다 최신값만 덮어쓴다. A3 수신 시에는
  // 새 문장을 기다리지 않고 이 스냅샷을 즉시 사용한다.
  if (isfinite(latitude) && isfinite(longitude) &&
      latitude >= -90.0 && latitude <= 90.0 &&
      longitude >= -180.0 && longitude <= 180.0) {
    latest_gga.valid = true;
    latest_gga.latitude = latitude;
    latest_gga.longitude = longitude;
    latest_gga.quality = quality;
    latest_gga.differential_age =
        strcmp(diff_age, "N/A") == 0 ? NAN : strtof(diff_age, nullptr);
    latest_gga.received_ms = last_gga_ms;
  }

  // MATLAB과 VS Code가 함께 읽을 수 있는 구조화 GNSS line.
  // G,tick,utc,lat,lon,alt,quality,sats,hdop,diff_age,status
  Serial.printf(
      "G,%lu,%s,%.8f,%.8f,%s,%u,%s,%s,%s,%s\r\n",
      static_cast<unsigned long>(last_gga_ms),
      fields[1][0] ? fields[1] : "N/A", latitude, longitude,
      fields[9][0] ? fields[9] : "nan", quality,
      fields[7][0] ? fields[7] : "0",
      fields[8][0] ? fields[8] : "nan",
      strcmp(diff_age, "N/A") == 0 ? "nan" : diff_age,
      GGAStatusName(quality));
}

void ProcessUM982Input() {
  if (um982 == nullptr) {
    return;
  }

  while (um982->available() > 0) {
    const int value = um982->read();
    if (value < 0) {
      break;
    }
    ++um982_received_bytes;
    const char byte = static_cast<char>(value);
    if (byte == '\n') {
      if (nmea_length > 0) {
        if (nmea_line[nmea_length - 1] == '\r') {
          --nmea_length;
        }
        nmea_line[nmea_length] = '\0';
        ++um982_lines;

        Serial.printf("[UM982 ALL] %s\r\n", nmea_line);

        HandleGGALine(nmea_line);
        nmea_length = 0;
      }
    } else if (nmea_length < NMEA_LINE_SIZE - 1) {
      nmea_line[nmea_length++] = byte;
    } else {
      ++nmea_overflows;
      nmea_length = 0;
    }
  }
}

void PrintOneSecondStatistics() {
  const uint32_t now = millis();
  const uint32_t interval_um982_bytes =
      um982_received_bytes - last_um982_received_bytes;
  const uint32_t interval_um982_lines = um982_lines - last_um982_lines;
  const uint32_t interval_gga_lines = gga_lines - last_gga_lines;
  const uint32_t gga_age_ms =
      last_gga_ms == 0 ? UINT32_MAX : static_cast<uint32_t>(now - last_gga_ms);
  const bool gga_rate_ok = interval_gga_lines >= UM982_EXPECTED_GGA_HZ - 1U &&
                           interval_gga_lines <= UM982_EXPECTED_GGA_HZ + 1U;

  Serial.printf(
      "[1s] LoRa=%lu pkt/%lu B RTCM=%lu frame/%lu B FWD=%lu/%lu B "
      "CRCerr=%lu PHYerr=%lu ReasmErr=%lu SeqMiss=%lu Dup=%lu "
      "FilterDrop=%lu UARTdrop=%lu UM982=%lu B/s,%lu lines/s "
      "GGA=%lu Hz(%s),age=%lu ms,NMEAovf=%lu\r\n",
      static_cast<unsigned long>(interval_stats.lora_packets),
      static_cast<unsigned long>(interval_stats.lora_bytes),
      static_cast<unsigned long>(interval_stats.completed_frames),
      static_cast<unsigned long>(interval_stats.completed_bytes),
      static_cast<unsigned long>(forwarded_frames - last_forwarded_frames),
      static_cast<unsigned long>(forwarded_bytes - last_forwarded_bytes),
      static_cast<unsigned long>(interval_stats.rtcm_crc_errors),
      static_cast<unsigned long>(interval_stats.lora_phy_crc_errors),
      static_cast<unsigned long>(interval_stats.reassembly_errors),
      static_cast<unsigned long>(interval_stats.sequence_misses),
      static_cast<unsigned long>(interval_stats.duplicate_packets),
      static_cast<unsigned long>(interval_stats.ground_filter_drops),
      static_cast<unsigned long>(uart_drops - last_uart_drops),
      static_cast<unsigned long>(interval_um982_bytes),
      static_cast<unsigned long>(interval_um982_lines),
      static_cast<unsigned long>(interval_gga_lines), gga_rate_ok ? "OK" : "WARN",
      static_cast<unsigned long>(gga_age_ms),
      static_cast<unsigned long>(nmea_overflows));
  last_forwarded_frames = forwarded_frames;
  last_forwarded_bytes = forwarded_bytes;
  last_uart_drops = uart_drops;
  last_um982_received_bytes = um982_received_bytes;
  last_um982_lines = um982_lines;
  last_gga_lines = gga_lines;
  interval_stats = Statistics{};
}

void PrintTenSecondTotals() {
  Serial.printf(
      "[TOTAL] LoRa=%lu pkt/%lu B RTCM=%lu frame/%lu B Forwarded=%lu "
      "frame/%lu B UM982_RX=%lu B GGA=%lu Lines=%lu NMEAovf=%lu "
      "CRCerr=%lu Timeout=%lu SeqMiss=%lu FilterDrop=%lu Unsupported=%lu\r\n",
      static_cast<unsigned long>(total.lora_packets),
      static_cast<unsigned long>(total.lora_bytes),
      static_cast<unsigned long>(total.completed_frames),
      static_cast<unsigned long>(total.completed_bytes),
      static_cast<unsigned long>(forwarded_frames),
      static_cast<unsigned long>(forwarded_bytes),
      static_cast<unsigned long>(um982_received_bytes),
      static_cast<unsigned long>(gga_lines),
      static_cast<unsigned long>(um982_lines),
      static_cast<unsigned long>(nmea_overflows),
      static_cast<unsigned long>(total.rtcm_crc_errors),
      static_cast<unsigned long>(total.fragment_timeouts),
      static_cast<unsigned long>(total.sequence_misses),
      static_cast<unsigned long>(total.ground_filter_drops),
      static_cast<unsigned long>(total.unsupported_packets));
}
}  // namespace

void LORARTK_Begin(HardwareSerial &um982_serial) {
  um982 = &um982_serial;
  ResetReassembly();
  total = Statistics{};
  interval_stats = Statistics{};
  sequence_initialized = false;
  latest_gga = LatestGga{};
  latest_waypoints = LatestWaypointCommand{};
  gps_sequence = 0;
  latest_detected_flag = 0;
  latest_person_count = 0;

  lora_setup();
  lora_freq();
  packet_set();
  fifo_set();
  rx_set();

  statistics_ms = millis();
  totals_ms = statistics_ms;

  Serial.print("\r\n=== LORARTK ESP32-S3 Drone Receiver ===\r\n");
  Serial.printf(
      "[PIN] CS=%d SCK=%d MISO=%d MOSI=%d EN=%d RST=%d DIO0=%d\r\n",
      LORA_PIN_CS, LORA_PIN_SCK, LORA_PIN_MISO, LORA_PIN_MOSI,
      LORA_PIN_EN, LORA_PIN_RESET, LORA_PIN_DIO0);
  Serial.printf(
      "[RADIO] SX1276 version=0x%02X, 922.1 MHz, SF7, BW125, CR4/5, "
      "Explicit, CRC ON\r\n",
      lora_read(0x42));
  Serial.print("[PACKET] A1=[type,seq,RTCM], "
               "A2=[type,seq,index,count,data], A3=DOWNLINK_END, "
               "Waypoint=[FF,F2,count,lat/lon...], "
               "uplink=[FE,seq,F3,1,lat,lon,det,count]\r\n");
  Serial.printf("[UART] UM982 RX=%d TX=%d, 115200 8N1; USB debug 115200\r\n",
                UM982_RX_PIN, UM982_TX_PIN);
  Serial.printf("[GNSS] Expected GGA=%u Hz, RX buffer=8192 bytes\r\n",
                UM982_EXPECTED_GGA_HZ);
}

void IRAM_ATTR LORARTK_NotifyRadioInterrupt() { radio_irq_pending = true; }

void LORARTK_SetDetection(uint8_t detected, uint8_t person_count) {
  latest_detected_flag = detected ? 1U : 0U;
  latest_person_count = person_count;
}

void LORARTK_Process() {
  ProcessUM982Input();

  // DIO0 interrupt가 주 경로다. 혹시 부팅/배선 타이밍에 edge를 놓쳤더라도
  // IRQ register를 주기적으로 확인해 수신 완료 packet을 회수한다.
  static uint32_t last_irq_poll_ms = 0;
  const uint32_t now = millis();
  bool check_radio = radio_irq_pending;
  if (static_cast<uint32_t>(now - last_irq_poll_ms) >= 20) {
    last_irq_poll_ms = now;
    check_radio = true;
  }
  if (check_radio) {
    radio_irq_pending = false;
    if ((lora_read(0x12) & 0x60U) != 0U) {
      ProcessLoRaPacket();
    }
  }

  uart_send_noise_rssi();

  // ProcessLoRaPacket()보다 앞에서 저장한 now를 사용하면, 이번 loop에서 기록된
  // last_fragment_ms가 now보다 커져 uint32_t 뺄셈이 언더플로될 수 있다.
  // 반드시 fragment 처리 이후의 현재 시간으로 timeout을 판정한다.
  const uint32_t timeout_now = millis();
  if (ReassemblyTimedOut(timeout_now)) {
    DiscardTimedOutReassembly(timeout_now);
  }

  if (static_cast<uint32_t>(now - statistics_ms) >= 1000) {
    statistics_ms = now;
    PrintOneSecondStatistics();
  }
  if (static_cast<uint32_t>(now - totals_ms) >= 10000) {
    totals_ms = now;
    PrintTenSecondTotals();
  }

  ProcessUM982Input();
}
