#ifndef PAYLOAD_STRUCT_H
#define PAYLOAD_STRUCT_H

#include <stdint.h>

enum PacketType : uint8_t {
    PKT_STATUS   = 0x01,   // handheld → base
    PKT_RESPONSE = 0x02    // base → handheld
};

enum StatusCode : uint8_t {
    STATUS_RED = 1,
    STATUS_GREEN = 2,
    STATUS_BLUE = 3
};

struct __attribute__((packed)) payload_t {
    uint32_t sent_time_ms; // for latency
    uint8_t  type;         // PacketType
    uint8_t  handheld_id;
    uint8_t tower_id;
    int32_t  latitude;     // lat * 1e7
    int32_t  longitude;    // lon * 1e7

    uint8_t  status;       // StatusCode
    uint16_t msg_id;

    // use for packet type 2
    uint8_t response_code; // response to 1 = alert, 2 = aid, 3 = safe 
}; // 18 bytes

struct __attribute__((packed)) ack_t {
    uint8_t  ack_ok;
    uint16_t msg_id;
}; // 3 bytes

#endif
