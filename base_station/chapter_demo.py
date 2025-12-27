import time
import struct
import requests
import json
from collections import deque
from datetime import datetime, timezone, timedelta
from pyrf24 import RF24, RF24_PA_MAX, RF24_250KBPS

# NRF24 object and pins
CE_PIN = 22
CSN_PIN = 0
radio = RF24(CE_PIN, CSN_PIN)

BASE_ADDR = b"BSE01"


# type, handheld_id, tower_id, lat, lon, status, msg_id
PAYLOAD_FMT = "<HBBIBBBiiBHB"  

# ack_ok, msg_id
ACK_FMT = "<BH"            

# status mapping
STATUS_MAP = {1: "ALERT",
              2: "SAFE",
              3: "AID"}

MESSAGE_RECEIVED = {0: False,
                    1: True,}

PACKET_TYPE = {1: "STATUS",
               2: "RESPONSE"}

# BACKEND_URL = "http://ip:5000/api/events"
BACKEND_URL = "http://ip:5000/api/events"

# Non-blocking timing & queue
LOOP_DELAY = 0.05  # seconds
last_loop_time = 0
msg_queue = deque()  # queue for messages to backend
MAX_RETRIES = 3

# Setup radio
def setup_radio():
    if not radio.begin():
        raise RuntimeError("NRF24 not found")

    radio.enableDynamicPayloads()
    radio.enableAckPayload()
    radio.setChannel(100)
    radio.setDataRate(RF24_250KBPS)
    radio.setPALevel(RF24_PA_MAX)

    radio.openReadingPipe(0, BASE_ADDR)
    radio.openWritingPipe(BASE_ADDR)
    radio.startListening()
    print("Base station ready")

# Backend sending with retries
def send_to_backend(msg):
    for attempt in range(1, MAX_RETRIES + 1):
        try:
            r = requests.post(BACKEND_URL, json=msg, timeout=3)
            print(f"Sent to backend: {r.status_code}, msg_id: {msg['msg_id']}")
            return True
        except Exception as e:
            print(f"Backend error (attempt {attempt}): {e}")
            time.sleep(0.5)  # short delay before retry
    print(f"Failed to send msg_id {msg['msg_id']} after {MAX_RETRIES} attempts")
    return False

# Process received packet
def process_packet():
    if radio.available():
        pipe_num = None
        expected_size = struct.calcsize(PAYLOAD_FMT)
        payload_size = radio.getDynamicPayloadSize()

        if payload_size != expected_size:
            dummy = radio.read(payload_size)
            print(f"Ignored invalid payload size: {payload_size}, expected {expected_size}")
            return

        raw = radio.read(payload_size)
        pkt = struct.unpack(PAYLOAD_FMT, raw)

        # reconstruct sent time
        sent_time = datetime(pkt[0], pkt[1], pkt[2], 0, 0, 0, tzinfo=timezone.utc) 
        sent_time += timedelta(seconds=pkt[3])
        
        #current time in UTC
        arricval_time = datetime.now(timezone.utc)

        # calculate latency in seconds
        latency_sec = (arricval_time - sent_time).total_seconds()

        # decode message
        msg = {
            "latency_ms": str(round(latency_sec * 1000, 2)) + " ms",
            "type": pkt[4],
            "type_str": PACKET_TYPE.get(pkt[4], "UNKNOWN"),
            "handheld_id": pkt[5],
            "tower_id": pkt[6],
            "lat": pkt[7] / 1e7,
            "lon": pkt[8] / 1e7,
            "status": pkt[9],
            "status_str": STATUS_MAP.get(pkt[9], "UNKNOWN"),
            "msg_id": pkt[10],
            "response_code": pkt[11],
            "response_bool": MESSAGE_RECEIVED.get(pkt[11], "UNKNOWN"),
        }

        print("\n--- MESSAGE RECEIVED ---")
        # print(msg)
        json_msg = json.dumps(msg, indent=4)
        print(json_msg)

        # queue message to backend
        msg_queue.append(msg)

        # ACK back to tower
        ack = struct.pack(ACK_FMT, 1, msg["msg_id"])
        radio.writeAckPayload(1, ack)
        print(f"ACK sent for msg_id {msg['msg_id']}")

# Process queued messages
def process_queue():
    if not msg_queue:
        return
    msg = msg_queue.popleft()
    success = send_to_backend(msg)
    if not success:
        # put it back to the end for retry
        msg_queue.append(msg)

# Main loop
def main():
    global last_loop_time
    while True:
        now = time.time()
        if now - last_loop_time < LOOP_DELAY:
            continue
        last_loop_time = now

        process_packet()
        process_queue()

# Entry point
if __name__ == "__main__":
    try:
        setup_radio()
        main()
    except KeyboardInterrupt:
        print("\nExiting...")
