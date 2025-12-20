import time
import struct
import requests
from collections import deque
from pyrf24 import RF24, RF24_PA_MAX, RF24_250KBPS

# NRF24 object and pins
CE_PIN = 22
CSN_PIN = 0
radio = RF24(CE_PIN, CSN_PIN)

BASE_ADDR = b"BSE01"


# type, handheld_id, tower_id, lat, lon, status, msg_id
PAYLOAD_FMT = "<BBBiiBH"  

# ack_ok, msg_id
ACK_FMT = "<BH"            

# status mapping
STATUS_MAP = {1: "EMERGENCY / CRITICAL",
              2: "SAFE / OK",
              3: "NEED FOOD / WATER / MEDICAL SUPPLIES"}

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

        # decode message
        msg = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()),
            "type": pkt[0],
            "handheld_id": pkt[1],
            "tower_id": pkt[2],
            "lat": pkt[3] / 1e7,
            "lon": pkt[4] / 1e7,
            "status": pkt[5],
            "status_str": STATUS_MAP.get(pkt[5], "UNKNOWN"),
            "msg_id": pkt[6]
        }

        print("\n--- MESSAGE RECEIVED ---")
        print(msg)

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
