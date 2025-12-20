#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "payload_struct.h"

// nrf24 Pins 
constexpr uint8_t CE_PIN = 4;
constexpr uint8_t CSN_PIN = 5;

// RGB LED pins
constexpr uint8_t LED_R = 25;
constexpr uint8_t LED_G = 26;
constexpr uint8_t LED_B = 27;

// Tower ID
constexpr uint8_t T_ID = 1;

// NRF object
RF24 radio(CE_PIN, CSN_PIN);

// Addresses
const uint8_t HND_ADDR[6] = "HND01";  // Handheld
const uint8_t BSE_ADDR[6] = "BSE01";  // Base station

// Helper function Set RGB LED 
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
    digitalWrite(LED_R, r);
    digitalWrite(LED_G, g);
    digitalWrite(LED_B, b);
}

void setup() {
    Serial.begin(115200);

    // Set LED pins as outputs
    pinMode(LED_R, OUTPUT);
    pinMode(LED_G, OUTPUT);
    pinMode(LED_B, OUTPUT);

    // Start with green idle
    setRGB(0, 1, 0);

    // Initialize NRF
    if (!radio.begin()) {
        Serial.println("NRF FAIL");
        while (1);
    }

    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    radio.setChannel(100);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_MAX);

    // Open reading and writing pipes
    radio.openReadingPipe(0, HND_ADDR);   // From handheld
    radio.openWritingPipe(BSE_ADDR);      // To base
    radio.startListening();

    Serial.println("Tower ready");
}

void loop() {
    uint8_t pipeNum;

    // Check for incoming packets
    if (!radio.available(&pipeNum)) return;

    uint8_t payloadSize = radio.getDynamicPayloadSize();

    // Only handle handheld packets
    if (pipeNum == 0) {
        if (payloadSize != sizeof(payload_t)) {
            uint8_t dummy[payloadSize];
            radio.read(dummy, payloadSize);      
            Serial.print("Ignored invalid packet of size: ");
            Serial.println(payloadSize);
            return;
        }

        payload_t pkt;
        radio.read(&pkt, sizeof(pkt));
        pkt.tower_id = T_ID; // assign tower id

        // Send ACK to handheld
        ack_t ack = {1, pkt.msg_id};
        radio.writeAckPayload(pipeNum, &ack, sizeof(ack));
        delay(2);
        Serial.print("ACK sent for msg_id: ");
        Serial.println(pkt.msg_id);

        //  Set LED based on status
        switch(pkt.status) {
            case STATUS_RED:   setRGB(1, 0, 0); break;
            case STATUS_GREEN: setRGB(0, 1, 0); break;
            case STATUS_BLUE:  setRGB(0, 0, 1); break;
            default:           setRGB(0, 1, 0); break; // fallback green
        }

        // Relay packet to base station
        radio.stopListening();
        bool ok = radio.write(&pkt, sizeof(pkt));
        if (!ok) Serial.println("Failed to relay packet to base");
        else Serial.println("Relayed packet to base");
        radio.startListening();
    }
    else {
        // Ignore other pipes (if any)
        uint8_t dummy[payloadSize];
        radio.read(dummy, payloadSize);
    }
}
