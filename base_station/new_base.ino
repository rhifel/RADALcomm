#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "payload_struct.h"

// RGB pins
constexpr uint8_t LED_R = 25;
constexpr uint8_t LED_G = 26;
constexpr uint8_t LED_B = 27;

//Buzzer
constexpr uint8_t BUZZ = 17;

// radio object, nrf24 pins 
constexpr uint8_t CE_PIN = 4;
constexpr uint8_t CSN_PIN = 5;
RF24 radio(CE_PIN, CSN_PIN);
constexpr uint8_t BSE_ADDR[6] = "BSE01";
constexpr uint8_t HND_ADDR[6] = "HND01";// Address


// buzzer beep helper function
void beepBUZZER(uint8_t pin, uint8_t times = 1, uint16_t delayMS = 100){
  for(uint8_t i = 0; i < times; i++){
      digitalWrite(pin, HIGH);
      delay(delayMS);
      digitalWrite(pin, LOW);
      delay(delayMS);
  }
}

void setRGB(uint8_t r, uint8_t g, uint8_t b){
  digitalWrite(LED_R, r);
  digitalWrite(LED_G, g);
  digitalWrite(LED_B, b);
}

const char* typeToStr(uint8_t t) {
    switch (t) {
        case 1: return "PKT_STATUS";
        case 2: return "PKT_RESPONSE";
        default: return "UNKNOWN";
    }
}

const char* statusToStr(uint8_t s) {
    switch (s) {
        case 1: return "ALERT";
        case 2: return "SAFE";
        case 3: return "AID";
        default: return "UNKNOWN";
    }
}

void handleIncomingRF(){

  uint8_t pipeNum;
  if(!radio.available(&pipeNum)) return;

  uint8_t payloadSize = radio.getDynamicPayloadSize();

  if(payloadSize != sizeof(payload_t)) return;

  payload_t pkt;
  radio.read(&pkt, sizeof(pkt));

  uint32_t arrival_time_ms = millis();
  uint32_t latency = arrival_time_ms - pkt.sent_time_ms; // rf transmission time

  if(pkt.type != PKT_STATUS) return;

  switch(pkt.status) {
    case STATUS_RED:   setRGB(1, 0, 0); break;
    case STATUS_GREEN: setRGB(0, 1, 0); break;
    case STATUS_BLUE:  setRGB(0, 0, 1); break;
    default:           setRGB(0, 0, 0); break; // off
  }

  float lat = pkt.latitude / 1e7;
  float lon = pkt.longitude / 1e7;

  char latencyStr[16];
  snprintf(latencyStr, sizeof(latencyStr), "%lu ms", latency);

  const char* type_str = typeToStr(pkt.type);
  const char* status_str = statusToStr(pkt.status);
  bool response_bool = false;

  char serialBuffer[256];
  snprintf(serialBuffer, sizeof(serialBuffer), 
         "<%s,%u,%s,%u,%u,%.6f,%.6f,%u,%s,%u,%u,",
         latencyStr,            
         pkt.type,
         type_str,              
         pkt.handheld_id,
         pkt.tower_id,
         lat,                   
         lon,                    
         pkt.status,
         status_str,             
         pkt.msg_id,
         pkt.response_code
        );
  Serial.print(serialBuffer);
  Serial.print(response_bool); Serial.print(">");

  ack_t ack = {1, pkt.msg_id};
  radio.writeAckPayload(pipeNum, &ack, sizeof(ack)); // send the acknowledgement to the handheld 
  delay(2);
}

void setup() {
  Serial.begin(115200); 
  
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(BUZZ, OUTPUT);

  if(!radio.begin()){
   digitalWrite(BUZZ, HIGH);
   while(1); 
  }

  beepBUZZER(BUZZ, 4, 200);
  
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setChannel(100);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setRetries(5, 15);
  radio.openWritingPipe(HND_ADDR);
  radio.openReadingPipe(1, BSE_ADDR);
  radio.startListening();
}

void loop() {

  handleIncomingRF();

}
