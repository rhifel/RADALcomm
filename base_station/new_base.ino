#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "payload_struct.h"

#include "esp_wifi.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

// RGB pins
constexpr uint8_t LED_R = 25;
constexpr uint8_t LED_G = 26;
constexpr uint8_t LED_B = 27;

//Buzzer
constexpr uint8_t BUZZ = 21;

// radio object, nrf24 pins 
constexpr uint8_t CE_PIN = 4;
constexpr uint8_t CSN_PIN = 5;
RF24 radio(CE_PIN, CSN_PIN);
constexpr uint8_t BSE_ADDR[6] = "BSE01";
constexpr uint8_t HND_ADDR[6] = "HND01";// Address

// serial module
constexpr uint8_t SERIAL2_RX = 16;
constexpr uint8_t SERIAL2_TX = 17;

String serialBufferIn = "";


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

const uint8_t statusColors[4][3] = {
  {0, 0, 0}, // Status 0: OFF
  {1, 0, 0}, // Status 1: RED
  {0, 0, 1}, // Status 2: BLUE
  {0, 1, 0}  // Status 3: GREEN
};

const char* typeToStr(uint8_t type) {
    switch (type) {
        case 1: return "PKT_STATUS";
        case 2: return "PKT_RESPONSE";
        default: return "UNKNOWN";
    }
}

const char* statusToStr(uint8_t status) {
    switch (status) {
        case 1: return "ALERT";
        case 2: return "AID";
        case 3: return "SAFE";
        default: return "UNKNOWN";
    }
}

const uint8_t daysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

bool isLeapYear(uint16_t y) {
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

void applyTimeOffset(uint8_t &hour, uint8_t &day, uint8_t &month, uint16_t &year, uint8_t offset) {
    hour += offset;
    while(hour >= 24){
        hour -= 24;
        day++;
        
        uint8_t maxDay = daysInMonth[month - 1];
        if (month == 2 && isLeapYear(year)) maxDay = 29;

        if (day > maxDay) {
            day = 1;
            month++;
            if (month > 12) {
                month = 1;
                year++;
            }
        }
    }
}


void handleIncomingPackets(){

  uint8_t pipeNum;
  if(!radio.available(&pipeNum)) return;
  Serial.println("Received Payload");

  uint8_t payloadSize = radio.getDynamicPayloadSize();

  if(payloadSize == 0 || payloadSize > 32) {
    radio.flush_rx();
    return;
}

  if(payloadSize != sizeof(payload_t)) {
    radio.flush_rx();
    return;
  }

  payload_t pkt;
  radio.read(&pkt, sizeof(pkt)); // auto acknowledgement

  if(pkt.type != PKT_STATUS) return;

  uint8_t s = (pkt.status <= 3) ? pkt.status : 0;

  setRGB(statusColors[s][0], statusColors[s][1], statusColors[s][2]);
  beepBUZZER(BUZZ, 2, 200);

/*
  switch(pkt.status) {
    case STATUS_RED:   setRGB(1, 0, 0); break;
    case STATUS_GREEN: setRGB(0, 1, 0); break;
    case STATUS_BLUE:  setRGB(0, 0, 1); break;
    default:           setRGB(0, 0, 0); break; // off
  }
*/  

  float lat = pkt.latitude / 1e7;
  float lon = pkt.longitude / 1e7;

  uint8_t hour = (uint8_t)(pkt.daySeconds / 3600);
  uint8_t minute = (uint8_t)((pkt.daySeconds / 60) % 60);
  uint8_t seconds = (uint8_t)(pkt.daySeconds % 60);

  uint8_t day = pkt.day;
  uint8_t month = pkt.month;
  uint16_t year = pkt.year;

  // UTC+8
  applyTimeOffset(hour, day, month, year, 8);

  // Convert 24-hour to 12-hour format
  const char* ampm = (hour >= 12) ? "PM" : "AM";
  uint8_t hour12 = hour % 12;
  if (hour12 == 0) hour12 = 12;

  char timeStamp[32]; 
  snprintf(timeStamp, sizeof(timeStamp), "%02u/%02u/%04u-%02u:%02u:%02u%s",  
         month,
         day, 
         year, 
         hour12, minute, seconds,
         ampm);

  char serialBuffer[256];
  snprintf(serialBuffer, sizeof(serialBuffer), 
         "<STATUS,%s,%u,%s,%u,%u,%.7f,%.7f,%u,%s,%u,%u,%d>\n",
         timeStamp, // timestamp
         pkt.type,
         typeToStr(pkt.type),              
         pkt.handheld_id,
         1, // placeholder tower
         lat,                   
         lon,
         pkt.status,                    
         statusToStr(pkt.status),           
         pkt.msg_id,
         pkt.response_code,
         0 // response bool
        );
  Serial2.print(serialBuffer);
  //Serial.print(serialBuffer);
}

void handleSerialCommands(){

  while(Serial2.available()){

  char c = Serial2.read();
  
  if(c == '\n'){
    processSerialPacket(serialBufferIn);
    serialBufferIn="";
  }else{
    serialBufferIn += c;
    }
  }    
}
  
void processSerialPacket(String line) {

  line.trim();

  if(!line.startsWith("<RESP")) return;
  if(!line.endsWith(">")) return;

  // remove < >
  line = line.substring(1, line.length() - 1);

  String parts[5];

  for(int i = 0; i < 5; i++){
    int idx = line.indexOf(',');

    if (idx == -1) {
      parts[i] = line;
      break;
    }

    parts[i] = line.substring(0, idx);
    line = line.substring(idx + 1);
  }

  String tag = parts[0];
  int handheld_id = parts[1].toInt();
  int msg_id = parts[2].toInt();
  int packet_type = parts[3].toInt();
  int response_code = parts[4].toInt();

  if (packet_type != 2) return;

  // Serial.println("Received CMD from dashboard:");
  // Serial.println("Handheld: " + String(handheld_id));
  // Serial.println("Msg ID: " + String(msg_id));
  // Serial.println("Response: " + String(response_code));

  sendRFResponse(handheld_id, msg_id, response_code);
}

void sendRFResponse(int handheld_id, int msg_id, int response_code) {

  payload_t pkt = {0};
  
  pkt.type = PKT_RESPONSE;
  pkt.handheld_id = handheld_id;
  pkt.msg_id = msg_id;
  pkt.response_code = response_code;

  radio.stopListening();

  bool ok = radio.write(&pkt, sizeof(pkt));

  radio.startListening();

  if (ok) {
      beepBUZZER(BUZZ, 3, 200); // to confirm autoack of if it fails the the response might be sent or not
  }
}

bool initHardware() {
  while (true) {
    if (!Serial2) {
      digitalWrite(BUZZ, HIGH);
      delay(2000);
      digitalWrite(BUZZ, LOW);
      delay(1000);
    } else if (!radio.begin()) {
      digitalWrite(BUZZ, HIGH);
      delay(2000);
      digitalWrite(BUZZ, LOW);
      delay(3000);
    } else {
      return true; // Both ready
    }
  }
}

void disableWireless() {
  esp_wifi_stop();
  esp_bluedroid_disable();
  esp_bluedroid_deinit();
  esp_bt_controller_disable();
  esp_bt_controller_deinit();
}

void setup() {
  Serial.begin(115200); 
  Serial2.begin(115200, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX);

  disableWireless();

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  pinMode(BUZZ, OUTPUT);

  initHardware();

  beepBUZZER(BUZZ, 4, 200);

  if (!radio.begin()) {
  Serial.println("NRF24L01 Hardware not responding!");
  while(1); // for testing
}
  //loop through colors
  for (uint8_t i = 1; i <= 3; i++) {
    setRGB(statusColors[i][0], statusColors[i][1], statusColors[i][2]);
    delay(500);
  }
  setRGB(0, 0, 0); //turn off

  radio.enableDynamicPayloads();
  radio.disableAckPayload();
  radio.setChannel(82);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setAutoAck(true);
  radio.setCRCLength(RF24_CRC_16);
  radio.setRetries(15, 15);
  radio.openWritingPipe(HND_ADDR);
  radio.openReadingPipe(1, BSE_ADDR);

  radio.startListening();

}

void loop() {

  handleIncomingPackets();
  handleSerialCommands();

}
