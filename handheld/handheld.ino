#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "payload_struct.h"
#include <Wire.h>
#include <Adafruit_SH110X.h>
#include <TinyGPS++.h>

// oled object and variables
constexpr uint8_t SCREEN_WIDTH  = 128;
constexpr uint8_t SCREEN_HEIGHT = 64;
constexpr uint8_t OLED_RESET    = -1;
constexpr uint8_t OLED_ADDR     = 0x3C;
Adafruit_SH1106G oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// gps obejct, uart, and variable
HardwareSerial GPSSerial(2);  
TinyGPSPlus gps;
constexpr uint8_t RX_PIN = 17;
constexpr uint8_t TX_PIN = 16;

// nrf24 object, device address, variables, and pins
constexpr uint8_t CE_PIN  = 4;
constexpr uint8_t CSN_PIN = 5;
RF24 radio(CE_PIN, CSN_PIN);
const uint8_t HND_ADDR[6] = "HND01";
constexpr uint8_t H_ID = 1;

// buttons and led pins
constexpr uint8_t BUTTON_R = 25;
constexpr uint8_t BUTTON_G = 26;
constexpr uint8_t BUTTON_B = 27;
constexpr uint8_t LED_G    = 33;

const uint8_t buttons[]  = {BUTTON_R, BUTTON_G, BUTTON_B};
const uint8_t statuses[] = {STATUS_RED, STATUS_GREEN, STATUS_BLUE};
const size_t numButtons  = sizeof(buttons)/sizeof(buttons[0]);

// variable helpers
static uint16_t msg_counter = 1; 
const uint16_t DEBOUNCE_MS = 50;
static uint32_t lastPressTime[numButtons] = {0,0,0}; // separate debounce times for the buttons

// OLED helper functions
void oledPrintAt(const String &msg, int x, int y, int textSize = 1, bool clear = false){
    if(clear) oled.clearDisplay();
    oled.setTextSize(textSize);
    oled.setCursor(x, y);
    oled.println(msg);
}

void oledPrintCenter(const String &msg, int textSize = 1, bool clear = false){
    int cX = SCREEN_WIDTH / 2;
    int cY = SCREEN_HEIGHT / 2;
    int textWidth  = msg.length() * 6 * textSize;
    int textHeight = 8 * textSize;
    int posX = cX - textWidth/2;
    int posY = cY - textHeight/2;
    oledPrintAt(msg, posX, posY, textSize, clear);
}

void oledPrintCenterY(const String &msg, int y, int textSize =1){
    int textWidth = msg.length() * 6  * textSize;
    int x = (128 - textWidth) / 2;
    oled.setTextSize(textSize);
    oled.setCursor(x, y);
    oled.print(msg);
}

// enums for statemachine homescreen
enum HomeScreenState { SCREEN_TIME, SCREEN_COORD, SCREEN_LEGEND };
HomeScreenState homeState = SCREEN_TIME;
uint32_t homeStateTimer = 0;
const uint32_t HOME_INTERVAL = 2000;

void oledShowTimeDate() {
    oled.clearDisplay();

    if (!gps.time.isValid() || !gps.date.isValid()) {
        oledPrintCenterY("ACQUIRING", 28, 2);
        oled.display();
        return;
    }

    uint8_t hour = gps.time.hour();
    uint8_t minute = gps.time.minute();
    uint8_t day   = gps.date.day();
    uint8_t month = gps.date.month();
    uint16_t year = gps.date.year();
    const char* ampm = "AM";

    // UTC+8
    hour += 8;
    if(hour >= 24){
        hour -= 24;
        day++;
        if(day>31){ 
            day=1; month++; 
            if(month>12){ 
                month=1; 
                year++; 
            } 
        }
    }

    if(hour >=12){ 
        ampm="PM"; 
        if(hour>12) hour-=12; 
    }
    if(hour==0) hour=12;

    char timeBuf[12];
    sprintf(timeBuf, "%02d:%02d %s", hour, minute, ampm);

    char dateBuf[16];
    sprintf(dateBuf, "%02d/%02d/%04d", month, day, year);

    oledPrintCenterY(timeBuf, 18, 2);
    oledPrintCenterY(dateBuf, 42, 1);
    oled.display();
}

void oledShowCoordinates(){
    oled.clearDisplay();

    if (!gps.location.isValid()) {
        oledPrintCenterY("GPS NO FIX", 28, 2);
        oled.display();
        return;
    }

    char coord[24];
    sprintf(coord, "%.5f, %.5f",
            gps.location.lat(),
            gps.location.lng());

    oledPrintCenterY("Latitude -- Longitude", 20, 1);
    oledPrintCenterY(coord, 34, 1);
    oled.display();
}

void oledShowLegend(){
    oled.clearDisplay();
    oledPrintAt("RED   [!] ALERT",  6, 18);
    oledPrintAt("GREEN [#] SAFE",   6, 32);
    oledPrintAt("BLUE  [+] AID",    6, 46);
    oled.display();
}

void oledHomeNonBlocking() {
    uint32_t now = millis();
    if(now - homeStateTimer < HOME_INTERVAL) return;
    homeStateTimer = now;

    switch(homeState){
        case SCREEN_TIME:  oledShowTimeDate();  homeState = SCREEN_COORD; break;
        case SCREEN_COORD: oledShowCoordinates(); homeState = SCREEN_LEGEND; break;
        case SCREEN_LEGEND: oledShowLegend(); homeState = SCREEN_TIME; break;
    }
}

// led blink helper fucntion
void indicateLED(bool ok){
    if(ok){
        digitalWrite(LED_G,HIGH); delay(300); digitalWrite(LED_G,LOW);
    } else {
        for(int i = 0; i < 2; i++){ 
            digitalWrite(LED_G,HIGH); 
            delay(100); 
            digitalWrite(LED_G,LOW); 
            delay(100);}
    }
}

// rf wait for ack
bool waitForAck(payload_t pkt, uint16_t timeout_ms){
    uint32_t start = millis();
    while(millis() - start < timeout_ms){
        if(radio.isAckPayloadAvailable()){
            uint8_t len = radio.getDynamicPayloadSize();
            if(len == sizeof(ack_t)){
                ack_t ack;
                radio.read(&ack,sizeof(ack));
                Serial.println("Ack received from tower");
                return ack.ack_ok && ack.msg_id==pkt.msg_id;
            }
        }
        delay(1);
    }
    Serial.println("ACK Timeout");
    return false;
}

bool sendSOS(uint8_t status){
    payload_t pkt;

    pkt.year  = gps.date.isValid() ? gps.date.year() : 0;
    pkt.month = gps.date.isValid() ? gps.date.month() : 0;
    pkt.day   = gps.date.isValid() ? gps.date.day() : 0;

    pkt.timestamp_utc = gps.time.isValid() ? 
                        gps.time.hour() * 3600 + 
                        gps.time.minute() * 60 + 
                        gps.time.second() : 0;

    pkt.type = PKT_STATUS;
    pkt.handheld_id = H_ID;
    pkt.tower_id = 0;

    pkt.latitude  = gps.location.isValid() ? 
                    (int32_t)(gps.location.lat()*1e7) : 0;

    pkt.longitude = gps.location.isValid() ? 
                    (int32_t)(gps.location.lng()*1e7) : 0;

    pkt.status = status;
    pkt.msg_id = msg_counter++;

    // added for the PKT_RESPONSE
    pkt.response_code = 0;

    radio.stopListening();
    bool ok = radio.write(&pkt,sizeof(pkt));
    radio.startListening();

    return ok && waitForAck(pkt,800);
}

// setup
void setup() {
    Serial.begin(115200);
    GPSSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

    oled.begin(OLED_ADDR,true);
    oled.clearDisplay();
    oled.setTextColor(SH110X_WHITE);

    pinMode(BUTTON_R, INPUT_PULLUP);
    pinMode(BUTTON_G, INPUT_PULLUP);
    pinMode(BUTTON_B, INPUT_PULLUP);
    pinMode(LED_G, OUTPUT);

    oled.clearDisplay();
    oledPrintCenter("BOOTING...", 2, true);
    oled.display();
    delay(700);
    
    oled.clearDisplay();
    if(!radio.begin()){
        oledPrintCenter("NRF FAIL", 2, true); 
        oled.display();
        while(1);
    } else {
        oledPrintCenter("NRF OK", 2, true); 
        oled.display();
        delay(700);
    }

    // LED flash to indicate ready
    for(int i = 0; i < 4; i++){ 
        digitalWrite(LED_G,HIGH); 
        delay(200); 
        digitalWrite(LED_G,LOW); 
        delay(200); }

    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    radio.setChannel(100);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_LOW);
    radio.setRetries(5,15);
    radio.openWritingPipe(HND_ADDR);
    radio.openReadingPipe(1,HND_ADDR);
    radio.stopListening();

    oled.clearDisplay();
    oledPrintCenter("ACTIVE", 2, true);
    oled.display();
    delay(700);
}

// Main Loop 
void loop() {
    while(GPSSerial.available()) gps.encode(GPSSerial.read());

    uint32_t now = millis();

    // Home screen refresh
    oledHomeNonBlocking();

    // Button handling
    for(size_t i = 0; i < numButtons; i++){
        if(digitalRead(buttons[i]) == LOW && now - lastPressTime[i] > DEBOUNCE_MS){
            lastPressTime[i] = now;

            bool ok = sendSOS(statuses[i]);

            oled.clearDisplay();
            oledPrintCenter(
                statuses[i] == STATUS_RED   ? "[!] ALERT" :
                statuses[i] == STATUS_GREEN ? "[#] SAFE" :
                                              "[+] AID",
                2, true
            );
            oled.display();

            indicateLED(ok);

            // Reset home screen after showing status
            homeStateTimer = millis();
            homeState = SCREEN_TIME;
        }
    }
}
