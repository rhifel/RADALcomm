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
const uint8_t HND_ADDR[6] = "address";
const uint8_t BSE_ADDR[6] = "address";
constexpr uint8_t H_ID = 1;

// buttons and led pins
constexpr uint8_t BUTTON_R = 25;
constexpr uint8_t BUTTON_G = 26;
constexpr uint8_t BUTTON_B = 27;
constexpr uint8_t BUTTON_SC = 32;
constexpr uint8_t LED_G    = 33;

// button and status arrays
const uint8_t buttons[]  = {BUTTON_R, BUTTON_G, BUTTON_B};
const uint8_t statuses[] = {STATUS_RED, STATUS_GREEN, STATUS_BLUE};
const size_t numButtons  = sizeof(buttons)/sizeof(buttons[0]);

// variable helpers
static uint16_t msg_counter = 1; 
const uint16_t DEBOUNCE_MS = 200;
// separate debounce times for the buttons
static uint32_t lastPressTime[numButtons] = {0,0,0}; 
static uint32_t lastScreenPressTime = 0;

// link-level base auto-ack 
bool receivedLinkAck = false;

// reponse from base for LAST RECV and checkForResponse
bool lastRecvResponse = false;
uint16_t lastRecvMsgID = 0;
uint8_t lastRecvStatus = 0;

// last sent status for LAST SENT and sendSOS
bool LastSentPkt = false;
uint16_t lastSentMsgID = 0;
uint8_t lastSentStatus = 0;

// time snapshots for LAST SENT
bool lastSentTimeValid = false;
uint8_t lastSentHour, lastSentMinute, lastSentSecond;
uint8_t lastSentDay, lastSentMonth;
uint8_t lastSentYear;

// time snapshots for LAST RECV
bool lastRecvTimeValid = false;
uint8_t lastRecvHour, lastRecvMinute, lastRecvSecond;
uint8_t lastRecvDay, lastRecvMonth;
uint8_t lastRecvYear;

// popup handling
bool showRecvPopup = false;
uint32_t recvPopupStart = 0;
const uint32_t RECV_POPUP_MS = 0;

// button confirmation
bool isWaitingConfirm = false;
uint8_t pendingStatus = 0;
uint32_t confirmTimeout = 0;
const uint16_t CONFIRM_WINDOW_MS = 5000; // 5 seconds to confirm button send

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

const char* statusToStr(uint8_t s) {
    switch (s) {
        case 1: return "ALERT";
        case 2: return "SAFE";
        case 3: return "AID";
        default: return "UNKNOWN";
    }
}

const char* statusToMsgResp(uint8_t r){
    switch(r){
        case 1: return "RESCUE ON THE WAY";
        case 2: return "FOOD & WATER INCOMING";
        case 3: return "CHECK-UP SCHEDULED";
        default: return "UNKNOWN";
    }
}

// enums for statemachine homescreen
// TDC is Time, Date, and Coordinates
// LSR is Last Sent and Received
enum HomeScreenState {SCREEN_TDC, SCREEN_LSR};
HomeScreenState homeState = SCREEN_TDC; 

const uint8_t daysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

bool isLeapYear(uint16_t y) {
    return (y % 4 == 0 && y % 100 != 0) || (y % 400 == 0);
}

void applyTimeOffset(uint8_t &hour, uint8_t &day, uint8_t &month, uint16_t &year, uint8_t offset) {
    hour += offset;
    if (hour >= 24) {
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

void oledShowTDC() {
    oled.clearDisplay();

    if (!gps.time.isValid() || !gps.date.isValid() || !gps.location.isValid()) {
        oledPrintCenterY("ACQUIRING", 28, 2);
        oled.display();
        return;
    }

    uint8_t hour   = gps.time.hour();
    uint8_t minute = gps.time.minute();
    uint8_t day    = gps.date.day();
    uint8_t month  = gps.date.month();
    uint16_t year  = gps.date.year();

    // UTC+8
    applyTimeOffset(hour, day, month, year, 8);

    // Convert to 12hr format
    const char* ampm = (hour >= 12) ? "PM" : "AM";
    uint8_t hour12 = hour % 12;
    if (hour12 == 0) hour12 = 12;

    char timeBuf[16];
    sprintf(timeBuf, "%02d:%02d %s", hour12, minute, ampm);

    char dateBuf[16];
    sprintf(dateBuf, "%02d/%02d/%04d", month, day, year);

    char coordBuf[24];
    sprintf(coordBuf, "%.5f, %.5f", gps.location.lat(), gps.location.lng());

    oledPrintCenterY(dateBuf, 4, 1);
    oledPrintCenterY(timeBuf, 16, 2);
    oledPrintCenterY("LAT , LON", 36, 1);
    oledPrintCenterY(coordBuf, 44, 1);

    oled.display();
}

void oledShowLSR() {
    oled.clearDisplay();
    char timeBuf[16];
    char idBuf[8];

    //  SECTION 1: LAST SENT 
    oledPrintAt("LAST SENT:", 0, 0, 1);

    if (LastSentPkt && lastSentTimeValid) {
        uint8_t hour   = lastSentHour;
        uint8_t minute = lastSentMinute;
        uint8_t second = lastSentSecond;
        uint8_t day    = lastSentDay;
        uint8_t month  = lastSentMonth;
        uint16_t year  = lastSentYear;

        applyTimeOffset(hour, day, month, year, 8);

        const char* ampm = (hour >= 12) ? "PM" : "AM";
        uint8_t hour12 = hour % 12;
        if (hour12 == 0) hour12 = 12;

        sprintf(timeBuf, "%02d:%02d:%02d%s", hour12, minute, second, ampm);
        oledPrintAt(timeBuf, 62, 0, 1); 

        // Large Status
        oledPrintAt(statusToStr(lastSentStatus), 0, 12, 2);

        // ID Label and Value
        oledPrintAt("ID:", 85, 12, 1);
        sprintf(idBuf, "%d", lastSentMsgID);
        oledPrintAt(idBuf, 105, 12, 2);
    } else {
        oledPrintAt("NO DATA", 0, 12, 2);
    }

    // Divider Line at middle
    oled.drawFastHLine(0, 31, 128, SH110X_WHITE);

    //  SECTION 2: LAST RECV 
    oledPrintAt("LAST RECV:", 0, 34, 1);

    if (lastRecvResponse && lastRecvTimeValid) {
        uint8_t hour   = lastRecvHour;
        uint8_t minute = lastRecvMinute;
        uint8_t second = lastRecvSecond;
        uint8_t day    = lastRecvDay;
        uint8_t month  = lastRecvMonth;
        uint16_t year  = lastRecvYear;

        applyTimeOffset(hour, day, month, year, 8);

        const char* ampm = (hour >= 12) ? "PM" : "AM";
        uint8_t hour12 = hour % 12;
        if (hour12 == 0) hour12 = 12;

        sprintf(timeBuf, "%02d:%02d:%02d%s", hour12, minute, second, ampm);
        oledPrintAt(timeBuf, 62, 34, 1);

        oledPrintAt("MSG_ID:", 0, 48, 1);
        sprintf(idBuf, "%d", lastRecvMsgID);
        oledPrintAt(idBuf, 45, 48, 1);
        
        oledPrintAt(statusToMsgResp(lastRecvStatus), 0, 56, 1);
    } else {
        oledPrintAt("NONE RECEIVED", 0, 48, 1);
    }

    oled.display();
}

// helper for switching screens
void refreshCurrentScreen() {
    if (homeState == SCREEN_TDC) oledShowTDC();
    else oledShowLSR();
}

void handleScreenButton(uint32_t now) {
    if (digitalRead(BUTTON_SC) == LOW && (now - lastScreenPressTime > DEBOUNCE_MS)) {

        lastScreenPressTime = now;

        homeState = (homeState == SCREEN_TDC) ? SCREEN_LSR : SCREEN_TDC;
        refreshCurrentScreen();
    }
}

// led blink helper fucntion
void blinkLED(uint8_t pin, uint8_t times = 1, uint16_t delayMs = 100){
    for(uint8_t i = 0; i < times; i++){
        digitalWrite(pin, HIGH);
        delay(delayMs);
        digitalWrite(pin, LOW);
        delay(delayMs);
    }
}

// Check for incoming response from base
void checkForResponse(){
    char idBuf[8];
    uint8_t pipe;
    if(radio.available(&pipe) && pipe == 1){
        uint8_t payloadSize = radio.getDynamicPayloadSize(); 
        if(payloadSize != sizeof(payload_t)){
            uint8_t dummy[payloadSize];
            radio.read(dummy, payloadSize);
            return;
        }
        payload_t resp; //response 
        radio.read(&resp, sizeof(resp));

        if(resp.type != PKT_RESPONSE || resp.handheld_id != H_ID) return;

        lastRecvResponse = true;
        lastRecvMsgID = resp.msg_id;
        lastRecvStatus = resp.response_code; // 1 = alertMsgResp, 2 = aidMsgResp, 3 = safeMsgResp 
        
        if(gps.time.isValid() && gps.date.isValid()){
            lastRecvHour   = gps.time.hour();
            lastRecvMinute = gps.time.minute();
            lastRecvSecond = gps.time.second();
            lastRecvDay    = gps.date.day();
            lastRecvMonth  = gps.date.month();
            lastRecvYear   = gps.date.year();
            lastRecvTimeValid = true;
    }

        showRecvPopup = true;
        recvPopupStart = millis();
        
        drawResponsePopup(resp.msg_id, resp.response_code);
    }
}

// recv popup UI
void drawResponsePopup(uint16_t id, uint8_t status) {
    char idBuf[8];
    oled.clearDisplay();
    oledPrintAt("MESSAGE RECEIVED", 0, 0, 1);
    oled.drawFastHLine(0, 10, 128, 1);
    
    oledPrintAt("ID:", 0, 15, 1);
    sprintf(idBuf, "%d", id);
    oledPrintAt(idBuf, 30, 15, 2); 
    
    oledPrintAt("STATUS:", 0, 35, 1);
    oledPrintAt(statusToMsgResp(status), 0, 45, 1);
    oled.display();
}

// rf wait for ack
bool waitForAck(payload_t pkt, uint16_t timeout_ms){
    uint32_t start = millis();

    while(millis() - start < timeout_ms){
        if(radio.isAckPayloadAvailable()){
            uint8_t payloadSize = radio.getDynamicPayloadSize();
            if(payloadSize == sizeof(ack_t)){
                ack_t ack;
                radio.read(&ack,sizeof(ack));
                if (ack.ack_ok && ack.msg_id == lastSentMsgID) {
                    receivedLinkAck = true;
                    if(receivedLinkAck) blinkLED(LED_G, 2, 200);
                    return true;
                }
            }
        }
        delay(1);
    }
    return false;
}

bool sendSOS(uint8_t status){
    lastSentStatus = status;   // ALERT / SAFE / AID
    LastSentPkt = true;
    receivedLinkAck = false;
    
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

    pkt.latitude  = gps.location.isValid() ? 
                    (int32_t)(gps.location.lat()*1e7) : 0;

    pkt.longitude = gps.location.isValid() ? 
                    (int32_t)(gps.location.lng()*1e7) : 0;

    pkt.status = status;
    pkt.msg_id = msg_counter++;
    pkt.response_code = 0;   // added for the PKT_RESPONSE
    
    lastSentMsgID = pkt.msg_id;
    
    // time snapshot
    if(gps.time.isValid() && gps.date.isValid()){
        lastSentHour   = gps.time.hour();
        lastSentMinute = gps.time.minute();
        lastSentSecond = gps.time.second();
        lastSentDay    = gps.date.day();
        lastSentMonth  = gps.date.month();
        lastSentYear   = gps.date.year();
        lastSentTimeValid = true;
    } 

    radio.stopListening();
    bool ok = radio.write(&pkt,sizeof(pkt));
    radio.startListening();

    return ok && waitForAck(pkt,800);
}

void handleUserButtons(uint32_t now) {
    for (size_t i = 0; i < numButtons; i++) {
        // check for press + debounce
        if (digitalRead(buttons[i]) == LOW && (now - lastPressTime[i] > DEBOUNCE_MS)) {
            lastPressTime[i] = now;

            // the 2nd press 
            if(isWaitingConfirm){
                if(statuses[i] == pendingStatus){ 
                    // match, send packet
                    sendSOS(pendingStatus);

                    oled.clearDisplay();
                    oledPrintCenterY("SENDING...", 10, 1);
                    oledPrintCenterY(statusToStr(pendingStatus), 30, 2);
                    oled.display();

                    // trigger the Popup state to freeze this screen for 3 seconds
                    showRecvPopup = true; 
                    recvPopupStart = now; 
                    isWaitingConfirm = false; // reset the confirmation state
                } else{ // user presses a different button
                    isWaitingConfirm = false;
                    oledPrintCenter("CANCELLED", 1, true);
                    oled.display(); 
                    delay(500);//small delay to show msg
                    refreshCurrentScreen();
                }
            }else{ // first press
                isWaitingConfirm = true;
                pendingStatus = statuses[i];
                confirmTimeout = now;

                oled.clearDisplay();
                oledPrintCenterY("CONFIRM SEND?", 5, 1);
                oledPrintCenterY(statusToStr(pendingStatus), 20, 2);
                oledPrintCenterY("Press same button", 45, 1);
                oledPrintCenterY("to send...", 55, 1);
                oled.display();
            }
        }
    }
    if(isWaitingConfirm && (now - confirmTimeout > CONFIRM_WINDOW_MS)){
        isWaitingConfirm = false;
        refreshCurrentScreen();
    }
}


// setup
void setup() {
    Serial.begin(115200);
    GPSSerial.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);

    oled.begin(OLED_ADDR,true);
    Wire.setClock(400000);
    oled.clearDisplay();
    oled.setTextColor(SH110X_WHITE);

    pinMode(BUTTON_R, INPUT_PULLUP);
    pinMode(BUTTON_G, INPUT_PULLUP);
    pinMode(BUTTON_B, INPUT_PULLUP);
    pinMode(BUTTON_SC, INPUT_PULLUP);
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

    // LED flash to indicate active
    blinkLED(LED_G, 4, 200);

    radio.enableDynamicPayloads();
    radio.enableAckPayload();
    radio.setChannel(100);
    radio.setDataRate(RF24_250KBPS);
    radio.setPALevel(RF24_PA_HIGH);
    radio.setRetries(5,15);
    radio.openWritingPipe(BSE_ADDR);
    radio.openReadingPipe(1, HND_ADDR);
    radio.startListening();

    oled.clearDisplay();
    oledPrintCenter("ACTIVE", 2, true);
    oled.display();
    delay(700);

    // Show initial screen
    oledShowTDC();
}

// Main Loop 
void loop() {
    while(GPSSerial.available()) gps.encode(GPSSerial.read());

    uint32_t now = millis();

    // check responses from base
    checkForResponse();

    // check for button presses
    handleScreenButton(now); // screen rotation button
    handleUserButtons(now);

    // popup transitions when receiving
    if (showRecvPopup){
        // If 3 seconds have passed, 
        // close the popup and return to main screen
        if (now - recvPopupStart > 3000){ 
            showRecvPopup = false;
            refreshCurrentScreen();
        }
    } 
    else if(isWaitingConfirm){
    // let handleUserButtons control the screen
        static uint32_t lastFlash = 0;
        if (now - lastFlash > 1000) { // flash every 1 second
            lastFlash = now;
            blinkLED(LED_G, 1, 50); // Short flash
        }
    }
    else{
        static uint32_t lastRefresh = 0;
        if(now - lastRefresh > 200){
            lastRefresh = now;
            refreshCurrentScreen();
        }
    }
}
