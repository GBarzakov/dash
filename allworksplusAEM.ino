

#include "driver/twai.h"
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include <HardwareSerial.h>
#include <qqqlab_GPS_UBLOX.h>
#include <qqqlab_AutoBaud.h>

#define EEPROM_SIZE 512
#define BRIGHTNESS_ADDR 0
#define N0_VALUE_ADDR 10  
#define X11_VALUE_ADDR 20  // Адрес след N0_VALUE_ADDR (10) и BRIGHTNESS_ADDR (0)
#define ECU_SELECT_ADDR 30  // Нов адрес за запазване на избора на ECU
#define N14_VAL_ADDR 40  // New address for n14.val storage
#define N12_VALUE_ADDR 50  // New address for n12.val storage
#define N15_VAL_ADDR 50  // New address for n14.val storage
#define NEXTION_START_MARKER 0xAA55
#define NEXTION_END_MARKER 0xBBBB
#define NEXTION_PACKET_SIZE (sizeof(NextionData) + 4)
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_18
#define NEXTION_RX_PIN 33
#define NEXTION_TX_PIN 32
#define LED_PIN_27 12
#define NUM_LEDS 7
#define RPM_AVG_SIZE 3
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_UPDATE_INTERVAL 75
#define SPEED_FILTER_SIZE 1

enum EcuType {
  ECU_MEGASQUIRT = 1,
  ECU_MAXXECU = 2,
  ECU_EMU = 3,
  ECU_LINK = 4,
  ECU_EMU_Serial = 5
};

#pragma pack(push, 1)
typedef struct {
  uint32_t n0;
  uint32_t n20;
  uint32_t x11;
  uint32_t n12;
  uint32_t n13;
  uint32_t n14;
  uint32_t n15;
} NextionData;
#pragma pack(pop)

// GPS обект и променливи
class GPS_UBLOX : public AP_GPS_UBLOX {
public:
  HardwareSerial *gps_serial;
 
  void begin(HardwareSerial *gps_serial) {
    this->gps_serial = gps_serial;
  }

  void I_setBaud(int baud) override { gps_serial->begin(baud); }
  inline int I_availableForWrite() override { return gps_serial->availableForWrite(); }
  inline int I_available() override { return gps_serial->available(); }
  inline int I_read(uint8_t* data, size_t len) override { return gps_serial->readBytes(data, len); }
  inline int I_write(uint8_t* data, size_t len) override { return gps_serial->write(data, len); }
  inline uint32_t I_millis() override { return ::millis(); }
  void I_print(const char *str) override { Serial.print(str); }
} gps;

#define GPS_SPEED_FILTER_SIZE 3  // Нов размер за GPS филтър
float gpsSpeedHistory[GPS_SPEED_FILTER_SIZE];
int gpsHistoryIndex = 0;
unsigned long lastGpsUpdateTime = 0;
float gpsSpeed = 0.0;          // Текуща GPS скорост
float filteredGpsSpeed = 0.0;  // Филтрирана GPS скорост
float vssSpeed = 0.0;          // VSS скорост (не се филтрира)

HardwareSerial SerialGPS(1);

// Глобални променливи
Adafruit_NeoPixel strip27(NUM_LEDS, LED_PIN_27, NEO_GRB + NEO_KHZ800);

int rpmBuffer[RPM_AVG_SIZE];
int rpmIndex = 0;
int rpmSum = 0;
bool rpmFilled = false;

// Глобални променливи
unsigned long previousRefreshTime = 0;
const long refreshInterval = 3000; // 5 секунди в милисекунди

static int last_gear = 255;
static float last_oil_p = 0.0f;
static float last_fuel_p = 0.0f;
static float last_oil_temp = 0.0f;

int brightness = 0;
int n0_value = 0;
int lastSavedBrightness = 0;

static NextionData nextionData;
static bool newNextionDataAvailable = false;
static uint32_t lastNextionUpdate = 0;

// CAN данни
float flex_g = 0.0f;

float maxxecuVssSpeed = 0;  // VSS скорост от MaxxECU
float megasquirtVssSpeed = 0.0;  // VSS скорост от Megasquirt
uint16_t rpm_g = 0;
uint16_t map_gRaw = 0;
uint16_t map_g = 0;
int16_t mat_g = 0;
int16_t clt_g = 0;
int tps_g = 0;
uint16_t batteryVoltageRaw = 0;
uint16_t batteryVoltage = 0;
uint16_t afr_g = 0;
float fuel_pressure = 0.0f;
float oilPressure = 0.0f;
int oil_temp = 0;
float oil_p = 0.0f;
float fuel_p = 0.0f;
static int last_afr_scaled = 0;
static int last_lambda_scaled = 0;
// Nextion променливи
uint16_t nextion_n0 = 0;
uint16_t nextion_x11 = 0;
uint16_t nextion_n12 = 0;
uint16_t nextion_n13 = 0;
uint16_t nextion_n14 = 0;
uint16_t nextion_n15 = 0;
uint16_t nextion_n20 = 0;
String nextionBuffer = "";
uint16_t currentX11 = 0;
// Последни изпратени стойности
static uint16_t last_map_g = 0;
static int16_t last_mat_g = 0;
static int16_t last_clt_g = 0;
static int last_tps_display = 0;
static uint16_t last_batteryVoltage = 0;
static uint16_t last_afr_g = 0;
float ignang = 0.0f;        // Ignition angle in degrees
float injpw = 0.0f;         // Injection pulse width in ms
uint16_t egt = 0;           // Exhaust gas temperature in °C

// RPM специфични променливи
static int lastRpmValue = 0;
static int lastRpmPicValue = 0;
static unsigned long lastRpmUpdate = 0;

// CAN таймаут
unsigned long lastCANReceived = 0;
const unsigned long canTimeout = 1000;

unsigned long lastNonCriticalUpdate = 0;
const unsigned long nonCriticalInterval = 60;

// Функция за филтриране на скоростта
float filterGpsSpeed(float newSpeed) {
    gpsSpeedHistory[gpsHistoryIndex] = newSpeed;
    gpsHistoryIndex = (gpsHistoryIndex + 1) % GPS_SPEED_FILTER_SIZE;
    
    float sum = 0;
    for (int i = 0; i < GPS_SPEED_FILTER_SIZE; i++) {
        sum += gpsSpeedHistory[i];
    }
    return sum / GPS_SPEED_FILTER_SIZE;
}


void saveN14ToEEPROM(uint16_t value) {
    EEPROM.writeUShort(N14_VAL_ADDR, value);
    EEPROM.commit();
}
void saveN15ToEEPROM(uint16_t value) {
    EEPROM.writeUShort(N15_VAL_ADDR, value);
    EEPROM.commit();
}

uint16_t loadN14FromEEPROM() {
    uint16_t value = EEPROM.readUShort(N14_VAL_ADDR);
    return (value == 0xFFFF) ? 1 : constrain(value, 1, 2); // Default to 1 if not set
}
uint16_t loadN15FromEEPROM() {
    uint16_t value = EEPROM.readUShort(N15_VAL_ADDR);
    return (value == 0xFFFF) ? 1 : constrain(value, 1, 2); // Default to 1 if not set
}

void saveN12ToEEPROM(uint8_t value) {
    EEPROM.write(N12_VALUE_ADDR, value);
    EEPROM.commit();
}



uint8_t loadN12FromEEPROM() {
    uint8_t value = EEPROM.read(N12_VALUE_ADDR);
    return (value == 0xFF) ? 0 : value; // Default to 0 if not set
}

void checkAndSendN12() {
    static uint8_t lastSentN12 = 0;
    if (nextion_n12 != lastSentN12) {
        sendN12ViaCAN(nextion_n12);
        lastSentN12 = nextion_n12;
    }
}
// Функция за пълно обновяване
void refreshAllValues() {
  
    
    if (nextion_n14 == 1) {
        // Refresh на GPS скорост
        nextionBuffer += "v3.txt=\"" + String(int(filteredGpsSpeed)) + "\"\xFF\xFF\xFF";
    } else if (nextion_n14 == 2) {
        // Refresh на VSS скорост
        nextionBuffer += "v3.txt=\"" + String(int(vssSpeed)) + "\"\xFF\xFF\xFF";
    }
    
    // Температури
    nextionBuffer += "x2.val=" + String(clt_g) + "\xFF\xFF\xFF";
    nextionBuffer += "x3.val=" + String(mat_g) + "\xFF\xFF\xFF";
    nextionBuffer += "x6.val=" + String(oil_temp) + "\xFF\xFF\xFF";
  /*  
   // Батерия
    nextionBuffer += "x1.val=" + String(batteryVoltage) + "\xFF\xFF\xFF";*/
  
    
    // t10.txt
    updateT10Text(nextion_n14);
    
    Serial.println("[REFRESH] All values updated");
}


void saveEcuSelectionToEEPROM(uint8_t ecuType) {
    if(ecuType >= 1 && ecuType <= 5) {
        EEPROM.write(ECU_SELECT_ADDR, ecuType);
        EEPROM.commit();
    }
}

uint8_t loadEcuSelectionFromEEPROM() {
    uint8_t ecuType = EEPROM.read(ECU_SELECT_ADDR);
    return (ecuType >= 1 && ecuType <= 5) ? ecuType : ECU_EMU; // По подразбиране EMU
}

void sendCAN_Speed(float speed_kmh) {
    twai_message_t message;
    message.identifier = 0x663;  // CAN ID за скорост
    message.extd = 0;
    message.data_length_code = 8;
    message.rtr = 0;

    // Преобразуване според документацията: speed_kmh × 16
    uint16_t can_speed_value = (uint16_t)(speed_kmh * 16.0f);

    // Записване на скоростта за всички колела
    // Формат: всяка двойка байтове съдържа скоростта (Little-Endian)
    for (int i = 0; i < 8; i += 2) {
        message.data[i] = (can_speed_value >> 8) & 0xFF;  // MSB
        message.data[i+1] = can_speed_value & 0xFF;       // LSB
    }

    // Изпращане
    if (twai_transmit(&message, pdMS_TO_TICKS(100)) != ESP_OK) {
        Serial.println("Failed to send CAN speed");
    } else {
        Serial.printf("Sent CAN speed: %.1f km/h (CAN value: %d = 0x%04X)\n", 
                     speed_kmh, can_speed_value, can_speed_value);
    }
}


void processGPSData() {
    if (nextion_n14 != 1) return;
    
    static unsigned long lastErrorPrintTime = 0;
    unsigned long currentMillis = millis();
    
    gps.update();
    
    if (currentMillis - lastGpsUpdateTime >= GPS_UPDATE_INTERVAL) {
        lastGpsUpdateTime = currentMillis;
        
        if (gps.state.status >= 2) {
            gpsSpeed = (gps.state.ground_speed / 1000.0) * 3.6;
            filteredGpsSpeed = filterGpsSpeed(gpsSpeed);  // Използваме новата функция
            
            if (nextion_n14 == 1) {
                sendCAN_Speed(filteredGpsSpeed);
                nextionBuffer += "v3.txt=\"" + String(int(filteredGpsSpeed)) + "\"\xFF\xFF\xFF";
            }
            
        }
    }
}

void sendN12ViaCAN(uint8_t value) {
    twai_message_t message;
    message.identifier = 101;  // CAN ID 101
    message.extd = 0;
    message.data_length_code = 8;  // Standard 8-byte message
    message.rtr = 0;
    
    // Clear all data bytes
    memset(message.data, 0, 8);
    
    // Set the n12 value at offset byte 0
    message.data[0] = value;
    
    if (twai_transmit(&message, pdMS_TO_TICKS(100)) != ESP_OK) {
        Serial.println("Failed to send n12 via CAN");
    } else {
        Serial.printf("Sent n12 value %d via CAN to ID 101\n", value);
    }
}


void handleNextionData() {
    if (!newNextionDataAvailable) return;

    nextion_n0 = nextionData.n0;
    nextion_n20 = nextionData.n20;
    nextion_x11 = nextionData.x11;
    nextion_n12 = nextionData.n12;
    nextion_n13 = nextionData.n13;
    nextion_n14 = nextionData.n14;
    nextion_n15 = nextionData.n15;

    // Добавете този блок за X11 мониторинг
    static uint16_t last_x11 = 0;
    if (nextion_x11 != last_x11) {
        currentX11 = nextion_x11;
        saveX11ToEEPROM(currentX11);
        last_x11 = nextion_x11;
        Serial.printf("[X11 UPDATE] New X11 value received from Nextion: %d\n", currentX11);
    }

     // Запазване на ECU избора при промяна
    static uint16_t last_n13 = 0;
    if (nextion_n13 != last_n13) {
        saveEcuSelectionToEEPROM(nextion_n13);
        last_n13 = nextion_n13;
        Serial.printf("ECU selection changed to: %d\n", nextion_n13);
    }
    
    // Handle n12 changes
    static uint8_t last_n12 = 0;
    if (nextion_n12 != last_n12) {
        saveN12ToEEPROM(nextion_n12);
        last_n12 = nextion_n12;
        Serial.printf("n12 value changed to: %d\n", nextion_n12);
    }
    
    // Add this block to handle n14 changes
    static uint16_t last_n14 = 0;
    if (nextion_n14 != last_n14) {
        saveN14ToEEPROM(nextion_n14);
        last_n14 = nextion_n14;
        updateT10Text(nextion_n14); // Update the display immediately
        Serial.printf("n14 value changed to: %d\n", nextion_n14);
    }

 // Add this block to handle n14 changes
    static uint16_t last_n15 = 0;
    if (nextion_n15 != last_n15) {
        saveN15ToEEPROM(nextion_n15);
        last_n15 = nextion_n15;
        updateT10Text(nextion_n15); // Update the display immediately
        Serial.printf("n15 value changed to: %d\n", nextion_n15);
    }

    // Rest of your existing code...
    updateBrightnessFromNextion(nextion_n20);
    n0_value = nextion_n0;
    newNextionDataAvailable = false;
}
    


void flushNextionBuffer() {
    if (nextionBuffer.length() > 0) {
        Serial2.print(nextionBuffer);
        nextionBuffer = "";
        lastRpmUpdate = millis();
    }
}

void saveN0ToEEPROM(uint16_t value) {
    EEPROM.writeUShort(N0_VALUE_ADDR, value);
    EEPROM.commit();
}

void saveX11ToEEPROM(uint16_t value) {
    EEPROM.writeUShort(X11_VALUE_ADDR, value);
    EEPROM.commit();
}

uint16_t loadX11FromEEPROM() {
    uint16_t value = EEPROM.readUShort(X11_VALUE_ADDR);
    return (value == 0xFFFF) ? 200 : constrain(value, 0, 400); // Стойност по подразбиране 200
}

uint16_t loadN0FromEEPROM() {
    uint16_t value = EEPROM.readUShort(N0_VALUE_ADDR);
    return (value == 0xFFFF) ? 5000 : constrain(value, 4000, 10000);
}

void updateBrightnessFromNextion(uint16_t n20_value) {
    if(n20_value > 100) return;
    
    int new_brightness = map(n20_value, 0, 100, 5, 255);
    new_brightness = constrain(new_brightness, 5, 255);
    
    if(new_brightness != brightness) {
        brightness = new_brightness;
        strip27.setBrightness(brightness);
        strip27.show();
        saveBrightnessToEEPROM(n20_value);
    }
}

void saveBrightnessToEEPROM(int value) {
    value = constrain(value, 0, 100);
    if (value != lastSavedBrightness) {
        EEPROM.write(BRIGHTNESS_ADDR, value);
        EEPROM.commit();
        lastSavedBrightness = value;
    }
}

void updateT10Text(uint16_t n14_value) {
    String text;
    if (n14_value == 1) {
        text = "GPS";
    } else if (n14_value == 2) {
        text = "VSS";
    } else {
        text = "UNK"; // Unknown value, shouldn't happen
    }
    nextionBuffer += "t10.txt=\"" + text + "\"\xFF\xFF\xFF";
}

void updateShiftLight(int rpm_g) {
    static unsigned long lastBlinkTime = 0;
    static bool ledState = false;

    const int blinkOnTime = 10;
    const int blinkOffTime = 100;

    strip27.clear();

    bool batteryAlarm = (batteryVoltage < 127 || batteryVoltage > 147);
    bool oilAlarm = (oil_p > 0.01 && (oil_p <= 0.5 || oil_p >= 7.8));
    bool mapAlarm = ((map_g - 100) >= currentX11);

    // Алармени диоди (те няма да се използват от RPM логиката)
    if (batteryAlarm) {
        strip27.setPixelColor(6, strip27.Color(100, 0, 255)); // Battery alarm on LED 6
    }
    if (oilAlarm) {
        strip27.setPixelColor(5, strip27.Color(0, 255, 0));   // Oil alarm on LED 5
    }
    if (mapAlarm) {
        strip27.setPixelColor(0, strip27.Color(100, 0, 255)); // MAP alarm on LED 0
    }

    // RPM логика (изключваме диоди, заети от аларми)
    if (rpm_g < n0_value) {
        if (rpm_g >= n0_value - 700 && !batteryAlarm) strip27.setPixelColor(6, strip27.Color(255, 0, 0));
        if (rpm_g >= n0_value - 600 && !oilAlarm)     strip27.setPixelColor(5, strip27.Color(255, 0, 0));
        if (rpm_g >= n0_value - 500) strip27.setPixelColor(4, strip27.Color(255, 0, 0));
        if (rpm_g >= n0_value - 400) strip27.setPixelColor(3, strip27.Color(255, 0, 0));
        if (rpm_g >= n0_value - 300) strip27.setPixelColor(2, strip27.Color(0, 255, 0));
        if (rpm_g >= n0_value - 200) strip27.setPixelColor(1, strip27.Color(0, 255, 0));
        if (rpm_g >= n0_value - 100 && !mapAlarm) strip27.setPixelColor(0, strip27.Color(0, 255, 0));
    } else {
        unsigned long currentTime = millis();
        if (currentTime - lastBlinkTime >= (ledState ? blinkOnTime : blinkOffTime)) {
            lastBlinkTime = currentTime;
            ledState = !ledState;

            for (int i = 0; i < NUM_LEDS; i++) {
                // Прескачаме алармените диоди при мигане
                if ((i == 6 && batteryAlarm) || 
                    (i == 5 && oilAlarm) || 
                    (i == 0 && mapAlarm)) {
                    continue;
                }
                strip27.setPixelColor(i, ledState ? strip27.Color(0, 255, 0) : strip27.Color(0, 0, 0));
            }
        }
    }

    strip27.setBrightness(brightness);
    strip27.show();
}



void processNextionSerial() {
  static uint8_t buffer[NEXTION_PACKET_SIZE];
  static uint8_t pos = 0;
  static bool inPacket = false;

  while (Serial2.available()) {
    uint8_t c = Serial2.read();

    if (!inPacket) {
      if (c == 0xAA) {
        inPacket = true;
        buffer[pos++] = c;
      }
      continue;
    }

    if (pos == 1 && c != 0x55) {
      inPacket = false;
      pos = 0;
      continue;
    }

    buffer[pos++] = c;

    if (pos >= NEXTION_PACKET_SIZE) {
      if (buffer[pos-2] == 0xBB && buffer[pos-1] == 0xBB) {
        memcpy(&nextionData, &buffer[2], sizeof(NextionData));
        
        nextionData.n0 = constrain(nextionData.n0, 4000, 10000);
        nextionData.n20 = constrain(nextionData.n20, 0, 100);
        nextionData.x11 = constrain(nextionData.x11, 0, 400);
        nextionData.n12 = (nextionData.n12 > 0) ? 1 : 0;
        nextionData.n13 = (nextionData.n13 - 1) % 5 + 1;
        nextionData.n14 = (nextionData.n14 - 1) % 2 + 1;
        nextionData.n15 = (nextionData.n15 - 1) % 2 + 1;
        newNextionDataAvailable = true;
        lastNextionUpdate = millis();
        
        static uint32_t last_n0 = 0;
        if (nextionData.n0 != last_n0) {
          saveN0ToEEPROM(nextionData.n0);
          last_n0 = nextionData.n0;
        }
      }
      inPacket = false;
      pos = 0;
    }
  }
}

void processCANMessages() {
    twai_message_t rx_message;
    static uint8_t currentEcu = loadEcuSelectionFromEEPROM();

    if (twai_receive(&rx_message, pdMS_TO_TICKS(0)) == ESP_OK) {
        lastCANReceived = millis();
        
        // Проверка дали ECU изборът е променен
        if (nextion_n13 != 0 && nextion_n13 != currentEcu) {
            currentEcu = nextion_n13;
            Serial.printf("Switched to ECU type: %d\n", currentEcu);
        }

        bool needsUpdate = false;
        String tempBuffer = "";

        // Обработка на съобщения според избраната ECU система
        switch(currentEcu) {
            case ECU_MEGASQUIRT:
                processMegasquirtMessage(rx_message, tempBuffer, needsUpdate);
                
                // Специална обработка за VSS ако сме в VSS режим
                if (nextion_n14 == 2 && rx_message.identifier == 1562) {
                    // Четене на VSS (16-bit, offset 0)
                    megasquirtVssSpeed = ((rx_message.data[1] << 8) | rx_message.data[0]);
                    
                    // Изпращане към дисплея
                    tempBuffer += "v3.txt=\"" + String(int(megasquirtVssSpeed)) + "\"\xFF\xFF\xFF";
                    
                    // Изпращане по CAN
                    sendCAN_Speed(megasquirtVssSpeed);
                    
                    needsUpdate = true;
                }
                break;
                
            case ECU_MAXXECU:
                processMaxxecuMessage(rx_message, tempBuffer, needsUpdate);
                
                // Специална обработка за VSS ако сме в VSS режим
                if (nextion_n14 == 2 && rx_message.identifier == 1562) {
                    // Четене на VSS (int16, offset 6, scale 0.1)
                    int16_t rawVss = (rx_message.data[1] << 8) | rx_message.data[0];
                    maxxecuVssSpeed = rawVss;
                    
                    // Изпращане към дисплея
                    tempBuffer += "v3.txt=\"" + String(int(maxxecuVssSpeed)) + "\"\xFF\xFF\xFF";
                    
                    // Изпращане по CAN
                    sendCAN_Speed(maxxecuVssSpeed);
                    
                    needsUpdate = true;
                }
                break;
                
            case ECU_EMU:
                processEmuMessage(rx_message, tempBuffer, needsUpdate);
                
                // Специална обработка за VSS ако сме в VSS режим
                if (nextion_n14 == 2 && rx_message.identifier == 1538) {
                    // Четене на VSS (16-bit, offset 0)
                    vssSpeed = ((rx_message.data[1] << 8) | rx_message.data[0]);
                    
                    // Изпращане към дисплея
                    tempBuffer += "v3.txt=\"" + String(int(vssSpeed)) + "\"\xFF\xFF\xFF";
                    
                    // Изпращане по CAN
                    sendCAN_Speed(vssSpeed);
                    
                    needsUpdate = true;
                }
                break;
                
            case ECU_LINK:
                processLinkMessage(rx_message, tempBuffer, needsUpdate);
                // Добавете VSS логика за LINK ако е необходимо
                break;
        }

        if (needsUpdate && tempBuffer.length() > 0) {
            nextionBuffer = tempBuffer;
        }
    } else {
        // CAN таймаут обработка
        unsigned long currentTime = millis();
        if (currentTime - lastCANReceived > canTimeout) {
            strip27.clear();
            strip27.show();
        }
    }
}


void processEmuMessage(twai_message_t &rx_message, String &tempBuffer, bool &needsUpdate) {
    switch (rx_message.identifier) {

        // ================= 1536 =================
        case 1536: {
            int rawRpm = ((rx_message.data[1] << 8) | rx_message.data[0]);
            rawRpm = constrain(rawRpm, 0, 16000);

            rpmSum = rpmSum - rpmBuffer[rpmIndex] + rawRpm;
            rpmBuffer[rpmIndex] = rawRpm;
            rpmIndex = (rpmIndex + 1) % RPM_AVG_SIZE;
            rpm_g = rpmSum / RPM_AVG_SIZE;

            if (rpm_g < 60) rpm_g = 0;

            if (abs(rpm_g - lastRpmValue) > 1) {
                lastRpmValue = rpm_g;
                needsUpdate = true;
            }

            if (abs(rpm_g - lastRpmPicValue) > 1) {
                int angle = map(rpm_g, 0, 9000, 0, 270);
                angle = constrain(angle, 0, 270);
                tempBuffer += "z0.val=" + String(angle) + "\xFF\xFF\xFF";
                lastRpmPicValue = rpm_g;
                needsUpdate = true;
            }

            // TPS, MAP, MAT
            tps_g = (rx_message.data[2]) / 2;
            tps_g = constrain(tps_g, 0, 100);

            mat_g = ((int8_t)rx_message.data[3]);
            map_g = ((rx_message.data[5] << 8) | rx_message.data[4]);

            if ((map_g - 100) != last_map_g || mat_g != last_mat_g) {
                tempBuffer += "x0.val=" + String(map_g - 100) + "\xFF\xFF\xFF";
                tempBuffer += "x3.val=" + String(mat_g) + "\xFF\xFF\xFF";
                last_map_g = map_g - 100;
                last_mat_g = mat_g;
                needsUpdate = true;
            }

            // MAP ъгъл → z0
            int map_angle = map(map_g, 0, 300, 0, 270);
            map_angle = constrain(map_angle, 0, 270);
            tempBuffer += "z1.val=" + String(map_angle) + "\xFF\xFF\xFF";
            needsUpdate = true;

            // TPS → x5
            if (tps_g != last_tps_display) {
                tempBuffer += "x5.val=" + String(tps_g) + "\xFF\xFF\xFF";
                last_tps_display = tps_g;
                needsUpdate = true;
            }

            // INJPW
            {
                uint16_t injpw_raw = (rx_message.data[7] << 8) | rx_message.data[6]; // Little endian
                float injpw_ms = injpw_raw * 0.016129f;
                tempBuffer += "t14.txt=\"" + String(injpw_ms, 1) + "\"\xFF\xFF\xFF";
                needsUpdate = true;
            }

            updateShiftLight(rpm_g);
            break;
        }

        // ================= 1538 =================
        case 1538: {
            static int last_oil_temp = 0;
            static float last_oil_p = 0.0f;
            static float last_fuel_p = 0.0f;
            static uint16_t last_vss = 0;
    
            // VSS
            uint16_t vss = ((rx_message.data[1] << 8) | rx_message.data[0]);
            vssSpeed = vss;
    
            if (vss != last_vss && nextion_n14 == 2) {
                nextionBuffer += "v3.txt=\"" + String(vss) + "\"\xFF\xFF\xFF";
                sendCAN_Speed(vss);
                last_vss = vss;
                needsUpdate = true;
            }

            oil_temp = rx_message.data[3];
            oil_p    = (rx_message.data[4] * 0.0625f);
            fuel_p   = (rx_message.data[5] * 0.03125f);
            clt_g    = ((int16_t)((rx_message.data[7] << 8) | rx_message.data[6]));

            if (oil_temp != last_oil_temp) {
                tempBuffer += "x6.val=" + String(oil_temp) + "\xFF\xFF\xFF";
                last_oil_temp = oil_temp;
                needsUpdate = true;
            }

            if (abs(oil_p - last_oil_p) > 0.01f) {
                tempBuffer += "x7.val=" + String((int)(oil_p * 100)) + "\xFF\xFF\xFF";
                last_oil_p = oil_p;
                needsUpdate = true;
            }

            if (abs(fuel_p - last_fuel_p) > 0.01f) {
                tempBuffer += "x8.val=" + String((int)(fuel_p * 200)) + "\xFF\xFF\xFF";
                last_fuel_p = fuel_p;
                needsUpdate = true;
            }

            if (clt_g != last_clt_g) {
                tempBuffer += "x2.val=" + String(clt_g) + "\xFF\xFF\xFF";
                last_clt_g = clt_g;
                needsUpdate = true;
            }
            break;
        }

        // ================= 1539 =================
        case 1539: {
            // Lambda / AFR
            float lambda = rx_message.data[2] * 0.0078125f;
            float afr = lambda * 14.7f;
            int afr_scaled = afr * 10;

            if (afr_scaled != last_afr_scaled) {
                tempBuffer += "x4.val=" + String(afr_scaled) + "\xFF\xFF\xFF";
                last_afr_scaled = afr_scaled;
                needsUpdate = true;
            }

            // ignang
            int8_t ignang_raw = (int8_t)rx_message.data[0];
            float ignang = ignang_raw * 0.5f;
            tempBuffer += "t15.txt=\"" + String(ignang, 1) + "\"\xFF\xFF\xFF";
            needsUpdate = true;

            // EGT
            uint16_t egt_raw = (rx_message.data[5] << 8) | rx_message.data[4];
            tempBuffer += "t16.txt=\"" + String(egt_raw) + "\"\xFF\xFF\xFF";
            needsUpdate = true;

            break;
        }

       // ================= 1540 =================
case 1540: {
    batteryVoltageRaw = ((uint16_t)(rx_message.data[3] << 8) | rx_message.data[2]) * 10;
    batteryVoltage = batteryVoltageRaw * 0.027f;

    if (batteryVoltage != last_batteryVoltage) {
        tempBuffer += "x1.val=" + String(batteryVoltage) + "\xFF\xFF\xFF";
        last_batteryVoltage = batteryVoltage;
        needsUpdate = true;
    }

    // GEAR (байт 0, както досега)
    static uint8_t last_gear = 255;
    uint8_t gear = rx_message.data[0];

    if (gear != last_gear) {
        String gearText;
        String gearColorCmd;

        if (gear == 0) {
            gearText = "N";
            gearColorCmd = "t0.pco=2016";
        } else if (gear == 255) {
            gearText = "R";
            gearColorCmd = "t0.pco=63488";
        } else {
            gearText = String(gear);
            gearColorCmd = "t0.pco=65535";
        }

        tempBuffer += "t0.txt=\"" + gearText + "\"\xFF\xFF\xFF";
        tempBuffer += gearColorCmd + "\xFF\xFF\xFF";
        last_gear = gear;
        needsUpdate = true;
    }

    // ETHANOL % (байт 7, 8-bit unsigned, 0–100, 1%/bit, little-endian — единичен байт)
    static uint8_t last_ethanol = 255; // невалидна стартова стойност
    uint8_t ethanol = rx_message.data[7];
    if (ethanol > 100) ethanol = 100;  // safety clamp

    if (ethanol != last_ethanol) {
        // към Nextion: t16.txt
        tempBuffer += "t16.txt=\"" + String(ethanol) + "\"\xFF\xFF\xFF";
        last_ethanol = ethanol;
        needsUpdate = true;
    }

    break;
}

// ================= 1545 → x9.val =================
case 1545: {
    static uint8_t last_x9 = 255;  // за процента (байт 6)
    uint8_t raw = rx_message.data[6];  // процент
    if (raw > 100) raw = 100;  // safety
    if (raw != last_x9) {
        tempBuffer += "x9.val=" + String(raw) + "\xFF\xFF\xFF";
        last_x9 = raw;
        needsUpdate = true;
    }

    // FUEL TEMP (байт 2, 8-bit unsigned) → t14.txt
    static uint8_t last_fuelTemp = 255; // невалидна стартова стойност
    uint8_t fuelTemp = rx_message.data[2]; // без скалиране, както поиска
    // по желание може да се добави safe clamp, напр. if (fuelTemp > 200) fuelTemp = 200;

    if (fuelTemp != last_fuelTemp) {
        tempBuffer += "t14.txt=\"" + String(fuelTemp) + "\"\xFF\xFF\xFF";
        last_fuelTemp = fuelTemp;
        needsUpdate = true;
    }

    break;
}

    }
}


void processMegasquirtMessage(twai_message_t &rx_message, String &tempBuffer, bool &needsUpdate) {
    switch (rx_message.identifier) {
        case 1520: {  // RPM данни
            
            // INJPW: байтове 2 и 3, unsigned 16-bit, Big Endian
             int16_t injpw_raw = (rx_message.data[2] << 8) | rx_message.data[3];
             float injpw_ms = injpw_raw * 0.001f;
             tempBuffer += "t14.txt=\"" + String(injpw_ms, 1) + "\"\xFF\xFF\xFF";

             int rawRpm = ((rx_message.data[6] << 8) | rx_message.data[7]);
            
            rpmSum = rpmSum - rpmBuffer[rpmIndex] + rawRpm;
            rpmBuffer[rpmIndex] = rawRpm;
            rpmIndex = (rpmIndex + 1) % RPM_AVG_SIZE;
            rpm_g = rpmSum / RPM_AVG_SIZE;
            
            if (rpm_g < 60) rpm_g = 0;

            if (abs(rpm_g - lastRpmValue) > 1) {
                lastRpmValue = rpm_g;
                needsUpdate = true;
            }

            if (abs(rpm_g - lastRpmPicValue) > 10) {
                int angle = map(rpm_g, 0, 9000, 0, 270);
                angle = constrain(angle, 0, 270);
                tempBuffer += "z0.val=" + String(angle) + "\xFF\xFF\xFF";
                lastRpmPicValue = rpm_g;
                needsUpdate = true;
            }

            updateShiftLight(rpm_g);
            break;
        }
        
case 1521: {  // IGN ANG и INJPW (и двете Big Endian)
    // IGN ANG: байтове 0 и 1, signed 16-bit, Big Endian
    int16_t ignang_raw = (rx_message.data[0] << 8) | rx_message.data[1];
    float ignang = ignang_raw * 0.1f;

    

    // Изпращане към Nextion
    tempBuffer += "t15.txt=\"" + String(ignang, 1) + "\"\xFF\xFF\xFF";
    

    needsUpdate = true;
    break;
}


        case 1522: {  // MAP, MAT, CLT
            map_gRaw = (rx_message.data[2] << 8) | rx_message.data[3];
            map_g = map_gRaw / 10;
            mat_g = ((int16_t)rx_message.data[4] << 8) | rx_message.data[5];
            clt_g = ((int16_t)rx_message.data[6] << 8) | rx_message.data[7];

            // Конвертиране от Fahrenheit към Celsius (ако е необходимо)
            mat_g = ((((mat_g - 320) * 5) / 9) / 10);
            clt_g = ((((clt_g - 320) * 5) / 9) / 10);

            if ((map_g - 100) != last_map_g || mat_g != last_mat_g || clt_g != last_clt_g) {
                tempBuffer += "x0.val=" + String(map_g - 100) + "\xFF\xFF\xFF";
                tempBuffer += "x3.val=" + String(mat_g) + "\xFF\xFF\xFF";
                tempBuffer += "x2.val=" + String(clt_g) + "\xFF\xFF\xFF";
                
                last_map_g = map_g - 100;
                last_mat_g = mat_g;
                last_clt_g = clt_g;
                needsUpdate = true;

                int map_angle = map(map_g, 0, 300, 0, 270);
                map_angle = constrain(map_angle, 0, 270);
                tempBuffer += "z1.val=" + String(map_angle) + "\xFF\xFF\xFF";
                needsUpdate = true;
            }
            break;
        }
        
       case 1523: {  // TPS, Battery, AFR
    tps_g = ((rx_message.data[0] << 8) | rx_message.data[1]);
    batteryVoltageRaw = (rx_message.data[2] << 8) | rx_message.data[3];
    batteryVoltage = batteryVoltageRaw;
    afr_g = (rx_message.data[4] << 8) | rx_message.data[5];

    int tps_display = 0;

    // Ако стойността е по-малка от 0 или необичайно висока — нула
    if (tps_g >= 0 && tps_g <= 1000) {
        tps_display = map(tps_g, 0, 1000, 0, 100);
    }

    // Ограничаване в обхвата 0–100
    tps_display = constrain(tps_display, 0, 100);
    
    if (tps_display != last_tps_display || batteryVoltage != last_batteryVoltage || afr_g != last_afr_g) {
        tempBuffer += "j0.val=" + String(tps_display) + "\xFF\xFF\xFF";
        tempBuffer += "x1.val=" + String(batteryVoltage) + "\xFF\xFF\xFF";
        tempBuffer += "x4.val=" + String(afr_g) + "\xFF\xFF\xFF";
        tempBuffer += "x5.val=" + String(tps_display) + "\xFF\xFF\xFF";

        last_tps_display = tps_display;
        last_batteryVoltage = batteryVoltage;
        last_afr_g = afr_g;
        needsUpdate = true;
    }
    break;
}

            case 1533: {  // Oil Pressure, Oil Temp, Fuel Pressure


            static int last_oil_temp = 0;
    static float last_oil_p = 0.0f;
    static float last_fuel_p = 0.0f;
    // Налягане на масло (0-1000 kPa)
    uint16_t oil_p_raw = (rx_message.data[0] << 8) | rx_message.data[1];
    oil_p = oil_p_raw / 10.0f;  // Преобразуване в float (пример: 500 → 50.0 kPa)

    // Температура на масло (°C, със знак)
    int16_t oil_temp_raw = (rx_message.data[2] << 8) | rx_message.data[3];
    oil_temp = oil_temp_raw / 10;  // Дали е необходимо деление зависи от ECU!

    // Налягане на бензин (0-1000 kPa)
    uint16_t fuel_p_raw = (rx_message.data[4] << 8) | rx_message.data[5];
    fuel_p = fuel_p_raw / 10.0f;

    // Изпращане към Nextion (същите променливи като в EMU)
    if (oil_temp != last_oil_temp) {
        tempBuffer += "x6.val=" + String(oil_temp) + "\xFF\xFF\xFF";
        last_oil_temp = oil_temp;
        needsUpdate = true;
    }

    if (abs(oil_p - last_oil_p) > 0.1f) {  // Праг за промяна 0.1 kPa
        tempBuffer += "x7.val=" + String(int(oil_p * 100)) + "\xFF\xFF\xFF";  // x100 за цяло число (5000 вместо 50.00)
        last_oil_p = oil_p;
        needsUpdate = true;
    }

    if (abs(fuel_p - last_fuel_p) > 0.1f) {
        tempBuffer += "x8.val=" + String(int(fuel_p * 100)) + "\xFF\xFF\xFF";  // Аналогично на EMU
        last_fuel_p = fuel_p;
        needsUpdate = true;
    }
    break;

        }

        case 1553: {  // Gear position from Megasquirt
    static int8_t last_gear_msq = -128;  // Извън валиден диапазон за инициализация
    int8_t gear = (int8_t)rx_message.data[6];

    if (gear != last_gear_msq) {
        String gearText;
        String gearColorCmd;

        if (gear == 0) {
            gearText = "N";
            gearColorCmd = "t0.pco=2016";  // Зелен
        } else if (gear < 0) {
            gearText = "R";
            gearColorCmd = "t0.pco=63488"; // Червен
        } else {
            gearText = String(gear);
            gearColorCmd = "t0.pco=65535"; // Бял
        }

        tempBuffer += "t0.txt=\"" + gearText + "\"\xFF\xFF\xFF";
        tempBuffer += gearColorCmd + "\xFF\xFF\xFF";
        last_gear_msq = gear;
        needsUpdate = true;
    }

    break;
}


case 1542: {  // EGT - Big Endian, байтове 0 и 1
    uint16_t egt_raw = (rx_message.data[0] << 8) | rx_message.data[1];

    tempBuffer += "t16.txt=\"" + String(egt_raw) + "\"\xFF\xFF\xFF";
    needsUpdate = true;
    break;
}


        case 1562: {  // VSS данни от Megasquirt
    if (nextion_n14 == 2 && nextion_n13 == ECU_MEGASQUIRT) {
        // Четене на VSS (16-bit, offset 0)
        megasquirtVssSpeed = ((rx_message.data[1] << 8) | rx_message.data[0]);
        
        // Изпращане към дисплея
        tempBuffer += "v3.txt=\"" + String(int(megasquirtVssSpeed)) + "\"\xFF\xFF\xFF";
        
        // Изпращане по CAN
        sendCAN_Speed(megasquirtVssSpeed);
        
        needsUpdate = true;
    }
    break;
}
    }
}

void processMaxxecuMessage(twai_message_t &rx_message, String &tempBuffer, bool &needsUpdate) {
    switch (rx_message.identifier) {
        case 0x520: {  // RPM, TPS, MAP, AFR
            // RPM (int16, offset 0, scale 1)
            int16_t rpm = (rx_message.data[1] << 8) | rx_message.data[0];
            rpm_g = rpm;
            
            // TPS (int16, offset 2, scale 0.1)
            int16_t tps_raw = (rx_message.data[3] << 8) | rx_message.data[2];
            tps_g = tps_raw * 0.1f;
            
            // MAP (int16, offset 4, scale 0.1)
            int16_t map_raw = (rx_message.data[5] << 8) | rx_message.data[4];
            map_g = map_raw * 0.1f;
            
            // AFR (int16, offset 6, scale 0.001) - умножаваме по 14.7
            int16_t afr_raw = (rx_message.data[7] << 8) | rx_message.data[6];
            afr_g = afr_raw * 0.01f * 14.7f;

            // RPM обработка
            if (abs(rpm_g - lastRpmValue) > 1) {
                int angle = map(rpm_g, 0, 9000, 0, 270);
                angle = constrain(angle, 0, 270);
                tempBuffer += "z0.val=" + String(angle) + "\xFF\xFF\xFF";
                lastRpmValue = rpm_g;
                needsUpdate = true;
            }

            // TPS (0-100%)
            if (abs(tps_g - last_tps_display) > 0.5f) {
                tempBuffer += "x5.val=" + String(static_cast<int>(tps_g)) + "\xFF\xFF\xFF";
                last_tps_display = tps_g;
                needsUpdate = true;
            }

if (abs(map_g - last_map_g) > 0.5f) {
    float vacuum = map_g - 100.0f;  // Изваждаме 100 за вакуум
    int vacuum_int = static_cast<int>(round(vacuum));  // Закръгляме до цяло число
    
    tempBuffer += "x0.val=" + String(vacuum_int) + "\xFF\xFF\xFF";  // Изпращане като цяло число
    last_map_g = map_g;
    needsUpdate = true;

      int map_angle = map(map_g, 0, 300, 0, 270);  // ако map_g е 0–400
    map_angle = constrain(map_angle, 0, 270);
    tempBuffer += "z1.val=" + String(map_angle) + "\xFF\xFF\xFF";

   
}

         // AFR обработка (x4.val е xfloat)
if (abs(afr_g - last_afr_g) > 0.1f) {
    // Вариант 1: Закръгляне до цяло число (ако Nextion очаква int)
    tempBuffer += "x4.val=" + String(static_cast<int>(round(afr_g))) + "\xFF\xFF\xFF";
    
    // ИЛИ Вариант 2: С 1 десетичен знак (ако Nextion очаква float)
    // tempBuffer += "x4.val=" + String(afr_g, 1) + "\xFF\xFF\xFF";
    
    last_afr_g = afr_g;
    needsUpdate = true;

}

            updateShiftLight(rpm_g);
            break;
        }
        case 0x521: {  // Нов случай за ignang от MaxxECU
    // Четене на ignang от байт 4 (int16 Little Endian)
    int16_t ignang_raw = (rx_message.data[5] << 8) | rx_message.data[4];
    float ignang = ignang_raw * 0.1f;  // Прилагаме резолюция 0.1

    // Изпращане към Nextion дисплея
    tempBuffer += "t15.txt=\"" + String(ignang, 1) + "\"\xFF\xFF\xFF";
    needsUpdate = true;

    break;
}


        case 0x522: {  // Нов случай за VSS от MaxxECU
    if (nextion_n14 == 2) {  // Ако сме във VSS режим
        // Четене на VSS (int16, offset 6, scale 0.1)
        int16_t rawVss = (rx_message.data[7] << 8) | rx_message.data[6];
        maxxecuVssSpeed = rawVss ;

        // Изпращане към дисплея
             int speed_int = static_cast<int>(round(maxxecuVssSpeed));
        tempBuffer += "v3.txt=\"" + String(speed_int) + "\"\xFF\xFF\xFF";

        // Изпращане по CAN (ако е необходимо)
        sendCAN_Speed(maxxecuVssSpeed);

        needsUpdate = true;
   }

    // --- INJPW (int16, offset 0, scale 0.01) ---
    int16_t injpw_raw = (rx_message.data[1] << 8) | rx_message.data[0];
    float injpw = injpw_raw * 0.001f;

    tempBuffer += "t14.txt=\"" + String(injpw, 1) + "\"\xFF\xFF\xFF";
    needsUpdate = true;
         
    break;
}


        case 0x530: {  // Battery, MAT, CLT
            // Battery Voltage (int16, offset 0, scale 0.01)
            int16_t batt_raw = (rx_message.data[1] << 8) | rx_message.data[0];
            batteryVoltage = batt_raw * 0.1f;
            
            // MAT (int16, offset 4, scale 0.1)
            int16_t mat_raw = (rx_message.data[5] << 8) | rx_message.data[4];
            mat_g = mat_raw * 0.1f;
            
            // CLT (int16, offset 6, scale 0.1)
            int16_t clt_raw = (rx_message.data[7] << 8) | rx_message.data[6];
            clt_g = clt_raw * 0.1f;

            // Battery
             if (batteryVoltage != last_batteryVoltage) {
                tempBuffer += "x1.val=" + String(batteryVoltage) + "\xFF\xFF\xFF";
                last_batteryVoltage = batteryVoltage;
                needsUpdate = true;
             

           
                tempBuffer += "x3.val=" + String(mat_g, 1) + "\xFF\xFF\xFF";
                last_mat_g = mat_g;
                needsUpdate = true;
            

           
                tempBuffer += "x2.val=" + String(clt_g, 1) + "\xFF\xFF\xFF";
                last_clt_g = clt_g;
                needsUpdate = true;
            }
            break;
        }

        case 0x531: {  // Нов случай за EGT от MaxxECU
    // Четене на EGT от байт 6 (int16 Little Endian)
    int16_t egt_raw = (rx_message.data[7] << 8) | rx_message.data[6];
    int egt_value = egt_raw * 1;  // Резолюция 1 (можеш да премахнеш *1, ако няма смисъл)

    // Изпращане към Nextion дисплея
    tempBuffer += "t16.txt=\"" + String(egt_value) + "\"\xFF\xFF\xFF";
    needsUpdate = true;

    break;
}


        case 0x536: {  // Gear, Oil Pressure, Oil Temp
            // Gear (int16, offset 0, scale 1)
            int16_t gear_raw = (rx_message.data[1] << 8) | rx_message.data[0];
            uint8_t gear = gear_raw;
            
            // Oil Pressure (int16, offset 4, scale 0.1)
            int16_t oil_p_raw = (rx_message.data[5] << 8) | rx_message.data[4];
            oil_p = oil_p_raw * 0.1f;
            
            // Oil Temp (int16, offset 6, scale 0.1)
            int16_t oil_temp_raw = (rx_message.data[7] << 8) | rx_message.data[6];
            oil_temp = oil_temp_raw * 0.1f;

            // Gear
            if (gear != last_gear) {
                String gearText;
                String gearColorCmd;
                if (gear == 0) {
                    gearText = "N";
                    gearColorCmd = "t0.pco=2016";  // Yellow
                } else if (gear == 255) {
                    gearText = "R";
                    gearColorCmd = "t0.pco=63488"; // Red
                } else {
                    gearText = String(gear);
                    gearColorCmd = "t0.pco=65535";  // White
                }
                tempBuffer += "t0.txt=\"" + gearText + "\"\xFF\xFF\xFF";
                tempBuffer += gearColorCmd + "\xFF\xFF\xFF";
                last_gear = gear;
                needsUpdate = true;
            }

            if (oil_temp != last_oil_temp) {
                    tempBuffer += "x6.val=" + String(oil_temp) + "\xFF\xFF\xFF";
                    last_oil_temp = oil_temp;
                    needsUpdate = true;
                }

                if (abs(oil_p - last_oil_p) > 0.01f) {
                    tempBuffer += "x7.val=" + String((int)(oil_p * 100)) + "\xFF\xFF\xFF";
                    last_oil_p = oil_p;
                    needsUpdate = true;
                }

            // Oil Temp
            if (abs(oil_temp - last_oil_temp) > 0.5f) {
                tempBuffer += "x6.val=" + String(oil_temp, 1) + "\xFF\xFF\xFF";
                last_oil_temp = oil_temp;
                needsUpdate = true;
            }
            break;
        }

        case 0x537: {  // Fuel Pressure
            // Fuel Pressure (int16, offset 0, scale 0.1)
            int16_t fuel_p_raw = (rx_message.data[1] << 8) | rx_message.data[0];
            fuel_p = fuel_p_raw * 0.1f;

               if (abs(fuel_p - last_fuel_p) > 0.01f) {
                    tempBuffer += "x8.val=" + String((int)(fuel_p)) + "\xFF\xFF\xFF";
                    last_fuel_p = fuel_p;
                    needsUpdate = true;
                }
            break;
        }
    }
}

void processLinkMessage(twai_message_t &rx_message, String &tempBuffer, bool &needsUpdate) {
    // AEM - ID: 0x01F0A000 (RPM, TPS, IAT, CLT)
    if (rx_message.identifier == 0x01F0A000) {
        uint16_t raw_rpm = (rx_message.data[0] << 8) | rx_message.data[1];
        float rpm = raw_rpm * 0.39063f;
        rpm_g = rpm;

        if (abs(rpm_g - lastRpmValue) > 1) {
            int angle = map(rpm_g, 0, 9000, 0, 270);
            angle = constrain(angle, 0, 270);
            tempBuffer += "z0.val=" + String(angle) + "\xFF\xFF\xFF";
            lastRpmValue = rpm_g;
            needsUpdate = true;
        }

        uint16_t raw_tps = (rx_message.data[4] << 8) | rx_message.data[5];
        float tps = raw_tps * 0.0015259f;
        tps_g = tps;

        if (abs(tps_g - last_tps_display) > 0.5f) {
            tempBuffer += "x5.val=" + String(static_cast<int>(tps_g)) + "\xFF\xFF\xFF";
            last_tps_display = tps_g;
            needsUpdate = true;
        }

        int8_t iat = static_cast<int8_t>(rx_message.data[6]);
        mat_g = iat;

        if (abs(mat_g - last_mat_g) > 0.5f) {
            tempBuffer += "x3.val=" + String(mat_g) + "\xFF\xFF\xFF";
            last_mat_g = mat_g;
            needsUpdate = true;
        }

        int8_t clt = static_cast<int8_t>(rx_message.data[7]);
        clt_g = clt;

        if (abs(clt_g - last_clt_g) > 0.5f) {
            tempBuffer += "x2.val=" + String(clt_g) + "\xFF\xFF\xFF";
            last_clt_g = clt_g;
            needsUpdate = true;
        }

        updateShiftLight(rpm_g);
    }

    // AEM - ID: 0x01F0A003 (Lambda, VSS, Gear, Ignition Timing, Battery Voltage)
    if (rx_message.identifier == 0x01F0A003) {
        uint8_t raw_lambda = rx_message.data[0];
        float afr = (raw_lambda * 0.00390625f + 0.5f) * 147.0f;
        afr_g = afr;

        if (abs(afr_g - last_afr_g) > 0.1f) {
            tempBuffer += "x4.val=" + String(static_cast<int>(round(afr_g))) + "\xFF\xFF\xFF";
            last_afr_g = afr_g;
            needsUpdate = true;
        }

        uint16_t raw_vss = (rx_message.data[2] << 8) | rx_message.data[3];
        float vss = raw_vss * 0.0062865f;
        maxxecuVssSpeed = vss;

        if (nextion_n14 == 2) {
            int speed_int = static_cast<int>(round(maxxecuVssSpeed));
            tempBuffer += "v3.txt=\"" + String(speed_int) + "\"\xFF\xFF\xFF";
            sendCAN_Speed(maxxecuVssSpeed);
            needsUpdate = true;
        }

        uint8_t gear = rx_message.data[4];
        if (gear != last_gear) {
            String gearText, gearColorCmd;
            if (gear == 0) {
                gearText = "N";
                gearColorCmd = "t0.pco=2016";
            } else if (gear == 255) {
                gearText = "R";
                gearColorCmd = "t0.pco=63488";
            } else {
                gearText = String(gear);
                gearColorCmd = "t0.pco=65535";
            }
            tempBuffer += "t0.txt=\"" + gearText + "\"\xFF\xFF\xFF";
            tempBuffer += gearColorCmd + "\xFF\xFF\xFF";
            last_gear = gear;
            needsUpdate = true;
        }

        uint8_t raw_ign = rx_message.data[5];
        float ignang = raw_ign * 0.35156f - 17.0f;
        tempBuffer += "t15.txt=\"" + String(ignang, 1) + "\"\xFF\xFF\xFF";
        needsUpdate = true;

        uint16_t raw_batt = (rx_message.data[6] << 8) | rx_message.data[7];
        float voltage = raw_batt * 0.002455f;
        batteryVoltage = voltage;

   if (batteryVoltage != last_batteryVoltage) {
                tempBuffer += "x1.val=" + String(batteryVoltage) + "\xFF\xFF\xFF";
                last_batteryVoltage = batteryVoltage;
                needsUpdate = true;
}
    }

// AEM - ID: 0x01F0A004 (MAP, Fuel Pressure, Oil Pressure)
    if (rx_message.identifier == 0x01F0A004) {
        uint16_t raw_map = (rx_message.data[0] << 8) | rx_message.data[1];
        map_g = (raw_map * 0.1f);

        if (abs(map_g - last_map_g) > 0.5f) {
            int vacuum_int = static_cast<int>(round(map_g - 100.0f));
            tempBuffer += "x0.val=" + String(vacuum_int) + "\xFF\xFF\xFF";
            last_map_g = map_g;
            needsUpdate = true;

                int map_angle = map(map_g, 0, 300, 0, 270);  // ако map_g е 0–400
                map_angle = constrain(map_angle, 0, 270);
                tempBuffer += "z1.val=" + String(map_angle) + "\xFF\xFF\xFF";
        }

        uint8_t raw_fp = rx_message.data[3];
        fuel_p = ((raw_fp * 0.580151f) * 6.8947);

        if (abs(fuel_p - last_fuel_p) > 0.1f) {
            tempBuffer += "x8.val=" + String(static_cast<int>(fuel_p)) + "\xFF\xFF\xFF";
            last_fuel_p = fuel_p;
            needsUpdate = true;
        }

        uint8_t raw_op = rx_message.data[4];
        oil_p = (((raw_op * 0.580151f) * 0.68947)) * 0.1;
        

       if (abs(oil_p - last_oil_p) > 0.01f) {
                    tempBuffer += "x7.val=" + String((int)(oil_p * 100)) + "\xFF\xFF\xFF";
                    last_oil_p = oil_p;
                    needsUpdate = true;
        }
    }




    // AEM - ID: 0x01F0A006 (Injector Pulse Width)
    if (rx_message.identifier == 0x01F0A006) {
        uint8_t inj_raw = rx_message.data[0];
        float inj = inj_raw * 0.1f;
        tempBuffer += "t14.txt=\"" + String(inj, 1) + "\"\xFF\xFF\xFF";
        needsUpdate = true;
    }



     // AEM - ID: 0x01F0A007 (Oil Temperature)
    if (rx_message.identifier == 0x01F0A007) {
        int8_t raw_temp = static_cast<int8_t>(rx_message.data[6]);
        oil_temp = raw_temp - 50;

        if (abs(oil_temp - last_oil_temp) > 0.5f) {
            tempBuffer += "x6.val=" + String(oil_temp, 1) + "\xFF\xFF\xFF";
            last_oil_temp = oil_temp;
            needsUpdate = true;
 }

 // Flex Content (byte 4)
    uint8_t raw_flex = rx_message.data[4];
    float flex = raw_flex * 0.392157f;
    flex_g = flex;

    if (abs(flex - flex_g) > 0.5f) {
        tempBuffer += "x9.val=" + String(flex, 1) + "\xFF\xFF\xFF";
        flex_g = flex;
        needsUpdate = true;
    }
    }
}







void setupCANBasedOnN15(uint16_t n15_value) {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config;

  if (n15_value == 1) {
    t_config = TWAI_TIMING_CONFIG_500KBITS();  // 500 kbps
    Serial.println("[CAN] Скорост: 500 kbps");
  } else if (n15_value == 2) {
    t_config = TWAI_TIMING_CONFIG_1MBITS();    // 1 Mbps
    Serial.println("[CAN] Скорост: 1 Mbps");
  } else {
    t_config = TWAI_TIMING_CONFIG_500KBITS();  // Default
    Serial.println("[CAN] Неизвестна стойност, по подразбиране 500 kbps");
  }

  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("[CAN] Драйвер инсталиран");
    if (twai_start() == ESP_OK) {
      Serial.println("[CAN] Стартиран успешно");
    } else {
      Serial.println("[CAN] Грешка при стартиране");
    }
  } else {
    Serial.println("[CAN] Грешка при инсталация");
  }
}

float last_flex_content = -1.0f;  // Последна известна стойност за Flex Content



void setup() {
    Serial.begin(115200);
    Serial2.begin(921600, SERIAL_8N1, NEXTION_RX_PIN, NEXTION_TX_PIN, false, 2048);
    Serial2.setRxBufferSize(2048);
    Serial2.setTxBufferSize(2048);
    
    // Инициализация на GPS
    SerialGPS.begin(230400, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    gps.begin(&SerialGPS);
    
    EEPROM.begin(EEPROM_SIZE);


     n0_value = loadN0FromEEPROM();
    nextion_x11 = loadX11FromEEPROM(); // Зареждане на запазена стойност
    nextion_n14 = loadN14FromEEPROM();
     nextion_n15 = loadN15FromEEPROM();
     updateT10Text(nextion_n14);
     nextion_n12 = loadN12FromEEPROM();
     nextion_n13 = loadEcuSelectionFromEEPROM(); // Add this line
     currentX11 = loadX11FromEEPROM();


    int savedValue = EEPROM.read(BRIGHTNESS_ADDR);
    savedValue = constrain(savedValue, 0, 100);
    brightness = map(savedValue, 0, 100, 50, 255);
      uint8_t currentEcu = loadEcuSelectionFromEEPROM();
    Serial.printf("Loaded ECU selection from EEPROM: %d\n", currentEcu);

    delay(20);

    setupCANBasedOnN15(nextion_n15);
    
    strip27.begin();
    strip27.clear();
    strip27.setBrightness(brightness);
    strip27.show();

    // Инициализация на филтъра за скорост
     for (int i = 0; i < GPS_SPEED_FILTER_SIZE; i++) {
        gpsSpeedHistory[i] = 0.0;
    }

    // CAN настройки
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t timing_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&general_config, &timing_config, &filter_config) == ESP_OK && 
        twai_start() == ESP_OK) {
        Serial.println("CAN initialized successfully");
    }

    // Nextion инициализация
    Serial2.write("\xFF\xFF\xFF");
    delay(100);
    Serial2.print("rest");
    Serial2.write("\xFF\xFF\xFF");
    delay(3000);
}



void loop() {
    static uint32_t lastLoopTime = 0;
    const uint32_t LOOP_DELAY = 1;

    processCANMessages();
    processNextionSerial();
    handleNextionData();
    processGPSData();
    
    // Автоматичен refresh на всеки 5 секунди
    unsigned long currentMillis = millis();
    if (currentMillis - previousRefreshTime >= refreshInterval) {
        refreshAllValues();
        previousRefreshTime = currentMillis;
    }
    checkAndSendN12();
    flushNextionBuffer();

    if (millis() - lastLoopTime < LOOP_DELAY) {
        delay(LOOP_DELAY - (millis() - lastLoopTime));
    }
    lastLoopTime = millis();

    taskYIELD();
}