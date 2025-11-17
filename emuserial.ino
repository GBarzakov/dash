#include <Arduino.h>
#include <EMUSerial.h>
#include <driver/twai.h>

EMUSerial emu(Serial1);

// Филтър параметри
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_18

#define FILTER_SIZE 2
float voltageBuffer[FILTER_SIZE] = {12, 12};
int voltageIndex = 0;
uint32_t txFailCount = 0;

float filteredVoltage(float newVoltage) {
    voltageBuffer[voltageIndex] = newVoltage;
    voltageIndex = (voltageIndex + 1) % FILTER_SIZE;

    float sum = 0;
    int count = 0;
    for (int i = 0; i < FILTER_SIZE; i++) {
        if (voltageBuffer[i] > 5 && voltageBuffer[i] < 16) {
            sum += voltageBuffer[i];
            count++;
        }
    }
    if (count > 0) return sum / count;
    return voltageBuffer[(voltageIndex + FILTER_SIZE - 1) % FILTER_SIZE];
}

void printEMUData() {
    const emu_data_t& d = emu.emu_data;
    float voltage = filteredVoltage(d.Batt);

    Serial.println("------ EMU Data ------");
    Serial.printf("RPM: %u\n", d.RPM);
    Serial.printf("MAP: %u kPa\n", d.MAP);
    Serial.printf("TPS: %u %%\n", d.TPS);
    Serial.printf("IAT: %d C\n", d.IAT);
    Serial.printf("Battery: %.2f V (filtered: %.2f V)\n", d.Batt, voltage);
    Serial.printf("Ign Angle: %.2f deg\n", d.IgnAngle);
    Serial.printf("PulseWidth: %.2f ms\n", d.pulseWidth);
    Serial.printf("Sec PulseWidth: %.2f ms\n", d.scondarypulseWidth);
    Serial.printf("EGT1: %u C\n", d.Egt1);
    Serial.printf("AFR: %.2f\n", d.wboAFR);
    Serial.printf("Gear: %d\n", d.gear);
    Serial.printf("Oil Temp: %u C\n", d.oilTemperature);
    Serial.printf("Oil Pressure: %.2f Bar\n", d.oilPressure);
    Serial.printf("Fuel Pressure: %.2f Bar\n", d.fuelPressure);
    Serial.printf("CLT: %d C\n", d.CLT);
    Serial.printf("Flex Ethanol: %.2f %%\n", d.flexFuelEthanolContent);
    Serial.printf("VSS: %.2f km/h\n", d.vssSpeed);
    Serial.println("------------------------");
}

void setupCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("CAN Driver installed");
    } else {
        Serial.println("Failed to install CAN driver");
        while(1);
    }

    if (twai_start() == ESP_OK) {
        Serial.println("CAN started");
    } else {
        Serial.println("Failed to start CAN");
        while(1);
    }
}


void sendCanMessage(uint32_t id, uint8_t *data, uint8_t len) {
    twai_message_t message = {};
    message.identifier = id;
    message.extd = 0;
    message.rtr = 0;
    message.data_length_code = len;
    memcpy(message.data, data, len);

    esp_err_t res = twai_transmit(&message, pdMS_TO_TICKS(10));

  if (res != ESP_OK) {
    txFailCount++;

    if (txFailCount % 20 == 0) {
        Serial.printf("CAN TX FAIL (count=%u, err=%d)\n", txFailCount, res);

        twai_status_info_t info;
        if (twai_get_status_info(&info) == ESP_OK) {

            Serial.printf(
                "TWAI state=%d, tx_err=%u, rx_err=%u, tx_q=%u, rx_q=%u\n",
                info.state,
                info.tx_error_counter,
                info.rx_error_counter,
                info.msgs_to_tx,
                info.msgs_to_rx
            );

            // ---- ТУК СЛАГАШ RECOVERY ----
            if (info.tx_error_counter >= 120) {
                Serial.println("FORCING TWAI RECOVERY (stop/start)");
                twai_stop();
                delay(10);
                twai_start();
            }
            // --------------------------------
        }
    }
}
}

void sendEmuDataOverCAN(const emu_data_t& d) {
    // ID 1536 - RPM, TPS, IAT, MAP, INJPW
    uint8_t data1536[8];
    uint16_t rpm = d.RPM;
    data1536[0] = rpm & 0xFF;
    data1536[1] = rpm >> 8;
    data1536[2] = d.TPS * 2;  
    data1536[3] = (int8_t)d.IAT;
    uint16_t map = d.MAP;
    data1536[4] = map & 0xFF;
    data1536[5] = map >> 8;
    uint16_t injpw_raw = d.pulseWidth / 0.016129f;
    data1536[6] = injpw_raw & 0xFF;
    data1536[7] = injpw_raw >> 8;
    sendCanMessage(1536, data1536, 8);

    // ID 1538 - VSS, Oil Temp, Oil P, Fuel P, CLT
    uint8_t data1538[8];
    uint16_t vss = d.vssSpeed;
    data1538[0] = vss & 0xFF;
    data1538[1] = vss >> 8;
    data1538[2] = 0; // padding
    data1538[3] = d.oilTemperature;
    data1538[4] = d.oilPressure / 0.0625f;
    data1538[5] = d.fuelPressure / 0.0625f;
    int16_t clt = d.CLT;
    data1538[6] = clt & 0xFF;
    data1538[7] = clt >> 8;
    sendCanMessage(1538, data1538, 8);

    // ID 1539 - IgnAng, AFR, padding, EGT
    uint8_t data1539[6];
    data1539[0] = (int8_t)(d.IgnAngle / 0.5f);
    data1539[1] = 0;
    data1539[2] = d.wboAFR / 14.7f / 0.0078125f;
    
    uint16_t egt = d.Egt1;
    data1539[3] = 0;  // reserved
    data1539[4] = egt & 0xFF;
    data1539[5] = egt >> 8;
    sendCanMessage(1539, data1539, 6);

    // ID 1540 - Gear, Battery, Byte7
  // ID 1540 - Gear, Battery, FlexFuel
uint8_t data1540[8] = {0};
data1540[0] = d.gear;
uint16_t batteryRaw = filteredVoltage(d.Batt) / 0.027f;
data1540[2] = batteryRaw & 0xFF;
data1540[3] = batteryRaw >> 8;
data1540[7] = (uint8_t)d.flexFuelEthanolContent;   // Flex fuel %
sendCanMessage(1540, data1540, 8);

// ID 1545 - Fuel Level (байт 6) — 0..100%
    uint8_t data1545[8] = {0};
    uint8_t fuelLevel = d.fuelLevel;
    if (fuelLevel > 100) fuelLevel = 100; // safety clamp
    data1545[6] = fuelLevel;  // точно където приемникът чете rx_message.data[6]
    sendCanMessage(1545, data1545, 8);


}


void setup() {
    Serial.begin(115200);
    Serial1.begin(19200, SERIAL_8N1, 34, -1);  // RX = GPIO34
    Serial.println("ECUMaster Serial Reader started");
    setupCAN();
}

void loop() {
    emu.checkEmuSerial();
    const emu_data_t& d = emu.emu_data;

    sendEmuDataOverCAN(d);  
    delay(35);              
}

