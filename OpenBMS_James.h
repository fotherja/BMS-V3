/*
* Description :   Software for a 16S BMS based on the BQ76952 and an ESP32. 
* Author      :   James Fotherby
* Date        :   08/04/2025
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*/

#ifndef OpenBMS_James_h
#define OpenBMS_James_h

#include "Arduino.h"

#define NUM_OF_CELLS        16

// Example patterns
enum LedMode : uint8_t {
  LED_OFF,
  LED_ON,
  LED_BLINK_SLOW,   // ~1 Hz
  LED_BLINK_FAST,   // ~5 Hz
  LED_BLINK_BALANCE, // e.g., 4 Hz while balancing
  LED_BLINK_COV,
  LED_BLINK_CUV,
  LED_BLINK_ALIVE
};

struct LedState {
  uint8_t pin;
  LedMode mode;
  bool level;
  uint32_t next_ms;
  uint16_t on_ms;
  uint16_t off_ms;
};

constexpr size_t LOG_LINE_MAX = 256;

struct LogMsg {
  uint16_t len;
  char text[LOG_LINE_MAX];
};



// Function prototypes:
void VEcan(void);
esp_err_t SendCAN_Packet(uint16_t Address, uint8_t Msg_Length, uint8_t * Msg);
void maintainConnections(void);
void callback(char* topic, byte* payload, unsigned int length);

void Task_CANHandle(void *pvParameters);
void Task_BMSHandle(void *pvParameters);
void Task_MQTTHandle(void *pvParameters);
void Task_LEDHandle(void *pvParameters);
void Task_ComsHandle(void *pvParameters);

#endif