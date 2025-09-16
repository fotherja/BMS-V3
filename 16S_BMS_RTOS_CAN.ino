/*
* Description :   Software for a 16S BMS based on the BQ76952 and an ESP32. 
* Author      :   James Fotherby
* Date        :   08/04/2025
* License     :   MIT
* This code is published as open source software. Feel free to share/modify.
*
* To Do:
*   - ODP programme chip
*   - Check mqtt current is coming through 
*   - Get rid of balance permission and just base it off current
*
*
* Future:
*   - Try serial using the optocouplers - 2400baud works well - Idea for this would be to have a mode which cancels the 2-signal BMS and instead uses a port to TX cell data to the next BMS
*  
*/
#include <BQ76952.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>  

#include "OpenBMS_James.h"
#include "driver/twai.h"
#include "esp_log.h"
#include <freertos/event_groups.h>
#include <freertos/queue.h>
#include "DVCC.h"
#include <esp_task_wdt.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//--------------------------------------------------
#define TWAI_DATA_LENGTH_CODE_MAX   8
#define CAN_TX_QUEUE_TIMEOUT_MS     2                               // wait to enqueue into TX queue
#define CAN_TX_RESULT_TIMEOUT_MS    100                             // wait for success/fail alert after enqueue

//------------- Pin definitions --------------------
#define LED_GREEN_PIN       21  //0
#define LED_RED_PIN         22  //1

#define ALERT               1

#define CAN_RX              18  //2
#define CAN_TX              20  //3
#define CAN_STB             19
#define CAN_EN              2

#define BUTTON_LOW_PIN      9
#define I2C_SDA_PIN         8
#define I2C_SCL_PIN         9                                       // Pulling to ground and reseting device enters BOOT mode
#define BOOT_BTN_PIN        I2C_SCL_PIN

#define AUX_1_PIN           10                                      // ESP - TX1 
#define AUX_2_PIN           11  //12                                // ESP - RX1
#define AUX_3_PIN           13  //11                                // ESP - TX2
#define AUX_4_PIN           12                                      // ESP - TX2

#define UART_TX             16
#define UART_RX             17

#define CUOV_LATCH_TIME      60000

// ---- Config UUIDs (pick your own if you like) ----
static const char* CELL_SERVICE_UUID = "df576e4d-c394-49b8-a13b-7b55b86439a7";
static const char* CELL_ARRAY_UUID   = "fc36e5ed-83ba-47f3-970c-233dfcff4ac8";

//------------- Variables --------------------
// Your credentials
const char*               ssid = "venus-HQ2306ZHQGJ-f3b";         
const char*               password = "zz6tzefc";                 
const char*               mqtt_server = "192.168.188.122";

BQ76952                   bms;
WiFiClient                espClient;
PubSubClient              client(espClient);
HardwareSerial            HWSerial(1);                               // use UART1

BLEServer*                gServer      = nullptr;
BLECharacteristic*        gCellChar    = nullptr;

volatile bool             gDeviceConnected     = false;
volatile bool             gOldDeviceConnected  = false;

EventGroupHandle_t        eg;
constexpr EventBits_t     BALANCING_BIT = BIT0;
constexpr EventBits_t     COV_BIT       = BIT4;
constexpr EventBits_t     CUV_BIT       = BIT5;

QueueHandle_t             logQ;

LedState                  ledG{LED_GREEN_PIN, LED_BLINK_SLOW, false, 0, 100, 900};
LedState                  ledR{LED_RED_PIN,   LED_OFF,        false, 0, 0,   0};

uint16_t                  Cell_Voltages[NUM_OF_CELLS];
uint32_t                  Cell_Balance_Times[NUM_OF_CELLS];

uint32_t                  BatteryVoltageSum_mV = 51200;
uint32_t                  StackVoltage_mV = 51200;

uint16_t                  CVL_Vx10, CCL_Ax10, DCL_Ax10;
Limits                    lim;

float                     PackTemperature = 20.5;
uint16_t                  SOC = 80, currentact = 0; 

// ####################################################################################################################
// --------------------------------------------------------------------------------------------------------------------
// ####################################################################################################################
void setup() {
  delay(1000);
  Serial.begin(115200);
  bms.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  bms.reset();
  delay(100); 

  // Configure the BQ76952:
  bms.setConnectedCells(NUM_OF_CELLS);  

  bms.writeByteToMemory(CUV_Threshold, 55);                         // Set Cell undervoltage protection to 2.78 volts (units: 50.6 mV)
  bms.writeByteToMemory(COV_Threshold, 71);                         // Set Cell overvoltage protection to 3.54 volts 
  bms.writeIntToMemory(CUV_Delay, 1513);                            // 5 Second delay before a CUV/COV triggers a fault. Default 100 mV hysterisis
  bms.writeIntToMemory(COV_Delay, 1513);

  bms.writeByteToMemory(Enabled_Protections_A, 0b00001100);         // Enable CUV, COV circuit protection
  bms.writeByteToMemory(Enabled_Protections_B, 0b01000000);         // Enable internal overtemp protection 80-65 celcius

  bms.writeIntToMemory(Cell_Balance_Min_Cell_V_Relaxed, 3350);      // Only balance when cells above 3350 mV (Top balance)
  bms.writeByteToMemory(Cell_Balance_Max_Cells, 3);                 // Allows up to 3 simultaneous cells to be balanced
  bms.writeByteToMemory(DA_Configuration, 0b00001101);              // Set chip die to be for cell temp measurment
  bms.writeByteToMemory(Cell_Open_Wire_Check_Time, 250);            // Set to 250 seconds to reduce unbalanced cell currents  

  bms.writeByteToMemory(TS1_Config, 0b00001011);                    // Configures TS1 as thermistor temperature measurement, reported but not used for protections
  bms.writeByteToMemory(Balancing_Configuration, 0b00000011);       // Enables autonomous balancing
  bms.writeIntToMemory(Power_Config, 0b0010100010110010);           // Disables SLEEP, slow measurement speed when balancing to 1/8th

  bms.writeByteToMemory(FET_Options, 0b00000000);                   // FETs will not be turned on
  bms.writeByteToMemory(Chg_Pump_Control, 0b00000000);              // Disable charge pump as not needed
  bms.writeIntToMemory(Mfg_Status_Init, 0x0000);                    // The BQ doesn't control any FETs.

  // Create Event groups
  eg = xEventGroupCreate();

  // Create Queue
  logQ = xQueueCreate(64, sizeof(LogMsg));  // tune depth for bursts

  // Create and start tasks
  xTaskCreate(Task_ComsHandle, "Task Coms", 8192, NULL, 2, NULL);  
  xTaskCreate(Task_LEDHandle, "Task LEDs", 4096, NULL, 2, NULL);
  xTaskCreate(Task_BMSHandle, "Task BMS", 4096, NULL, 2, NULL);    
  xTaskCreate(Task_CANHandle, "Task CAN", 4096, NULL, 2, NULL);  
  xTaskCreate(Task_MQTTHandle, "Task MQTT", 8192, NULL, 2, NULL);
  xTaskCreate(Task_BLEHandle, "Task BLE", 8192, NULL, 2, NULL);
  Serial.println("Tasks started");  
}

void loop() {
  vTaskDelete(NULL);   // kill the Arduino loop task
}

// ####################################################################################################################
// MQTT Callback function----------------------------------------------------------------------------------------------
// ####################################################################################################################
void callback(char* topic, byte* payload, unsigned int length)
{
  // Log topic + payload as text (bounded). Note: payload may be binary.
  const unsigned int show = (length > 200) ? 200 : length; // cap for logs
  logf("MQTT IN topic='%s' len=%u payload='%.*s'\n", topic, length, (int)show, (const char*)payload);

  if (strcmp(topic, "Balancing/permission") == 0) {
    if (length >= 2 && payload[0] == 'N' && payload[1] == 'o') {
      uint8_t data[2] = {0, 0};  // renamed to avoid shadowing 'payload'
      bms.subCommandWriteData(CB_ACTIVE_CELLS, data, sizeof(data));       // disables balancing for a balancing interval
    }
  }

  if (strcmp(topic, "battery/soc") == 0) {
    // Convert payload to string and ensure it's null-terminated
    char soc_str[16] = {0};  
    unsigned int copy_len = (length < sizeof(soc_str) - 1) ? length : (sizeof(soc_str) - 1);
    memcpy(soc_str, payload, copy_len);
    soc_str[copy_len] = '\0';

    // Convert to float
    float value = atof(soc_str);

    // clamp value between 0 and 100
    if (value >= 0.0 && value <= 100.0) {
      SOC = round(value);
      logf("battery/soc: %.1f%%\n", value); 
    } 
  }

  if (strcmp(topic, "battery/current") == 0) {
    // Convert payload to string and ensure it's null-terminated
    char I_str[16] = {0};  
    unsigned int copy_len = (length < sizeof(I_str) - 1) ? length : (sizeof(I_str) - 1);
    memcpy(I_str, payload, copy_len);
    I_str[copy_len] = '\0';

    // Convert to float
    float value = atof(I_str);
    currentact = round(value);
    logf("battery/current: %.1f%%\n", value); 
  }
}

// ####################################################################################################################
// Regularly call this to re-establish MQTT and WiFi connections if lost-----------------------------------------------
// ####################################################################################################################
void maintainConnections() {
  // 1) Ensure WiFi
  if (WiFi.status() != WL_CONNECTED) {
    logf("WiFi not connected. Attempting reconnect (ssid: %s)...\n", ssid);
    WiFi.begin(ssid, password);     // returns immediately
    return;                         // don't continue to MQTT until WiFi is back
  }

  // 2) One-time MQTT config when WiFi is up
  static bool mqttConfigured = false;
  if (!mqttConfigured) {
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
    mqttConfigured = true;
    logf("MQTT configured: server=%s:1883\n", mqtt_server);
  }

  // 3) Reconnect MQTT if needed
  if (!client.connected()) {
    logf("MQTT not connected. Attempting reconnect...\n");

    // Build a stable client ID once (helps brokers that reject duplicates)
    static bool idInit = false;
    static char clientId[32];
    if (!idInit) {
      uint32_t id = (uint32_t)ESP.getEfuseMac();
      snprintf(clientId, sizeof(clientId), "ESP32Client%08X", (unsigned)id);
      idInit = true;
    }

    if (client.connect(clientId)) {
      logf("MQTT connected as %s\n", clientId);
      client.subscribe("Balancing/permission");
      logf("MQTT subscribed: %s\n", "Balancing/permission");
      client.subscribe("battery/soc");
      logf("MQTT subscribed: %s\n", "battery/soc");
      client.subscribe("battery/current");
      logf("MQTT subscribed: %s\n", "battery/current");
    } else {
      logf("MQTT connect failed. state=%d\n", client.state());
      // Non-blocking; will try again on next maintainConnections()
    }
  }
}

// ####################################################################################################################
// Send a CAN packet and keep tally of errors/successful transmissions-------------------------------------------------
// ####################################################################################################################
esp_err_t SendCAN_Packet(uint16_t Address, uint8_t Msg_Length, uint8_t * Msg)
{
  static uint16_t can_tx_ok = 0;
  static uint16_t can_tx_fail = 0;
  static uint32_t last_report_ticks = 0;

  static bool     s_recovery_initiated = false;

  // 0) Validate arguments
  if (!Msg || Msg_Length == 0 || Msg_Length > TWAI_DATA_LENGTH_CODE_MAX) {
    return ESP_ERR_INVALID_ARG;
  }

  // 1) Check controller state cheaply; avoid TX while BUS_OFF/RECOVERING
  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK) {
    if (st.state == TWAI_STATE_BUS_OFF) {
      if (!s_recovery_initiated) {
        (void)twai_initiate_recovery();   // kick off recovery once
        s_recovery_initiated = true;
      }
      can_tx_fail++;                      // count this attempt as a fail
      // Optional: brief yield to avoid hammering this path
      vTaskDelay(pdMS_TO_TICKS(1));
      return ESP_ERR_INVALID_STATE;
    } else if (st.state == TWAI_STATE_RECOVERING) {
      // Still waiting for 128*11 recessive bits; TX won’t work yet
      can_tx_fail++;
      vTaskDelay(pdMS_TO_TICKS(1));
      return ESP_ERR_INVALID_STATE;
    }
  }

  // 2) Drain any stale alerts quickly so we don't misattribute outcomes
  {
    uint32_t dummy;
    // Non-blocking reads to clear backlog (max a few iterations)
    for (int i = 0; i < 4; ++i) {
      if (twai_read_alerts(&dummy, 0) != ESP_OK) break;
    }
  }

  // 3) Build single-shot message (no automatic retransmission)
  twai_message_t message = {0};
  message.identifier       = Address;
  message.data_length_code = Msg_Length;
  message.extd             = 0;                 // standard ID
  message.rtr              = 0;                 // data frame
  message.flags            = TWAI_MSG_FLAG_SS;  // SINGLE SHOT: try once, don't retry
  memcpy(message.data, Msg, Msg_Length);

  // 4) Try to enqueue the frame (short, bounded wait)
  esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(CAN_TX_QUEUE_TIMEOUT_MS));
  if (err != ESP_OK) {
    // NOT_STARTED / INVALID_STATE / QUEUE_FULL / TIMEOUT, etc.
    can_tx_fail++;
  } else {
    // Single blocking wait for the transmission outcome
    uint32_t alerts = 0;

    if (twai_read_alerts(&alerts, pdMS_TO_TICKS(CAN_TX_RESULT_TIMEOUT_MS)) != ESP_OK) {
      // No alert within window (bus quiet/disconnected?)
      can_tx_fail++;
      err = ESP_ERR_TIMEOUT;
    } else {
      // Prioritise hard error first
      if (alerts & TWAI_ALERT_BUS_OFF) {
        (void)twai_initiate_recovery();
        s_recovery_initiated = true;
        can_tx_fail++;
        err = ESP_ERR_INVALID_STATE;
      } else if (alerts & TWAI_ALERT_TX_SUCCESS) {
        // Frame went out and was ACKed
        can_tx_ok++;
        err = ESP_OK;
      } else if (alerts & (TWAI_ALERT_TX_FAILED | TWAI_ALERT_ARB_LOST)) {
        // Single-shot: no retry; treat as failed attempt
        can_tx_fail++;
        err = ESP_FAIL;
      } else {
        // Got some other alert (e.g., ERR_PASS/BUS_ERROR/TX_IDLE) without a decisive result
        can_tx_fail++;
        err = ESP_FAIL;
      }

      // Informational: clear recovery flag if notified
      if (alerts & TWAI_ALERT_BUS_RECOVERED) {
        s_recovery_initiated = false;
      }
    }
  }

  // 5) Periodic stats (every 5 s)
  {
    TickType_t now = xTaskGetTickCount();
    if ((now - last_report_ticks) >= pdMS_TO_TICKS(5000)) {
      logf("CAN TX: ok=%u fail=%u\n", can_tx_ok, can_tx_fail);
      last_report_ticks = now;
    }
  }

  return err; // ESP_OK only if ACKed; otherwise ESP_FAIL/ESP_ERR_TIMEOUT/ESP_ERR_INVALID_STATE etc.
}

// OLD CODE FOR DELETION ONCE NEW CODE ABOVE VALIDATED
//-------------------------------------------------------------------------------------
// #define CAN_TX_TIMEOUT_MS           100

// esp_err_t SendCAN_Packet(uint16_t Address, uint8_t Msg_Length, uint8_t * Msg) 
// {
//   static uint16_t can_tx_ok = 0;
//   static uint16_t can_tx_fail = 0;
//   static uint32_t last_report_ticks = 0;

//   // 1) Validate arguments
//   if (!Msg || Msg_Length == 0 || Msg_Length > TWAI_DATA_LENGTH_CODE_MAX) {
//       //logf("CAN ERR: invalid args ptr=%p len=%u", Msg, Msg_Length);
//       return ESP_ERR_INVALID_ARG;
//   }

//   // 2) Prepare message (zero-init for flags)
//   twai_message_t message = { 0 };
//   message.identifier       = Address;
//   message.data_length_code = Msg_Length;
//   message.extd             = 0;             // standard ID
//   message.rtr              = 0;             // no remote frame
//   memcpy(message.data, Msg, Msg_Length);

//   // 3) Transmit
//   esp_err_t err = twai_transmit(&message, pdMS_TO_TICKS(CAN_TX_TIMEOUT_MS));
//   if (err == ESP_OK) {
//       can_tx_ok++;
//   } else {
//       can_tx_fail++;
//   }

//   // Report every 5000 ms (5 s)
//   TickType_t now = xTaskGetTickCount();
//   if ((now - last_report_ticks) >= pdMS_TO_TICKS(5000)) {
//       logf("CAN TX: ok=%u fail=%u\n", can_tx_ok, can_tx_fail);
//       last_report_ticks = now;
//   }

//   return err;
// }

// ####################################################################################################################
// communication with Victron system over CAN--------------------------------------------------------------------------
// ####################################################################################################################
void VEcan() 
{
  uint8_t mes[8], SOH = 100;
  uint8_t bmsmanu[8] = {'M', 'I', 'N', 'I', ' ', 'B', 'M', 'S'};

  mes[0] = lowByte(CVL_Vx10);                         mes[1] = highByte(CVL_Vx10);
  mes[2] = lowByte(CCL_Ax10);                         mes[3] = highByte(CCL_Ax10);
  mes[4] = lowByte(DCL_Ax10);                         mes[5] = highByte(DCL_Ax10);
  mes[6] = lowByte(480);                              mes[7] = highByte(480);

  SendCAN_Packet(0x351, 8, mes);
  vTaskDelay(5);

  mes[0] = lowByte(SOC);                              mes[1] = highByte(SOC);
  mes[2] = lowByte(SOH);                              mes[3] = highByte(SOH);
  mes[4] = lowByte(SOC * 10);                         mes[5] = highByte(SOC * 10);
  mes[6] = 0;                                         mes[7] = 0;

  SendCAN_Packet(0x355, 8, mes);
  vTaskDelay(5);

  mes[0] = lowByte(uint16_t(StackVoltage_mV));        mes[1] = highByte(uint16_t(StackVoltage_mV));
  mes[2] = lowByte(uint16_t(currentact * 10));        mes[3] = highByte(uint16_t(currentact * 10));
  mes[4] = lowByte(uint16_t(PackTemperature * 10));   mes[5] = highByte(uint16_t(PackTemperature * 10));
  mes[6] = 0;                                         mes[7] = 0;

  SendCAN_Packet(0x356, 8, mes);
  vTaskDelay(5);

  SendCAN_Packet(0x35E, 8, bmsmanu); 
  vTaskDelay(5);

  mes[0] = 0; mes[1] = 0; mes[2] = 0; mes[3] = 0; mes[4] = 0; mes[5] = 0; mes[6] = 0; mes[7] = 0;

  SendCAN_Packet(0x35A, 8, mes);
  vTaskDelay(5);

  SendCAN_Packet(0x35B, 8, mes);
  vTaskDelay(5);  
}

// ####################################################################################################################
// --------------------------------------------------------------------------------------------------------------------
// ####################################################################################################################
static inline void applyMode(LedState& L)
{
  switch (L.mode) {
    case LED_OFF:           L.on_ms = 0;    L.off_ms = 0;     L.level = LOW;  break;
    case LED_ON:            L.on_ms = 0;    L.off_ms = 0;     L.level = HIGH; break;
    case LED_BLINK_SLOW:    L.on_ms = 1000; L.off_ms = 1000;                  break;                       
    case LED_BLINK_FAST:    L.on_ms = 100;  L.off_ms = 100;                   break;         
    case LED_BLINK_BALANCE: L.on_ms= 80;    L.off_ms = 170;                   break;                        
    case LED_BLINK_COV:     L.on_ms= 2000;  L.off_ms = 200;                   break;                        
    case LED_BLINK_CUV:     L.on_ms= 100;   L.off_ms = 200;                   break;               
    case LED_BLINK_ALIVE:   L.on_ms= 50;    L.off_ms = 5000;                  break;        
  }
}

// ####################################################################################################################
// --------------------------------------------------------------------------------------------------------------------
// ####################################################################################################################
static inline void updateLed(LedState& L, uint32_t now)
{
  if (L.mode == LED_OFF) { if (L.level != LOW)  { L.level = LOW;  digitalWrite(L.pin, LOW); }  return; }
  if (L.mode == LED_ON)  { if (L.level != HIGH) { L.level = HIGH; digitalWrite(L.pin, HIGH); } return; }

  if ((int32_t)(now - L.next_ms) >= 0) {
    // toggle
    L.level = !L.level;
    digitalWrite(L.pin, L.level ? HIGH : LOW);
    L.next_ms = now + (L.level ? L.on_ms : L.off_ms);
  }
}

// ####################################################################################################################
// Use from any task (non-ISR)-----------------------------------------------------------------------------------------
// ####################################################################################################################
void logf(const char* fmt, ...) 
{
  char tmp[LOG_LINE_MAX];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(tmp, sizeof(tmp), fmt, ap);
  va_end(ap);
  if (n <= 0) return;

  LogMsg m{};
  m.len = (uint16_t) ((n >= (int)sizeof(m.text)) ? (LOG_LINE_MAX - 1) : n);
  memcpy(m.text, tmp, m.len);
  m.text[m.len] = '\0';

  // If queue is full, drop or block. Here: drop.
  (void)xQueueSend(logQ, &m, 0);
}

// ####################################################################################################################
// BLE server connect/disconnect flags --------------------------------------------------------------------------------
// ####################################################################################################################
class MyServerCallbacks : public BLEServerCallbacks 
{
  void onConnect(BLEServer* s) override { gDeviceConnected = true;  }
  void onDisconnect(BLEServer* s) override { gDeviceConnected = false; }
};

// ####################################################################################################################
// Pack & send the 16 cells (mV) as 32 bytes LE -----------------------------------------------------------------------
// ####################################################################################################################
static void notifyCellVoltages() 
{
  if (!gCellChar) return;

  uint8_t buf[NUM_OF_CELLS * 2];
  for (int i = 0; i < NUM_OF_CELLS; ++i) {
    uint16_t mv = Cell_Voltages[i];
    buf[2*i + 0] = (uint8_t)(mv & 0xFF);
    buf[2*i + 1] = (uint8_t)(mv >> 8);
  }
  gCellChar->setValue(buf, sizeof(buf));
  gCellChar->notify();
}

// ####################################################################################################################
// --------------------------------------------------------------------------------------------------------------------
// ####################################################################################################################
// --------------------------------------------------------------------------------------------------------------------
// ####################################################################################################################
void Task_CANHandle(void *pvParameters)
{
  const TickType_t period = pdMS_TO_TICKS(200);

  // Initialize CAN configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX, (gpio_num_t)CAN_RX, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); 
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver 
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    logf("Driver installed\n");
  } else {
    logf("Failed to install driver\n");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    logf("Driver started\n");
  } else {
    logf("Failed to start driver\n");
    return;
  }

  // Reconfigure alerts to detect TX alerts and Bus-Off errors
  uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_ARB_LOST | 
                              TWAI_ALERT_BUS_ERROR | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    logf("CAN Alerts reconfigured\n");
  } else {
    logf("Failed to reconfigure alerts\n");
    return;
  }

  // Configure Enable / stanby
  // digitalWrite(CAN_EN, HIGH); pinMode(CAN_EN, OUTPUT);           // CAN transciever is enabled with this line commented out       
  digitalWrite(CAN_STB, LOW); pinMode(CAN_STB, OUTPUT);             // This takes the chip out of standby mode 

  for (;;)
  {
    vTaskDelay(period);    
    VEcan();
  }
}

// --------------------------------------------------------------------------------------------------------------------
void Task_BMSHandle(void *pvParameters)
{
  const TickType_t tick200ms = pdMS_TO_TICKS(200);
  const TickType_t tick3s    = pdMS_TO_TICKS(3000);

  TickType_t last200ms = xTaskGetTickCount();
  TickType_t last3s  = xTaskGetTickCount();

  static uint16_t MaxCellVoltage_mV = 3200;
  static uint16_t MinCellVoltage_mV = 3200; 

  static TickType_t cuv_latch_until = 0;
  static TickType_t cov_latch_until = 0;

  // Register the current task for monitoring by the watchdog
  ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
  logf("TWDT configured\n");
  esp_task_wdt_reset();

  // Configure pins
  //HWSerial.begin(2400, SERIAL_8N1, -1, 10); //HWSerial.println("COV");        // Future comms mode for daisy chaining BMS modules
  pinMode(AUX_1_PIN, OUTPUT); digitalWrite(AUX_1_PIN, HIGH);
  pinMode(AUX_2_PIN, INPUT);
  pinMode(AUX_3_PIN, OUTPUT); digitalWrite(AUX_3_PIN, HIGH);  
  pinMode(AUX_4_PIN, INPUT);

  for(;;)  {
    TickType_t now = xTaskGetTickCount();
    if (now - last3s >= tick3s) {
      last3s = now; 

      esp_task_wdt_reset();                                           // This task is subject to a watchdog because it is vital that it continues to run properly
               
      int8_t Alert_A = bms.directCommandRead(DIR_CMD_FPROTEC);        // Read the protection registers
      int8_t Alert_B = bms.directCommandRead(DIR_CMD_FTEMP);
    
      // Re-arm the latches if a fault bit is present
      if (Alert_A & 0b0100) { // CUV
          cuv_latch_until = now + pdMS_TO_TICKS(CUOV_LATCH_TIME);
      }
      if (Alert_A & 0b1000) { // COV
          cov_latch_until = now + pdMS_TO_TICKS(CUOV_LATCH_TIME);
      }

      // Wrap-safe expiry check written inline:
      bool cov_active = (Alert_A & 0b1000) || ((int32_t)(cov_latch_until - now) > 0);      
      bool cuv_active = (Alert_A & 0b0100) || ((int32_t)(cuv_latch_until - now) > 0);

      // Apply actions
      if (cov_active) {
          digitalWrite(AUX_1_PIN, HIGH);          
          xEventGroupSetBits(eg, COV_BIT);
      } else {
          digitalWrite(AUX_1_PIN, LOW);          
          xEventGroupClearBits(eg, COV_BIT);
      }

      if (cuv_active) {
          digitalWrite(AUX_3_PIN, HIGH);
          xEventGroupSetBits(eg, CUV_BIT);
      } else {
          digitalWrite(AUX_3_PIN, LOW);
          xEventGroupClearBits(eg, CUV_BIT);
      }

      if(bms.isBalancing()) {
        xEventGroupSetBits(eg, BALANCING_BIT);
      }
      else  {
        xEventGroupClearBits(eg, BALANCING_BIT);
      }

      // Faults
      {
        char aBin[33], bBin[33];   
        utoa((unsigned)Alert_A, aBin, 2);
        utoa((unsigned)Alert_B, bBin, 2);
        logf("Fault A: %s, Fault B: %s \n", aBin, bBin);
      }

      // Cell Voltages
      {
        char buf[LOG_LINE_MAX];
        int pos = snprintf(buf, sizeof(buf), "Cell Voltages: ");
        for (byte i = 0; i < NUM_OF_CELLS && pos > 0 && pos < (int)sizeof(buf); i++) {
          Cell_Voltages[i] = bms.getCellVoltage(i + 1);
          pos += snprintf(buf + pos, sizeof(buf) - pos, "%u%s", (unsigned)Cell_Voltages[i], (i < NUM_OF_CELLS - 1) ? ", " : "");
        }
        logf("%s\n", buf);
      }

      // Cell Balance Times
      {
        char buf[LOG_LINE_MAX];
        bms.GetCellBalancingTimes(Cell_Balance_Times);      
        int pos = snprintf(buf, sizeof(buf), "Cell Balance Times: ");
        for (int i = 0; i < NUM_OF_CELLS && pos > 0 && pos < (int)sizeof(buf); i++) {
          pos += snprintf(buf + pos, sizeof(buf) - pos, "%u%s", (unsigned)Cell_Balance_Times[i], (i < NUM_OF_CELLS - 1) ? ", " : "");
        }
        logf("%s\n", buf);
      }

      // Temperatures (float)
      {
        float ti = bms.getInternalTemp();        
        PackTemperature = bms.getThermistorTemp(TS1);

        // If Pack temperature is < -25C it'll be because the thermistor is not connected. So default to 25C
        if(PackTemperature < -25.0) {PackTemperature = 25.0;}  

        logf("BQ Temp: %.1f, TS1 Temp: %.1f\n", (double)ti, (double)PackTemperature);
      }

      // DASTATUS_5 and Stack Voltage
      {
        byte* voltage_data = bms.subCommandwithdata(DASTATUS_5, 32); 
        uint16_t rawMax  = (voltage_data[5] << 8) | voltage_data[4];
        uint16_t rawMin  = (voltage_data[7] << 8) | voltage_data[6];
        uint32_t rawPack = ((voltage_data[9] << 8) | voltage_data[8]) * 10;

        MaxCellVoltage_mV = constrain(rawMax, CELL_ABS_MIN_mV, CELL_ABS_MAX_mV);
        MinCellVoltage_mV = constrain(rawMin, CELL_ABS_MIN_mV, CELL_ABS_MAX_mV);        
        BatteryVoltageSum_mV = constrain(rawPack, PACK_ABS_MIN_mV, PACK_ABS_MAX_mV);
        StackVoltage_mV = bms.directCommandRead(DIR_CMD_VSTACK) * 10;

        logf("Max=%u mV, Min=%u mV, Cell sum=%u mV, Stack=%u mV  ->  CVL=%lu mV  CCL=%lu mA  DCL=%lu mA\n\n",
            (unsigned)MaxCellVoltage_mV,
            (unsigned)MinCellVoltage_mV,
            (unsigned)BatteryVoltageSum_mV,
            (unsigned)StackVoltage_mV,
            (unsigned long)lim.CVL_mV,
            (unsigned long)lim.CCL_mA,
            (unsigned long)lim.DCL_mA);         
      }
    }      

    // This section runs every 200 ms and calculated the DVCC values that get sent over CAN
    Telemetry tele { .maxCell_mV = MaxCellVoltage_mV, .minCell_mV = MinCellVoltage_mV, .pack_mV = StackVoltage_mV, .pack_temp = PackTemperature };
    static LimitsState st;
    computeLimits(tele, st, lim, 0.2f);                             // dt=0.2 s if you call at 5 Hz
    toVictronScaling(lim, CVL_Vx10, CCL_Ax10, DCL_Ax10);     

    vTaskDelayUntil(&last200ms, tick200ms);
  }
}

// --------------------------------------------------------------------------------------------------------------------
void Task_MQTTHandle(void *pvParameters)
{
  const TickType_t tick200ms  = pdMS_TO_TICKS(200);
  const TickType_t tick20s    = pdMS_TO_TICKS(20000);
  const TickType_t tick60s    = pdMS_TO_TICKS(60000);
  const TickType_t tick15m    = pdMS_TO_TICKS(900000);

  TickType_t last200ms        = xTaskGetTickCount();
  TickType_t last20s          = xTaskGetTickCount() - tick20s;
  TickType_t last60s          = xTaskGetTickCount();
  TickType_t last15m          = xTaskGetTickCount();

  TickType_t now;

  for (;;)
  {
    vTaskDelayUntil(&last200ms, tick200ms);

    // Every 200 milliseconds we service our MQTT client
    client.loop();

    // Every 20 seconds we ensure wifi and MQTT servers are connected
    now = xTaskGetTickCount();
    if (now - last20s >= tick20s) {
      last20s = now; 

      maintainConnections();      
    }

    // Every 60 seconds we attempt to Tx our cell voltages over MQTT
    now = xTaskGetTickCount();
    if (now - last60s >= tick60s) {      
      last60s = now;  // reset period from actual time

      char voltageJson[256];
      StaticJsonDocument<300> doc;
      JsonArray arr = doc.to<JsonArray>();
      for (int i = 0; i < NUM_OF_CELLS; i++) {
        arr.add(Cell_Voltages[i]);
      }
      serializeJson(doc, voltageJson, sizeof(voltageJson));
      logf("%s\n", voltageJson);
      client.publish("battery/cell_voltages", voltageJson);
    }

    // Every 15 minutes we attempt to Tx our cell balance times over MQTT
    now = xTaskGetTickCount();
    if (now - last15m >= tick15m) {      
      last15m = now;  // reset period from actual time

      char balanceJson[512];
      StaticJsonDocument<600> doc;
      JsonArray arr = doc.to<JsonArray>();
      for (int i = 0; i < NUM_OF_CELLS; i++) {
        arr.add(Cell_Balance_Times[i]);
      }
      serializeJson(doc, balanceJson, sizeof(balanceJson));
      logf("%s\n", balanceJson);
      client.publish("battery/balance_times", balanceJson); 
    }        
  }
}

// --------------------------------------------------------------------------------------------------------------------
void Task_LEDHandle(void *pvParameters)
{
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_RED_PIN,   OUTPUT);

  // initialize modes
  applyMode(ledG);
  applyMode(ledR);
  digitalWrite(ledG.pin, ledG.level);
  digitalWrite(ledR.pin, ledR.level);
  uint32_t now = millis();
  ledG.next_ms = now + (ledG.level ? ledG.on_ms : ledG.off_ms);
  ledR.next_ms = now + (ledR.level ? ledR.on_ms : ledR.off_ms);

  const TickType_t tick = pdMS_TO_TICKS(10); // 10 ms scheduler tick for smooth timing
  TickType_t last = xTaskGetTickCount();

  for (;;)
  {
    // Read system state using an Event Group
    EventBits_t b = xEventGroupGetBits(eg);

    // Green LED:
    bool bms_is_balancing = (b & BALANCING_BIT);
    ledG.mode = bms_is_balancing ? LED_BLINK_BALANCE : LED_BLINK_SLOW;

    // Red LED
    bool cov_fault = (b & COV_BIT);
    bool cuv_fault = (b & CUV_BIT);
    
    if (cuv_fault) {
        ledR.mode = LED_BLINK_CUV;     
    } else if (cov_fault) {
        ledR.mode = LED_BLINK_COV;
    } else {
        ledR.mode = LED_BLINK_ALIVE;
    }

    // If modes changed since last loop, re-apply timing
    static LedMode prevG = (LedMode)255, prevR = (LedMode)255;
    if (ledG.mode != prevG) { applyMode(ledG); ledG.next_ms = millis(); prevG = ledG.mode; }
    if (ledR.mode != prevR) { applyMode(ledR); ledR.next_ms = millis(); prevR = ledR.mode; }

    // Update LEDs
    uint32_t t = millis();
    updateLed(ledG, t);
    updateLed(ledR, t);

    // Yield with stable cadence
    vTaskDelayUntil(&last, tick);
  }
}

// --------------------------------------------------------------------------------------------------------------------
void Task_ComsHandle(void *pvParameters)
{
  for (;;) {
    LogMsg m;
    if (xQueueReceive(logQ, &m, portMAX_DELAY) == pdTRUE) {
      Serial.write((const uint8_t*)m.text, m.len);
    }
  }
}

// --------------------------------------------------------------------------------------------------------------------
void Task_BLEHandle(void *pvParameters)
{
  const TickType_t tick3s       = pdMS_TO_TICKS(3000);  
  const TickType_t period500ms  = pdMS_TO_TICKS(500);  
  const TickType_t period10ms   = pdMS_TO_TICKS(10);

  TickType_t lastPush           = xTaskGetTickCount();

  TickType_t now;

  // Init BLE once
  BLEDevice::init("MiniBMS");
  BLEDevice::setMTU(100);             // helps ensure 32B payload fits in one packet (browser will negotiate)

  // Create server + callbacks
  gServer = BLEDevice::createServer();
  gServer->setCallbacks(new MyServerCallbacks());

  // Service
  BLEService* svc = gServer->createService(CELL_SERVICE_UUID);

  // Characteristic: READ + NOTIFY (no write needed)
  gCellChar = svc->createCharacteristic(
    CELL_ARRAY_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  gCellChar->addDescriptor(new BLE2902()); // enables CCCD for notifications

  svc->start();

  // Advertise service
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(CELL_SERVICE_UUID);
  // adv->setScanResponse(true); // default is fine
  BLEDevice::startAdvertising();

  logf("BLE waiting for a client...");


  for (;;) {
    now = xTaskGetTickCount();

    // If connected, update the cell values and notify every ~3s
    if (gDeviceConnected) {
      if (now - lastPush >= tick3s) {
        lastPush = now;                
        notifyCellVoltages();        
        logf("Pushed cell voltages\n");
      }
    }

    // Handle connect/disconnect transitions
    if (!gDeviceConnected && gOldDeviceConnected) {
      logf("Client disconnected; restarting advertising.\n");
      vTaskDelay(period500ms);
      gServer->startAdvertising();
      gOldDeviceConnected = gDeviceConnected;
    }
    if (gDeviceConnected && !gOldDeviceConnected) {
      logf("Client connected.\n");
      notifyCellVoltages();
      lastPush = now;
      gOldDeviceConnected = gDeviceConnected;
    }

    // keep the loop light
    vTaskDelay(period10ms);
  }
}









