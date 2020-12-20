#include <Arduino.h>
#include <SPI.h>
#include <BLEPeripheral.h>
#include "nrf_soc.h"
#include "nrf_nvic.h"

#define ADVERTISING_INTERVAL 2500
#define PIN_SW_SENSOR PIN_BUTTON1
#define PIN_LED_BLE PIN_LED2
#define TX_POWER 4

///
void characteristicWrittenCallback(BLECentral&, BLECharacteristic&);
///***/
void updateAdvertisingScanData();
void sensorValueChanged();
void digitalWriteLog(uint32_t ulPin, uint32_t ulVal);
volatile uint32_t g_counterValue = 0;
uint32_t g_old_counterValue = g_counterValue;
volatile bool g_sensorValueChanged = false;

const char g_localName[] = "casa-verde-humi";
const char g_deviceName[] = "CasaVerde Plant Humidity";
///

BLEPeripheral blePeripheral = BLEPeripheral();

BLEService ledService = BLEService("19b10000e8f2537e4f6cd104768a1214");
BLECharCharacteristic ledCharacteristic = BLECharCharacteristic("19b10001e8f2537e4f6cd104768a1214", BLERead | BLEWrite);

void setup() {
  #ifdef ENABLE_LOG
    Serial.begin(115200);
  #endif

  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED_BLE, OUTPUT);

  pinMode(PIN_SW_SENSOR, INPUT_PULLUP);


  blePeripheral.setAdvertisedServiceUuid(ledService.uuid());
  blePeripheral.addAttribute(ledService);
  blePeripheral.addAttribute(ledCharacteristic);
  blePeripheral.setLocalName(g_localName);
  blePeripheral.setDeviceName(g_deviceName);
  
  blePeripheral.setAdvertisingInterval(ADVERTISING_INTERVAL);
  ledCharacteristic.setEventHandler(BLEWritten, characteristicWrittenCallback);
  
  blePeripheral.begin();
  blePeripheral.setTxPower(TX_POWER);

  updateAdvertisingScanData();

  attachInterruptLowAccuracy(digitalPinToInterrupt(PIN_SW_SENSOR), sensorValueChanged, FALLING);
  //** Not BLE Interrupt handling ***/
  //attachInterrupt(digitalPinToInterrupt(PIN_SW_SENSOR), sensorValueChanged, FALLING);

  // enable low power mode without interrupt
  sd_power_mode_set(NRF_POWER_MODE_LOWPWR);

}

void loop() {
  digitalWriteLog(PIN_LED_BLE, LOW);
  // Enter Low power mode
  sd_app_evt_wait();
  // Exit Low power mode
  
  // Clear IRQ flag to be able to go to sleep if nothing happens in between
  sd_nvic_ClearPendingIRQ(SWI2_IRQn);

  digitalWriteLog(PIN_LED_BLE, HIGH);
  if (g_sensorValueChanged) {
      g_sensorValueChanged = false;
      #ifdef ENABLE_LOG
        Serial.print(F("g_sensorValueChanged..."));
      #endif  
      updateAdvertisingScanData();
      //digitalWriteLog(PIN_LED_SENSOR, LOW);
  }

  // poll peripheral
  blePeripheral.poll();
}

void characteristicWrittenCallback(BLECentral& central, BLECharacteristic& characteristic) {
  // central wrote new value to characteristic, update LED
  if (ledCharacteristic.value()) {
    digitalWrite(PIN_LED1, HIGH);
  } else {
    digitalWrite(PIN_LED1, LOW);
  }
}

void check_irq() {
  if (g_old_counterValue != g_counterValue) {
      #ifdef ENABLE_LOG
        Serial.print(F("sensorValueChanged..."));
        Serial.println(g_counterValue);
      #endif  
      g_old_counterValue = g_counterValue;
      //g_sensorValueChanged = true;
  }
}

void sensorValueChanged()
{
#if 1
  g_counterValue++;
  g_sensorValueChanged = true;
  //Serial.print(F("sensorValueChanged..."));
#else    
  uint32_t diff = millis() - g_lastSensorValueChanged;
  if (diff > DEBOUNCE_SENSOR_MS || diff < 0)
  {
    g_sensorValueChanged = true;
    g_counterValue = g_counterValue + 1;
  }
  g_lastSensorValueChanged = millis();
#endif  
}

// https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile/
void updateAdvertisingScanData()
{
  unsigned char srData[31];
  unsigned char srDataLen = 0;
  int scanDataSize = 3;
  BLEEirData scanData[scanDataSize];

  // - Local name
  scanData[0].length = strlen(g_localName);
  scanData[0].type = 0x09;
  memcpy(scanData[0].data, g_localName, scanData[0].length);

  // - Tx Power
  scanData[1].length = 1;
  scanData[1].type = 0x0A;
  scanData[1].data[0] = TX_POWER;

  // - Manufacturer Data
  scanData[2].length = 2 + 4;
  scanData[2].type = 0xFF;
  // Manufacturer ID
  scanData[2].data[0] = 0xFF;
  scanData[2].data[1] = 0xFF;
  // Manufacturer data content
  scanData[2].data[2] = g_counterValue & 0xFF;
  scanData[2].data[3] = (g_counterValue >> 8) & 0xFF;
  scanData[2].data[4] = (g_counterValue >> 16) & 0xFF;
  scanData[2].data[5] = (g_counterValue >> 24) & 0xFF;

  if (scanDataSize && scanData)
  {
    for (int i = 0; i < scanDataSize; i++)
    {
      srData[srDataLen + 0] = scanData[i].length + 1;
      srData[srDataLen + 1] = scanData[i].type;
      srDataLen += 2;

      memcpy(&srData[srDataLen], scanData[i].data, scanData[i].length);

      srDataLen += scanData[i].length;
    }
  }

  // - Sets only avertising scan data
  sd_ble_gap_adv_data_set(NULL, 0, srData, srDataLen);
}

void digitalWriteLog(uint32_t ulPin, uint32_t ulVal)
{
#ifdef ENABLE_LOG
  digitalWrite(ulPin, ulVal);
#endif
}