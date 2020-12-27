#include <Arduino.h>
#include <SPI.h>
#include <BLEPeripheral.h>
#include "nrf_soc.h"
#include "nrf_nvic.h"

//Infos:
/// Datasheet
/// https://infocenter.nordicsemi.com/pdf/nRF51822_PS_v3.1.pdf
//
/// LE
/// https://www.forward.com.au/pfod/BLE/LowPower/index.html#lp_timer
//  #include <lp_timer.h>

//good balance bwtween LE and discorvery...
#define ADVERTISING_INTERVAL 2500
#define VBAT_MAX_IN_MV 3000

#define PIN_SW_SENSOR PIN_BUTTON1
#define PIN_LED_BLE PIN_LED2

//Max Transmition-Power
#define TX_POWER 4

///
void characteristicWrittenCallback(BLECentral&, BLECharacteristic&);
///***/
void updateAdvertisingScanData();
void sensorValueChanged();
void digitalWriteLog(uint32_t ulPin, uint32_t ulVal);
unsigned char getBatteryLevel(void);
void bleConnectedCallback(BLECentral &);


volatile uint32_t g_counterValue = 0;
uint32_t g_old_counterValue = g_counterValue;
volatile bool g_sensorValueChanged = false;

const char g_localName[] = "casa-verde-humi";
const char g_deviceName[] = "CasaVerde Plant Humidity";
///

BLEPeripheral blePeripheral = BLEPeripheral();

//BLEService ledService =                              BLEService("19b10000e8f2537e4f6cd104768a1214");
//BLECharCharacteristic ledCharacteristic = BLECharCharacteristic("19b10001e8f2537e4f6cd104768a1214", BLERead | BLEWrite);


BLEService mainBleService                        ("838c2bb4-42d1-11eb-b378-0242ac130002");
BLEUnsignedIntCharacteristic sensorCharacteristic("7f461cfe-42d1-11eb-b378-0242ac130002", BLERead | BLEWrite);
BLECharCharacteristic  ledCharacteristic         ("5f58b230-42d1-11eb-b378-0242ac130002", BLERead | BLEWrite);

BLEService                                batteryService("180F");
BLEUnsignedCharCharacteristic batteryLevelCharacteristic("2A19", BLERead);
void sensorCharacteristicWrittenCallback(BLECentral &, BLECharacteristic &);


void setup() {
  #ifdef ENABLE_LOG
    Serial.begin(115200);
  #endif

  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED_BLE, OUTPUT);

  pinMode(PIN_SW_SENSOR, INPUT_PULLUP);

  blePeripheral.setLocalName(g_localName);
  blePeripheral.setDeviceName(g_deviceName);

  blePeripheral.setAdvertisedServiceUuid(mainBleService.uuid());
  blePeripheral.addAttribute(mainBleService);
  blePeripheral.addAttribute(ledCharacteristic);
  blePeripheral.addAttribute(sensorCharacteristic);

  blePeripheral.addAttribute(batteryService);
  blePeripheral.addAttribute(batteryLevelCharacteristic);

  blePeripheral.setAdvertisingInterval(ADVERTISING_INTERVAL);
  ledCharacteristic.setEventHandler(BLEWritten, characteristicWrittenCallback);

  sensorCharacteristic.setValue(g_counterValue);
  sensorCharacteristic.setEventHandler(BLEWritten, sensorCharacteristicWrittenCallback);
 
  blePeripheral.setEventHandler(BLEConnected, bleConnectedCallback);



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
        Serial.print(F("g_sensorValueChanged, count: "));
        Serial.print(g_counterValue);
        Serial.println(F("."));
      #endif  
      sensorCharacteristic.setValue(g_counterValue);
      updateAdvertisingScanData();
      //digitalWriteLog(PIN_LED_SENSOR, LOW);
  }

  // poll peripheral
  blePeripheral.poll();
}

void bleConnectedCallback(BLECentral &bleCentral)
{
  unsigned char batteryLevel = getBatteryLevel();
  if (batteryLevel > 100)
  {
    batteryLevel = 100;
  }
  batteryLevelCharacteristic.setValue(batteryLevel);
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

void sensorCharacteristicWrittenCallback(BLECentral &central, BLECharacteristic &characteristic)
{
  // The new value has already been written on characteristic
  // We still save it and flag the value as changed to allow the advertising packet to be updated
  g_sensorValueChanged = true;
  g_counterValue = *characteristic.value();
}

unsigned char getBatteryLevel(void)
{
  // Configure ADC
  NRF_ADC->CONFIG = (ADC_CONFIG_RES_8bit << ADC_CONFIG_RES_Pos) |
                    (ADC_CONFIG_INPSEL_SupplyOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) |
                    (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos) |
                    (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos) |
                    (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos);
  NRF_ADC->EVENTS_END = 0;
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;

  NRF_ADC->EVENTS_END = 0; // Stop any running conversions.
  NRF_ADC->TASKS_START = 1;

  while (!NRF_ADC->EVENTS_END)
  {
  }

  uint16_t vbg_in_mv = 1200;
  uint8_t adc_max = 255;
  uint16_t vbat_current_in_mv = (NRF_ADC->RESULT * 3 * vbg_in_mv) / adc_max;

  NRF_ADC->EVENTS_END = 0;
  NRF_ADC->TASKS_STOP = 1;
  NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;

  return (unsigned char)((vbat_current_in_mv * 100) / VBAT_MAX_IN_MV);
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