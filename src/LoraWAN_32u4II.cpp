/*******************************************************************************
 *  Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *  Copyright (c) 2017 Redferne
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
*******************************************************************************/

/*
Board information here:
http://www.diymalls.com/index.php?route=product/product&path=74&product_id=88
*/

/* Select OTAA / ABP mode by define/undefine LORA_ABP below.
   Register your device on the network and populate NWKSKEY, APPSKEY, APPEUI,
   DEVEUI, DEVADDR
*/

#include <Arduino.h>
#include <SPI.h>
#include <elapsedMillis.h>

// These must be set in arduino-lmic/lmic/config.h
// Debug and printf adds 2802
//#define LMIC_DEBUG_LEVEL 1
//#define LMIC_PRINTF_TO Serial
//#define LMIC_FAILURE_TO Serial
//#define LMIC_SPI_FREQ 10E6
// Uncomment these to save 4986 bytes
// #define DISABLE_JOIN
// #define DISABLE_PING
// #define DISABLE_BEACONS

// Disable to save 2928 bytes
// #define LORA_INFO

#include <lmic.h>
#include <hal/hal.h>

#define DEBUG_BAUD 57600

// Arduino pins

#define SERIAL1_RX   0    // HW Serial (free for GPS etc)
#define SERIAL1_TX   1    // HW Serial (free for GPS etc)
#define I2C_SDA      2    // Free to use
#define I2C_SCL      3    // Free to use
#define RFM95_RESET  4
#define RFM95_DIO_1  5
#define FREE_PIN_1   6    // Free to use
#define RFM95_DIO_0  7
#define RFM95_CS     8
#define RFM95_DIO_2  9
#define FREE_PIN_2  10    // Free to use
#define FREE_PIN_3  11    // Free to use
#define FREE_PIN_4  12    // Free to use
#define FREE_PIN_5  13    // Free to use
#define RFM95_MISO  14
#define RFM95_SCK   15
#define RFM95_MOSI  16

#define BATTERY_SENSE_PIN A0    // VBAT -> 1MOhm -> A0 -> 3MOhm -> GND
#define FREE_ANALOG_1     A1
#define FREE_ANALOG_2     A2
#define FREE_ANALOG_3     A3
#define FREE_ANALOG_4     A4
#define FREE_ANALOG_5     A5

// Lora Config

#define LORA_DEFAULT_CH 1 // Use default 3 channel config
//#define LORA_ABP        // Undefined == OTAA see below
#define LINK_CHECK 1
#define LINK_ADR 1
#define UPLINK_ACK 0
#define TX_PORT 1

float vbatt;
float lmic_freq;
uint8_t pbatt;

bool lmic_running = false;

elapsedMillis led_tick = 0;
elapsedMillis sensors_tick = 0;
uint32_t sensors_interval = 10;
uint32_t uplink_interval = 60;
bool led_blink = false;
bool tx_completed = false;


static osjob_t interval_job;
void SendSensorData(osjob_t* j);

void ReadBattery(void);
void SerialPrintSensors(void);
void PrepareSensorData(void);
void ActivityLed(void);
void PrintHex8(uint8_t *, uint8_t);
void LoRa_ReBoot(void);

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, true);

  // Wait for Serial to become available...
  while (!Serial && millis() < 5000);
  Serial.begin(DEBUG_BAUD);
  Serial.println(F("BOOT"));

  pinMode(BATTERY_SENSE_PIN, INPUT);

  ReadBattery();

  // TODO Battery too low, we should disable all and
  // enter lowest power mode here!

  Serial.println(F("Init LoRa"));

  // return;

  lmic_running = true;

  // LMIC init
  os_init();

  LoRa_ReBoot();

  SendSensorData(&interval_job);

  ActivityLed();
}

void loop()
{
  if (tx_completed && lmic_running) {
    os_setTimedCallback(&interval_job, os_getTime()+sec2osticks(uplink_interval), SendSensorData);
    tx_completed = false;
  }

  if (lmic_running) {
    os_runloop_once();
  }

  if (led_tick > 100 && led_blink) { // LED ON 100ms
      led_blink = false;
      digitalWrite(LED_BUILTIN, false);
  }

  if (sensors_tick > (sensors_interval * 1000)) {
      ReadBattery();
      #if 0
      ReadSensors();
      PrepareSensorData();
      #endif
      SerialPrintSensors();
      sensors_tick = 0;
  }
}

// Pin mapping for the new Moteino Mega R2 with solder bridges
const lmic_pinmap lmic_pins = {
  .nss = RFM95_CS,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = RFM95_RESET,
  .dio = {RFM95_DIO_0, RFM95_DIO_1, RFM95_DIO_2}, // DIO0, 1, 2
};

#if defined(LORA_INFO)

void ParseLMICOpmode(void);

ev_t lora_event = EV_RESET;
uint16_t lora_mode = 0;
char buffer[32] = {0};

const static char EV_STRING0[] PROGMEM = "None";
const static char EV_STRING1[] PROGMEM = "Scan Timeout";
const static char EV_STRING2[] PROGMEM = "Beacon Found";
const static char EV_STRING3[] PROGMEM = "Beacon Miss";
const static char EV_STRING4[] PROGMEM = "Beacon Track";
const static char EV_STRING5[] PROGMEM = "Joining";
const static char EV_STRING6[] PROGMEM = "Joined";
const static char EV_STRING7[] PROGMEM = "Uplink 1";
const static char EV_STRING8[] PROGMEM = "Join Fail";
const static char EV_STRING9[] PROGMEM = "Rejoin Fail";
const static char EV_STRING10[] PROGMEM = "RX2&TX Done";
const static char EV_STRING11[] PROGMEM = "Lost TSync";
const static char EV_STRING12[] PROGMEM = "Reset";
const static char EV_STRING13[] PROGMEM = "RX Complete";
const static char EV_STRING14[] PROGMEM = "Link Dead";
const static char EV_STRING15[] PROGMEM = "Link Alive";
const static char EV_STRING16[] PROGMEM = "Mode Switch";
const static char EV_STRING17[] PROGMEM = "TX Done";
const static char EV_STRING18[] PROGMEM = "RX1 Done";
const static char EV_STRING19[] PROGMEM = "Build TX";

const char* const EV_STRING[] PROGMEM = {
  EV_STRING0, EV_STRING1, EV_STRING2, EV_STRING3, EV_STRING4, EV_STRING5,
  EV_STRING6, EV_STRING7, EV_STRING8, EV_STRING9, EV_STRING10, EV_STRING11,
  EV_STRING12, EV_STRING13, EV_STRING14, EV_STRING15, EV_STRING16, EV_STRING17,
  EV_STRING18, EV_STRING19 };

const static char OP_STRING0[] PROGMEM = "None";
const static char OP_STRING1[] PROGMEM = "Scan";
const static char OP_STRING2[] PROGMEM = "Track";
const static char OP_STRING3[] PROGMEM = "TX";
const static char OP_STRING4[] PROGMEM = "Poll";
const static char OP_STRING5[] PROGMEM = "Join";
const static char OP_STRING6[] PROGMEM = "Fail";
const static char OP_STRING7[] PROGMEM = "SHTD";
const static char OP_STRING8[] PROGMEM = "TXRX";
const static char OP_STRING9[] PROGMEM = "DTX";
const static char OP_STRING10[] PROGMEM = "PingI";
const static char OP_STRING11[] PROGMEM = "PingA";
const static char OP_STRING12[] PROGMEM = "Ch+";
const static char OP_STRING13[] PROGMEM = "Dead";
const static char OP_STRING14[] PROGMEM = "Mode";
const static char OP_STRING15[] PROGMEM = "Test";

const char* const OP_STRING[] PROGMEM = {
  OP_STRING0, OP_STRING1, OP_STRING2, OP_STRING3, OP_STRING4, OP_STRING5,
  OP_STRING6, OP_STRING7, OP_STRING8, OP_STRING9, OP_STRING10, OP_STRING11,
  OP_STRING12, OP_STRING13, OP_STRING14, OP_STRING15 };

#endif

// LoRaWAN ABP
#if defined(LORA_ABP)
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }
static const u1_t PROGMEM NWKSKEY[16] = { 0x0E, 0xC5, 0x3B, 0x74, 0xB2, 0x37, 0xA6, 0xEE, 0x46, 0xEB, 0x20, 0xBE, 0x48, 0x4D, 0xBB, 0xFE };
#endif
// LoRaWAN ABP/OTAA
static const u1_t PROGMEM APPSKEY[16] = { 0x0E, 0xC5, 0x3B, 0x74, 0xB2, 0x37, 0xA6, 0xEE, 0x46, 0xEB, 0x20, 0xBE, 0x48, 0x4D, 0xBB, 0xFE };
// LoRaWAN ABP end-device address (DevAddr)
static const u4_t DEVADDR = 0x02000011 ; // <-- Change this address!
#if !defined(LORA_ABP)
static const u1_t PROGMEM APPEUI[8]   = { 0x65, 0x6e, 0x72, 0x65, 0x66, 0x64, 0x65, 0x52 };
// Little endian
static const u1_t PROGMEM DEVEUI[8] =   { 0x09, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01 };
void os_getArtEui (u1_t* buf) {memcpy_P(buf, APPEUI, 8);}
void os_getDevKey (u1_t* buf) {memcpy_P(buf, APPSKEY, 16);}
void os_getDevEui (u1_t* buf) {memcpy_P(buf, DEVEUI, 8);}
#endif

// Example payload data struct
#if 0
struct tx_data_t {
  uint32_t epoch; // Timestamp on message!    4
  int32_t gpslat;  // * 10 000 000 (1e7)      8
  int32_t gpslon;  // * 10 000 000 (1e7)      12
  int32_t gpsalt; // fix.geoidHeight_cm       16
  uint16_t gpsacc; // accuracy * 100 (2 dec)  18
  int16_t temp;  // temp in C * 100 (2 dec)   20
  int16_t stemp;  // temp in C * 100 (2 dec) SOIL 22
  int16_t pres;  // (100000) - hPa * 100 (2dec) 24
  int16_t alt;   // m (0 dec) 26
  int16_t dew;   // dewpoint in C * 100 (2dec)  28
  uint16_t lux;   // light in lux 30
  int16_t volt;  // batt voltage * 100  32
  uint16_t hum;  // hum in % * 10 (1 dec) 34
  uint16_t shum;  // hum in % * 10 (1 dec) SOIL 36
  uint8_t pcnt;  // batt in percent 37
  uint8_t gps;  // GPS Status 38
  uint8_t move;  // Motion  39
} __attribute__ ((__packed__));
#endif

struct tx_data_t {
  char text[13] = "Hello World!";
} __attribute__ ((__packed__));

struct tx_data_t tx_data;

// Example sensor formatting
#if 0
void PrepareSensorData(void) {
  tx_data.temp = (int16_t) (temp * 100);
  tx_data.hum = (uint16_t) round((hum * 10));
  tx_data.pres = (int16_t) round((1000.00 - pres)*100);
  tx_data.alt = (int16_t) round(alt);
  tx_data.dew = (int16_t) (dew * 100);
  tx_data.lux = (uint16_t) lux;
  tx_data.volt = (uint16_t) bV;
  tx_data.pcnt = (uint8_t) pbatt;
  tx_data.gps = fix.status;
  if (fix.valid.time && fix.valid.date) {
    tx_data.epoch = fix.dateTime;
  }
  if (fix.valid.location) {
    tx_data.gpslat = fix.latitudeL();
    tx_data.gpslon = fix.longitudeL();
  } else {
    tx_data.gpslat = 0;
    tx_data.gpslon = 0;
  }
  if (fix.valid.lat_err && fix.valid.lon_err) {
    if (fix.lat_err() > fix.lon_err())
      tx_data.gpsacc = (uint16_t) round(fix.lat_err() * 100);
    else
      tx_data.gpsacc = (uint16_t) round(fix.lon_err() * 100);
  } else {
    tx_data.gpsacc = 0;
  }
  if(fix.valid.altitude)
    tx_data.gpsalt = fix.altitude_cm();
  else
    tx_data.gpsalt = 0;
}
#endif

void ReadBattery(void) {

  uint16_t raw = analogRead(BATTERY_SENSE_PIN);

  vbatt = ((float) raw * ( 4.42 / 1023));
  uint16_t vint = round(vbatt * 100);

  if (vint <= 330)
    pbatt = 0;
  else if (vint <= 340)
    pbatt = map(vint, 330, 340, 0, 9);
  else if (vint <= 350)
    pbatt = map(vint, 340, 350, 9, 18);
  else if (vint <= 360)
    pbatt = map(vint, 350, 360, 18, 37);
  else if (vint <= 370)
    pbatt = map(vint, 360, 370, 37, 51);
  else if (vint <= 380)
    pbatt = map(vint, 370, 380, 51, 60);
  else if (vint <= 390)
    pbatt = map(vint, 380, 390, 60, 74);
  else if (vint <= 400)
    pbatt = map(vint, 390, 400, 74, 84);
  else if (vint <= 410)
    pbatt = map(vint, 400, 410, 84, 95);
  else if (vint <= 420)
    pbatt = map(vint, 410, 420, 95, 100);
  else
    pbatt = 100;
}

void SerialPrintSensors(void) {
#if 0
  Serial.print(F("T: "));
  Serial.print(temp);
  Serial.print(F("C H: "));
  Serial.print(hum);
  Serial.print(F("% B: "));
  Serial.print(pres);
  Serial.print(F("hPa A: "));
  Serial.print(alt);
  Serial.print(F(" m D: "));
  Serial.print(dew);
  Serial.print(F("C L: "));
  Serial.print(lux);
  Serial.print(F("lx B: "));
#endif
  Serial.print(vbatt);
  Serial.print(F("V B: "));
  Serial.print(pbatt);
  Serial.println(F("%"));
}

void ActivityLed() {
  digitalWrite(LED_BUILTIN, true);
  led_blink = true;
  led_tick = 0;
}

void PrintHex8(uint8_t *data, uint8_t length)
{
  Serial.print(F("0x"));
  for (int i=0; i<length; i++) {
    if (data[i]<0x10) {Serial.print(F("0"));}
    Serial.print(data[i],HEX);
    Serial.print(" ");
  }
}

#if defined(LORA_INFO)

void ParseLMICOpmode(void) {
  if (LMIC.opmode) {
    for (uint8_t i = 0; i < 15; i++) {
      if (LMIC.opmode & (1 << i)) {
        strcpy_P(buffer, (char*)pgm_read_ptr(&(OP_STRING[i+1])));
        Serial.print(buffer);
        Serial.print(F("-"));
      }
    }
  } else {
    strcpy_P(buffer, (char*)pgm_read_ptr(&(OP_STRING[0])));
    Serial.print(buffer);
  }
}

bool ParseLMICMAC(void) {
  bool valid = false;
  if (LMIC.mcmd) {
  } else {
    return false;
  }
  if (LMIC.mcmd & (1UL<<MCMD_LCHK_ANS)) {
    Serial.print(F("(LinkCheck Margin:"));
    Serial.print(LMIC.gwMargin);
    Serial.print(F("dB GwCnt:"));
    Serial.print(LMIC.nGws);
    LMIC.mcmd &= ~(1UL<<MCMD_LCHK_ANS);
    Serial.println(F(")"));
    valid = true;
  }
  if (LMIC.mcmd & (1UL<<MCMD_LADR_REQ)) {
    Serial.print(F("(LinkADR Pwr:"));
    Serial.print(LMIC.adrTxPow);
    Serial.print(F("dBm SF"));
    Serial.print(getSf(updr2rps(LMIC.datarate)) + 6);
    Serial.print(F(" ChMask:"));
    Serial.print(LMIC.channelMap,HEX);
    LMIC.mcmd &= ~(1UL<<MCMD_LADR_REQ);
    Serial.println(F(")"));
    valid = true;
  }
  if (LMIC.mcmd & (1UL<<MCMD_DCAP_REQ)) {
    Serial.print(F("(DutyCycle "));
    if (LMIC.opmode & OP_SHUTDOWN) {
      Serial.print(F("DISABLE TX)"));
    } else {
      Serial.print(LMIC.globalDutyRate);
      ostime_t airtime = calcAirTime(LMIC.rps, LMIC.dataLen);
      Serial.print(F(" Airtime:"));
      Serial.print((float) (airtime/us2osticks(10000000)) * (100-(1/(1<<LMIC.globalDutyRate))),2);
      Serial.println(F(")"));
    }
    valid = true;
    LMIC.mcmd &= ~(1UL<<MCMD_DCAP_REQ);
  }
  if (LMIC.mcmd & (1UL<<MCMD_DN2P_SET)) {
    Serial.print(F("(RXParams SF"));
    Serial.print(getSf(dndr2rps(LMIC.dn2Dr)) + 6);
    Serial.print(F(" Freq:"));
    float rx2freq = ( (float) LMIC.dn2Freq / 1000000);
    Serial.print(rx2freq, 1);
    LMIC.mcmd &= ~(1UL<<MCMD_DN2P_SET);
    Serial.println(F("MHz)"));
    valid = true;
  }
  if (LMIC.mcmd & (1UL<<MCMD_DEVS_REQ)) {
    Serial.print(F("(DevStatus Batt:"));
    Serial.print(os_getBattLevel());
    Serial.print(F(" Margin:"));
    Serial.print(LMIC.margin);
    Serial.println(F("dB)"));
    LMIC.mcmd &= ~(1UL<<MCMD_DEVS_REQ);
  }
  if (LMIC.mcmd & (1UL<<MCMD_SNCH_REQ)) {
    Serial.print(F("(NewChannel "));
    for( u1_t i=0; i<MAX_CHANNELS; i++) {
      if (LMIC.channelMap & (1<<i)) {
        Serial.print(i);
        Serial.print("=");
        float freq = ( (float) LMIC.channelFreq[i] / 1000000);
        Serial.println(freq, 1);
        Serial.print(F(" DR:"));
        Serial.print(LMIC.channelDrMap[i],HEX);
      }
    }
    Serial.println(F(")"));
    valid = true;
    LMIC.mcmd &= ~(1UL<<MCMD_SNCH_REQ);
  }
#ifdef CLASS_B
  if (LMIC.mcmd & (1UL<<MCMD_PING_SET)) {
    Serial.println(F("(PingSlot)"));
    valid = true;
    LMIC.mcmd &= ~(1UL<<MCMD_PING_SET);
  }
  if (LMIC.mcmd & (1UL<<MCMD_BCNI_ANS)) {
    Serial.println(F("(BeaconTiming)"));
    valid = true;
    LMIC.mcmd &= ~(1UL<<MCMD_BCNI_ANS);
  }
#endif
  if (LMIC.mcmd) {
    Serial.print(F("*** Unknown MAC:"));
    Serial.println(LMIC.mcmd, HEX);
    return true;
  }
  return valid;
}

#endif // LORA_INFO

void onEvent (ev_t ev) {

  ActivityLed();

  #if defined(LORA_INFO)
  ParseLMICMAC();

  lora_event = ev;
  Serial.print(F("Event: "));
  strcpy_P(buffer, (char*)pgm_read_ptr(&(EV_STRING[lora_event])));
  Serial.print(buffer);

  Serial.print(F(" ( "));
  ParseLMICOpmode();
  Serial.print(F(") "));
  #endif // LORA_INFO

  switch(ev) {
    case EV_LINK_ALIVE:
    case EV_RX1_DONE:
      #if defined(LORA_INFO)
      Serial.print(F("(RSSI:"));
      Serial.print(LMIC.rssi);
      Serial.print(F("dBm("));
      Serial.print(map(LMIC.rssi,0,255,-196,63));
      Serial.print(F(")"));
      Serial.print(F("dBm SNR:"));
      Serial.print(LMIC.snr/4);
      Serial.print(F("dB)"));
      #endif
      break;

    case EV_JOINED:
      LMIC_setLinkCheckMode(LINK_CHECK);
      break;
    case EV_REJOIN_FAILED:
    case EV_LINK_DEAD:
      LMIC_setLinkCheckMode(LINK_CHECK);
      LMIC_setLinkCheckRequestOnce(LINK_CHECK);
    case EV_OP_MODE:
    case EV_RXCOMPLETE:
    case EV_TXCOMPLETE: // INCLUDES RX2
      #if defined(LORA_INFO)
      Serial.print(F("(RSSI:"));
      Serial.print(LMIC.rssi);
      Serial.print(F("dBm("));
      Serial.print(map(LMIC.rssi,0,255,-196,63));
      Serial.print(F(")"));
      Serial.print(F("dBm SNR:"));
      Serial.print(LMIC.snr/4);
      Serial.print(F("dB)"));
      #endif
    case EV_TX_DONE:
      #if defined(LORA_INFO)
      lora_mode = LMIC.opmode;
      {
        if ( (LMIC.freq) ){
          uint8_t sf = getSf(LMIC.rps) + 6; // 1 == SF7
          uint8_t bw = getBw(LMIC.rps);
          uint8_t cr = getCr(LMIC.rps) + 5; // 4/x
          lmic_freq = ( (float) LMIC.freq / 1000000);
          Serial.print(lmic_freq, 1);
          Serial.print(F("MHz SF"));
          Serial.print(sf);
          Serial.print(F(" "));
          Serial.print(bw == BW125 ? 125 : (bw == BW250 ? 250 : 500));
          Serial.print(F("kHz CR4/"));
          Serial.print(cr);
          Serial.print(F(" IH:"));
          Serial.print(getIh(LMIC.rps));
          Serial.print(F(" Pow:"));
          Serial.print(LMIC.txpow);
          Serial.print(F("dBm Margin:"));
          Serial.print(LMIC.margin);
          Serial.print(F(" AckCnt:"));
          Serial.print(LMIC.adrAckReq);
          Serial.print(F(" Len: "));
          Serial.print(LMIC.dataLen);
        }
      }
      #endif
      if (ev == EV_TXCOMPLETE) {
        if (LMIC.txrxFlags & TXRX_ACK) {
          ActivityLed();
          Serial.print(F(" Received Ack"));
        } else if (LMIC.txrxFlags & TXRX_NACK) {
          ActivityLed();
          Serial.print(F(" Ack Timeout! "));
        }
        if (LMIC.dataLen) {
          ActivityLed();
          Serial.print(F("RX "));
          Serial.print(LMIC.dataLen);
          Serial.print(F(" bytes "));
          if (LMIC.txrxFlags & TXRX_PORT) {
            Serial.print(F("Port: "));
            Serial.print(LMIC.frame[LMIC.dataBeg-1]);
          }
          Serial.print(F(" HEX: "));
          PrintHex8(&LMIC.frame[LMIC.dataBeg],LMIC.dataLen);
          Serial.print(F(" ASCII: "));
          for (int pos = LMIC.dataBeg; pos < LMIC.dataBeg + LMIC.dataLen; pos++) {
            Serial.write(LMIC.frame[pos]);
          }
        }
        tx_completed = true;
      }
      break;
  }
  Serial.println();
}

void SendSensorData(osjob_t* j){
  // Check if there is not a current TX/RX job running
  ActivityLed();
  if ( ((LMIC.opmode & (OP_REJOIN|OP_LINKDEAD)) == 0 ) && (LMIC.opmode & OP_TXRXPEND) ) {
    Serial.println(F("TX already pending, abort!"));
  } else {
    Serial.print(F("Queue uplink packet, size: "));
    Serial.println(sizeof(struct tx_data_t));
    if(LMIC_setTxData2(TX_PORT, (xref2u1_t) &tx_data, sizeof(struct tx_data_t), UPLINK_ACK)) {
      Serial.println(F("Fail setTxData2"));
    }
  }
}

void LoRa_Session(u4_t netid, devaddr_t devaddr, xref2u1_t nwkKey, xref2u1_t artKey, uint8_t cfg) {
  LMIC.netid = netid;
  LMIC.devaddr = devaddr;
  os_copyMem(LMIC.nwkKey, nwkKey, 16);
  os_copyMem(LMIC.artKey, artKey, 16);
  if (cfg == LORA_DEFAULT_CH)  // EU868 DEFAULT 3 CHANNELS
    LMIC_initDefaultChannels(0);
  LMIC.opmode &= ~(OP_JOINING|OP_TRACK|OP_REJOIN|OP_TXRXPEND|OP_PINGINI);
  LMIC.opmode |= OP_NEXTCHNL;
  LMIC_stateJustJoined();
}

void LoRa_ReBoot(void) {

  LMIC_reset();

  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  #ifdef LORA_ABP
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LoRa_Session(0x1, DEVADDR, nwkskey, appskey, LORA_DEFAULT_CH);
    #else
    LoRa_Session(0x1, DEVADDR, NWKSKEY, APPSKEY, LORA_DEFAULT_CH);
    #endif

    #if !defined(LORA_DEFAULT_CH)
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);
    LMIC_setupBand(BAND_MILLI, 14, 1000); // 0.1%
    LMIC_setupBand(BAND_CENTI, 14, 100);  //   1%
    LMIC_setupBand(BAND_DECI, 27, 10);    //  10%
    #endif
    LMIC.enabledChannels = LMIC.channelMap;
    LMIC_setLinkCheckMode(LINK_CHECK);
    LMIC_setLinkCheckRequestOnce(LINK_CHECK);
    LMIC_setAdrMode(LINK_ADR);
    // Use SF9 for RX2 window.
    LMIC.dn2Dr = DR_SF9;
    LMIC_setDrTxpow(DR_SF7,14);
  #endif
}
