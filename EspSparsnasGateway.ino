 /*
  * TODO
  * "CRC ERR" in debug mode is not sent as Json
  * 
  */
 
 /*
 * Based on code from user Sommarlov @ EF: http://elektronikforumet.com/forum/viewtopic.php?f=2&t=85006&start=255#p1357610
 * Which in turn is based on Strigeus work: https://github.com/strigeus/sparsnas_decoder
 * 
 */

 // Settings for the Mqtt broker:

// Set this to the value of your energy meter
//#define PULSES_PER_KWH 1000 // <- samma här, står på elmätaren
// The code from the Sparnas tranmitter. Under the battery lid there's a sticker with digits like '400 643 654'.
// Set SENSOR_ID to the last six digits, ie '643654'.
//#define SENSOR_ID 643654 

#define DEBUG 1
//#define DEBUG_CONFIG 1

// You dont have to change anything below

#define appname "EspSparsnasGateway"

const char compile_date[] = __DATE__ " " __TIME__;

#include <FS.h>
#include "RFM69registers.h"
#include <Arduino.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

// Mqtt
#include <PubSubClient.h>
WiFiClient espClient;
PubSubClient client(espClient);

#define RF69_MODE_SLEEP 0      // XTAL OFF
#define RF69_MODE_STANDBY 1    // XTAL ON
#define RF69_MODE_SYNTH 2      // PLL ON
#define RF69_MODE_RX 3    // RX MODE
#define RF69_MODE_TX 4    // TX MODE

uint32_t FXOSC = 32000000;
uint32_t TwoPowerToNinteen = 524288; // 2^19
float RF69_FSTEP = (1.0 * FXOSC) / TwoPowerToNinteen; // p13 in datasheet
uint32_t FREQUENCY = 868000000;
uint16_t BITRATE = FXOSC / 40000; // 40kBps
uint16_t FREQUENCYDEVIATION = 10000 / RF69_FSTEP; // 10kHz
uint16_t SYNCVALUE = 0xd201;
uint8_t RSSITHRESHOLD = 210; // must be set to dBm = (-Sensitivity / 2), default is 0xE4 = 228 so -114dBm
uint8_t PAYLOADLENGTH = 20;

#define _interruptNum 5

static volatile uint8_t DATA[21];
static volatile uint8_t TEMPDATA[21];
static volatile uint8_t DATALEN;
static volatile uint16_t RSSI; // Most accurate RSSI during reception (closest to the reception)
static volatile uint8_t _mode;
static volatile bool inInterrupt = false; // Fake Mutex
uint8_t enc_key[5];

unsigned long lastCheckedForUpdate = millis();
unsigned long lastRecievedData = millis();

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40] = "192.168.168.1";
char mqtt_port[6] = "1883";
char mqtt_user[64] = "";
char mqtt_pass[64] = "";
char mqtt_status_topic[64] = "EspSparsnasGateway/values";
char pulses_per_kwh[10] = "1000";
char sensor_id[7] = "558109";
uint32 ui_pulses_per_kwh;
uint32 ui_sensor_id;

//flag for saving data
bool shouldSaveConfig = false;

// ----------------------------------------------------

// Callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Welcome to EspSparsnasGateway");
  Serial.print ("Compiled at:");
  Serial.println(compile_date);
  #ifdef DEBUG
     Serial.println("Debug on");
  #endif

#ifdef DEBUG_CONFIG
  Serial.println("Debug mode, erasing FS");
  SPIFFS.format();
#endif

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        //json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(mqtt_user, json["mqtt_user"]);
          strcpy(mqtt_pass, json["mqtt_pass"]);
          strcpy(mqtt_status_topic, json["mqtt_status_topic"]);
          strcpy(pulses_per_kwh, json["pulses_per_kwh"]);
          strcpy(sensor_id, json["sensor_id"]);
          
        } else {
          Serial.println("failed to load json config");
        }
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  //end read
  
  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_mqtt_user("user", "mqtt username", mqtt_user, 64);
  WiFiManagerParameter custom_mqtt_pass("pass", "mqtt password", mqtt_pass, 64);
  WiFiManagerParameter custom_mqtt_status_topic("status_topic", "mqtt status topic", mqtt_status_topic, 64);
  WiFiManagerParameter custom_pulses_per_kwh("pulses_per_kwh", "pulses/kwh", pulses_per_kwh, 10);
  WiFiManagerParameter custom_sensor_id("sensor_id", "sensor id (last 6 digits)", sensor_id, 7);

    //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
  
  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  wifiManager.addParameter(&custom_mqtt_status_topic);
  wifiManager.addParameter(&custom_pulses_per_kwh);
  wifiManager.addParameter(&custom_sensor_id);

  if (!wifiManager.autoConnect("EspSparsnasGateway")) {
    Serial.println("failed to connect and hit timeout. Sleeping for 10 seconds.");
    ESP.deepSleep(10*1e6);
  }
  WiFi.hostname(appname);

  //reset settings - for testing
#ifdef DEBUG_CONFIG
  wifiManager.resetSettings();
#endif

  //if you get here you have connected to the WiFi
  Serial.println("WiFi connected");

  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());
  strcpy(mqtt_status_topic, custom_mqtt_status_topic.getValue());
  strcpy(pulses_per_kwh, custom_pulses_per_kwh.getValue());
  strcpy(sensor_id, custom_sensor_id.getValue());

  //save the custom parameters to FS if needed
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["mqtt_user"] = mqtt_user;
    json["mqtt_pass"] = mqtt_pass;
    json["mqtt_status_topic"] = mqtt_status_topic;
    json["pulses_per_kwh"] = pulses_per_kwh;
    json["sensor_id"] = sensor_id;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  // We are connected, print the local IP
  Serial.println("");
  Serial.println("local ip");
  Serial.println(WiFi.localIP());

  ui_pulses_per_kwh = atoi(pulses_per_kwh);
  ui_sensor_id = atoi(sensor_id);
  
/*  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  WiFi.hostname(appname);
*/

 ArduinoOTA.setHostname(appname);
 ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();

  client.setServer(mqtt_server, 1883);
  //client.setCallback(callback); // What to do when a Mqtt message arrives
  if (!client.connected()) {
      reconnect();
  }

  // Publish some info, first via serial, then Mqtt
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  IPAddress ip = WiFi.localIP();
  char buf[60];
  sprintf(buf, "%s @ IP:%d.%d.%d.%d SSID: %s", appname, WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3], "");//ssid );
  client.publish(mqtt_status_topic, buf);
  
  // Calc encryption key, used for bytes 5-17
  const uint32_t sensor_id_sub = ui_sensor_id - 0x5D38E8CB;
  enc_key[0] = (uint8_t)(sensor_id_sub >> 24);
  enc_key[1] = (uint8_t)(sensor_id_sub);
  enc_key[2] = (uint8_t)(sensor_id_sub >> 8);
  enc_key[3] = 0x47;
  enc_key[4] = (uint8_t)(sensor_id_sub >> 16);

  if (!initialize(FREQUENCY)) {
    char mess[ ] = "Unable to initialize the radio. Exiting.";
    Serial.println(mess);
    client.publish(mqtt_status_topic, mess);

    while (1) {
      yield();
    }
  }
  #ifdef DEBUG
    String temp = "Listening on " + String(getFrequency()) + "hz. Done in setup.";
    char mess[100];
    temp.toCharArray(mess,100);
    Serial.println(temp);
    client.publish(mqtt_status_topic, mess);
  #endif
}

bool initialize(uint32_t frequency) {
  frequency = frequency / RF69_FSTEP;

  const uint8_t CONFIG[][2] = {
    /* 0x01 */ {REG_OPMODE, RF_OPMODE_SEQUENCER_ON | RF_OPMODE_LISTEN_OFF | RF_OPMODE_STANDBY},
    /* 0x02 */ {REG_DATAMODUL, RF_DATAMODUL_DATAMODE_PACKET | RF_DATAMODUL_MODULATIONTYPE_FSK | RF_DATAMODUL_MODULATIONSHAPING_01},
    /* 0x03 */ {REG_BITRATEMSB, (uint8_t)(BITRATE >> 8)},
    /* 0x04 */ {REG_BITRATELSB, (uint8_t)(BITRATE)},
    /* 0x05 */ {REG_FDEVMSB, (uint8_t)(FREQUENCYDEVIATION >> 8)},
    /* 0x06 */ {REG_FDEVLSB, (uint8_t)(FREQUENCYDEVIATION)},
    /* 0x07 */ {REG_FRFMSB, (uint8_t)(frequency >> 16)},
    /* 0x08 */ {REG_FRFMID, (uint8_t)(frequency >> 8)},
    /* 0x09 */ {REG_FRFLSB, (uint8_t)(frequency)},
    /* 0x19 */ {REG_RXBW, RF_RXBW_DCCFREQ_010 | RF_RXBW_MANT_16 | RF_RXBW_EXP_4}, // p26 in datasheet, filters out noise
    /* 0x25 */ {REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_01},             // PayloadReady
    /* 0x26 */ {REG_DIOMAPPING2, RF_DIOMAPPING2_CLKOUT_OFF},          // DIO5 ClkOut disable for power saving
    /* 0x28 */ {REG_IRQFLAGS2, RF_IRQFLAGS2_FIFOOVERRUN},             // writing to this bit ensures that the FIFO & status flags are reset
    /* 0x29 */ {REG_RSSITHRESH, RSSITHRESHOLD},                    
    /* 0x2D */ {REG_PREAMBLELSB, 3}, // default 3 preamble bytes 0xAAAAAA
    /* 0x2E */ {REG_SYNCCONFIG, RF_SYNC_ON | RF_SYNC_FIFOFILL_AUTO | RF_SYNC_SIZE_2 | RF_SYNC_TOL_0},
    /* 0x2F */ {REG_SYNCVALUE1, (uint8_t)(SYNCVALUE >> 8)},
    /* 0x30 */ {REG_SYNCVALUE2, (uint8_t)(SYNCVALUE)},
    /* 0x37 */ {REG_PACKETCONFIG1, RF_PACKET1_FORMAT_FIXED | RF_PACKET1_DCFREE_OFF | RF_PACKET1_CRC_OFF | RF_PACKET1_CRCAUTOCLEAR_ON | RF_PACKET1_ADRSFILTERING_OFF},
    /* 0x38 */ {REG_PAYLOADLENGTH, PAYLOADLENGTH},
    /* 0x3C */ {REG_FIFOTHRESH, RF_FIFOTHRESH_TXSTART_FIFONOTEMPTY | RF_FIFOTHRESH_VALUE},               // TX on FIFO not empty
    /* 0x3D */ {REG_PACKETCONFIG2, RF_PACKET2_RXRESTARTDELAY_2BITS | RF_PACKET2_AUTORXRESTART_ON | RF_PACKET2_AES_OFF}, // RXRESTARTDELAY must match transmitter PA ramp-down time (bitrate dependent)
    /* 0x6F */ {REG_TESTDAGC, RF_DAGC_IMPROVED_LOWBETA0}, // run DAGC continuously in RX mode for Fading Margin Improvement, recommended default for AfcLowBetaOn=0
    {255, 0}
  };

  digitalWrite(SS, HIGH);
  pinMode(SS, OUTPUT);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  // decided to slow down from DIV2 after SPI stalling in some instances,
  // especially visible on mega1284p when RFM69 and FLASH chip both present
  SPI.setClockDivider(SPI_CLOCK_DIV4);

  unsigned long start = millis();
  uint8_t timeout = 50;
  do {
    writeReg(REG_SYNCVALUE1, 0xAA);
    yield();
  } while (readReg(REG_SYNCVALUE1) != 0xaa && millis() - start < timeout);
  start = millis();
  do {
    writeReg(REG_SYNCVALUE1, 0x55);
    yield();
  } while (readReg(REG_SYNCVALUE1) != 0x55 && millis() - start < timeout);

  for (uint8_t i = 0; CONFIG[i][0] != 255; i++) {
    writeReg(CONFIG[i][0], CONFIG[i][1]);
    yield();
  }

  setMode(RF69_MODE_STANDBY);
  start = millis();
  while (((readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) && millis() - start < timeout) {
    // wait for ModeReady
    //codeLibrary.wait(1);
    delay(1);
  }
  if (millis() - start >= timeout) {
    char mess[ ] = "Failed on waiting for ModeReady()";
    Serial.println(mess);
    client.publish(mqtt_status_topic, mess);
    return false;
  }
  attachInterrupt(_interruptNum, interruptHandler, RISING);

  #ifdef DEBUG
    char mess[ ] = "RFM69 init done";
    Serial.println(mess);
    client.publish(mqtt_status_topic, mess);
  #endif
  return true;
}

uint32_t getFrequency() {
  return RF69_FSTEP * (((uint32_t)readReg(REG_FRFMSB) << 16) + ((uint16_t)readReg(REG_FRFMID) << 8) + readReg(REG_FRFLSB));
}

void receiveBegin() {
  DATALEN = 0;
  RSSI = 0;
  if (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY) {
    uint8_t val = readReg(REG_PACKETCONFIG2);
    // avoid RX deadlocks
    writeReg(REG_PACKETCONFIG2, (val & 0xFB) | RF_PACKET2_RXRESTART);
  }
  setMode(RF69_MODE_RX);
}

// checks if a packet was received and/or puts transceiver in receive (ie RX or listen) mode
bool receiveDone() {
  // noInterrupts(); // re-enabled in unselect() via setMode() or via
  // receiveBegin()

  if (_mode == RF69_MODE_RX && DATALEN > 0) {
    setMode(RF69_MODE_STANDBY); // enables interrupts
    return true;

  } else if (_mode == RF69_MODE_RX) {
    // already in RX no payload yet
    // interrupts(); // explicitly re-enable interrupts
    return false;
  }

  receiveBegin();
  return false;
}

// get the received signal strength indicator (RSSI)
uint16_t readRSSI(bool forceTrigger = false) {
  uint16_t rssi = 0;
  if (forceTrigger) {
    // RSSI trigger not needed if DAGC is in continuous mode
    writeReg(REG_RSSICONFIG, RF_RSSI_START);
    while ((readReg(REG_RSSICONFIG) & RF_RSSI_DONE) == 0x00) {
      // wait for RSSI_Ready
      yield();
    }
  }
  rssi = -readReg(REG_RSSIVALUE);
  rssi >>= 1;
  return rssi;
}

uint8_t readReg(uint8_t addr) {
  select();
  SPI.transfer(addr & 0x7F);
  uint8_t regval = SPI.transfer(0);
  unselect();
  return regval;
}

void writeReg(uint8_t addr, uint8_t value) {
  select();
  SPI.transfer(addr | 0x80);
  SPI.transfer(value);
  unselect();
}

// select the RFM69 transceiver (save SPI settings, set CS low)
void select() {
  // noInterrupts();
  digitalWrite(SS, LOW);
}

// unselect the RFM69 transceiver (set CS high, restore SPI settings)
void unselect() {
  digitalWrite(SS, HIGH);
  // interrupts();
}

void interruptHandler() {
  
  if (inInterrupt) {
    //Serial.println("Already in interruptHandler.");
    return;
  }
  inInterrupt = true;


  if (_mode == RF69_MODE_RX && (readReg(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY)) {
    setMode(RF69_MODE_STANDBY);
    DATALEN = 0;
    select();

    // Init reading
    SPI.transfer(REG_FIFO & 0x7F);

    // Read 20 bytes
    for (uint8_t i = 0; i < 20; i++) {
      TEMPDATA[i] = SPI.transfer(0);
    }

    // CRC is done BEFORE decrypting message
    uint16_t crc = crc16(TEMPDATA, 18);
    uint16_t packet_crc = TEMPDATA[18] << 8 | TEMPDATA[19];

    #ifdef DEBUG
       Serial.println("Got rf data");
       Serial.print("Sensor ID: ");
       Serial.println(ui_sensor_id);
       Serial.print("Pulses/kWh ");
       Serial.println(ui_pulses_per_kwh);
    #endif

    // Decrypt message
    for (size_t i = 0; i < 13; i++) {
      TEMPDATA[5 + i] = TEMPDATA[5 + i] ^ enc_key[i % 5];
    }

    uint32_t rcv_sensor_id = TEMPDATA[5] << 24 | TEMPDATA[6] << 16 | TEMPDATA[7] << 8 | TEMPDATA[8];
    Serial.print("Recevied sensor ID: ");
    Serial.println(rcv_sensor_id);
    String output;

    // Bug fix from https://github.com/strigeus/sparsnas_decoder/pull/7/files
    // if (data_[0] != 0x11 || data_[1] != (SENSOR_ID & 0xFF) || data_[3] != 0x07 || rcv_sensor_id != SENSOR_ID) { 
    // if (TEMPDATA[0] != 0x11 || TEMPDATA[1] != (SENSOR_ID & 0xFF) || TEMPDATA[3] != 0x07 || TEMPDATA[4] != 0x0E || rcv_sensor_id != SENSOR_ID) {
    if (TEMPDATA[0] != 0x11 || TEMPDATA[1] != (ui_sensor_id & 0xFF) || TEMPDATA[3] != 0x07 || rcv_sensor_id != ui_sensor_id) {
      
      Serial.print("Bad package: ");
      for (int i = 0; i < 18; i++) {
        Serial.print(TEMPDATA[i], HEX);
        Serial.print(" ");
      }
      Serial.println("");
      
    } else {
      /*
        0: uint8_t length;        // Always 0x11
        1: uint8_t sender_id_lo;  // Lowest byte of sender ID
        2: uint8_t unknown;       // Not sure
        3: uint8_t major_version; // Always 0x07 - the major version number of the sender.
        4: uint8_t minor_version; // Always 0x0E - the minor version number of the sender.
        5: uint32_t sender_id;    // ID of sender
        9: uint16_t time;         // Time in units of 15 seconds.
        11:uint16_t effect;       // Current effect usage
        13:uint32_t pulses;       // Total number of pulses
        17:uint8_t battery;       // Battery level, 0-100.
      */

      Serial.print("Good package: ");
      for (int i = 0; i < 18; i++) {
        Serial.print(TEMPDATA[i], HEX);
        Serial.print(" ");
      }
      Serial.println("");

      int seq = (TEMPDATA[9] << 8 | TEMPDATA[10]);
      int power = (TEMPDATA[11] << 8 | TEMPDATA[12]);
      int pulse = (TEMPDATA[13] << 24 | TEMPDATA[14] << 16 | TEMPDATA[15] << 8 | TEMPDATA[16]);
      int battery = TEMPDATA[17];
      
      // Bug fix from https://github.com/strigeus/sparsnas_decoder/pull/7/files
      // float watt =  (float)((3600000 / ui_pulses_per_kwh) * 1024) / (power);
      float watt = power * 24;
      int data4 = TEMPDATA[4]^0x0f;
      //  Note that data_[4] cycles between 0-3 when you first put in the batterys in t$
      if(data4 == 1){
           watt = (float)((3600000 / ui_pulses_per_kwh) * 1024) / (power);
      }
      /* m += sprintf(m, "%5d: %7.1f W. %d.%.3d kWh. Batt %d%%. FreqErr: %.2f", seq, watt, pulse/PULSES_PER_KWH, pulse%PULSES_PER_KWH, battery, freq);
      'So in the example 10 % 3, 10 divided by 3 is 3 with remainder 1, so the answer is 1.'
      */
      output = "Seq " + String(seq) + ": ";
      output += String(watt) + " W, total: ";
      output += String(pulse / ui_pulses_per_kwh) + " kWh, battery ";
      output += String(battery) + "% ";

      output += (crc == packet_crc ? "" : "CRC ERR");
      String err = (crc == packet_crc ? "" : "CRC ERR");
      Serial.println(output);

    if (err=="CRC ERR") {
      Serial.println(err);
      #ifdef DEBUG
        client.publish(mqtt_status_topic, "CRC ERR");
      #endif

    }
    else {
      //For Json output
      StaticJsonBuffer<150> jsonBuffer;
      JsonObject& root = jsonBuffer.createObject();
      char msg[150];
      root["seq"] = seq;
      root["power"] = String(watt);
      root["total"] = String(pulse / ui_pulses_per_kwh);
      root["battery"] = battery;
      root.printTo((char*)msg, root.measureLength() + 1);
      client.publish(mqtt_status_topic, msg);  // Wants a char
    }
    }

    unselect();
    setMode(RF69_MODE_RX);
  }
  RSSI = readRSSI();

  inInterrupt = false;
}

void setMode(uint8_t newMode) {
  if (newMode == _mode) {
    return;
  }

  uint8_t val = readReg(REG_OPMODE);
  switch (newMode) {
    case RF69_MODE_TX:
      writeReg(REG_OPMODE, (val & 0xE3) | RF_OPMODE_TRANSMITTER);
      break;
    case RF69_MODE_RX:
      writeReg(REG_OPMODE, (val & 0xE3) | RF_OPMODE_RECEIVER);
      break;
    case RF69_MODE_SYNTH:
      writeReg(REG_OPMODE, (val & 0xE3) | RF_OPMODE_SYNTHESIZER);
      break;
    case RF69_MODE_STANDBY:
      writeReg(REG_OPMODE, (val & 0xE3) | RF_OPMODE_STANDBY);
      break;
    case RF69_MODE_SLEEP:
      writeReg(REG_OPMODE, (val & 0xE3) | RF_OPMODE_SLEEP);
      break;
    default:
      return;
  }

  // we are using packet mode, so this check is not really needed but waiting for mode ready is necessary when
  // going from sleep because the FIFO may not be immediately available from previous mode.
  unsigned long start = millis();
  uint16_t timeout = 500;
  while (_mode == RF69_MODE_SLEEP && millis() - start < timeout && (readReg(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00) {
    // wait for ModeReady
    yield();
  }
  if (millis() - start >= timeout) {
      //Timeout when waiting for getting out of sleep
  }

  _mode = newMode;
}


uint16_t crc16(volatile uint8_t *data, size_t n) {
  uint16_t crcReg = 0xffff;
  size_t i, j;
  for (j = 0; j < n; j++) {
    uint8_t crcData = data[j];
    for (i = 0; i < 8; i++) {
      if (((crcReg & 0x8000) >> 8) ^ (crcData & 0x80))
        crcReg = (crcReg << 1) ^ 0x8005;
      else
        crcReg = (crcReg << 1);
      crcData <<= 1;
    }
  }
  return crcReg;
}

void loop() {
  ArduinoOTA.handle();

  // Mqtt
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  if (receiveDone()) {
    lastRecievedData = millis();
    // Send data to Mqtt server
    Serial.println("We got data to send.");
    // Wait a bit
    //codeLibrary.wait(500);
    delay(500);
  }
}

void reconnect() {
  boolean r;
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if(strlen(mqtt_user)) {
      r = client.connect(appname, mqtt_user, mqtt_pass);
    } else {
      r = client.connect(appname);
    }
    if (r) {
      Serial.println("Connected to Mqtt broker");
    } else {
      Serial.print("Mqtt connection failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 30 seconds");
      // Wait 30 seconds before retrying
      delay(30000);
    }
  }
}
