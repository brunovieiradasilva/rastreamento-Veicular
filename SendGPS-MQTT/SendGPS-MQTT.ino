#ifndef SerialAT
#define SerialAT Serial1
#endif

#ifndef SerialGPS
#define SerialGPS Serial2
#endif

#define BOARD_MODEM_DTR_PIN 25
#define BOARD_MODEM_TX_PIN 26
#define BOARD_MODEM_RX_PIN 27
#define BOARD_MODEM_PWR_PIN 4
#define BOARD_ADC_PIN 35
#define BOARD_POWER_ON_PIN 12
#define BOARD_MODEM_RI_PIN 33
#define BOARD_RST_PIN 5
#define BOARD_SDCARD_MISO 2
#define BOARD_SDCARD_MOSI 15
#define BOARD_SDCARD_SCLK 14
#define BOARD_SDCARD_CS 13

#define BOARD_GPS_TX_PIN 21
#define BOARD_GPS_RX_PIN 22
#define BOARD_GPS_PPS_PIN 23
#define BOARD_GPS_WAKEUP_PIN 19
#define TINY_GSM_RX_BUFFER 1024

#define DUMP_AT_COMMANDS
#include "utilities.h"
#include <TinyGsmClient.h>
#include <Arduino.h>
#include <TinyGPS++.h>
TinyGPSPlus gps;

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// #define NETWORK_APN     ""             

// MQTT
const char *broker = "io.adafruit.com";
const uint16_t broker_port = 8883;
const char *broker_username = "user";
const char *broker_password = "password";
const char *client_id = "id";

const char *loc_topic = "user/feeds/loc/csv";
const char *loctxt_topic = "user/feeds/loctxt";
const uint8_t mqtt_client_id = 0;
uint32_t check_connect_millis = 0;

bool mqtt_connect() {
  Serial.print("Conectando no broker  ");
  Serial.print(broker);

  bool ret = modem.mqtt_connect(mqtt_client_id, broker, broker_port, client_id, broker_username, broker_password);
  if (!ret) {
    Serial.println("Falha!");
    return false;
  }
  Serial.println("Sucesso.");

  if (modem.mqtt_connected()) {
    Serial.println("Conexão MQTT realizada com sucesso!");
  } else {
    return false;
  }
  return true;
}


void setup() {
  pinMode(BOARD_POWER_ON_PIN, OUTPUT);
  digitalWrite(BOARD_POWER_ON_PIN, HIGH);

  pinMode(BOARD_RST_PIN, OUTPUT);
  digitalWrite(BOARD_RST_PIN, LOW);

  pinMode(BOARD_MODEM_PWR_PIN, OUTPUT);
  digitalWrite(BOARD_MODEM_PWR_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_MODEM_PWR_PIN, HIGH);
  delay(1000);
  digitalWrite(BOARD_MODEM_PWR_PIN, LOW);

  Serial.begin(115200);
  //Modem Serial port
  SerialAT.begin(115200, SERIAL_8N1, BOARD_MODEM_RX_PIN, BOARD_MODEM_TX_PIN);
  //GPS Serial port
  SerialGPS.begin(9600, SERIAL_8N1, BOARD_GPS_RX_PIN, BOARD_GPS_TX_PIN);

  delay(2000);

  Serial.println("Start Sketch");

#ifdef BOARD_POWERON_PIN
  pinMode(BOARD_POWERON_PIN, OUTPUT);
  digitalWrite(BOARD_POWERON_PIN, HIGH);
#endif

  pinMode(MODEM_RESET_PIN, OUTPUT);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
  delay(100);
  digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
  delay(2600);
  digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);

  pinMode(BOARD_PWRKEY_PIN, OUTPUT);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, HIGH);
  delay(100);
  digitalWrite(BOARD_PWRKEY_PIN, LOW);

  Serial.println("Inicializando modem...");

  int retry = 0;
  while (!modem.testAT(1000)) {
    Serial.println(".");
    if (retry++ > 10) {
      digitalWrite(BOARD_PWRKEY_PIN, LOW);
      delay(100);
      digitalWrite(BOARD_PWRKEY_PIN, HIGH);
      delay(1000);
      digitalWrite(BOARD_PWRKEY_PIN, LOW);
      retry = 0;
    }
  }
  Serial.println();

  // Check if SIM card is online
  SimStatus sim = SIM_ERROR;
  while (sim != SIM_READY) {
    sim = modem.getSimStatus();
    switch (sim) {
      case SIM_READY:
        Serial.println("SIM card online");
        break;
      case SIM_LOCKED:
        Serial.println("Cartão SIM trancado.");
        // const char *SIMCARD_PIN_CODE = "123456";
        // modem.simUnlock(SIMCARD_PIN_CODE);
        break;
      default:
        break;
    }
    delay(1000);
  }

#ifndef TINY_GSM_MODEM_SIM7672
  if (!modem.setNetworkMode(MODEM_NETWORK_AUTO)) {
    Serial.println("Set network mode failed!");
  }
  String mode = modem.getNetworkModes();
  Serial.print("Current network mode : ");
  Serial.println(mode);
#endif

#ifdef NETWORK_APN
  Serial.printf("Set network apn : %s\n", NETWORK_APN);
  modem.sendAT(GF("+CGDCONT=1,\"IP\",\""), NETWORK_APN, "\"");
  if (modem.waitResponse() != 1) {
    Serial.println("Set network apn error !");
  }
#endif

  //network registration and signal status
  int16_t sq;
  Serial.print("Wait for the modem to register with the network.");
  RegStatus status = REG_NO_RESULT;
  while (status == REG_NO_RESULT || status == REG_SEARCHING || status == REG_UNREGISTERED) {
    status = modem.getRegistrationStatus();
    switch (status) {
      case REG_UNREGISTERED:
      case REG_SEARCHING:
        sq = modem.getSignalQuality();
        Serial.printf("[%lu] Qualidade do Sinal:%d\n", millis() / 1000, sq);
        delay(1000);
        break;
      case REG_DENIED:
        Serial.println("Registro na rede negado, verifique a APN.");
        return;
      case REG_OK_HOME:
        Serial.println("Registro online realizado com sucesso.");
        break;
      case REG_OK_ROAMING:
        Serial.println("Registro online realizado com sucesso, modo roaming.");
        break;
      default:
        Serial.printf("Status de Registro:%d\n", status);
        delay(1000);
        break;
    }
  }
  Serial.println();


  Serial.printf("Status de Registro:%d\n", status);
  delay(1000);

  String ueInfo;
  if (modem.getSystemInformation(ueInfo)) {
    Serial.print("Inquiring UE system information:");
    Serial.println(ueInfo);
  }

  if (!modem.setNetworkActive()) {
    Serial.println("Falha na ativação da rede!");
  }

  delay(5000);

  String ipAddress = modem.getLocalIP();
  Serial.print("IP na rede:");
  Serial.println(ipAddress);

  // Initialize MQTT
  modem.mqtt_begin(true);


  if (!mqtt_connect()) {
    return;
  }

  if (millis() > check_connect_millis) {
    check_connect_millis = millis() + 10000UL;
    if (!modem.mqtt_connected()) {
      mqtt_connect();
    } else {
      String payload = "RunTime:" + String(millis() / 1000);
    }
  }
  modem.mqtt_handle();
  delay(5);
}

void loop() {

  while (SerialGPS.available()) {
    int c = SerialGPS.read();
    if (gps.encode(c)) {
      displayInfo();
    }
  }

  if (millis() > 30000 && gps.charsProcessed() < 10) {
    Serial.println(F("GPS não conectado"));
    delay(1000);
  }

  delay(1);
}

void displayInfo() {
  if (gps.location.isValid() && millis() > 1200000) {
    char sendbuffer[100];
    char loctxt[100];
    char *p = sendbuffer;
    char *p2 = loctxt;
    // Velocidade
    dtostrf(0, 2, 6, p);
    p += strlen(p);
    p[0] = ',';
    p++;
    // latitude
    dtostrf(gps.location.lat(), 2, 6, p);
    p += strlen(p);
    p[0] = ',';
    p++;
    // longitude
    dtostrf(gps.location.lng(), 3, 6, p);
    p += strlen(p);
    p[0] = ',';
    p++;
    // altitude
    dtostrf(0, 2, 6, p);
    p += strlen(p);
    p[0] = 0;

    // latitude
    dtostrf(gps.location.lat(), 2, 6, p2);
    p2 += strlen(p2);
    p2[0] = ',';
    p2++;
    // longitude
    dtostrf(gps.location.lng(), 3, 6, p2);
    p2 += strlen(p2);
    p[0] = 0;

    Serial.println(sendbuffer);
    Serial.println(loctxt);
    modem.mqtt_publish(0, loc_topic, sendbuffer);
    delay(1000);
    modem.mqtt_publish(0, loctxt_topic, loctxt);
  }

  Serial.print(F("Localização: "));
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print(F("INVALIDO"));
  }

  Serial.print(F("  Data/Hora: "));
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  } else {
    Serial.print(F("INVALIDO"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid()) {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  } else {
    Serial.print(F("INVALIDO"));
  }

  Serial.println();
  delay(7000);
}


#ifndef TINY_GSM_FORK_LIBRARY
#error "No correct definition detected, Please copy all the [lib directories](https://github.com/Xinyuan-LilyGO/LilyGO-T-A76XX/tree/main/lib) to the arduino libraries directory , See README"
#endif