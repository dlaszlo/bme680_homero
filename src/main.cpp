#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <bsec.h>

#include "settings.h"

#define WIFI_TIMEOUT (60 * 1000)
#define LOOP_DELAY (1 * 1000)
#define SERIAL_BAUD 9600
#define MQTT_SEND_TIMEOUT (1 * 60 * 1000)

const uint8_t bsec_config_iaq[] = {
#include "config/generic_33v_3s_4d/bsec_iaq.txt"
};

#define STATE_SAVE_PERIOD UINT32_C(360 * 60 * 1000)

uint64 wifiStartTime;
uint64 mqttSendTime;
boolean connected;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

Bsec iaqSensor;
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;

void errorLeds(void)
{
  for (uint8_t i = 0; i < 50; i++)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
  }
  ESP.restart();
}

void connectMqtt()
{
  if (!mqttClient.connected())
  {
    Serial.print(F("MQTT: Csatlakozás: "));
    Serial.print(mqtt_server);
    Serial.print(F(":"));
    Serial.println(mqtt_port);
    if (mqttClient.connect(mqtt_client, mqtt_user, mqtt_password))
    {
      Serial.println(F("MQTT: Csatlakozva"));
    }
    else
    {
      Serial.print(F("MQTT: A csatlakozás sikertelen: "));
      Serial.println(mqttClient.state());
      errorLeds();
    }
  }
}

void checkWiFi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(LED_BUILTIN, LOW);
    if (connected)
    {
      connected = false;
      wifiStartTime = millis();
      Serial.println(F("WiFi: Nincs csatlakozva"));
    }
    if ((wifiStartTime + WIFI_TIMEOUT) < millis())
    {
      Serial.println(F("WiFi: Csatlakozás időtúllépés"));
      errorLeds();
    }
  }
  else
  {
    digitalWrite(LED_BUILTIN, HIGH);
    if (!connected)
    {
      Serial.print(F("WiFi: Csatlakozva, IP cím: "));
      Serial.println(WiFi.localIP());
      delay(1000);
      connectMqtt();
      connected = true;
    }
    mqttClient.loop();
  }
}

void checkIaqSensorStatus()
{
  if (iaqSensor.status != BSEC_OK)
  {
    Serial.print(F("BSEC státusz: "));
    Serial.println(iaqSensor.status);
    if (iaqSensor.status < BSEC_OK)
    {
      errorLeds();
    }
  }
  if (iaqSensor.bme680Status != BME680_OK)
  {
    Serial.print(F("BME680 státusz: "));
    Serial.println(iaqSensor.bme680Status);
    if (iaqSensor.bme680Status < BME680_OK)
    {
      errorLeds();
    }
  }
}

void loadState()
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE)
  {
    Serial.println("EEPROM: BSEC státusz betöltése");
    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
      bsecState[i] = EEPROM.read(i + 1);
      Serial.print(bsecState[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  }
  else
  {
    Serial.println("EEPROM: Törlés");
    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
    {
      EEPROM.write(i, 0);
    }
    EEPROM.commit();
  }
}

void updateState()
{
  boolean update = false;
  if (stateUpdateCounter == 0)
  {
    if (iaqSensor.iaqAccuracy >= 3)
    {
      update = true;
      stateUpdateCounter++;
    }
  }
  else
  {
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis())
    {
      update = true;
      stateUpdateCounter++;
    }
  }
  if (update)
  {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();
    Serial.println("EEPROM: BSEC státusz mentése");
    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++)
    {
      EEPROM.write(i + 1, bsecState[i]);
      Serial.print(bsecState[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}

void setupBME680()
{
  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1);
  iaqSensor.begin(BME680_I2C_ADDR_SECONDARY, Wire);

  Serial.print(F("BSEC library version "));
  Serial.print(iaqSensor.version.major);
  Serial.print(F("."));
  Serial.print(iaqSensor.version.minor);
  Serial.print(F("."));
  Serial.print(iaqSensor.version.major_bugfix);
  Serial.print(F("."));
  Serial.println(iaqSensor.version.minor_bugfix);

  checkIaqSensorStatus();

  iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();

  loadState();

  bsec_virtual_sensor_t sensorList[13] = {
      BSEC_OUTPUT_RAW_TEMPERATURE,
      BSEC_OUTPUT_RAW_PRESSURE,
      BSEC_OUTPUT_RAW_HUMIDITY,
      BSEC_OUTPUT_RAW_GAS,
      BSEC_OUTPUT_IAQ,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
      BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
      BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
      BSEC_OUTPUT_CO2_EQUIVALENT,
      BSEC_OUTPUT_COMPENSATED_GAS,
      BSEC_OUTPUT_GAS_PERCENTAGE,
      BSEC_OUTPUT_STATIC_IAQ,
      BSEC_OUTPUT_STABILIZATION_STATUS
  };

  iaqSensor.updateSubscription(sensorList, 13, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();

  Serial.println(F("time, rawTemperature, pressure, rawHumidity, gasResistance, gasResistance, iaq, iaqAccuracy, temperature, humidity, breathVocEquivalent, breathVocAccuracy, co2Equivalent, co2Accuracy, compGasValue, compGasAccuracy, gasPercentage, gasPercentageAcccuracy, staticIaq, staticIaqAccuracy, stabStatus"));

}

void publish(char *payload, int length)
{
  connectMqtt();
  if (mqttClient.connected())
  {
    Serial.print(F("MQTT: adatok küldése: "));
    if (mqttClient.publish(mqtt_topic, payload))
    {
      Serial.print(mqtt_topic);
      Serial.print(F(" = "));
      Serial.println(payload);
    }
    else
    {
      Serial.println(F("az adatok küldése nem sikerült"));
      errorLeds();
    }
  }
  else
  {
    Serial.println(F("MQTT: Az MQTT kliens nincs csatlakozva, az adatok küldése nem sikerült"));
    errorLeds();
  }
}

void setup(void)
{
  connected = false;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(SERIAL_BAUD);
  Wire.begin();

  setupBME680();

  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_STA); 
  delay(100);
  WiFi.begin(ssid, password);
  delay(100);
  
  mqttClient.setServer(mqtt_server, mqtt_port);

  wifiStartTime = millis();
  mqttSendTime = millis();
}

void loop(void)
{
  checkWiFi();

  unsigned long time_trigger = millis();
  if (iaqSensor.run())
  {
    Serial.print(time_trigger);
    Serial.print(F(", "));
    Serial.print(iaqSensor.rawTemperature);
    Serial.print(F(", "));
    Serial.print(iaqSensor.pressure);
    Serial.print(F(", "));
    Serial.print(iaqSensor.rawHumidity);
    Serial.print(F(", "));
    Serial.print(iaqSensor.gasResistance);
    Serial.print(F(", "));
    Serial.print(iaqSensor.iaq);
    Serial.print(F(", "));
    Serial.print(iaqSensor.iaqAccuracy);
    Serial.print(F(", "));
    Serial.print(iaqSensor.temperature);
    Serial.print(F(", "));
    Serial.print(iaqSensor.humidity);
    Serial.print(F(", "));
    Serial.print(iaqSensor.breathVocEquivalent);
    Serial.print(F(", "));
    Serial.print(iaqSensor.breathVocAccuracy);
    Serial.print(F(", "));
    Serial.print(iaqSensor.co2Equivalent);
    Serial.print(F(", "));
    Serial.print(iaqSensor.co2Accuracy);
    Serial.print(F(", "));
    Serial.print(iaqSensor.compGasValue);
    Serial.print(F(", "));
    Serial.print(iaqSensor.compGasAccuracy);
    Serial.print(F(", "));
    Serial.print(iaqSensor.gasPercentage);
    Serial.print(F(", "));
    Serial.print(iaqSensor.gasPercentageAcccuracy);
    Serial.print(F(", "));
    Serial.print(iaqSensor.staticIaq);
    Serial.print(F(", "));
    Serial.print(iaqSensor.staticIaqAccuracy);
    Serial.print(F(", "));
    Serial.println(iaqSensor.stabStatus);

    if (connected && (mqttSendTime + MQTT_SEND_TIMEOUT) < millis())
    {
      digitalWrite(LED_BUILTIN, LOW);
      StaticJsonDocument<1000> doc;
      JsonObject root = doc.to<JsonObject>();
      root["time"] = time_trigger;
      root["rawTemperature"] = iaqSensor.rawTemperature;
      root["pressure"] = iaqSensor.pressure;
      root["rawHumidity"] = iaqSensor.rawHumidity;
      root["gasResistance"] = iaqSensor.gasResistance;
      root["iaq"] = iaqSensor.iaq;
      root["iaqAccuracy"] = iaqSensor.iaqAccuracy;
      root["temperature"] = iaqSensor.temperature;
      root["humidity"] = iaqSensor.humidity;
      root["breathVocEquivalent"] = iaqSensor.breathVocEquivalent;
      root["breathVocAccuracy"] = iaqSensor.breathVocAccuracy;
      root["co2Equivalent"] = iaqSensor.co2Equivalent;
      root["co2Accuracy"] = iaqSensor.co2Accuracy;
      root["compGasValue"] = iaqSensor.compGasValue;
      root["compGasAccuracy"] = iaqSensor.compGasAccuracy;
      root["gasPercentage"] = iaqSensor.gasPercentage;
      root["gasPercentageAcccuracy"] = iaqSensor.gasPercentageAcccuracy;
      root["staticIaq"] = iaqSensor.staticIaq;
      root["staticIaqAccuracy"] = iaqSensor.staticIaqAccuracy;
      root["stabStatus"] = iaqSensor.stabStatus;

      char json[1000];
      int length = measureJson(doc) + 1;
      serializeJson(doc, json, length);

      publish(json, length);

      mqttSendTime = millis();
    }

    updateState();
  }
  else
  {
    checkIaqSensorStatus();
  }

  delay(LOOP_DELAY);
}
