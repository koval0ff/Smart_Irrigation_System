#include <WiFi.h>
#include <WiFiManager.h>
#include <ArduinoOTA.h>
#include <WebServer.h>

#include <ModbusIP_ESP8266.h>

#include <Wire.h>
#include <Adafruit_INA3221.h>

#include <OneWire.h>
#include <DallasTemperature.h>

/*
 * ============================================================
 * Smart Irrigation System
 * ESP32 Controller Firmware
 * ------------------------------------------------------------
 *
 * Author : Danylo Kovalov
 * Version: 2.1
 *
 * Description:
 * Firmware for the ESP32 controller used in the
 * Smart Irrigation System project.
 *
 * Responsibilities:
 * - Read connected sensors
 * - Publish process values through Modbus TCP
 * - Receive the pump command from OpenPLC
 * - Control the physical pump output
 * - Provide a local diagnostic web interface
 * - Support OTA firmware updates
 *
 * The irrigation decision logic is implemented in OpenPLC.
 *
 * Repository:
 * https://github.com/koval0ff/Smart_Irrigation_System
 * ============================================================
 */

// =====================================================
// ESP32 DEVKIT PIN CONFIGURATION
// =====================================================

const uint8_t MOISTURE_PIN = 32;
const uint8_t LIGHT_PIN = 34;
const uint8_t TEMPERATURE_PIN = 4;

const uint8_t MOTOR_PIN = 21;

// Float Hall-effect sensor
const uint8_t HALL_PIN = 27;

// INA3221
const uint8_t I2C_SDA_PIN = 25;
const uint8_t I2C_SCL_PIN = 26;

// =====================================================
// MOTOR OUTPUT
// =====================================================

const uint8_t MOTOR_ON_LEVEL = HIGH;
const uint8_t MOTOR_OFF_LEVEL = LOW;

/*
  If the relay or motor driver is active LOW, use:

  const uint8_t MOTOR_ON_LEVEL = LOW;
  const uint8_t MOTOR_OFF_LEVEL = HIGH;
*/

// =====================================================
// HALL-EFFECT SENSOR
// =====================================================

/*
  Most digital Hall-effect sensor modules are active LOW:

  Magnet detected:
  GPIO = LOW

  Magnet not detected:
  GPIO = HIGH
*/
const uint8_t HALL_ACTIVE_LEVEL = LOW;

/*
  true:
  A detected magnet means the float has moved down
  and the water tank is empty.

  If the mechanical arrangement works in reverse,
  change this value to false.
*/
const bool HALL_MAGNET_MEANS_WATER_EMPTY = true;

// Hall sensor debounce time
const unsigned long HALL_DEBOUNCE_MS = 50;

// =====================================================
// INA3221
// =====================================================

const uint8_t INA3221_ADDRESS = 0x40;

/*
  INA3221 library channel numbering:

  0 = CH1
  1 = CH2
  2 = CH3
*/
const uint8_t INA_MOTOR_CHANNEL = 0;

// R100 shunt resistor
const float INA_SHUNT_RESISTANCE_OHMS = 0.1f;

Adafruit_INA3221 ina3221;

bool inaFound = false;
bool inaOk = false;

uint8_t inaErrorCount = 0;
const uint8_t INA_ERROR_LIMIT = 5;

// =====================================================
// MODBUS MAP: INPUT REGISTERS
// =====================================================

/*
  FC4 - Input Registers

  0 - Soil moisture, raw 0...4095
  1 - Light level, raw 0...4095
  2 - Temperature multiplied by 100
  3 - DS18B20 status
  4 - Battery voltage in millivolts
  5 - Pump current in milliamps
  6 - INA3221 status
  7 - Shunt voltage in microvolts
*/

const uint16_t IREG_MOISTURE = 0;
const uint16_t IREG_LIGHT = 1;
const uint16_t IREG_TEMPERATURE = 2;
const uint16_t IREG_TEMPERATURE_OK = 3;

const uint16_t IREG_VOLTAGE_MV = 4;
const uint16_t IREG_CURRENT_MA = 5;
const uint16_t IREG_INA_OK = 6;
const uint16_t IREG_SHUNT_UV = 7;

// =====================================================
// MODBUS MAP: COILS
// =====================================================

// FC15 / Coil 0 - Motor command from OpenPLC
const uint16_t COIL_MOTOR_COMMAND = 0;

// =====================================================
// MODBUS MAP: DISCRETE INPUTS
// =====================================================

/*
  FC2 - Discrete Inputs

  0 - Motor output is physically active
  1 - Water is present in the tank
  2 - Magnet is detected by the Hall sensor
*/

const uint16_t ISTS_MOTOR_ACTUAL = 0;
const uint16_t ISTS_WATER_PRESENT = 1;
const uint16_t ISTS_HALL_MAGNET_DETECTED = 2;

// =====================================================
// OBJECTS
// =====================================================

ModbusIP modbus;
WebServer server(80);

OneWire oneWire(TEMPERATURE_PIN);
DallasTemperature temperatureSensor(&oneWire);

// =====================================================
// UPDATE INTERVALS
// =====================================================

const unsigned long ANALOG_INTERVAL_MS = 1000;
const unsigned long INA_INTERVAL_MS = 250;

const unsigned long TEMPERATURE_INTERVAL_MS = 2000;
const unsigned long TEMPERATURE_CONVERSION_MS = 800;

const unsigned long SERIAL_INTERVAL_MS = 1000;

unsigned long previousAnalogTime = 0;
unsigned long previousInaTime = 0;
unsigned long previousTemperatureCycleTime = 0;
unsigned long temperatureRequestTime = 0;
unsigned long previousSerialTime = 0;

bool temperatureConversionRunning = false;

// =====================================================
// SENSOR VALUES
// =====================================================

uint16_t moistureRaw = 0;
uint16_t lightRaw = 0;

float temperatureC = 0.0f;
bool temperatureOk = false;

float batteryVoltageV = 0.0f;
float motorCurrentMa = 0.0f;
float shuntVoltageMv = 0.0f;

// =====================================================
// HALL SENSOR STATE
// =====================================================

// Raw GPIO state
bool hallRawState = HIGH;

// Stable state after debounce filtering
bool hallStableState = HIGH;

// True when a magnet is detected
bool hallMagnetDetected = false;

// True when water is present in the tank
bool waterPresent = true;

bool previousHallRawState = HIGH;
unsigned long hallLastChangeTime = 0;

// =====================================================
// MOTOR CONTROL
// =====================================================

// Command received from OpenPLC
bool plcMotorCommand = false;

// Actual state of GPIO21
bool motorActualOn = false;

// Manual override from the web interface
bool webManualOverride = false;
bool webManualMotorCommand = false;

// =====================================================
// EMBEDDED DIAGNOSTIC WEB PAGE
// =====================================================

const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="ru">
<head>
  <meta charset="UTF-8">

  <meta
    name="viewport"
    content="width=device-width, initial-scale=1"
  >

  <title>ESP32 Plant Controller</title>

  <style>
    body {
      margin: 0;
      padding: 20px;
      font-family: Arial, sans-serif;
      background: #f2f4f7;
      color: #202124;
    }

    .container {
      max-width: 900px;
      margin: auto;
    }

    h1 {
      margin-top: 0;
    }

    .status,
    .controls {
      padding: 16px;
      margin-bottom: 15px;
      background: white;
      border-radius: 12px;
    }

    .grid {
      display: grid;
      grid-template-columns:
        repeat(auto-fit, minmax(210px, 1fr));
      gap: 12px;
    }

    .card {
      padding: 16px;
      background: white;
      border-radius: 12px;
    }

    .label {
      color: #666;
      font-size: 14px;
    }

    .value {
      margin-top: 7px;
      font-size: 23px;
      font-weight: bold;
    }

    .ok {
      color: #198754;
    }

    .bad {
      color: #dc3545;
    }

    .warning {
      color: #b26a00;
    }

    button {
      min-width: 125px;
      margin: 5px;
      padding: 12px 18px;
      border: none;
      border-radius: 8px;
      color: white;
      cursor: pointer;
      font-size: 16px;
    }

    .on {
      background: #198754;
    }

    .off {
      background: #dc3545;
    }

    .auto {
      background: #0d6efd;
    }

    .small {
      margin-top: 10px;
      color: #666;
      font-size: 13px;
    }
  </style>
</head>

<body>
<div class="container">

  <h1>ESP32 Plant Controller</h1>

  <div class="status">
    <div>IP: <strong id="ip">—</strong></div>
    <div>Wi-Fi: <strong id="ssid">—</strong></div>
    <div>Uptime: <strong id="uptime">—</strong></div>
  </div>

  <div class="grid">

    <div class="card">
      <div class="label">Влажность почвы</div>
      <div class="value" id="moisture">—</div>
    </div>

    <div class="card">
      <div class="label">Освещённость</div>
      <div class="value" id="light">—</div>
    </div>

    <div class="card">
      <div class="label">Температура</div>
      <div class="value" id="temperature">—</div>
      <div class="small" id="temperatureStatus">—</div>
    </div>

    <div class="card">
      <div class="label">Вода в баке</div>
      <div class="value" id="waterPresent">—</div>
      <div class="small" id="hallStatus">—</div>
    </div>

    <div class="card">
      <div class="label">Напряжение батареи</div>
      <div class="value" id="voltage">—</div>
    </div>

    <div class="card">
      <div class="label">Падение на шунте</div>
      <div class="value" id="shunt">—</div>
    </div>

    <div class="card">
      <div class="label">Ток насоса</div>
      <div class="value" id="current">—</div>
    </div>

    <div class="card">
      <div class="label">INA3221</div>
      <div class="value" id="inaOk">—</div>
      <div class="small" id="inaErrors">—</div>
    </div>

    <div class="card">
      <div class="label">Команда OpenPLC</div>
      <div class="value" id="plcCommand">—</div>
    </div>

    <div class="card">
      <div class="label">Мотор фактически</div>
      <div class="value" id="motorActual">—</div>
    </div>

    <div class="card">
      <div class="label">Источник управления</div>
      <div class="value" id="controlMode">—</div>
    </div>

  </div>

  <div class="controls">
    <h2>Ручное управление мотором</h2>

    <button class="on" id="motorOn">
      ВКЛ
    </button>

    <button class="off" id="motorOff">
      ВЫКЛ
    </button>

    <button class="auto" id="motorAuto">
      AUTO / OpenPLC
    </button>

    <div class="small">
      В ручном режиме команда OpenPLC временно игнорируется.
      ESP32 пока не блокирует запуск при отсутствии воды:
      эту защиту мы сделаем в OpenPLC.
    </div>
  </div>

</div>

<script>
  function setText(id, value) {
    document.getElementById(id).textContent = value;
  }

  function setBoolState(id, enabled) {
    const element = document.getElementById(id);

    element.textContent = enabled ? "ВКЛ" : "ВЫКЛ";
    element.className =
      enabled ? "value ok" : "value bad";
  }

  async function updateStatus() {
    try {
      const response = await fetch("/api/status", {
        cache: "no-store"
      });

      if (!response.ok) {
        throw new Error("HTTP " + response.status);
      }

      const data = await response.json();

      setText("ip", data.ip);
      setText("ssid", data.ssid);
      setText("uptime", data.uptime_s + " сек");

      setText("moisture", data.moisture_raw);
      setText("light", data.light_raw);

      const temperatureElement =
        document.getElementById("temperature");

      if (data.temperature_ok) {
        temperatureElement.textContent =
          data.temperature_c.toFixed(2) + " °C";

        temperatureElement.className = "value";
        setText("temperatureStatus", "DS18B20: OK");
      } else {
        temperatureElement.textContent = "ОШИБКА";
        temperatureElement.className = "value bad";
        setText("temperatureStatus", "DS18B20 не найден");
      }

      const waterElement =
        document.getElementById("waterPresent");

      if (data.water_present) {
        waterElement.textContent = "ЕСТЬ";
        waterElement.className = "value ok";
      } else {
        waterElement.textContent = "НЕТ";
        waterElement.className = "value bad";
      }

      setText(
        "hallStatus",
        data.hall_magnet_detected
          ? "Магнит обнаружен"
          : "Магнит не обнаружен"
      );

      setText(
        "voltage",
        data.voltage_v.toFixed(3) + " V"
      );

      setText(
        "shunt",
        data.shunt_mv.toFixed(4) + " mV"
      );

      setText(
        "current",
        data.current_ma.toFixed(2) + " mA"
      );

      const inaElement =
        document.getElementById("inaOk");

      inaElement.textContent =
        data.ina_ok ? "OK" : "ОШИБКА";

      inaElement.className =
        data.ina_ok ? "value ok" : "value bad";

      setText(
        "inaErrors",
        "Ошибок подряд: " + data.ina_error_count
      );

      setBoolState(
        "plcCommand",
        data.plc_motor_command
      );

      setBoolState(
        "motorActual",
        data.motor_actual
      );

      const modeElement =
        document.getElementById("controlMode");

      if (data.manual_override) {
        modeElement.textContent = "РУЧНОЙ WEB";
        modeElement.className = "value warning";
      } else {
        modeElement.textContent = "OpenPLC";
        modeElement.className = "value ok";
      }
    } catch (error) {
      console.error(error);
    }
  }

  async function sendCommand(path) {
    try {
      await fetch(path, {
        method: "POST",
        cache: "no-store"
      });

      await updateStatus();
    } catch (error) {
      console.error(error);
    }
  }

  document
    .getElementById("motorOn")
    .addEventListener("click", function() {
      sendCommand("/api/motor/on");
    });

  document
    .getElementById("motorOff")
    .addEventListener("click", function() {
      sendCommand("/api/motor/off");
    });

  document
    .getElementById("motorAuto")
    .addEventListener("click", function() {
      sendCommand("/api/motor/auto");
    });

  updateStatus();
  setInterval(updateStatus, 1000);
</script>

</body>
</html>
)HTML";

// =====================================================
// HELPER FUNCTION
// =====================================================

uint16_t toUint16(float value) {
  if (!isfinite(value) || value <= 0.0f) {
    return 0;
  }

  if (value >= 65535.0f) {
    return 65535;
  }

  return static_cast<uint16_t>(value + 0.5f);
}

// =====================================================
// WI-FI
// =====================================================

void connectWiFi() {
  WiFi.mode(WIFI_STA);

  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(180);

  Serial.println();
  Serial.println("Подключение к Wi-Fi...");

  bool connected = wifiManager.autoConnect(
    "PlantController-Setup",
    "plant1234"
  );

  if (!connected) {
    Serial.println("Не удалось подключиться к Wi-Fi");
    Serial.println("Перезапуск ESP32...");

    delay(2000);
    ESP.restart();
  }

  WiFi.setSleep(false);

  Serial.println("Wi-Fi подключён");

  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

// =====================================================
// OTA
// =====================================================

void setupOTA() {
  ArduinoOTA.setHostname("plant-controller");
  ArduinoOTA.setPassword("plant1234");

  ArduinoOTA.onStart([]() {
    // Stop the motor before starting an OTA update
    digitalWrite(MOTOR_PIN, MOTOR_OFF_LEVEL);
    motorActualOn = false;

    Serial.println();
    Serial.println("OTA началось");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println();
    Serial.println("OTA завершено");
  });

  ArduinoOTA.onProgress([](
    unsigned int progress,
    unsigned int total
  ) {
    if (total == 0) {
      return;
    }

    unsigned int percent =
        static_cast<unsigned int>(
          progress * 100ULL / total
        );

    Serial.print("OTA: ");
    Serial.print(percent);
    Serial.println("%");
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.print("Ошибка OTA: ");
    Serial.println(static_cast<int>(error));
  });

  ArduinoOTA.begin();

  Serial.println("OTA запущено");
}

// =====================================================
// MODBUS TCP
// =====================================================

void setupModbus() {
  modbus.server();

  modbus.addIreg(IREG_MOISTURE, 0);
  modbus.addIreg(IREG_LIGHT, 0);
  modbus.addIreg(IREG_TEMPERATURE, 0);
  modbus.addIreg(IREG_TEMPERATURE_OK, 0);

  modbus.addIreg(IREG_VOLTAGE_MV, 0);
  modbus.addIreg(IREG_CURRENT_MA, 0);
  modbus.addIreg(IREG_INA_OK, 0);
  modbus.addIreg(IREG_SHUNT_UV, 0);

  modbus.addCoil(COIL_MOTOR_COMMAND, false);

  modbus.addIsts(ISTS_MOTOR_ACTUAL, false);
  modbus.addIsts(ISTS_WATER_PRESENT, true);
  modbus.addIsts(ISTS_HALL_MAGNET_DETECTED, false);

  Serial.println("Modbus TCP запущен");
  Serial.println("Порт: 502");
}

// =====================================================
// INA3221
// =====================================================

void setupINA3221() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

  inaFound = ina3221.begin(
    INA3221_ADDRESS,
    &Wire
  );

  if (!inaFound) {
    inaOk = false;

    Serial.println("INA3221 не найден");
    return;
  }

  ina3221.setShuntResistance(
    INA_MOTOR_CHANNEL,
    INA_SHUNT_RESISTANCE_OHMS
  );

  /*
    The INA3221 VPU pin must be connected to 3.3 V
    to provide stable I2C pull-up voltage.
  */
  inaErrorCount = 0;
  inaOk = true;

  Serial.println("INA3221 найден");

  Serial.print("Используется программный канал: CH");
  Serial.println(INA_MOTOR_CHANNEL + 1);
}

void readINA3221() {
  if (!inaFound) {
    inaOk = false;
    modbus.Ireg(IREG_INA_OK, 0);

    return;
  }

  float newBusVoltage =
      ina3221.getBusVoltage(INA_MOTOR_CHANNEL);

  float newShuntVoltageV =
      ina3221.getShuntVoltage(INA_MOTOR_CHANNEL);

  /*
    Current is calculated from the measured shunt voltage
    using Ohm's law:

    I = U / R
  */
  float newCurrentMa =
      fabsf(
        (
          newShuntVoltageV
          / INA_SHUNT_RESISTANCE_OHMS
        )
        * 1000.0f
      );

  bool valuesValid =
      isfinite(newBusVoltage)
      && isfinite(newShuntVoltageV)
      && isfinite(newCurrentMa)
      && newBusVoltage >= 0.0f
      && newBusVoltage <= 26.0f
      && fabsf(newShuntVoltageV) <= 0.164f;

  if (!valuesValid) {
    if (inaErrorCount < 255) {
      inaErrorCount++;
    }

    if (inaErrorCount >= INA_ERROR_LIMIT) {
      inaOk = false;
      modbus.Ireg(IREG_INA_OK, 0);
    }

    return;
  }

  inaErrorCount = 0;
  inaOk = true;

  batteryVoltageV = newBusVoltage;
  shuntVoltageMv = newShuntVoltageV * 1000.0f;
  motorCurrentMa = newCurrentMa;

  modbus.Ireg(
    IREG_VOLTAGE_MV,
    toUint16(batteryVoltageV * 1000.0f)
  );

  modbus.Ireg(
    IREG_CURRENT_MA,
    toUint16(motorCurrentMa)
  );

  modbus.Ireg(
    IREG_SHUNT_UV,
    toUint16(fabsf(shuntVoltageMv) * 1000.0f)
  );

  modbus.Ireg(IREG_INA_OK, 1);
}

// =====================================================
// SOIL MOISTURE AND LIGHT SENSORS
// =====================================================

void readAnalogSensors() {
  moistureRaw =
      static_cast<uint16_t>(
        analogRead(MOISTURE_PIN)
      );

  lightRaw =
      static_cast<uint16_t>(
        analogRead(LIGHT_PIN)
      );

  modbus.Ireg(IREG_MOISTURE, moistureRaw);
  modbus.Ireg(IREG_LIGHT, lightRaw);
}

// =====================================================
// HALL-EFFECT SENSOR
// =====================================================

void updateHallSensor() {
  unsigned long now = millis();

  hallRawState = digitalRead(HALL_PIN);

  /*
    Restart the debounce timer whenever
    the raw input state changes.
  */
  if (hallRawState != previousHallRawState) {
    previousHallRawState = hallRawState;
    hallLastChangeTime = now;
  }

  /*
    Accept a new state only after it has remained
    stable for at least the configured debounce time.
  */
  if (
    now - hallLastChangeTime
    >= HALL_DEBOUNCE_MS
  ) {
    hallStableState = hallRawState;
  }

  hallMagnetDetected =
      hallStableState == HALL_ACTIVE_LEVEL;

  if (HALL_MAGNET_MEANS_WATER_EMPTY) {
    waterPresent = !hallMagnetDetected;
  } else {
    waterPresent = hallMagnetDetected;
  }

  modbus.Ists(
    ISTS_WATER_PRESENT,
    waterPresent
  );

  modbus.Ists(
    ISTS_HALL_MAGNET_DETECTED,
    hallMagnetDetected
  );
}

// =====================================================
// NON-BLOCKING DS18B20 READING
// =====================================================

void updateTemperature() {
  unsigned long now = millis();

  if (!temperatureConversionRunning) {
    if (
      now - previousTemperatureCycleTime
      >= TEMPERATURE_INTERVAL_MS
    ) {
      previousTemperatureCycleTime = now;
      temperatureRequestTime = now;

      temperatureSensor.requestTemperatures();
      temperatureConversionRunning = true;
    }

    return;
  }

  if (
    now - temperatureRequestTime
    < TEMPERATURE_CONVERSION_MS
  ) {
    return;
  }

  float newTemperature =
      temperatureSensor.getTempCByIndex(0);

  temperatureOk =
      newTemperature != DEVICE_DISCONNECTED_C
      && isfinite(newTemperature);

  if (temperatureOk) {
    temperatureC = newTemperature;

    int16_t scaledTemperature =
        static_cast<int16_t>(
          temperatureC * 100.0f
        );

    modbus.Ireg(
      IREG_TEMPERATURE,
      static_cast<uint16_t>(
        scaledTemperature
      )
    );

    modbus.Ireg(IREG_TEMPERATURE_OK, 1);
  } else {
    temperatureC = 0.0f;

    modbus.Ireg(IREG_TEMPERATURE, 0);
    modbus.Ireg(IREG_TEMPERATURE_OK, 0);
  }

  temperatureConversionRunning = false;
}

// =====================================================
// MOTOR OUTPUT
// =====================================================

void updateMotor() {
  plcMotorCommand =
      modbus.Coil(COIL_MOTOR_COMMAND);

  bool requestedMotorState;

  if (webManualOverride) {
    requestedMotorState =
        webManualMotorCommand;
  } else {
    requestedMotorState =
        plcMotorCommand;
  }

  /*
    IMPORTANT:

    The ESP32 does not make irrigation safety decisions here.
    It only publishes waterPresent to OpenPLC.

    Pump protection logic is implemented in the PLC program.
  */
  motorActualOn = requestedMotorState;

  digitalWrite(
    MOTOR_PIN,
    motorActualOn
      ? MOTOR_ON_LEVEL
      : MOTOR_OFF_LEVEL
  );

  modbus.Ists(
    ISTS_MOTOR_ACTUAL,
    motorActualOn
  );
}

// =====================================================
// STATUS JSON FOR THE WEB INTERFACE
// =====================================================

String buildStatusJson() {
  String json;
  json.reserve(850);

  json += "{";

  json += "\"ip\":\"";
  json += WiFi.localIP().toString();
  json += "\",";

  json += "\"ssid\":\"";
  json += WiFi.SSID();
  json += "\",";

  json += "\"uptime_s\":";
  json += String(millis() / 1000UL);
  json += ",";

  json += "\"moisture_raw\":";
  json += String(moistureRaw);
  json += ",";

  json += "\"light_raw\":";
  json += String(lightRaw);
  json += ",";

  json += "\"temperature_c\":";
  json += String(temperatureC, 2);
  json += ",";

  json += "\"temperature_ok\":";
  json += temperatureOk ? "true" : "false";
  json += ",";

  json += "\"water_present\":";
  json += waterPresent ? "true" : "false";
  json += ",";

  json += "\"hall_magnet_detected\":";
  json += hallMagnetDetected ? "true" : "false";
  json += ",";

  json += "\"hall_raw\":";
  json += hallStableState ? "1" : "0";
  json += ",";

  json += "\"voltage_v\":";
  json += String(batteryVoltageV, 3);
  json += ",";

  json += "\"shunt_mv\":";
  json += String(shuntVoltageMv, 4);
  json += ",";

  json += "\"current_ma\":";
  json += String(motorCurrentMa, 2);
  json += ",";

  json += "\"ina_ok\":";
  json += inaOk ? "true" : "false";
  json += ",";

  json += "\"ina_error_count\":";
  json += String(inaErrorCount);
  json += ",";

  json += "\"plc_motor_command\":";
  json += plcMotorCommand ? "true" : "false";
  json += ",";

  json += "\"manual_override\":";
  json += webManualOverride ? "true" : "false";
  json += ",";

  json += "\"manual_motor_command\":";
  json += webManualMotorCommand ? "true" : "false";
  json += ",";

  json += "\"motor_actual\":";
  json += motorActualOn ? "true" : "false";

  json += "}";

  return json;
}

// =====================================================
// HTTP
// =====================================================

void handleRoot() {
  server.send_P(
    200,
    "text/html; charset=utf-8",
    INDEX_HTML
  );
}

void handleStatus() {
  server.sendHeader(
    "Cache-Control",
    "no-store"
  );

  server.send(
    200,
    "application/json",
    buildStatusJson()
  );
}

void handleMotorOn() {
  webManualOverride = true;
  webManualMotorCommand = true;

  updateMotor();

  server.send(
    200,
    "application/json",
    "{\"ok\":true,\"mode\":\"manual\",\"motor\":true}"
  );
}

void handleMotorOff() {
  webManualOverride = true;
  webManualMotorCommand = false;

  updateMotor();

  server.send(
    200,
    "application/json",
    "{\"ok\":true,\"mode\":\"manual\",\"motor\":false}"
  );
}

void handleMotorAuto() {
  webManualOverride = false;

  updateMotor();

  server.send(
    200,
    "application/json",
    "{\"ok\":true,\"mode\":\"openplc\"}"
  );
}

void handleNotFound() {
  server.send(
    404,
    "text/plain; charset=utf-8",
    "Страница не найдена"
  );
}

void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);

  server.on(
    "/api/motor/on",
    HTTP_POST,
    handleMotorOn
  );

  server.on(
    "/api/motor/off",
    HTTP_POST,
    handleMotorOff
  );

  server.on(
    "/api/motor/auto",
    HTTP_POST,
    handleMotorAuto
  );

  server.onNotFound(handleNotFound);

  server.begin();

  Serial.println("HTTP-сервер запущен");

  Serial.print("Страница: http://");
  Serial.println(WiFi.localIP());
}

// =====================================================
// SERIAL DIAGNOSTICS
// =====================================================

void printStatus() {
  Serial.println();
  Serial.println("================================");

  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("Влажность почвы: ");
  Serial.println(moistureRaw);

  Serial.print("Освещённость: ");
  Serial.println(lightRaw);

  Serial.print("Температура: ");

  if (temperatureOk) {
    Serial.print(temperatureC, 2);
    Serial.println(" C");
  } else {
    Serial.println("ОШИБКА");
  }

  Serial.print("Hall GPIO27 raw: ");
  Serial.println(hallStableState ? "HIGH" : "LOW");

  Serial.print("Магнит обнаружен: ");
  Serial.println(
    hallMagnetDetected ? "ДА" : "НЕТ"
  );

  Serial.print("Вода в баке: ");
  Serial.println(
    waterPresent ? "ЕСТЬ" : "НЕТ"
  );

  Serial.print("Напряжение INA: ");
  Serial.print(batteryVoltageV, 3);
  Serial.println(" V");

  Serial.print("Падение на шунте: ");
  Serial.print(shuntVoltageMv, 4);
  Serial.println(" mV");

  Serial.print("Ток насоса: ");
  Serial.print(motorCurrentMa, 2);
  Serial.println(" mA");

  Serial.print("INA3221: ");
  Serial.println(inaOk ? "OK" : "ОШИБКА");

  Serial.print("Ошибок INA подряд: ");
  Serial.println(inaErrorCount);

  Serial.print("Команда OpenPLC: ");
  Serial.println(
    plcMotorCommand ? "ВКЛ" : "ВЫКЛ"
  );

  Serial.print("Режим управления: ");
  Serial.println(
    webManualOverride
      ? "РУЧНОЙ WEB"
      : "OPENPLC"
  );

  Serial.print("Мотор фактически: ");
  Serial.println(
    motorActualOn ? "ВКЛ" : "ВЫКЛ"
  );
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  pinMode(MOISTURE_PIN, INPUT);
  pinMode(LIGHT_PIN, INPUT);

  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, MOTOR_OFF_LEVEL);

  // Use the ESP32 internal pull-up for the Hall sensor input
  pinMode(HALL_PIN, INPUT_PULLUP);

  // Initialize Hall sensor state
  hallRawState = digitalRead(HALL_PIN);
  hallStableState = hallRawState;
  previousHallRawState = hallRawState;
  hallLastChangeTime = millis();

  analogReadResolution(12);

  temperatureSensor.begin();
  temperatureSensor.setWaitForConversion(false);

  connectWiFi();
  setupOTA();
  setupModbus();
  setupINA3221();
  setupWebServer();

  Serial.println();
  Serial.println("ESP32 готова");

  Serial.print("Modbus TCP: ");
  Serial.print(WiFi.localIP());
  Serial.println(":502");

  Serial.print("Web: http://");
  Serial.println(WiFi.localIP());
}

// =====================================================
// LOOP
// =====================================================

void loop() {
  ArduinoOTA.handle();
  modbus.task();
  server.handleClient();

  unsigned long now = millis();

  // Continuously update the Hall sensor state
  updateHallSensor();

  // Apply the OpenPLC command or temporary web override
  updateMotor();

  // Read INA3221 at the configured interval
  if (
    now - previousInaTime
    >= INA_INTERVAL_MS
  ) {
    previousInaTime = now;
    readINA3221();
  }

  // Read moisture and light sensors at the configured interval
  if (
    now - previousAnalogTime
    >= ANALOG_INTERVAL_MS
  ) {
    previousAnalogTime = now;
    readAnalogSensors();
  }

  // Update the non-blocking temperature conversion
  updateTemperature();

  // Print full diagnostic output at the configured interval
  if (
    now - previousSerialTime
    >= SERIAL_INTERVAL_MS
  ) {
    previousSerialTime = now;
    printStatus();
  }

  delay(1);
}