#include <EEPROM.h>

#define SENSOR_SEND_MAX_DELAY 60000
#define BME280_READ_DELAY 1000
#define BME680_READ_DELAY 1000
#define SDS011_READ_DELAY 1000

#define STATE_SAVE_PERIOD  UINT32_C(360 * 60 * 1000) // 360 minutes - 4 times a day
uint8_t bsecState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
uint16_t stateUpdateCounter = 0;


void init_sensors() {
  //  init_bme680();
  init_sds011(&dustSerial);

  init_bme680();
}

String read_sensors() {
  read_bme680();
  return read_sds011();
}


void init_bme680() {

  EEPROM.begin(BSEC_MAX_STATE_BLOB_SIZE + 1); // 1st address for the length


  iaqSensor.begin(0x77 /*BME680_I2C_ADDR_PRIMARY*/ , Wire);
  bme680Msg1 = "\nBSEC library version " + String(iaqSensor.version.major) + "." + String(iaqSensor.version.minor) + "." + String(iaqSensor.version.major_bugfix) + "." + String(iaqSensor.version.minor_bugfix);
  Serial.println(output);

  //iaqSensor.setConfig(bsec_config_iaq);
  checkIaqSensorStatus();
  loadState();

  bsec_virtual_sensor_t sensorList[14] = {
    BSEC_OUTPUT_RAW_TEMPERATURE,
    BSEC_OUTPUT_RAW_PRESSURE,
    BSEC_OUTPUT_RAW_HUMIDITY,
    BSEC_OUTPUT_RAW_GAS,
    BSEC_OUTPUT_IAQ,
    BSEC_OUTPUT_STATIC_IAQ,
    BSEC_OUTPUT_CO2_EQUIVALENT,
    BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
    BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
    BSEC_OUTPUT_STABILIZATION_STATUS,
    BSEC_OUTPUT_RUN_IN_STATUS,
    BSEC_OUTPUT_COMPENSATED_GAS,
    BSEC_OUTPUT_GAS_PERCENTAGE
  };

  iaqSensor.updateSubscription(sensorList, 14, BSEC_SAMPLE_RATE_LP);
  checkIaqSensorStatus();


}

int i;
void read_bme680() {
  unsigned long int ct;

  if (iaqSensor.run(&ct)) { // If new data is available
      bme680Msg1 = "reading bme680";
    process_bme680();
  } else {
    checkIaqSensorStatus();
  }
}

void process_bme680() {
  //   String outputms = String(time_trigger) + "ms, ";
  //    outputms += String(stateUpdateCounter) + " updates";

  bme680Msg1 = String(iaqSensor.rawTemperature) + "r°C,";
  bme680Msg1 += String(iaqSensor.rawHumidity) + "r%,";

  bme680Msg2 = String(iaqSensor.gasResistance) + "Ohm,";
  bme680Msg2 += String(iaqSensor.iaqEstimate) + "IAQ/" + String(iaqSensor.iaqAccuracy);

  String outputE = "stab:" + String(iaqSensor.stabStatus);
  outputE += " runIn:" + String(iaqSensor.runInStatus);
  outputE += " gas%:" + String(iaqSensor.gasPercentage);
  outputE += " co2A:" + String(iaqSensor.co2Accuracy);
  outputE += " gas%a:" + String(iaqSensor.gasPercentageAcccuracy);


  String output = String(iaqSensor.rawTemperature) + "r°C";
  output += ", " + String(iaqSensor.pressure) + "hPa";

  String output2 = String(iaqSensor.rawHumidity) + "r%";
  output2 += ", " + String(iaqSensor.gasResistance) + "Ohm";

  String output3 = String(iaqSensor.iaqEstimate) + "IAQ";
  output3 += ", " + String(iaqSensor.iaqAccuracy) + "a";
  output3 += ", " + String(iaqSensor.temperature) + "°C";

  String output4 = String(iaqSensor.humidity) + "%";
  output4 += ", " + String(iaqSensor.staticIaq) + "sIAQ";
  String output5 = String(iaqSensor.co2Equivalent) + "CO2 eq";
  output5 += ", " + String(iaqSensor.breathVocEquivalent) + "VocEq";


  
  Serial.println(outputE);

  Serial.println("gas " + String(iaqSensor.compGasValue) + " accuracy " + String(iaqSensor.compGasAccuracy)); 
  i++;

  updateState();

  if (i % 5 == 0) {
    String output = String(iaqSensor.rawTemperature) + "r°C";
    output += ", " + String(iaqSensor.pressure) + "hPa";
    output += ", " + String(iaqSensor.rawHumidity) + "r%";
    output += ", " + String(iaqSensor.gasResistance) + "Ohm";

    output += ", " + String(iaqSensor.iaqEstimate) + "IAQ";
    output += ", " + String(iaqSensor.iaqAccuracy) + "a";
    output += ", " + String(iaqSensor.temperature) + "°C";

    output += ", " + String(iaqSensor.humidity) + "%";
    output += ", " + String(iaqSensor.staticIaq) + "sIAQ";
    output += ", " + String(iaqSensor.co2Equivalent) + "CO2 eq";
    output += ", " + String(iaqSensor.breathVocEquivalent) + "VocEq";

    Serial.println(output);
  }
}
/*
  void init_bme680() {
  Serial.print(F("INIT BME680: "));
  if (bme680.begin()) {
    Serial.println(F("found"));
    // Set up oversampling and filter initialization
    bme680.setTemperatureOversampling(BME680_OS_8X);
    bme680.setHumidityOversampling(BME680_OS_2X);
    bme680.setPressureOversampling(BME680_OS_4X);
    bme680.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme680.setGasHeater(320, BME680_HEATING_TIME); // 320*C for 150 ms, from settings.h
    bme680_ok = 1;
  } else {
    Serial.println(F("not found"));
  }
  }

  void read_bme680() {
  // Read BME680 if it has been initialised successfully and it is time to read it
  if ((bme680_ok == 1) && (millis() > (bme680_lastRead + BME680_READ_DELAY))) {
    if (! bme680.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    } else {
      bme680_lastRead = millis();
      float temp = bme680.temperature;
      float humi = bme680.humidity;
      float pres = bme680.pressure / 100.0F;
      float gas = bme680.gas_resistance / 1000.0F;
      // Do something for the data here
      bme680_lastSend = millis();
      bme680_lastTemp = temp;
      bme680_lastHumi = humi;
      bme680_lastPres = pres;
      bme680_lastGas = gas;
    }
  }
  }
*/

void init_sds011(HardwareSerial * serial) {
  Serial.print(F("INIT sds011: "));
  sds011.setup(serial);
  sds011.onData([](float pm25Value, float pm10Value) {
    if (pm_array_counter < pm_array_size) {
      sds011_pm25[pm_array_counter] = pm25Value;
      sds011_pm10[pm_array_counter] = pm10Value;
      pm_array_counter++;
    } else {
      Serial.println("Array full!");
      pm_array_counter = 0;
    }

    Serial.print("PM2.5: ");
    Serial.print(pm25Value);
    Serial.print(" PM10: ");
    Serial.println(pm10Value);
  });
  /*
    sds011.onResponse([](){
      // Serial.println("Response");
      // command has been executed
    });
  */
  sds011.onError([](int8_t error) {
    Serial.println("EROR");
    // error happened
    // -1: CRC error
  });
  sds011.setWorkingPeriod(5);
}

String read_sds011() {
  return sds011.loop();
}




void checkIaqSensorStatus(void)
{
  if (iaqSensor.status != BSEC_OK) {
    if (iaqSensor.status < BSEC_OK) {
      bme680Msg1 = "BSEC error code : " + String(iaqSensor.status);
      Serial.println(bme680Msg1);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      bme680Msg1 = "BSEC warning code : " + String(iaqSensor.status);
      Serial.println(bme680Msg1);
    }
  }

  if (iaqSensor.bme680Status != BME680_OK) {
    if (iaqSensor.bme680Status < BME680_OK) {
      bme680Msg1 = "BME680 error code : " + String(iaqSensor.bme680Status);
      Serial.println(bme680Msg1);
      for (;;)
        errLeds(); /* Halt in case of failure */
    } else {
      bme680Msg1 = "BME680 warning code : " + String(iaqSensor.bme680Status);
      Serial.println(bme680Msg1);
    }
  }
}

void errLeds(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}


void loadState(void)
{
  if (EEPROM.read(0) == BSEC_MAX_STATE_BLOB_SIZE) {
    // Existing state in EEPROM
    Serial.println("Reading state from EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE; i++) {
      bsecState[i] = EEPROM.read(i + 1);
      Serial.println(bsecState[i], HEX);
    }

    iaqSensor.setState(bsecState);
    checkIaqSensorStatus();
  } else {
    // Erase the EEPROM with zeroes
    Serial.println("Erasing EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE + 1; i++)
      EEPROM.write(i, 0);

    EEPROM.commit();
  }
}

void updateState(void)
{
  bool update = false;
  if (stateUpdateCounter == 0) {
    /* First state update when IAQ accuracy is >= 3 */
    if (iaqSensor.iaqAccuracy >= 3) {
      update = true;
      stateUpdateCounter++;
    }
  } else {
    /* Update every STATE_SAVE_PERIOD minutes */
    if ((stateUpdateCounter * STATE_SAVE_PERIOD) < millis()) {
      update = true;
      stateUpdateCounter++;
    }
  }

  if (update) {
    iaqSensor.getState(bsecState);
    checkIaqSensorStatus();

    Serial.println("Writing state to EEPROM");

    for (uint8_t i = 0; i < BSEC_MAX_STATE_BLOB_SIZE ; i++) {
      EEPROM.write(i + 1, bsecState[i]);
      Serial.println(bsecState[i], HEX);
    }

    EEPROM.write(0, BSEC_MAX_STATE_BLOB_SIZE);
    EEPROM.commit();
  }
}
