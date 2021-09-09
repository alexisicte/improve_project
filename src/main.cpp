#include <ArduinoBLE.h>
#include "LSM9DS1.h"
#include "core_defines.h"

//Services
BLEService DeviceInformation("180A");
BLEService BatteryService("180F");
BLEService SensorService(SENSOR_SERVICE);
//Characteristics
BLEUnsignedCharCharacteristic batteryLevelChar("2A19", BLERead | BLENotify);

BLEStringCharacteristic manName("2A29", BLERead, 10);
BLEStringCharacteristic serialNum("2A25", BLERead, 28);
BLEStringCharacteristic firmwareVersion("2A26", BLERead, 10);
BLEStringCharacteristic hardwareVersion("2A27", BLERead, 10);
BLEStringCharacteristic patient_id("2AC3", BLERead | BLEWrite, 28);

BLEStringCharacteristic accelerometer_data(CHAR_ACC, BLERead | BLENotify, 48);
BLEStringCharacteristic gyroscope_data(CHAR_GYRO, BLERead | BLENotify, 48);
BLEStringCharacteristic magnetometer_data(CHAR_MAG, BLERead | BLENotify, 48);

BLEStringCharacteristic debug_system(CHAR_DEBUG, BLERead | BLENotify, 48);

String values_to_send = " ";
float x_accel, y_accel, z_accel, x_gyro, y_gyro, z_gyro, x_mag, y_mag, z_mag;
float x_accel_f, y_accel_f, z_accel_f, x_gyro_f, y_gyro_f, z_gyro_f = 0;
String debug_to_send = " ";
int old_battery_lvl = 0;

void handle_sensors(int flag){
  if(!flag){
    digitalWrite(PIN_ENABLE_SENSORS_3V3, LOW);
    digitalWrite(PIN_ENABLE_I2C_PULLUP, LOW);
  }
  else{
      digitalWrite(PIN_ENABLE_SENSORS_3V3, HIGH);
      digitalWrite(PIN_ENABLE_I2C_PULLUP, HIGH);
      if (!lsm9ds1.begin()) {
        Serial.println("Failed to initialize lsm9ds1!");
        while (1);
      }
      lsm9ds1.init();// initialize sensors and parameters
  }

}

void updateBatteryLevel() {
  int batteryLevel = map(analogRead(A0), 940, 1020, 100, 0);
  if(batteryLevel != old_battery_lvl){
    old_battery_lvl = batteryLevel;
    batteryLevelChar.writeValue(batteryLevel);  // and update the battery level characteristic
  } 
}
void update_temperature() {
  debug_to_send = " ";
  lsm9ds1.read_system_temperature();
  debug_to_send += String(lsm9ds1.temperature) + "|";
  debug_to_send += String(analogRead(A0));
  debug_system.writeValue(debug_to_send);
}

String low_pass_filter(float x_axis, float y_axis, float z_axis, float alpha, int sensor) {
  values_to_send = " ";// re initialize array
  if (!sensor) { // sensor = accelerometer
    x_accel_f = alpha * x_accel_f + (1 - alpha) * x_axis;
    y_accel_f = alpha * y_accel_f + (1 - alpha) * y_axis;
    z_accel_f = alpha * z_accel_f + (1 - alpha) * z_axis;
    values_to_send += String(x_accel_f) + "|";
    values_to_send += String(y_accel_f) + "|";
    values_to_send += String(z_accel_f);
  }
  else {
    x_gyro_f = alpha * x_gyro_f + (1 - alpha) * x_axis;
    y_gyro_f = alpha * y_gyro_f + (1 - alpha) * y_axis;
    z_gyro_f = alpha * z_gyro_f + (1 - alpha) * z_axis;
    values_to_send += String(int(x_gyro_f)) + "|";
    values_to_send += String(int(y_gyro_f)) + "|";
    values_to_send += String(int(z_gyro_f));
  }
  return values_to_send;
}

void setup() {
  Serial.begin(9600);
  //BLE.debug(Serial);
  pinMode(LED_PWR, OUTPUT);
  digitalWrite(LED_PWR, LOW);

  handle_sensors(0);//close sensors
  
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
  
  // set advertised local name and service UUID:
  BLE.setLocalName(SENSOR_NAME);
  BLE.setAdvertisedService(DeviceInformation);
  BLE.setAdvertisedService(BatteryService);
  BLE.setAdvertisedService(SensorService);
  // add the characteristic to the service
  DeviceInformation.addCharacteristic(manName);
  DeviceInformation.addCharacteristic(serialNum);
  DeviceInformation.addCharacteristic(firmwareVersion);
  DeviceInformation.addCharacteristic(hardwareVersion);
  DeviceInformation.addCharacteristic(patient_id);
  manName.writeValue(MANUFACTURER);
  serialNum.writeValue(SERIAL_NUMBER);
  firmwareVersion.writeValue(FIRMWARE_VERSION);
  hardwareVersion.writeValue(HARDWARE_VERSION);
  patient_id.writeValue(" ");
  
  BatteryService.addCharacteristic(batteryLevelChar);
  SensorService.addCharacteristic(accelerometer_data);
  SensorService.addCharacteristic(gyroscope_data);
  SensorService.addCharacteristic(magnetometer_data);
  SensorService.addCharacteristic(debug_system);
  // add services
  BLE.addService(DeviceInformation);
  BLE.addService(BatteryService);
  BLE.addService(SensorService);
}

void loop() {
  BLE.advertise(); // start advertising again 
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    handle_sensors(1);//open sensors
    Serial.print("Connected to central: ");Serial.println(central.address());
    Serial.print("Accelerometer binary status: "); Serial.println(lsm9ds1.read_accel_status(), BIN);
    Serial.print("Gyroscope binary status: "); Serial.println(lsm9ds1.read_gyro_status(), BIN);
    Serial.print("Mangnetomenter binary status: "); Serial.println(lsm9ds1.read_mag_status(), BIN);

    BLE.stopAdvertise();
    // while the central is still connected to peripheral:
    while (central.connected()) {
      if (lsm9ds1.accelerationAvailable()) {
        lsm9ds1.readAcceleration(x_accel, y_accel, z_accel);
        // set alpha to zero if you don't want to apply the low pass filter
        accelerometer_data.writeValue(low_pass_filter(x_accel, y_accel, z_accel, 0.01, 0));
      }
      if (lsm9ds1.gyroscopeAvailable()) {
        lsm9ds1.readGyroscope(x_gyro, y_gyro, z_gyro);
        gyroscope_data.writeValue(low_pass_filter(x_gyro, y_gyro, z_gyro, 0.01, 1));
      }
      updateBatteryLevel();
      update_temperature();
      if (lsm9ds1.magneticFieldAvailable()) {
        values_to_send = " ";// re initialize array
        lsm9ds1.readMagneticField(x_mag, y_mag, z_mag);
        values_to_send += String(x_mag) + "|";
        values_to_send += String(y_mag) + "|";
        values_to_send += String(z_mag);
        magnetometer_data.writeValue(values_to_send);
      }
    }
    handle_sensors(0);//close sensors
    central.disconnect();
    // when the central disconnects, print it out:
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
  
}