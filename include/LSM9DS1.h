#include <Arduino.h>
#include <Wire.h>
#include "LSM9DS1_Registers.h"

#define LED_PWR     (25u)
class LSM9DS1
{
public:
  LSM9DS1(TwoWire &wire);
  virtual ~LSM9DS1();

  int begin();
  void init();
  void end();
  // Accelerometer
  virtual int readAcceleration(float &x, float &y, float &z);
  virtual int accelerationAvailable();                
  virtual int read_accel_status();

  // Gyroscope
  virtual int readGyroscope(float &x, float &y, float &z); // Results are in degrees/second.
  virtual int gyroscopeAvailable();                        // Number of samples in the FIFO.
  virtual int read_gyro_status();

    // Magnetometer
    virtual int readMagneticField(float& x, float& y, float& z); // Results are in uT (micro Tesla).
    virtual int magneticFieldAvailable(); // Number of samples in the FIFO.
    virtual int read_mag_status(); // Number of samples in the FIFO.

  int16_t temperature; // Chip temperature
  uint8_t tempAvailable();
  void read_system_temperature();

private:
  int readRegister(uint8_t slaveAddress, uint8_t address);
  int readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t *data, size_t length);
  int writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value);
  //for temperature
  uint8_t xgReadByte(uint8_t subAddress);
  uint8_t xgReadBytes(uint8_t subAddress, uint8_t *dest, uint8_t count);
  uint8_t I2CreadByte(uint8_t address, uint8_t subAddress);
  uint8_t I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t *dest, uint8_t count);

private:
  TwoWire *_wire;
};

extern LSM9DS1 lsm9ds1;
