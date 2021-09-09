#include "LSM9DS1.h"

LSM9DS1::LSM9DS1(TwoWire &wire) : _wire(&wire)
{
}

LSM9DS1::~LSM9DS1()
{
}

int LSM9DS1::begin()
{
  _wire->begin();
  // reset
  writeRegister(LSM9DS1_ADDRESS, CTRL_REG8, 0x05);
  writeRegister(LSM9DS1_ADDRESS, LSM9DS1_CTRL_REG2_M, 0x0C);
  delay(10);
  if (readRegister(LSM9DS1_ADDRESS, WHO_AM_I) != 0x68)
  {
    end();
    return 0;
  }

  return 1;
}

void LSM9DS1::init()
{
  //config accel and gyro
  //119 Hz, 2000 dps 0111 1000 -> 0x78
  // 119 Hz, 245 dps 0110 0000 -> 0x60
  //14.9 Hz, 245 dps 0010 0000 -> 0x20
  writeRegister(LSM9DS1_ADDRESS, CTRL_REG1_G, 0x20);  
  // 119 Hz, 16g,0111 1000 -> 0x78
  // 119 Hz, 8g, 0111 0000 -> 0x70
  // 119 Hz, 2g, 0110 0000 -> 0x60
  // 10 Hz,  2g, 0010 0000 -> 0x20
  writeRegister(LSM9DS1_ADDRESS, CTRL_REG6_XL, 0x60); 
  //set gyro to lowpower mode -> 0x80
  writeRegister(LSM9DS1_ADDRESS, CTRL_REG3_G, 0x80);
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG1_M, 0x80); // Temperature compensation enable, low power mode, 20 Hz
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG2_M, 0x00); // 4 Gauss
  writeRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG3_M, 0x00); // Continuous conversion mode
}

void LSM9DS1::end()
{
  writeRegister(LSM9DS1_ADDRESS, CTRL_REG8, 0x03);// Disable magnetometer if it is not by default
  writeRegister(LSM9DS1_ADDRESS, CTRL_REG1_G, 0x00);
  writeRegister(LSM9DS1_ADDRESS, CTRL_REG6_XL, 0x00);

  _wire->end();
}

int LSM9DS1::readAcceleration(float &x, float &y, float &z)
{
  int16_t data[3];

  if (!readRegisters(LSM9DS1_ADDRESS, OUT_X_L_XL, (uint8_t *)data, sizeof(data)))
  {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 2.0 / 32768.0; //(16 bit resolution)
  y = data[1] * 2.0 / 32768.0;
  z = data[2] * 2.0 / 32768.0;

  return 1;
}

int LSM9DS1::accelerationAvailable()
{
  if (readRegister(LSM9DS1_ADDRESS, STATUS_REG_0) & 0x01)
  {
    return 1;
  }

  return 0;
}

int LSM9DS1::read_mag_status()
{
  return readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_CTRL_REG1_M);
}

int LSM9DS1::readMagneticField(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM9DS1_ADDRESS_M, LSM9DS1_OUT_X_L_M, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 4.0 * 100.0 / 32768.0;
  y = data[1] * 4.0 * 100.0 / 32768.0;
  z = data[2] * 4.0 * 100.0 / 32768.0;

  return 1;
}

int LSM9DS1::magneticFieldAvailable()
{
  if (readRegister(LSM9DS1_ADDRESS_M, LSM9DS1_STATUS_REG_M) & 0x08) {
    return 1;
  }

  return 0;
}

int LSM9DS1::read_accel_status()
{
  return readRegister(LSM9DS1_ADDRESS, CTRL_REG6_XL);
}

int LSM9DS1::read_gyro_status()
{
  return readRegister(LSM9DS1_ADDRESS, CTRL_REG1_G);
}

int LSM9DS1::readGyroscope(float &x, float &y, float &z)
{
  int16_t data[3];

  if (!readRegisters(LSM9DS1_ADDRESS, OUT_X_L_G, (uint8_t *)data, sizeof(data)))
  {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }
  //Serial.print("data[0]: ");Serial.print(data[0]);Serial.print(" data[1]: ");Serial.print(data[1]);Serial.print(" data[2]: ");Serial.println(data[2]);
  x = data[0] * 245 / 32768.0;
  y = data[1] * 245 / 32768.0;
  z = data[2] * 245 / 32768.0;

  return 1;
}

int LSM9DS1::gyroscopeAvailable()
{
  if (readRegister(LSM9DS1_ADDRESS, STATUS_REG_0) & 0x02)
  {
    return 1;
  }
  return 0;
}

int LSM9DS1::readRegister(uint8_t slaveAddress, uint8_t address)
{
  _wire->beginTransmission(slaveAddress);
  _wire->write(address);
  if (_wire->endTransmission() != 0)
  {
    return -1;
  }

  if (_wire->requestFrom(slaveAddress, 1) != 1)
  {
    return -1;
  }
  return _wire->read();
}

int LSM9DS1::readRegisters(uint8_t slaveAddress, uint8_t address, uint8_t *data, size_t length)
{
  _wire->beginTransmission(slaveAddress);
  _wire->write(0x80 | address);
  if (_wire->endTransmission(false) != 0)
  {
    return -1;
  }

  if (_wire->requestFrom(slaveAddress, length) != length)
  {
    return 0;
  }

  for (size_t i = 0; i < length; i++)
  {
    *data++ = _wire->read();
  }

  return 1;
}

int LSM9DS1::writeRegister(uint8_t slaveAddress, uint8_t address, uint8_t value)
{
  _wire->beginTransmission(slaveAddress);
  _wire->write(address);
  _wire->write(value);
  if (_wire->endTransmission() != 0)
  {
    return 0;
  }

  return 1;
}

uint8_t LSM9DS1::tempAvailable()
{
	uint8_t status = xgReadByte(STATUS_REG_1);
	
	return ((status & (1<<2)) >> 2);
}

void LSM9DS1::read_system_temperature()
{
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp	
	if ( xgReadBytes(OUT_TEMP_L, temp, 2) == 2 ) // Read 2 bytes, beginning at OUT_TEMP_L
	{
		int16_t offset = 25;  // Per datasheet sensor outputs 0 typically @ 25 degrees centigrade
		temperature = offset + ((((int16_t)temp[1] << 8) | temp[0]) >> 8) ;
	}
}

uint8_t LSM9DS1::xgReadByte(uint8_t subAddress)
{
	return I2CreadByte(LSM9DS1_ADDRESS, subAddress);
}

uint8_t LSM9DS1::xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	return I2CreadBytes(LSM9DS1_ADDRESS, subAddress, dest, count);
}

uint8_t LSM9DS1::I2CreadByte(uint8_t address, uint8_t subAddress)
{
	uint8_t data; // `data` will store the register data	
	
	_wire->beginTransmission(address);         // Initialize the Tx buffer
	_wire->write(subAddress);	                 // Put slave register address in Tx buffer
	_wire->endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
	_wire->requestFrom(address, (uint8_t) 1);  // Read one byte from slave register address 
	
	data = _wire->read();                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

uint8_t LSM9DS1::I2CreadBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{
	byte retVal;
	_wire->beginTransmission(address);      // Initialize the Tx buffer
	// Next send the register to be read. OR with 0x80 to indicate multi-read.
	_wire->write(subAddress | 0x80);        // Put slave register address in Tx buffer
	retVal = _wire->endTransmission(false); // Send Tx buffer, send a restart to keep connection alive
	if (retVal != 0) // endTransmission should return 0 on success
		return 0;
	
	retVal = _wire->requestFrom(address, count);  // Read bytes from slave register address 
	if (retVal != count)
		return 0;
	
	for (int i=0; i<count;)
		dest[i++] = _wire->read();
	
	return count;
}

#ifdef ARDUINO_ARDUINO_NANO33BLE
LSM9DS1 lsm9ds1(Wire1);
#else
LSM9DS1 lsm9ds1(Wire);
#endif
