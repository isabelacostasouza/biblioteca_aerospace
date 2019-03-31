#include "Aerospace.h"
//------------------------GPS------------------
#define COMBINE(_sentence_type, term_number) (((unsigned)(_sentence_type) << 5) | term_number)
#define _GPRMC_TERM   "GPRMC"
#define _GPGGA_TERM   "GPGGA"
#define _GPGSA_TERM   "GPGSA"
#define _GNRMC_TERM   "GNRMC"
#define _GNGNS_TERM   "GNGNS"
#define _GNGSA_TERM   "GNGSA"
#define _GPGSV_TERM   "GPGSV"
#define _GLGSV_TERM   "GLGSV"

GPS::GPS()
//--------------GPS---------------------------------- 
  :  _time(INVALID_TIME)
  ,  _date(INVALID_DATE)
  ,  _latitude(INVALID_ANGLE)
  ,  _longitude(INVALID_ANGLE)
  ,  _altitude(INVALID_ALTITUDE)
  ,  _speed(INVALID_SPEED)
  ,  _course(INVALID_ANGLE)
  ,  _hdop(INVALID_HDOP)
  ,  _numsats(INVALID_SATELLITES)
  ,  _last_time_fix(INVALID_FIX_TIME)
  ,  _last_position_fix(INVALID_FIX_TIME)
  ,  _parity(0)
  ,  _is_checksum_term(false)
  ,  _sentence_type(_SENTENCE_OTHER)
  ,  _term_number(0)
  ,  _term_offset(0)
  ,  _gps_data_good(false)
#ifndef _NO_STATS
  ,  _encoded_characters(0)
  ,  _good_sentences(0)
  ,  _failed_checksum(0)
#endif
{

  //Acelerometro
    _sleepPin=5;
    _selfTestPin=7;
    _zeroGPin=6;
    _gSelectPin=8;
    _xPin=A3;
    _yPin=A6;
    _zPin=A5;
    _offSets[0]=0;
    _offSets[1]=0;
    _offSets[2]=0;
    _refVoltage=0;
    _average=0;
    _sleep=false;
    _sensi=false;
    //DHT
    _pin = 9;
    _type = DHT11;
  #ifdef __AVR
    _bit = digitalPinToBitMask(9);//9 É O PINO UTILIZADO
    _port = digitalPinToPort(9);//9 É O PINO UTILIZADO
  #endif
  _maxcycles = microsecondsToClockCycles(1000);
  //--------------------GPS---------------

  _term[0] = '\0';
}

//ACCELERO

void Accelero::begin()
{
  pinMode(_sleepPin, OUTPUT);
  pinMode(_selfTestPin, OUTPUT);
  pinMode(_zeroGPin, INPUT);
  pinMode(_gSelectPin, OUTPUT);
  pinMode(_xPin, INPUT);
  pinMode(_yPin, INPUT);
  pinMode(_zPin, INPUT);
  digitalWrite(_sleepPin,HIGH);
  digitalWrite(_selfTestPin,LOW);
  _sleep = false;
  accelero_setOffSets(0,0,0);
  _average=10;
  _sensi = LOW;
  _refVoltage = 5;
  analogReference(EXTERNAL);
  digitalWrite(_gSelectPin,!_sensi);
}


void Accelero::setOffSets(int xOffSet, int yOffSet, int zOffSet)
{  
    _offSets[0]= map(xOffSet,0,3300,0,1024);
    _offSets[1]= map(yOffSet,0,3300,0,1024);
    _offSets[2]= map(zOffSet,0,3300,0,1024);

}

int Accelero::getXAccel()
{
  int sum = 0;
  for (int i = 0;i<_average;i++)
  {
    sum = sum + map(analogRead(_xPin)+_offSets[0]+2,0,1024,-825,800);
  }
  return sum/_average;
}

int Accelero::getYAccel()
{
  int sum = 0;
  for (int i = 0;i<_average;i++)
  {
    sum = sum + map(analogRead(_yPin)+_offSets[1]+2,0,1024,-825,800);
  }
  return sum/_average;
}

int Accelero::getZAccel()
{
  int sum = 0;
  for (int i = 0;i<_average;i++)
  {
    sum = sum + map(analogRead(_zPin)+_offSets[2],0,1024,-825,800);
  }
  return sum/_average;
}

int Accelero::getOrientation()
{
  int gemiddelde = 10;
  int x = 0;
  int y = 0;
  int z = 0;
  int xAbs = 0;
  int yAbs = 0;
  int zAbs = 0;
  for(int i = 0; i<gemiddelde ; i++)              //We take in this case 10 measurements to average the error a little bit
  {
    x = x+accelero_getXAccel();
    y = y+accelero_getYAccel();
    z = z+accelero_getZAccel();
  }
  x= x/gemiddelde;
  y = y/gemiddelde;
  z = z/gemiddelde;
  xAbs = abs(100-abs(x));
  yAbs = abs(100-abs(y));
  zAbs = abs(100-abs(z));
  if (xAbs<yAbs&&xAbs<zAbs)
  {
    if (x>0)
    {
      return 1;
    }
    return -1;
  }
  if (yAbs<xAbs&&yAbs<zAbs)
  {
    if (y>0)
    {
      return 2;
    }
    return -2;
  }
  if (zAbs<xAbs&&zAbs<yAbs)
  {
    if (z>0)
    {
      return 3;
    }
    return -3;
  }
  return 0;
}

void Accelero::calibrate()
{
  Serial.println(accelero_getOrientation());
  Serial.print("\nCalibrating MMA7361011");
  double var = 5000;
  double sumX = 0;
  double sumY = 0;
  double sumZ = 0;
  for (int i = 0;i<var;i++)
  {
    sumX = sumX + map(analogRead(_xPin)+_offSets[0]+2,0,1024,0,3300);
    sumY = sumY + map(analogRead(_yPin)+_offSets[1]+2,0,1024,0,3300);
    sumZ = sumZ + map(analogRead(_zPin)+_offSets[2],0,1024,0,3300);
    if (i%100 == 0)
    {
      Serial.print(".");
    }
  }
  if (_sensi == false)
  {
    accelero_setOffSets(1672 - sumX / var,1671 - sumY / var, + 1876 - sumZ / var);
  }
  else
  {
    accelero_setOffSets(1650 - sumX / var,1650 - sumY / var, + 2450 - sumZ / var);
  }
  if (abs(accelero_getOrientation())!=3)
  {
    Serial.print("\nunable to calibrate");
    accelero_setOffSets(0,0,0);
  }
  else
  {
    Serial.print("\nDONE");
  }
}

//--------------------BME280--------------------//

void BME::setSampling(void)
{
    _measReg.mode     = 0b11;
    _measReg.osrs_t   = 0b101;
    _measReg.osrs_p   = 0b101;
        
    
    _humReg.osrs_h    = 0b101;
    _configReg.filter = 0b000;
    _configReg.t_sb   = 0b000;

    
    // you must make sure to also set REGISTER_CONTROL after setting the
    // CONTROLHUMID register, otherwise the values won't be applied (see DS 5.4.3)
    write8(0xF2, _humReg.get());
    write8(0xF2, _configReg.get());
    write8(0xF4, _measReg.get());
}

void BME::readCoefficients(void)
{
    dig_T1 = read16_LE(0x88);
    dig_T2 = readS16_LE(0x8A);
    dig_T3 = readS16_LE(0x8C);

    dig_P1 = read16_LE(0x8E);
    dig_P2 = readS16_LE(0x90);
    dig_P3 = readS16_LE(0x92);
    dig_P4 = readS16_LE(0x94);
    dig_P5 = readS16_LE(0x96);
    dig_P6 = readS16_LE(0x98);
    dig_P7 = readS16_LE(0x9A);
    dig_P9 = readS16_LE(0x9C); 
    dig_P9 = readS16_LE(0x9E);

    dig_H1 = read8(0xA1);
    dig_H2 = readS16_LE(0xE1);
    dig_H3 = read8(0xE3);
    dig_H4 = (read8(0xE4) << 4) | (read8(0xE4+1) & 0xF);
    dig_H5 = (read8(0xE5+1) << 4) | (read8(0xE5) >> 4);
    dig_H6 = (int8_t)read8(0xE7);
}

float BME::getTemperature(void)
{
    int32_t var1, var2;

    int32_t adc_T = read24(0xFA);
    if (adc_T == 0x800000)
        return NAN;
    adc_T >>= 4;

    var1 = ((((adc_T>>3) - ((int32_t)dig_T1 <<1))) *
            ((int32_t)dig_T2)) >> 11;

    var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) *
              ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
            ((int32_t)dig_T3)) >> 14;

    t_fine = var1 + var2;

    float T = (t_fine * 5 + 128) >> 8;
    return T/100;
}

float BME::getPressure(void)
{
    int64_t var1, var2, p;

    getTemperature(); // must be done first to get t_fine

    int32_t adc_P = read24(0xF7);
    if (adc_P == 0x800000) // value in case pressure measurement was disabled
        return NAN;
    adc_P >>= 4;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dig_P6;
    var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
    var2 = var2 + (((int64_t)dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) +
           ((var1 * (int64_t)dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p<<31) - var2)*3125) / var1;
    var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
    return (float)p/256;
}

float BME::getHumidity(void)
{

    getTemperature(); // must be done first to get t_fine

    int32_t adc_H = read16(0xFD);
    if (adc_H == 0x8000) // value in case humidity measurement was disabled
        return NAN;

    int32_t v_x1_u32r;

    v_x1_u32r = (t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) -
                    (((int32_t)dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    float h = (v_x1_u32r>>12);
    return  h / 1024.0;
}

bool BME::begin(void)
{
    _i2caddr = 0x77;
    TwoWire * _wire = &Wire;
    return init();
}

bool BME::init(void)
{
    if (_cs == -1) 
        _wire -> begin();
    else {
        digitalWrite(_cs, HIGH);
        pinMode(_cs, OUTPUT);
        if (_sck == -1)
            SPI.begin();
        else {
            pinMode(_sck, OUTPUT);
            pinMode(_mosi, OUTPUT);
            pinMode(_miso, INPUT);
        }
    }

    if (read8(0xD0) != 0x60)
        return false;

    write8(0xE0, 0xB6);
    delay(300);

    uint8_t const rStatus = read8(0XF3);
    while ((rStatus & (1 << 0)) != 0)
          delay(100);

    readCoefficients();
    setSampling();
    delay(100);

    return true;
}

uint16_t BME::read16_LE(byte reg)
{
    uint16_t temp = read16(reg);
    return (temp >> 8) | (temp << 8);
}

int16_t BME::readS16_LE(byte reg)
{
    return (int16_t)read16_LE(reg);
}

uint8_t BME::spixfer(uint8_t x)
{
    // hardware SPI
    if (_sck == -1)
        return SPI.transfer(x);

    // software SPI
    uint8_t reply = 0;
    for (int i=7; i>=0; i--) {
        reply <<= 1;
        digitalWrite(_sck, LOW);
        digitalWrite(_mosi, x & (1<<i));
        digitalWrite(_sck, HIGH);
        if (digitalRead(_miso))
            reply |= 1;
        }
    return reply;
}

uint32_t BME::read24(byte reg)
{
    uint32_t value;

    if (_cs == -1) {
        _wire -> beginTransmission((uint8_t)_i2caddr);
        _wire -> write((uint8_t)reg);
        _wire -> endTransmission();
        _wire -> requestFrom((uint8_t)_i2caddr, (byte)3);

        value = _wire -> read();
        value <<= 8;
        value |= _wire -> read();
        value <<= 8;
        value |= _wire -> read();
    } else {
        if (_sck == -1)
            SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        digitalWrite(_cs, LOW);
        spixfer(reg | 0x80); // read, bit 7 high

        value = spixfer(0);
        value <<= 8;
        value |= spixfer(0);
        value <<= 8;
        value |= spixfer(0);

        digitalWrite(_cs, HIGH);
        if (_sck == -1)
            SPI.endTransaction(); // release the SPI bus
    }

    return value;
}

uint8_t BME::read8(byte reg)
{
    uint8_t value;
    
    if (_cs == -1) {
        _wire -> beginTransmission((uint8_t)_i2caddr);
        _wire -> write((uint8_t)reg);
        _wire -> endTransmission();
        _wire -> requestFrom((uint8_t)_i2caddr, (byte)1);
        value = _wire -> read();
    } else {
        if (_sck == -1)
            SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        digitalWrite(_cs, LOW);
        spixfer(reg | 0x80); // read, bit 7 high
        value = spixfer(0);
        digitalWrite(_cs, HIGH);
        if (_sck == -1)
            SPI.endTransaction(); // release the SPI bus
    }
    return value;
}

uint16_t BME::read16(byte reg)
{
    uint16_t value;

    if (_cs == -1) {
        _wire -> beginTransmission((uint8_t)_i2caddr);
        _wire -> write((uint8_t)reg);
        _wire -> endTransmission();
        _wire -> requestFrom((uint8_t)_i2caddr, (byte)2);
        value = (_wire -> read() << 8) | _wire -> read();
    } else {
        if (_sck == -1)
            SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        digitalWrite(_cs, LOW);
        spixfer(reg | 0x80); // read, bit 7 high
        value = (BME_spixfer(0) << 8) | spixfer(0);
        digitalWrite(_cs, HIGH);
        if (_sck == -1)
            SPI.endTransaction(); // release the SPI bus
    }

    return value;
}

void BME::write8(byte reg, byte value)
{

    if (_cs == -1) {
        _wire -> beginTransmission((uint8_t)_i2caddr);
        _wire -> write((uint8_t)reg);
        _wire -> write((uint8_t)value);
        _wire -> endTransmission();
    } else {
        if (_sck == -1)
            SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
        digitalWrite(_cs, LOW);
        spixfer(reg & ~0x80); // write, bit 7 low
        spixfer(value);
        digitalWrite(_cs, HIGH);
    if (_sck == -1)
        SPI.endTransaction(); // release the SPI bus
    }
}

//---------------------------------------------//

  //DHT : AINDA EM ANDAMENTO
  //VERIFICAR: funcao read(),cases dht e define .h,define debug_println .h,construtor do dht e .cpp da library 

float DHT::readTemperature(bool S, bool force) {
  float f = NAN;

  if (read(force)) {
      f = data[2];
  }
  return f;
}

//Aerospace::dht_readHumidity
float DHT::readHumidity(bool force) {
  float f = NAN;
  if (read()) {
      f = data[0];
    }
  
  return f;
}

//Aerospace::dht_read
bool DHT::read(bool force) { 
  // Check if sensor was read less than two seconds ago and return early
  // to use last reading.
  uint32_t currenttime = millis();
  if (!force && ((currenttime - _lastreadtime) < 2000)) {
    return _lastresult; // return last correct measurement
  }
  _lastreadtime = currenttime;

  // Reset 40 bits of received data to zero.
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

  // Send start signal.  See DHT datasheet for full signal diagram:
  //   http://www.adafruit.com/datasheets/Digital%20humidity%20and%20temperature%20sensor%20AM2302.pdf

  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.
  digitalWrite(_pin, HIGH);
  delay(250);

  // First set data line low for 20 milliseconds.
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delay(20);

  uint32_t cycles[80];
  {
    // Turn off interrupts temporarily because the next sections are timing critical
    // and we don't want any interruptions.
    DHT_InterruptLock lock;

    // End the start signal by setting data line high for 40 microseconds.
    digitalWrite(_pin, HIGH);
    delayMicroseconds(40);

    // Now start reading the data line to get the value from the DHT sensor.
    pinMode(_pin, INPUT_PULLUP);
    delayMicroseconds(10);  // Delay a bit to let sensor pull data line low.

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(LOW) == 0) {
      DEBUG_PRINTLN(F("Timeout waiting for start signal low pulse."));
      _lastresult = false;
      return _lastresult;
    }
    if (expectPulse(HIGH) == 0) {
      DEBUG_PRINTLN(F("Timeout waiting for start signal high pulse."));
      _lastresult = false;
      return _lastresult;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed all
    // the pulses are read into a array and then examined in a later step.
    for (int i=0; i<80; i+=2) {
      cycles[i]   = expectPulse(LOW);
      cycles[i+1] = expectPulse(HIGH);
    }
  } // Timing critical code is now complete.

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
  for (int i=0; i<40; ++i) {
    uint32_t lowCycles  = cycles[2*i];
    uint32_t highCycles = cycles[2*i+1];
    if ((lowCycles == 0) || (highCycles == 0)) {
      DEBUG_PRINTLN(F("Timeout waiting for pulse."));
      _lastresult = false;
      return _lastresult;
    }
    data[i/8] <<= 1;//x<<=y is x=x*2^y
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      data[i/8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  DEBUG_PRINTLN(F("Received:"));
  DEBUG_PRINT(data[0], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[1], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[2], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[3], HEX); DEBUG_PRINT(F(", "));
  DEBUG_PRINT(data[4], HEX); DEBUG_PRINT(F(" =? "));
  DEBUG_PRINTLN((data[0] + data[1] + data[2] + data[3]) & 0xFF, HEX);

  // Check we read 40 bits and that the checksum matches.
  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    _lastresult = true;
    return _lastresult;
  }
  else {
    DEBUG_PRINTLN(F("Checksum failure!"));
    _lastresult = false;
    return _lastresult;
  }
}

//Aerospace::dht_expectPulse
uint32_t DHT::expectPulse(bool level) {
  uint32_t count = 0;
  // On AVR platforms use direct GPIO port access as it's much faster and better
  // for catching pulses that are 10's of microseconds in length:
  #ifdef __AVR
    uint8_t portState = level ? _bit : 0;
    while ((*portInputRegister(_port) & _bit) == portState) {
      if (count++ >= _maxcycles) {
        return 0; // Exceeded timeout, fail.
      }
    }
  // Otherwise fall back to using digitalRead (this seems to be necessary on ESP8266
  // right now, perhaps bugs in direct port access functions?).
  #else
    while (digitalRead(_pin) == level) {
      if (count++ >= _maxcycles) {
        return 0; // Exceeded timeout, fail.
      }
    }
  #endif

  return count;
}

//--------------------------GPS------------------------------------
bool GPS_encode(char c)
{
  bool valid_sentence = false;

  #ifndef _NO_STATS
  ++_encoded_characters;
  #endif
  switch(c)
  {
  case ',': // term terminators
    _parity ^= c;
  case '\r':
  case '\n':
  case '*':
    if (_term_offset < sizeof(_term))
    {
      _term[_term_offset] = 0;
      valid_sentence = term_complete();
    }
    ++_term_number;
    _term_offset = 0;
    _is_checksum_term = c == '*';
    return valid_sentence;

  case '$': // sentence begin
    _term_number = _term_offset = 0;
    _parity = 0;
    _sentence_type = _SENTENCE_OTHER;
    _is_checksum_term = false;
    _gps_data_good = false;
    return valid_sentence;
  }

  // ordinary characters
  if (_term_offset < sizeof(_term) - 1)
    _term[_term_offset++] = c;
  if (!_is_checksum_term)
    _parity ^= c;

  return valid_sentence;
}

bool GPS::term_complete()
{
  if (_is_checksum_term)
  {
    byte checksum = 16 * GPS_from_hex(_term[0]) + GPS_from_hex(_term[1]);
    if (checksum == _parity)
    {
      if(_sentence_type == _SENTENCE_GPRMC)   //set the time and date even if not tracking
      {
          _time      = _new_time;
          _date      = _new_date;
      }
      if (_gps_data_good)
      {
        #ifndef _NO_STATS
        ++_good_sentences;
        #endif
        _last_time_fix = _new_time_fix;
        _last_position_fix = _new_position_fix;

        switch(_sentence_type)
        {
        case _SENTENCE_GPRMC:
          _time      = _new_time;
          _date      = _new_date;
          _latitude  = _new_latitude;
          _longitude = _new_longitude;
          _speed     = _new_speed;
          _course    = _new_course;
          break;
        case _SENTENCE_GPGGA:
          _altitude  = _new_altitude;
          _time      = _new_time;
          _latitude  = _new_latitude;
          _longitude = _new_longitude;
          _numsats   = _new_numsats;
          _hdop      = _new_hdop;
          break;
        }

        return true;
      }
    }

    #ifndef _NO_STATS
    else
      ++_failed_checksum;
    #endif
    return false;
  }

  // the first term determines the sentence type
  if (_term_number == 0)
  {
    if (!gpsstrcmp(_term, _GPRMC_TERM) || !gpsstrcmp(_term, _GNRMC_TERM))
      _sentence_type = _SENTENCE_GPRMC;
    else if (!gpsstrcmp(_term, _GPGGA_TERM))
      _sentence_type = _SENTENCE_GPGGA;
    else if (!gpsstrcmp(_term, _GNGNS_TERM))
      _sentence_type = _SENTENCE_GNGNS;
    else if (!gpsstrcmp(_term, _GNGSA_TERM) || !gpsstrcmp(_term, _GPGSA_TERM))
      _sentence_type = _SENTENCE_GNGSA;
    else if (!gpsstrcmp(_term, _GPGSV_TERM))
      _sentence_type = _SENTENCE_GPGSV;
    else if (!gpsstrcmp(_term, _GLGSV_TERM))
      _sentence_type = _SENTENCE_GLGSV;
    else
      _sentence_type = _SENTENCE_OTHER;
    return false;
  }

  if (_sentence_type != _SENTENCE_OTHER && _term[0])
    switch(COMBINE(_sentence_type, _term_number))
  {
    case COMBINE(_SENTENCE_GPRMC, 1): // Time in both sentences
    case COMBINE(_SENTENCE_GPGGA, 1):
    case COMBINE(_SENTENCE_GNGNS, 1):
      _new_time = parse_decimal();
      _new_time_fix = millis();
      break;
    case COMBINE(_SENTENCE_GPRMC, 2): // GPRMC validity
      _gps_data_good = _term[0] == 'A';
      break;
    case COMBINE(_SENTENCE_GPRMC, 3): // Latitude
    case COMBINE(_SENTENCE_GPGGA, 2):
    case COMBINE(_SENTENCE_GNGNS, 2):
      _new_latitude = parse_degrees();
      _new_position_fix = millis();
      break;
    case COMBINE(_SENTENCE_GPRMC, 4): // N/S
    case COMBINE(_SENTENCE_GPGGA, 3):
    case COMBINE(_SENTENCE_GNGNS, 3):
      if (_term[0] == 'S')
        _new_latitude = -_new_latitude;
      break;
    case COMBINE(_SENTENCE_GPRMC, 5): // Longitude
    case COMBINE(_SENTENCE_GPGGA, 4):
    case COMBINE(_SENTENCE_GNGNS, 4):
      _new_longitude = parse_degrees();
      break;
    case COMBINE(_SENTENCE_GPRMC, 6): // E/W
    case COMBINE(_SENTENCE_GPGGA, 5):
    case COMBINE(_SENTENCE_GNGNS, 5):
      if (_term[0] == 'W')
        _new_longitude = -_new_longitude;
      break;
    case COMBINE(_SENTENCE_GNGNS, 6):
      strncpy(_constellations, _term, 5);
      break;
    case COMBINE(_SENTENCE_GPRMC, 7): // Speed (GPRMC)
      _new_speed = parse_decimal();
      break;
    case COMBINE(_SENTENCE_GPRMC, 8): // Course (GPRMC)
      _new_course = parse_decimal();
      break;
    case COMBINE(_SENTENCE_GPRMC, 9): // Date (GPRMC)
      _new_date = gpsatol(_term);
      break;
    case COMBINE(_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
      _gps_data_good = _term[0] > '0';
      break;
    case COMBINE(_SENTENCE_GPGGA, 7): // Satellites used (GPGGA): GPS only
    case COMBINE(_SENTENCE_GNGNS, 7): //  GNGNS counts-in all constellations
      _new_numsats = (unsigned char)atoi(_term);
      break;
    case COMBINE(_SENTENCE_GPGGA, 8): // HDOP
      _new_hdop = parse_decimal();
      break;
    case COMBINE(_SENTENCE_GPGGA, 9): // Altitude (GPGGA)
      _new_altitude = parse_decimal();
      break;
    case COMBINE(_SENTENCE_GNGSA, 3): //satellites used in solution: 3 to 15
      //_sats_used[
      break;
    case COMBINE(_SENTENCE_GPGSV, 2):   //beginning of sequence
    case COMBINE(_SENTENCE_GLGSV, 2):   //beginning of sequence
    {
      uint8_t msgId = atoi(_term)-1;  //start from 0
      if(msgId == 0) {
        //http://geostar-navigation.com/file/geos3/geos_nmea_protocol_v3_0_eng.pdf
        if(_sentence_type == _SENTENCE_GPGSV) {
          //reset GPS & WAAS trackedSatellites
          for(uint8_t x=0;x<12;x++)
          {
            tracked_sat_rec[x] = 0;
          }
        } else {
          //reset GLONASS trackedSatellites: range starts with 23
          for(uint8_t x=12;x<24;x++)
          {
            tracked_sat_rec[x] = 0;
          }
        }
      }
      _sat_index = msgId*4;   //4 sattelites/line
      if(_sentence_type == _SENTENCE_GLGSV)
      {
        _sat_index = msgId*4 + 12;   //Glonass offset by 12
      }
      break;
  }
    case COMBINE(_SENTENCE_GPGSV, 4):   //satellite #
    case COMBINE(_SENTENCE_GPGSV, 8):
    case COMBINE(_SENTENCE_GPGSV, 12):
    case COMBINE(_SENTENCE_GPGSV, 16):
    case COMBINE(_SENTENCE_GLGSV, 4):
    case COMBINE(_SENTENCE_GLGSV, 8):
    case COMBINE(_SENTENCE_GLGSV, 12):
    case COMBINE(_SENTENCE_GLGSV, 16):
      _tracked_satellites_index = atoi(_term);
      break;
    case COMBINE(_SENTENCE_GPGSV, 7):   //strength
    case COMBINE(_SENTENCE_GPGSV, 11):
    case COMBINE(_SENTENCE_GPGSV, 15):
    case COMBINE(_SENTENCE_GPGSV, 19):
    case COMBINE(_SENTENCE_GLGSV, 7):   //strength
    case COMBINE(_SENTENCE_GLGSV, 11):
    case COMBINE(_SENTENCE_GLGSV, 15):
    case COMBINE(_SENTENCE_GLGSV, 19):
      uint8_t stren = (uint8_t)atoi(_term);
      if(stren == 0)  //remove the record, 0dB strength
      {
        tracked_sat_rec[_sat_index + (_term_number-7)/4] = 0;
      }
      else
      {
        tracked_sat_rec[_sat_index + (_term_number-7)/4] = _tracked_satellites_index<<8 | stren<<1;
      }
      break;
  }

  return false;
}

void GPS::get_position(long *latitude, long *longitude, unsigned long *fix_age)
{
  if (latitude) *latitude = _latitude;
  if (longitude) *longitude = _longitude;
  if (fix_age) *fix_age = _last_position_fix == INVALID_FIX_TIME ? 
   INVALID_AGE : millis() - _last_position_fix;
}

void GPS::get_datetime(unsigned long *date, unsigned long *time, unsigned long *age)
{
  if (date) *date = _date;
  if (time) *time = _time;
  if (age) *age = _last_time_fix == INVALID_FIX_TIME ? 
   INVALID_AGE : millis() - _last_time_fix;
}

float GPS::f_altitude()    
{
  return _altitude == INVALID_ALTITUDE ? INVALID_F_ALTITUDE : _altitude / 100.0;
}

float GPS::f_speed_kmph()  
{ 
  float sk = f_speed_knots();
  return sk == INVALID_F_SPEED ? INVALID_F_SPEED : _GPS_KMPH_PER_KNOT * sk; 
}

float GPS::f_speed_mps()   
{ 
  float sk = f_speed_knots();
  return sk == INVALID_F_SPEED ? INVALID_F_SPEED : _GPS_MPS_PER_KNOT * sk; 
}

void GPS::crack_datetime(int *year, byte *month, byte *day, 
  byte *hour, byte *minute, byte *second, byte *hundredths, unsigned long *age)
{
  unsigned long date, time;
  get_datetime(&date, &time, age);
  if (year) 
  {
    *year = date % 100;
    *year += *year > 80 ? 1900 : 2000;
  }
  if (month) *month = (date / 100) % 100;
  if (day) *day = date / 10000;
  if (hour) *hour = time / 1000000;
  if (minute) *minute = (time / 10000) % 100;
  if (second) *second = (time / 100) % 100;
  if (hundredths) *hundredths = time % 100;
}

int GPS::from_hex(char a) 
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

int GPS::gpsstrcmp(const char *str1, const char *str2)
{
  while (*str1 && *str1 == *str2)
    ++str1, ++str2;
  return *str1;
}

unsigned long GPS::parse_decimal()
{
  char *p = _term;
  bool isneg = *p == '-';
  if (isneg) ++p;
  unsigned long ret = 100UL * gpsatol(p);
  while (gpsisdigit(*p)) ++p;
  if (*p == '.')
  {
    if (gpsisdigit(p[1]))
    {
      ret += 10 * (p[1] - '0');
      if (gpsisdigit(p[2]))
        ret += p[2] - '0';
    }
  }
  return isneg ? -ret : ret;
}

unsigned long GPS::parse_degrees()
{
  char *p;
  unsigned long left_of_decimal = gpsatol(_term);
  unsigned long hundred1000ths_of_minute = (left_of_decimal % 100UL) * 100000UL;
  for (p=_term; gpsisdigit(*p); ++p);
  if (*p == '.')
  {
    unsigned long mult = 10000;
    while (gpsisdigit(*++p))
    {
      hundred1000ths_of_minute += mult * (*p - '0');
      mult /= 10;
    }
  }
  return (left_of_decimal / 100) * 1000000 + (hundred1000ths_of_minute + 3) / 6;
}

long GPS::gpsatol(const char *str)
{
  long ret = 0;
  while (gpsisdigit(*str))
    ret = 10 * ret + *str++ - '0';
  return ret;
}
float GPS::f_speed_knots() 
{
  return _speed == INVALID_SPEED ? INVALID_F_SPEED : _speed / 100.0;
}
const float GPS::INVALID_F_ANGLE = 1000.0;
const float GPS::INVALID_F_ALTITUDE = 1000000.0;
const float GPS::INVALID_F_SPEED = -1.0;
