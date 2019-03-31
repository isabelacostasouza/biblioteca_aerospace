

#ifndef Aerospace_h
#define Aerospace_h

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#include <pins_arduino.h>
#endif

//DHT
#define DEBUG_PRINTER Serial

// Setup debug printing macros.
#ifdef DHT_DEBUG
  #define DEBUG_PRINT(...) { DEBUG_PRINTER.print(__VA_ARGS__); }
  #define DEBUG_PRINTLN(...) { DEBUG_PRINTER.println(__VA_ARGS__); }
#else
  #define DEBUG_PRINT(...) {}
  #define DEBUG_PRINTLN(...) {}
#endif

// Define types of sensors.
#define DHT11 11


//------------------GPS----------------

#define _GPS_VERSION 13 // software version of this library
#define _GPS_MPH_PER_KNOT 1.15077945
#define _GPS_MPS_PER_KNOT 0.51444444
#define _GPS_KMPH_PER_KNOT 1.852
#define _GPS_MILES_PER_METER 0.00062137112
#define _GPS_KM_PER_METER 0.001


class GPS
{
  public:
  GPS(); 
    
    static const float INVALID_F_ANGLE, INVALID_F_ALTITUDE, INVALID_F_SPEED;

    enum {
      INVALID_AGE = 0xFFFFFFFF,      INVALID_ANGLE = 999999999, 
      INVALID_ALTITUDE = 999999999,  INVALID_DATE = 0,
      INVALID_TIME = 0xFFFFFFFF,     INVALID_SPEED = 999999999, 
      INVALID_FIX_TIME = 0xFFFFFFFF, INVALID_SATELLITES = 0xFF,
      INVALID_HDOP = 0xFFFFFFFF
    };

    bool encode(char c);
    void get_position(long *latitude, long *longitude, unsigned long *fix_age = 0);
    void get_datetime(unsigned long *date, unsigned long *time, unsigned long *age = 0);
    void gcrack_datetime(int *year, byte *month, byte *day,byte *hour, byte *minute, byte *second, byte *hundredths = 0, unsigned long *fix_age = 0);
    float f_altitude();
    float f_speed_kmph();
    float f_speed_knots();
    float f_speed_mps();

  private:
    int gpsstrcmp(const char *str1, const char *str2);
    unsigned long parse_decimal();
    unsigned long parse_degrees();
    long gpsatol(const char *str);
    bool gpsisdigit(char c) { return c >= '0' && c <= '9'; }
    enum {_SENTENCE_GPGGA, _SENTENCE_GPRMC, _SENTENCE_GNGNS, _SENTENCE_GNGSA,
      _SENTENCE_GPGSV, _SENTENCE_GLGSV, _SENTENCE_OTHER};
    // properties
    unsigned long _time, _new_time;
    unsigned long _date, _new_date;
    long _latitude, _new_latitude;
    long _longitude, _new_longitude;
    long _altitude, _new_altitude;
    unsigned long  _speed, _new_speed;
    unsigned long  _course, _new_course;
    unsigned long  _hdop, _new_hdop;
    unsigned short _numsats, _new_numsats;

    unsigned long _last_time_fix, _new_time_fix;
    unsigned long _last_position_fix, _new_position_fix;

    // parsing state variables
    byte _parity;
    bool _is_checksum_term;
    char _term[15];
    byte _sentence_type;
    byte _term_number;
    byte _term_offset;
    bool _gps_data_good;
    struct TrackedSattelites {
      uint8_t prn;      //"pseudo-random noise" sequences, or Gold codes. GPS sats are listed here http://en.wikipedia.org/wiki/List_of_GPS_satellites
      uint8_t strength; //in dB
    };

    char _constellations[5];
    uint8_t _sat_used_count;

    //format:
    //bit 0-7: sat ID
    //bit 8-14: snr (dB), max 99dB
    //bit 15: used in solution (when tracking)
    uint32_t tracked_sat_rec[24]; //TODO: externalize array size
    int _tracked_satellites_index;
    uint8_t _sat_index;

    #ifndef _NO_STATS
    // statistics
    unsigned long _encoded_characters;
    unsigned short _good_sentences;
    unsigned short _failed_checksum;
    unsigned short _passed_checksum;
    #endif
    int from_hex(char a);
    bool term_complete();
}

class Accelero
{
  public:
    Accelero();
    void begin();
    int getXAccel();
    int getYAccel();
    int getZAccel();
    void setOffSets(int xOffSet, int yOffSet, int zOffSet);
    void calibrate();                             // only to be executed when Z-axis is oriented to the ground
    int getOrientation();

  private:
    int _sleepPin;
    int _selfTestPin;
    int _zeroGPin;
    int _gSelectPin;
    int _xPin;
    int _yPin;
    int _zPin;
    int _offSets[3];
    double _refVoltage;
    int _average;
    boolean _sleep;
    boolean _sensi;
}

class DHT
{
  public:
    DHT();
    float readTemperature(bool S=false, bool force=false);
    float readHumidity(bool force=false);
    bool read(bool force=false);

  private:
    uint8_t data[5];
    uint8_t _pin, _type;
    #ifdef __AVR
    // Use direct GPIO access on an 8-bit AVR so keep track of the port and bitmask
    // for the digital pin connected to the DHT.  Other platforms will use digitalRead.
    uint8_t _bit, _port;
    #endif
    uint32_t _lastreadtime, _maxcycles;
    bool _lastresult;
    uint32_t expectPulse(bool level);
}

class BME
{
  public:
    BME();
    bool BME_begin(void);
    bool BME_init();


    float BME_getTemperature(void);
    float BME_getPressure(void);
    float BME_getHumidity(void);
        
    float BME_getAltitude(float seaLevel);

  private:
    TwoWire *_wire;
    uint8_t spixfer(uint8_t x);

    void write8(byte reg, byte value);
    uint8_t   read8(byte reg);
    uint16_t  read16(byte reg);
    uint32_t  read24(byte reg);
    int16_t   readS16(byte reg);
    uint16_t  read16_LE(byte reg); // little endian
    int16_t   readS16_LE(byte reg); // little endian
    uint8_t   _i2caddr;
    int8_t _cs, _mosi, _miso, _sck;



}

class DHT_InterruptLock {
  public:
   DHT_InterruptLock() {
    noInterrupts();
   }
   ~DHT_InterruptLock() {
    interrupts();
   }

};

#endif
