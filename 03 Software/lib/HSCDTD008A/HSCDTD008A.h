
#ifndef Compass_h
#define Compass_h

#include "Arduino.h"
#include <Wire.h>

class HSCDTD008A
{
private:
    TwoWire &_twi;
    const uint8_t _addr;
    int16_t _x, _y, _z;

public:
    HSCDTD008A(TwoWire &twi = Wire, uint8_t addr = 0x0F) : _twi(twi), _addr(addr), _x(0), _y(0), _z(0) {}

    inline bool begin(bool initTwi = true)
    {
        if (initTwi)
            _twi.begin();
        _twi.beginTransmission(_addr);
        _twi.write(0x1B);       //  Control1
        _twi.write(0b10001010); //  Active Mode, Force State
        if (_twi.endTransmission())
            return false;
        _twi.beginTransmission(_addr);
        _twi.write(0x1C);       //  Control2
        _twi.write(0b00011100); //  DRDY high + FIFO
        if (_twi.endTransmission())
            return false;
        _twi.beginTransmission(_addr);
        _twi.write(0x1D);       //  Control3
        _twi.write(0b00000000); //  Nothing special
        if (_twi.endTransmission())
            return false;
        _twi.beginTransmission(_addr);
        _twi.write(0x1E);       //  Control4
        _twi.write(0b10000000); //  14bit resolution
        if (_twi.endTransmission())
            return false;
        return true;
    }

    inline bool calibrate()
    {
        //  Ensure we are in active force state ...
        _twi.beginTransmission(_addr);
        _twi.write(0x1B);       //  Control1
        _twi.write(0b10001010); //  Active Mode, Force State
        if (_twi.endTransmission())
            return false;

        //  Now trigger temperature calibration and
        //  wait for it to finish.
        _twi.beginTransmission(_addr);
        _twi.write(0x1D);       //  Control3
        _twi.write(0b00000010); //  Start Temp Calibration
        if (_twi.endTransmission())
            return false;

        //  Wait a bit and then check whether done ...
        for (;;)
        {
            delay(1);
            _twi.beginTransmission(_addr);
            _twi.write(0x1D);
            if (_twi.endTransmission())
                return false;
            _twi.requestFrom(_addr, 1, true);
            if (_twi.available() >= 1)
            {
                if (!(_twi.read() & 0b00000010))
                    break;
            }
        }

        //  Now basically the same thing, but for the offset
        //  calibration ...
        _twi.beginTransmission(_addr);
        _twi.write(0x1D);       //  Control3
        _twi.write(0b00000001); //  Start Offset Calibration
        if (_twi.endTransmission())
            return false;

        //  Wait a bit and then check whether done ...
        for (;;)
        {
            delay(1);
            _twi.beginTransmission(_addr);
            _twi.write(0x1D);
            if (_twi.endTransmission())
                return false;
            _twi.requestFrom(_addr, 1, true);
            if (_twi.available() >= 1)
            {
                if (!(_twi.read() & 0b00000001))
                    break;
            }
        }

        return true;
    }

    inline bool measure()
    {
        bool success = false;

        //  Check whether there is data ready ...
        _twi.beginTransmission(_addr);
        _twi.write(0x18);
        if (_twi.endTransmission())
            return false;
        _twi.requestFrom(_addr, 1, true);
        if (_twi.available() >= 1)
        {
            if (_twi.read() & 0b01000000)
            {
                //  There is data pending ... Updating our
                //  coordinates ...
                _twi.beginTransmission(_addr);
                _twi.write(0x10);
                if (_twi.endTransmission())
                    return false;
                _twi.requestFrom(_addr, 6, true);
                if (_twi.available() >= 6)
                {
                    _x = (int16_t)(_twi.read() | (((uint16_t)_twi.read()) << 8));
                    _y = (int16_t)(_twi.read() | (((uint16_t)_twi.read()) << 8));
                    _z = (int16_t)(_twi.read() | (((uint16_t)_twi.read()) << 8));
                    success = true;
                }
            }
        }

        //  Trigger a force mode read if not still active ...
        _twi.beginTransmission(_addr);
        _twi.write(0x1D); //  Control3
        if (_twi.endTransmission())
            return success;
        _twi.requestFrom(_addr, 1, true);
        if (_twi.available() >= 1)
        {
            if (!(_twi.read() & 0b01000000))
            {
                _twi.beginTransmission(_addr);
                _twi.write(0x1D); //  Control3
                _twi.write(0b01000000);
                _twi.endTransmission();
            }
        }
        return success;
    }

    inline int16_t x() { return _x; }
    inline int16_t y() { return _y; }
    inline int16_t z() { return _z; }
};
#endif