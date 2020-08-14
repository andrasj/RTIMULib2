////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib
//
//  Copyright (c) 2014-2015, richards-tech, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

//  The MPU-9250/ICM-20948 and SPI driver code is based on code generously supplied by
//  staslock@gmail.com (www.clickdrive.io)


#ifndef _RTIMUICM20948_H
#define	_RTIMUICM20948_H

#include "RTIMU.h"

//  Define this symbol to use cache mode

#define ICM20948_CACHE_MODE

//  FIFO transfer size

#define ICM20948_FIFO_CHUNK_SIZE     12                      // gyro and accels take 12 bytes

#ifdef ICM20948_CACHE_MODE

//  Cache mode defines

#define ICM20948_CACHE_SIZE          16                      // number of chunks in a block
#define ICM20948_CACHE_BLOCK_COUNT   16                      // number of cache blocks

typedef struct
{
    unsigned char data[ICM20948_FIFO_CHUNK_SIZE * ICM20948_CACHE_SIZE];
    int count;                                              // number of chunks in the cache block
    int index;                                              // current index into the cache
    unsigned char compass[9];                               // the raw compass readings for the block

} ICM20948_CACHE_BLOCK;

#endif


class RTIMUICM20948 : public RTIMU
{
public:
    RTIMUICM20948(RTIMUSettings *settings);
    ~RTIMUICM20948();

    virtual const char *IMUName() { return "ICM-20948"; }
    virtual int IMUType() { return RTIMU_TYPE_ICM20948; }
    virtual bool IMUInit();
    virtual bool IMURead();
    virtual int IMUGetPollInterval();

private:
    bool HALOpen() {return m_settings->HALOpen();}
    void HALClose(){m_settings->HALClose();}
    bool SelectBank(unsigned char slaveAddr, unsigned short addr); // ensures the correct bank is activated in ICM20948 before performing the real write
    bool HALRead(unsigned char slaveAddr, unsigned short addr, unsigned char length,
                 unsigned char *data, const char *errorMsg);    // normal read with register select
    bool HALWrite(unsigned char slaveAddr, unsigned short addr,
                  unsigned char length, unsigned char const *data, const char *errorMsg);
    bool HALWrite(unsigned char slaveAddr, unsigned short addr,
                  unsigned char const data, const char *errorMsg);
    bool foo_debug();


private:
    //validate & precalculate chip values
    bool setGyroLpf(unsigned char lpf);
    bool setGyroFsr(unsigned char fsr);
    bool setAccelLpf(unsigned char lpf);
    bool setAccelFsr(unsigned char fsr);
    bool setSampleRate(int rate);
    bool setCompassRate(int rate);

    //configure chip
    bool resetFifo();
    bool setGyroConfig();
    bool setAccelConfig();
    bool compassSetup();
    bool bypassOn();
    bool bypassOff();


private:
    uint8_t m_lastbank; //last set bank

    bool m_firstTime;                                       // if first sample

    unsigned char m_slaveAddr;                              // I2C address of ICM20948

    unsigned char m_gyroLpf;                                // gyro low pass filter setting
    unsigned char m_accelLpf;                               // accel low pass filter setting
    unsigned char m_compassRate;                            // compass sample rate mode
    unsigned char m_gyroFsr;
    unsigned char m_accelFsr;

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;


#ifdef ICM20948_CACHE_MODE

    ICM20948_CACHE_BLOCK m_cache[ICM20948_CACHE_BLOCK_COUNT]; // the cache itself
    int m_cacheIn;                                          // the in index
    int m_cacheOut;                                         // the out index
    int m_cacheCount;                                       // number of used cache blocks

#endif

};

#endif // _RTIMUICM20948_H
