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

#include "RTIMUICM20948.h"
#include "RTIMUSettings.h"
#include "RTIMUICM20948Def.h"

RTIMUICM20948::RTIMUICM20948(RTIMUSettings *settings) : RTIMU(settings)
{

}

RTIMUICM20948::~RTIMUICM20948()
{
}

bool RTIMUICM20948::IMUInit()
{
    unsigned char result;

    m_firstTime = true;

#ifdef ICM20948_CACHE_MODE
    m_cacheIn = m_cacheOut = m_cacheCount = 0;
#endif

    // set validity flags

    m_imuData.fusionPoseValid = false;
    m_imuData.fusionQPoseValid = false;
    m_imuData.gyroValid = true;
    m_imuData.accelValid = true;
    m_imuData.compassValid = true;
    m_imuData.pressureValid = false;
    m_imuData.temperatureValid = false;
    m_imuData.humidityValid = false;

    //  configure IMU

    m_slaveAddr = m_settings->m_I2CSlaveAddress;

    setSampleRate(m_settings->m_ICM20948GyroAccelSampleRate);
    // setCompassRate(m_settings->m_ICM20948CompassSampleRate);
    setGyroLpf(m_settings->m_ICM20948GyroLpf);
    setAccelLpf(m_settings->m_ICM20948AccelLpf);
    setGyroFsr(m_settings->m_ICM20948GyroFsr);
    setAccelFsr(m_settings->m_ICM20948AccelFsr);

    setCalibrationData();


    //  enable the bus

    if (!m_settings->HALOpen())
        return false;

    //  reset the ICM20948

    if (!m_settings->HALWrite(m_slaveAddr,  ICM20948_REG_PWR_MGMT_1,  ICM20948_BIT_H_RESET, "Failed to initiate ICM20948 reset"))
        return false;

    m_settings->delayMs(100);

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_REG_WHO_AM_I, 1, &result, "Failed to read ICM20948 id"))
        return false;

    if (result != ICM20948_DEVICE_ID) {
        HAL_ERROR2("Incorrect %s id %d\n", IMUName(), result);
        return false;
    }

    /* Auto selects the best available clock source  PLL if ready, else use the Internal oscillator */
    if (!m_settings->HALWrite(m_slaveAddr,  ICM20948_REG_PWR_MGMT_1,  ICM20948_BIT_CLK_PLL, "Failed to select clock"))
        return false;

    /* PLL startup time - maybe it is too long but better be on the safe side, no spec in the datasheet */
    m_settings->delayMs(30);

    HAL_INFO("Clock selected\n");

    //  now configure the various components

    if (!setGyroConfig())
        return false;

    if (!setAccelConfig())
        return false;

    if (!setSampleRate())
        return false;

    // if(!compassSetup()) {
    //     return false;
    // }

    // if (!setCompassRate())
    //     return false;

    //  enable the sensors

    uint8_t pwrManagement1;
    uint8_t pwrManagement2;

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_REG_PWR_MGMT_1, 1, &pwrManagement1, "Failed to read current power state"))
         return false;
    pwrManagement2 = 0;

    /* To enable the accelerometer clear the DISABLE_ACCEL bits in PWR_MGMT_2 */
    pwrManagement2 &= ~(ICM20948_BIT_PWR_ACCEL_STBY);

    /* To enable gyro clear the DISABLE_GYRO bits in PWR_MGMT_2 */
    pwrManagement2 &= ~(ICM20948_BIT_PWR_GYRO_STBY);

    // /* To enable the temperature sensor clear the TEMP_DIS bit in PWR_MGMT_1 */
    // pwrManagement1 &= ~(ICM20948_BIT_TEMP_DIS);

    /* Write back the modified values */
    if (!m_settings->HALWrite(m_slaveAddr,  ICM20948_REG_PWR_MGMT_1,  pwrManagement1, "Failed to set REG_PWR_MGMT_1"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr,  ICM20948_REG_PWR_MGMT_2,  pwrManagement2, "Failed to set REG_PWR_MGMT_2"))
        return false;

    //  select the data to go into the FIFO and enable

     if (!resetFifo())
         return false;

    // gyroBiasInit();

    HAL_INFO1("%s init complete\n", IMUName());
    return true;
}

bool RTIMUICM20948::setSampleRate(int rate)
{
    if ((rate <ICM20948_SAMPLERATE_MIN) || (rate > ICM20948_SAMPLERATE_MAX)) {
        HAL_ERROR1("Illegal sample rate %d\n", rate);
        return false;
    }

    //  Note: rates interact with the lpf settings

    if ((rate < ICM20948_SAMPLERATE_MAX) && (rate >= 8000))
        rate = 8000;

    if ((rate < 8000) && (rate >= 1000))
        rate = 1000;

    if (rate < 1000) {
        int sampleDiv = (1000 / rate) - 1;
        m_sampleRate = 1000 / (1 + sampleDiv);
    } else {
        m_sampleRate = rate;
    }
    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;
    return true;
}

bool RTIMUICM20948::setGyroLpf(unsigned char lpf)
{
    switch (lpf) {
    case ICM20648_GYRO_LPF_12100HZ:
    case ICM20648_GYRO_LPF_360HZ  :
    case ICM20648_GYRO_LPF_200HZ  :
    case ICM20648_GYRO_LPF_150HZ  :
    case ICM20648_GYRO_LPF_120HZ  :
    case ICM20648_GYRO_LPF_51HZ   :
    case ICM20648_GYRO_LPF_24HZ   :
    case ICM20648_GYRO_LPF_12HZ   :
    case ICM20648_GYRO_LPF_6HZ    :
        m_gyroLpf = lpf;
        return true;

    default:
        HAL_ERROR1("Illegal ICM20648 gyro lpf %d\n", lpf);
        return false;
    }
}

bool RTIMUICM20948::setAccelLpf(unsigned char lpf)
{
    switch (lpf) {
    case ICM20648_ACCEL_LPF_1210HZ:
    case ICM20648_ACCEL_LPF_470HZ :
    case ICM20648_ACCEL_LPF_246HZ :
    case ICM20648_ACCEL_LPF_111HZ :
    case ICM20648_ACCEL_LPF_50HZ  :
    case ICM20648_ACCEL_LPF_24HZ  :
    case ICM20648_ACCEL_LPF_12HZ  :
    case ICM20648_ACCEL_LPF_6HZ   :
        m_accelLpf = lpf;
        return true;

    default:
        HAL_ERROR1("Illegal ICM20948 accel lpf %d\n", lpf);
        return false;
    }
}


bool RTIMUICM20948::setCompassRate(int rate)
{
    if ((rate < MPU9250_COMPASSRATE_MIN) || (rate > MPU9250_COMPASSRATE_MAX)) {
        HAL_ERROR1("Illegal compass rate %d\n", rate);
        return false;
    }
    m_compassRate = rate;
    return true;
}

bool RTIMUICM20948::setGyroFsr(unsigned char fsr)
{
    switch (fsr) {
    case ICM20648_GYRO_FULLSCALE_250DPS:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (131.0 * 180.0);
        return true;

    case ICM20648_GYRO_FULLSCALE_500DPS:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (62.5 * 180.0);
        return true;

    case ICM20648_GYRO_FULLSCALE_1000DPS:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (32.8 * 180.0);
        return true;

    case ICM20648_GYRO_FULLSCALE_2000DPS:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (16.4 * 180.0);
        return true;

    default:
        HAL_ERROR1("Illegal ICM20948 gyro fsr %d\n", fsr);
        return false;
    }
}

bool RTIMUICM20948::setAccelFsr(unsigned char fsr)
{
    switch (fsr) {
    case ICM20648_ACCEL_FULLSCALE_2G:
        m_accelFsr = fsr;
        m_accelScale = 1.0/16384.0;
        return true;

    case ICM20648_ACCEL_FULLSCALE_4G:
        m_accelFsr = fsr;
        m_accelScale = 1.0/8192.0;
        return true;

    case ICM20648_ACCEL_FULLSCALE_8G:
        m_accelFsr = fsr;
        m_accelScale = 1.0/4096.0;
        return true;

    case ICM20648_ACCEL_FULLSCALE_16G:
        m_accelFsr = fsr;
        m_accelScale = 1.0/2048.0;
        return true;

    default:
        HAL_ERROR1("Illegal ICM20948 accel fsr %d\n", fsr);
        return false;
    }
}


bool RTIMUICM20948::resetFifo(){

     if (!m_settings->HALWrite(m_slaveAddr, ICM20948_REG_USER_CTRL, 0, "Disabling the fifo"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_REG_FIFO_EN_1, 0, "Writing fifo disable"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_REG_FIFO_EN_2, 0, "disabling fifo for Acc & Gyro"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_REG_FIFO_RST, 0x0F, "Resetting fifo"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_REG_FIFO_RST, 0x00, "Undo Resetting fifo"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_REG_FIFO_EN_2, ICM20948_BIT_ACCEL_FIFO_EN|ICM20948_BITS_GYRO_FIFO_EN, "enabling fifo for Acc & Gyro"))
        return false;

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_REG_FIFO_MODE, 0x0F, "Set fifo as snapshot"))
        return false;
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_REG_USER_CTRL, ICM20948_BIT_FIFO_EN, "Enabling the fifo"))
        return false;

    m_settings->delayMs(50);
    return true;
}

bool RTIMUICM20948::setGyroConfig()
{
    unsigned char gyroConfig = m_gyroFsr | m_gyroLpf;

    HAL_INFO1("Set GYRO_CONFIG_1 to 0x%x\n",gyroConfig)

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_REG_GYRO_CONFIG_1, gyroConfig, "Failed to write gyro config"))
         return false;

    return true;
}

bool RTIMUICM20948::setAccelConfig()
{
    unsigned char accelConfig = m_accelFsr | m_accelLpf;
    HAL_INFO3("Set ACCEL_CONFIG to 0x%x (FS: 0x%x, LPF: 0x%x)\n",accelConfig,m_accelFsr,m_accelLpf);

    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_REG_ACCEL_CONFIG, accelConfig, "Failed to write accel config"))
         return false;

    return true;
}

bool RTIMUICM20948::setSampleRate()
{
    uint8_t gyroDiv;
    float gyroSampleRate;

    /* Calculate the sample rate divider */
    gyroSampleRate = (1125.0 / m_sampleRate) - 1.0;

    /* Check if it fits in the divider register */
    if ( gyroSampleRate > 255.0 ) {
        gyroSampleRate = 255.0;
    }

    if ( gyroSampleRate < 0 ) {
        gyroSampleRate = 0.0;
    }

    /* Write the value to the register */
    gyroDiv = (uint8_t) gyroSampleRate;
    
    HAL_INFO1("Set GYRO_SMPLRT_DIV to 0x%x\n",gyroDiv)
  
    if (!m_settings->HALWrite(m_slaveAddr, ICM20948_REG_GYRO_SMPLRT_DIV, gyroDiv, "Failed to set gyro sample rate"))
        return false;


    //Accel
    uint16_t accelDiv;
    float accelSampleRate;

    /* Calculate the sample rate divider */
    accelSampleRate = (1125.0 / m_sampleRate) - 1.0;

    /* Check if it fits in the divider registers */
    if ( accelSampleRate > 4095.0 ) {
        accelSampleRate = 4095.0;
    }

    if ( accelSampleRate < 0 ) {
        accelSampleRate = 0.0;
    }

    /* Write the value to the registers */
    accelDiv = (uint16_t) accelSampleRate;
    HAL_INFO1("Set ACCEL_SMPLRT_DIV to 0x%x\n",accelDiv)
   if (!m_settings->HALWrite(m_slaveAddr, ICM20948_REG_ACCEL_SMPLRT_DIV_1, (uint8_t) (accelDiv >> 8), "Failed to set MSB accel sample rate"))
        return false;
   if (!m_settings->HALWrite(m_slaveAddr, ICM20948_REG_ACCEL_SMPLRT_DIV_2, (uint8_t) (accelDiv & 0xFF), "Failed to set LSB accel sample rate"))
        return false;

    return true;
}

int RTIMUICM20948::IMUGetPollInterval()
{
    if (m_sampleRate > 400)
        return 1;
    else
        return (400 / m_sampleRate);
}

bool RTIMUICM20948::IMURead()
{
    unsigned char fifoCount[2];
    unsigned int count;
    unsigned char fifoData[12];
    unsigned char compassData[8];

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_REG_FIFO_COUNT_H, 2, fifoCount, "Failed to read fifo count"))
         return false;


    count = ((unsigned int)fifoCount[0] << 8) + fifoCount[1];

//    HAL_INFO1("Nb FIFO bytes: %d\n",count)

    if (count == 512) {
        HAL_INFO("ICM-20948 fifo has overflowed");
        resetFifo();
        m_imuData.timestamp += m_sampleInterval * (512 / ICM20948_FIFO_CHUNK_SIZE + 1); // try to fix timestamp
        return false;
    }

#ifdef ICM20948_CACHE_MODE
    if ((m_cacheCount == 0) && (count  >= ICM20948_FIFO_CHUNK_SIZE) && (count < (ICM20948_CACHE_SIZE * ICM20948_FIFO_CHUNK_SIZE))) {
        // special case of a small fifo and nothing cached - just handle as simple read

        if (!m_settings->HALRead(m_slaveAddr, ICM20948_REG_FIFO_R_W, ICM20948_FIFO_CHUNK_SIZE, fifoData, "Failed to read fifo data"))
            return false;

        // if (!m_settings->HALRead(m_slaveAddr, ICM20948_EXT_SENS_DATA_00, 8, compassData, "Failed to read compass data"))
        //     return false;
    } else {
        if (count >= (ICM20948_CACHE_SIZE * ICM20948_FIFO_CHUNK_SIZE)) {
            if (m_cacheCount == ICM20948_CACHE_BLOCK_COUNT) {
                // all cache blocks are full - discard oldest and update timestamp to account for lost samples
                m_imuData.timestamp += m_sampleInterval * m_cache[m_cacheOut].count;
                if (++m_cacheOut == ICM20948_CACHE_BLOCK_COUNT)
                    m_cacheOut = 0;
                m_cacheCount--;
            }

            int blockCount = count / ICM20948_FIFO_CHUNK_SIZE;   // number of chunks in fifo

            if (blockCount > ICM20948_CACHE_SIZE)
                blockCount = ICM20948_CACHE_SIZE;

            if (!m_settings->HALRead(m_slaveAddr, ICM20948_REG_FIFO_R_W, ICM20948_FIFO_CHUNK_SIZE * blockCount,
                    m_cache[m_cacheIn].data, "Failed to read fifo data"))
                return false;

            // if (!m_settings->HALRead(m_slaveAddr, ICM20948_EXT_SENS_DATA_00, 8, m_cache[m_cacheIn].compass, "Failed to read compass data"))
            //     return false;

            m_cache[m_cacheIn].count = blockCount;
            m_cache[m_cacheIn].index = 0;

            m_cacheCount++;
            if (++m_cacheIn == ICM20948_CACHE_BLOCK_COUNT)
                m_cacheIn = 0;

        }

        //  now fifo has been read if necessary, get something to process

        if (m_cacheCount == 0)
            return false;

        memcpy(fifoData, m_cache[m_cacheOut].data + m_cache[m_cacheOut].index, ICM20948_FIFO_CHUNK_SIZE);
        memcpy(compassData, m_cache[m_cacheOut].compass, 8);

        m_cache[m_cacheOut].index += ICM20948_FIFO_CHUNK_SIZE;

        if (--m_cache[m_cacheOut].count == 0) {
            //  this cache block is now empty

            if (++m_cacheOut == ICM20948_CACHE_BLOCK_COUNT)
                m_cacheOut = 0;
            m_cacheCount--;
        }
    }

#else

    if (count > ICM20948_FIFO_CHUNK_SIZE * 40) {
        // more than 40 samples behind - going too slowly so discard some samples but maintain timestamp correctly
        while (count >= ICM20948_FIFO_CHUNK_SIZE * 10) {
            if (!m_settings->HALRead(m_slaveAddr, ICM20948_FIFO_R_W, ICM20948_FIFO_CHUNK_SIZE, fifoData, "Failed to read fifo data"))
                return false;
            count -= ICM20948_FIFO_CHUNK_SIZE;
            m_imuData.timestamp += m_sampleInterval;
        }
    }

    if (count < ICM20948_FIFO_CHUNK_SIZE)
        return false;

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_FIFO_R_W, ICM20948_FIFO_CHUNK_SIZE, fifoData, "Failed to read fifo data"))
        return false;

    if (!m_settings->HALRead(m_slaveAddr, ICM20948_EXT_SENS_DATA_00, 8, compassData, "Failed to read compass data"))
        return false;

#endif

    RTMath::convertToVector(fifoData, m_imuData.accel, m_accelScale, true);
    RTMath::convertToVector(fifoData + 6, m_imuData.gyro, m_gyroScale, true);
    RTMath::convertToVector(compassData + 1, m_imuData.compass, 0.6f, false);

    //  sort out gyro axes

    m_imuData.gyro.setX(m_imuData.gyro.x());
    m_imuData.gyro.setY(-m_imuData.gyro.y());
    m_imuData.gyro.setZ(-m_imuData.gyro.z());

    //  sort out accel data;

    m_imuData.accel.setX(-m_imuData.accel.x());

    //  use the compass fuse data adjustments

    m_imuData.compass.setX(m_imuData.compass.x() * m_compassAdjust[0]);
    m_imuData.compass.setY(m_imuData.compass.y() * m_compassAdjust[1]);
    m_imuData.compass.setZ(m_imuData.compass.z() * m_compassAdjust[2]);

    //  sort out compass axes

    float temp;

    temp = m_imuData.compass.x();
    m_imuData.compass.setX(m_imuData.compass.y());
    m_imuData.compass.setY(-temp);

    //  now do standard processing

    handleGyroBias();
    calibrateAverageCompass();
    calibrateAccel();

    if (m_firstTime)
        m_imuData.timestamp = RTMath::currentUSecsSinceEpoch();
    else
        m_imuData.timestamp += m_sampleInterval;

    m_firstTime = false;

    //  now update the filter

    updateFusion();

    return true;
}

