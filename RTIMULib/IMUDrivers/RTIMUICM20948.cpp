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

RTIMUICM20948::RTIMUICM20948(RTIMUSettings *settings) : RTIMU(settings),m_lastbank(0xff)
{

}

RTIMUICM20948::~RTIMUICM20948()
{
}

bool RTIMUICM20948::SelectBank(unsigned char slaveAddr, unsigned short addr)
{
    uint8_t bank = (uint8_t) (addr >> 7);
    // uint8_t regAddr = (uint8_t) (addr & 0x7F);

    // uint8_t curDevBank;
    // if (!m_settings->HALRead(slaveAddr, ICM20948_REG_BANK_SEL, 1,&curDevBank, "Failed to read Bank"))
    //     return false;
    // if (m_lastbank!= 0xff && (curDevBank >>4) != m_lastbank)
    // {
    //     HAL_INFO1("HOLY MOLY BANKY went to %Xh\n",curDevBank>>4);
    // }

    if (bank != m_lastbank)
    {
//        HAL_INFO4("Switching from bank %d to %d for accessing addr: 0x%X (regAddr: 0x%X)\n",m_lastbank,bank,addr,regAddr);
        if (!m_settings->HALWrite(slaveAddr, ICM20948_REG_BANK_SEL, (bank << 4), "Failed to select Bank"))
            return false;
        m_lastbank = bank;
    }
    return true;
}

bool RTIMUICM20948::HALRead(unsigned char slaveAddr, unsigned short addr, unsigned char length,
                    unsigned char *data, const char *errorMsg)
{
    if (!SelectBank(slaveAddr,addr))
        return false;
    if (!m_settings->HALRead(slaveAddr, addr & 0x7F, length, data, errorMsg))
        return false;
    return true;
}

bool RTIMUICM20948::HALWrite(unsigned char slaveAddr, unsigned short addr,
                   unsigned char const data, const char *errorMsg)
{
    if (!SelectBank(slaveAddr,addr))
        return false;
    if (!m_settings->HALWrite(slaveAddr, addr & 0x7F, data, errorMsg))
        return false;
    // m_settings->delayMs(10);
    return true;
}

bool RTIMUICM20948::HALWrite(unsigned char slaveAddr, unsigned short addr,
                   unsigned char length, unsigned char const *data, const char *errorMsg)
{
    if (!SelectBank(slaveAddr,addr))
        return false;
    if (!m_settings->HALWrite(slaveAddr, addr & 0x7F, length, data, errorMsg))
        return false;
    // m_settings->delayMs(10);
    return true;
}

bool RTIMUICM20948::foo_debug()
{
    //disable all secondary I2C slaves
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_CTRL,  0, "Failed to disable SLV0")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV1_CTRL,  0, "Failed to disable SLV1")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV2_CTRL,  0, "Failed to disable SLV2")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV3_CTRL,  0, "Failed to disable SLV3")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV4_CTRL,  0, "Failed to disable SLV4")) return false;

    //enable I2C master mode
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_USER_CTRL,  ICM20948_BIT_I2C_MST_EN, "Failed to use SLV0")) return false;
    m_settings->delayMs(20);

while (true)
{
    /* code */

    unsigned char akId = 0xff;
    //inv_icm20948_execute_read_secondary
    // inv_icm20948_read_secondary
    char len=1;
    unsigned char akAddr = AK09916_ADDRESS;
    unsigned char addr = ICM20948_INV_MPU_BIT_I2C_READ | akAddr;

    //set reading parameters for AK company ID (see datasheet AK09916)
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_ADDR,  addr, "Failed to use SLV0")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_REG,  REG_AK09916_WIA1, "Failed to use SLV0")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_CTRL,  ICM20948_INV_MPU_BIT_SLV_EN | len, "Failed to use SLV0")) return false;
    m_settings->delayMs(20);//wait for read to happen
    if (!HALRead(m_slaveAddr,  ICM20948_REG_EXT_SLV_SENS_DATA_00,  1, &akId, "Failed to disable SLV0")) return false;
    HAL_INFO1("AK company ID: %Xh\n",akId);

    akId=0;
    //set reading parameters for AK device ID (see datasheet AK09916)
//    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_ADDR,  addr, "Failed to use SLV0")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_REG,  REG_AK09916_WIA2, "Failed to use SLV0")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_CTRL,  ICM20948_INV_MPU_BIT_SLV_EN | len, "Failed to use SLV0")) return false;
     for (size_t i = 0; i < 10; i++) // read very fast until the value is present
     {
         if (!HALRead(m_slaveAddr,  ICM20948_REG_EXT_SLV_SENS_DATA_00,  1, &akId, "Failed to disable SLV0")) return false;
         HAL_INFO2("Reading incomming data, attempt: %d, value: %Xh\n",i+1,akId);
         if (akId==0 || akId==0x48)
             m_settings->delayMs(3);
         else
             break;
     }
    HAL_INFO1("AK Dev ID: %Xh\n",akId);

 //       break; //while(true)
    m_settings->delayMs(1000);
    }

    return false;
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

    if (!setGyroLpf(m_settings->m_ICM20948GyroLpf)) return false;
    if (!setAccelLpf(m_settings->m_ICM20948AccelLpf)) return false;
    if (!setGyroFsr(m_settings->m_ICM20948GyroFsr)) return false;
    if (!setAccelFsr(m_settings->m_ICM20948AccelFsr)) return false;
    if (!setSampleRate(m_settings->m_ICM20948GyroAccelSampleRate)) return false;
    if (!setCompassRate(m_settings->m_ICM20948CompassSampleRate)) return false;
    
    setCalibrationData();


    //  enable the bus

    if (!HALOpen())
        return false;

    //  reset the ICM20948

    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_CTRL,  0, "Failed to disable SLV0")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV1_CTRL,  0, "Failed to disable SLV1")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV2_CTRL,  0, "Failed to disable SLV2")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV3_CTRL,  0, "Failed to disable SLV3")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV4_CTRL,  0, "Failed to disable SLV4")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_USER_CTRL,  0, "Failed to disable fifo and possible active I2C functions")) return false;
    m_settings->delayMs(50);

    if (!HALWrite(m_slaveAddr,  ICM20948_REG_PWR_MGMT_1,  ICM20948_BIT_H_RESET, "Failed to initiate ICM20948 reset"))
        return false;

    m_settings->delayMs(100);

    if (!HALRead(m_slaveAddr, ICM20948_REG_WHO_AM_I, 1, &result, "Failed to read ICM20948 id"))
        return false;

    if (result != ICM20948_DEVICE_ID) {
        HAL_ERROR2("Incorrect %s id %d\n", IMUName(), result);
        return false;
    }

    //Set clock and pull out of sleep mode (which is default after reset)
    /* Auto selects the best available clock source  PLL if ready, else use the Internal oscillator */
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_PWR_MGMT_1,  ICM20948_BIT_CLK_PLL, "Failed to select clock"))
        return false;

    /* PLL startup time - maybe it is too long but better be on the safe side, no spec in the datasheet */
    m_settings->delayMs(30);

    uint8_t curPwrMgmt1;
    if (!HALRead(m_slaveAddr,  ICM20948_REG_PWR_MGMT_1, 1, &curPwrMgmt1, "Failed to read PWR_MGMT_1"))
        return false;


    // foo_debug();
    // return false;
    //  now configure the various components

    //  enable Gyro && Accel sensors
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_PWR_MGMT_2,  0, "Failed to set REG_PWR_MGMT_2"))
        return false;

    if (!setGyroConfig())
        return false;

    if (!setAccelConfig())
        return false;

    if(!compassSetup()) {
        return false;
    }

    //  select the data to go into the FIFO and enable
     if (!resetFifo())
         return false;

    // gyroBiasInit();

    HAL_INFO1("%s init complete\n", IMUName());
    return true;
}

bool RTIMUICM20948::setSampleRate(int rate)
{
    m_sampleRate = rate;
    m_sampleInterval = (uint64_t)1000000 / m_sampleRate;
    return true;
}

bool RTIMUICM20948::setGyroLpf(unsigned char lpf)
{
    switch (lpf) {
    case ICM20948_GYRO_LPF_12100HZ:
    case ICM20948_GYRO_LPF_360HZ  :
    case ICM20948_GYRO_LPF_200HZ  :
    case ICM20948_GYRO_LPF_150HZ  :
    case ICM20948_GYRO_LPF_120HZ  :
    case ICM20948_GYRO_LPF_51HZ   :
    case ICM20948_GYRO_LPF_24HZ   :
    case ICM20948_GYRO_LPF_12HZ   :
    case ICM20948_GYRO_LPF_6HZ    :
        m_gyroLpf = lpf;
        break;
    default:
        HAL_ERROR1("Illegal ICM20648 gyro lpf %d\n", lpf);
        return false;
    }
    return true;
}

bool RTIMUICM20948::setAccelLpf(unsigned char lpf)
{
    switch (lpf) {
    case ICM20948_ACCEL_LPF_1210HZ:
    case ICM20948_ACCEL_LPF_470HZ :
    case ICM20948_ACCEL_LPF_246HZ :
    case ICM20948_ACCEL_LPF_111HZ :
    case ICM20948_ACCEL_LPF_50HZ  :
    case ICM20948_ACCEL_LPF_24HZ  :
    case ICM20948_ACCEL_LPF_12HZ  :
    case ICM20948_ACCEL_LPF_6HZ   :
        m_accelLpf = lpf;
        break;
    default:
        HAL_ERROR1("Illegal ICM20948 accel lpf %d\n", lpf);
        return false;
    }
    return true;
}


bool RTIMUICM20948::setCompassRate(int rate)
{
    switch (rate)
    {
    case 10:
        m_compassRate = 0x02;
        break;
    case 20:
        m_compassRate = 0x04;
        break;
    case 50:
        m_compassRate = 0x06;
        break;
    case 100:
        m_compassRate = 0x8;
        break;
    
    default:
        HAL_ERROR1("Illegal ICM20948 compas sample rate %d\n", rate);
        return false;
    }
    return true;
}

bool RTIMUICM20948::setGyroFsr(unsigned char fsr)
{
    switch (fsr) {
    case ICM20948_GYRO_FULLSCALE_250DPS:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (131.0 * 180.0);
        return true;

    case ICM20948_GYRO_FULLSCALE_500DPS:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (65.5 * 180.0);
        return true;

    case ICM20948_GYRO_FULLSCALE_1000DPS:
        m_gyroFsr = fsr;
        m_gyroScale = RTMATH_PI / (32.8 * 180.0);
        return true;

    case ICM20948_GYRO_FULLSCALE_2000DPS:
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
    case ICM20948_ACCEL_FULLSCALE_2G:
        m_accelFsr = fsr;
        m_accelScale = 1.0/16384.0;
        return true;

    case ICM20948_ACCEL_FULLSCALE_4G:
        m_accelFsr = fsr;
        m_accelScale = 1.0/8192.0;
        return true;

    case ICM20948_ACCEL_FULLSCALE_8G:
        m_accelFsr = fsr;
        m_accelScale = 1.0/4096.0;
        return true;

    case ICM20948_ACCEL_FULLSCALE_16G:
        m_accelFsr = fsr;
        m_accelScale = 1.0/2048.0;
        return true;

    default:
        HAL_ERROR1("Illegal ICM20948 accel fsr %d\n", fsr);
        return false;
    }
}


bool RTIMUICM20948::resetFifo()
{
    unsigned char origUSER_CTRL;
    if (!HALRead(m_slaveAddr, ICM20948_REG_USER_CTRL, 1, &origUSER_CTRL, "Disabling the fifo")) return false;
    if (!HALWrite(m_slaveAddr, ICM20948_REG_USER_CTRL, origUSER_CTRL & ~ICM20948_BIT_FIFO_EN, "Disabling the fifo")) return false;

    if (!HALWrite(m_slaveAddr, ICM20948_REG_FIFO_EN_1, 0, "Writing fifo disable")) return false;
    if (!HALWrite(m_slaveAddr, ICM20948_REG_FIFO_EN_2, 0, "disabling fifo for Acc & Gyro")) return false;
    if (!HALWrite(m_slaveAddr, ICM20948_REG_FIFO_RST, 0x0F, "Resetting fifo")) return false;
    if (!HALWrite(m_slaveAddr, ICM20948_REG_FIFO_RST, 0x00, "Undo Resetting fifo")) return false;
    if (!HALWrite(m_slaveAddr, ICM20948_REG_FIFO_EN_2, ICM20948_BIT_ACCEL_FIFO_EN|ICM20948_BITS_GYRO_FIFO_EN, "enabling fifo for Acc & Gyro")) return false;

    if (!HALWrite(m_slaveAddr, ICM20948_REG_FIFO_MODE, 0x0F, "Set fifo as snapshot")) return false;
    if (!HALWrite(m_slaveAddr, ICM20948_REG_USER_CTRL, origUSER_CTRL | ICM20948_BIT_FIFO_EN, "Enabling the fifo")) return false;

    m_settings->delayMs(50);
    return true;
}

bool RTIMUICM20948::setGyroConfig()
{
    if (!(m_gyroLpf & ICM20948_BIT_GYRO_FCHOICE))
    {
        HAL_ERROR1("Cannot calculate gyro sampleDivider rate  with current LPF 0x%02X\n",m_gyroLpf)
        return false;
    }

    unsigned char gyroConfig = m_gyroFsr | m_gyroLpf;

    HAL_INFO1("Set GYRO_CONFIG_1 to 0x%02X\n",gyroConfig)

    if (!HALWrite(m_slaveAddr, ICM20948_REG_GYRO_CONFIG_1, gyroConfig, "Failed to write gyro config"))
         return false;

    float gyroSampleDivider = (1125.0 / m_sampleRate) - 1.0;
    if (gyroSampleDivider > 255)
    {
        HAL_ERROR1("gyro sampleDivider out of range %f\n",gyroSampleDivider)
        return false;
    }
    if (!HALWrite(m_slaveAddr, ICM20948_REG_GYRO_SMPLRT_DIV, (unsigned char)gyroSampleDivider, "Failed to write gyro SMPLRT_DIV"))
         return false;


    return true;
}

bool RTIMUICM20948::setAccelConfig()
{
    if (!(m_accelLpf & ICM20948_BIT_ACCEL_FCHOICE))
    {
        HAL_ERROR1("Cannot calculate acc sampleDivider rate with current LPF 0x%02X\n",m_accelLpf)
        return false;
    }

    unsigned char accelConfig = m_accelFsr | m_accelLpf;
    HAL_INFO3("Set ACCEL_CONFIG to 0x%x (FS: 0x%x, LPF: 0x%02X)\n",accelConfig,m_accelFsr,m_accelLpf);

    if (!HALWrite(m_slaveAddr, ICM20948_REG_ACCEL_CONFIG, accelConfig, "Failed to write accel config"))
         return false;

    float accSampleDivider = (1125.0 / m_sampleRate) - 1.0;
    if (accSampleDivider > 4095)
    {
        HAL_ERROR1("acc sampleDivider out of range 0x%f\n",accSampleDivider)
        return false;
    }
    uint16_t divider = accSampleDivider;

    HAL_INFO2("Set ACCEL_SMPL to 0x%04x Scale: %f )\n",divider,m_accelScale);
    if (!HALWrite(m_slaveAddr, ICM20948_REG_ACCEL_SMPLRT_DIV_1, (unsigned char)(divider >> 8), "Failed to write acc SMPLRT_DIV"))
         return false;
    if (!HALWrite(m_slaveAddr, ICM20948_REG_ACCEL_SMPLRT_DIV_2, (unsigned char)(divider & 0xFF), "Failed to write acc SMPLRT_DIV"))
         return false;
    return true;
}

int RTIMUICM20948::IMUGetPollInterval()
{
    return (400 / m_sampleRate);
}

bool RTIMUICM20948::IMURead()
{
    unsigned char fifoCount[2];
    unsigned int count;
    unsigned char fifoData[12];
    unsigned char compassData[9];

    if (!HALRead(m_slaveAddr, ICM20948_REG_FIFO_COUNT_H, 2, fifoCount, "Failed to read fifo count"))
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

        if (!HALRead(m_slaveAddr, ICM20948_REG_FIFO_R_W, ICM20948_FIFO_CHUNK_SIZE, fifoData, "Failed to read fifo data"))
            return false;

        if (!HALRead(m_slaveAddr, ICM20948_REG_EXT_SLV_SENS_DATA_00, 9, compassData, "Failed to read compass data"))
             return false;
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

            if (!HALRead(m_slaveAddr, ICM20948_REG_FIFO_R_W, ICM20948_FIFO_CHUNK_SIZE * blockCount,
                    m_cache[m_cacheIn].data, "Failed to read fifo data"))
                return false;

            if (!HALRead(m_slaveAddr, ICM20948_REG_EXT_SLV_SENS_DATA_00, 9, m_cache[m_cacheIn].compass, "Failed to read compass data"))
                 return false;

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
        memcpy(compassData, m_cache[m_cacheOut].compass, 9);

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
            if (!HALRead(m_slaveAddr, ICM20948_REG_FIFO_R_W, ICM20948_FIFO_CHUNK_SIZE, fifoData, "Failed to read fifo data"))
                return false;
            count -= ICM20948_FIFO_CHUNK_SIZE;
            m_imuData.timestamp += m_sampleInterval;
        }
    }

    if (count < ICM20948_FIFO_CHUNK_SIZE)
        return false;

    if (!HALRead(m_slaveAddr, ICM20948_REG_FIFO_R_W, ICM20948_FIFO_CHUNK_SIZE, fifoData, "Failed to read fifo data"))
        return false;

    if (!HALRead(m_slaveAddr, ICM20948_REG_EXT_SLV_SENS_DATA_00, 9, compassData, "Failed to read compass data"))
        return false;

#endif

    RTMath::convertToVector(fifoData, m_imuData.accel, m_accelScale, true);
    RTMath::convertToVector(fifoData + 6, m_imuData.gyro, m_gyroScale, true);
    RTMath::convertToVector(compassData + 1, m_imuData.compass, 0.15f, false);

    //printf("Accel: %5.2f %5.2f %5.2f\n",m_imuData.accel.x(),m_imuData.accel.y(),m_imuData.accel.z());

    //  sort out gyro axes
    m_imuData.gyro.setX(m_imuData.gyro.x());
    m_imuData.gyro.setY(-m_imuData.gyro.y());
    m_imuData.gyro.setZ(-m_imuData.gyro.z());

    //  sort out accel data;
    m_imuData.accel.setX(-m_imuData.accel.x());

    //  sort out compass axes
    //  done, seems to be correct

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

bool RTIMUICM20948::compassSetup() {
    if (m_settings->m_busIsI2C) {
        // I2C mode

        bypassOn();

        // get fuse ROM data

        if (!m_settings->HALWrite(AK09916_ADDRESS, REG_AK09916_CNTL2, 0, "Failed to set compass in power down")) {
            bypassOff();
            return false;
        }

        if (!m_settings->HALWrite(AK09916_ADDRESS, REG_AK09916_CNTL2, m_compassRate, "Failed to set compass in mode 2")) {
            bypassOff();
            return false;
        }

        bypassOff();

    } else {
    //  SPI mode

        //disable all secondary I2C slaves
        if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_CTRL,  0, "Failed to disable SLV0")) return false;
        if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV1_CTRL,  0, "Failed to disable SLV1")) return false;
        if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV2_CTRL,  0, "Failed to disable SLV2")) return false;
        if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV3_CTRL,  0, "Failed to disable SLV3")) return false;
        if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV4_CTRL,  0, "Failed to disable SLV4")) return false;

        {//enable I2C master mode
            unsigned char curUSER_CTRL;
            if (!HALRead(m_slaveAddr, ICM20948_REG_USER_CTRL, 1, &curUSER_CTRL, "Failed enable I2C master mode")) return false;
            if (!HALWrite(m_slaveAddr,  ICM20948_REG_USER_CTRL,  curUSER_CTRL | ICM20948_BIT_I2C_MST_EN, "Failed enable I2C master mode")) return false;
            m_settings->delayMs(20);
        }
        {//verify compass device ID
            const uint8_t len=1;
            if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_ADDR, ICM20948_INV_MPU_BIT_I2C_READ | AK09916_ADDRESS, "Failed to set slave 0 address")) return false;
            if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_REG, REG_AK09916_WIA2, "Failed to set slave 0 reg")) return false;
            if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_CTRL, ICM20948_INV_MPU_BIT_SLV_EN | len, "Failed to set slave 0 ctrl")) return false;
            m_settings->delayMs(50);
            unsigned char wia;
            if (!HALRead(m_slaveAddr, ICM20948_REG_EXT_SLV_SENS_DATA_00, len, &wia, "Failed to read WIA")) return false;

            if (wia != AK09916_WIA2_ID)
            {
                HAL_ERROR2("Invalid magnetometer ID, expected %Xh but was %Xh\n",AK09916_WIA2_ID,wia);
                return false;
            }
        }
    }

    {//Configure Magnetometer output
        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV1_ADDR, AK09916_ADDRESS, "Failed to set slave 0 address")) return false;
        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV1_REG, REG_AK09916_CNTL2, "Failed to set slave 0 reg")) return false;
        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV1_CTRL, ICM20948_INV_MPU_BIT_SLV_EN | 1, "Failed to set slave 0 ctrl")) return false;
        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV1_DO, 0x00, "Failed to disable compass")) return false;
        m_settings->delayMs(20);
        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV1_DO, 0x02, "Failed to enable compass mode2")) return false;
        m_settings->delayMs(20);

        //disable SLV1 again.
        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV1_CTRL, 0, "Failed to set slave 0 ctrl")) return false;
        m_settings->delayMs(20);
    }

    {//verify compass mode change
        const uint8_t len=1;
        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_ADDR, ICM20948_INV_MPU_BIT_I2C_READ | AK09916_ADDRESS, "Failed to set slave 0 address")) return false;
        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_REG, REG_AK09916_CNTL2, "Failed to set slave 0 reg")) return false;
        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_CTRL, ICM20948_INV_MPU_BIT_SLV_EN | len, "Failed to set slave 0 ctrl")) return false;
        m_settings->delayMs(20);
        unsigned char compasMode;
        if (!HALRead(m_slaveAddr, ICM20948_REG_EXT_SLV_SENS_DATA_00, len, &compasMode, "Failed to read compasMode")) return false;

        if (compasMode != 0x02)
        {
            HAL_ERROR1("Magnetometer mode not properly set: %Xh\n",compasMode);
            return false;
        }
        else
        {
            HAL_INFO("Magnetometer active\n");
        }
    }

    {//Configure for reading
        uint8_t len=9; // ST1 (1byte) + magData (6 bytes) + tmps (1byte) + ST2 (1byte)
        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_ADDR, ICM20948_INV_MPU_BIT_I2C_READ | AK09916_ADDRESS, "Failed to set slave 0 address")) return false;
        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_REG, REG_AK09916_STATUS1, "Failed to set slave 0 reg")) return false;
        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_CTRL, ICM20948_INV_MPU_BIT_SLV_EN | len, "Failed to set slave 0 ctrl")) return false;

        m_settings->delayMs(30);

        //verify single sample
        unsigned char compMeas[9];
        if (!HALRead(m_slaveAddr, ICM20948_REG_EXT_SLV_SENS_DATA_00, 9, compMeas, "Failed to set slave 0 ctrl")) return false;
    }

    return true;
}


bool RTIMUICM20948::bypassOn()
{
    unsigned char userControl;

    if (!HALRead(m_slaveAddr, ICM20948_REG_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl &= ~0x20;

    if (!HALWrite(m_slaveAddr, ICM20948_REG_USER_CTRL, 1, &userControl, "Failed to write user_ctrl reg"))
        return false;

    m_settings->delayMs(50);

    if (!HALWrite(m_slaveAddr, ICM20948_REG_INT_PIN_CFG, 0x82, "Failed to write int_pin_cfg reg"))
        return false;

    m_settings->delayMs(50);
    return true;
}


bool RTIMUICM20948::bypassOff()
{
    unsigned char userControl;

    if (!HALRead(m_slaveAddr, ICM20948_REG_USER_CTRL, 1, &userControl, "Failed to read user_ctrl reg"))
        return false;

    userControl |= 0x20;

    if (!HALWrite(m_slaveAddr,ICM20948_REG_USER_CTRL, 1, &userControl, "Failed to write user_ctrl reg"))
        return false;

    m_settings->delayMs(50);

    if (!HALWrite(m_slaveAddr, ICM20948_REG_INT_PIN_CFG, 0x80, "Failed to write int_pin_cfg reg"))
         return false;

    m_settings->delayMs(50);
    return true;
}


