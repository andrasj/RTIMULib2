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
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_USER_CTRL,  ICM20948_BIT_I2C_MST_EN, "Failed to use SLV0")) return false;
    m_settings->delayMs(5);


    //bypassOff();
    // if (!HALWrite(m_slaveAddr,  ICM20948_REG_INT_PIN_CFG, 0x30, "Failed to disable SLV0")) return false;
    // if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_MST_CTRL, 0x4D, "Failed to disable SLV0")) return false;
    // if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_MST_DELAY_CTRL, 0x01, "Failed to disable SLV0")) return false;

        // self.write(ICM20948_I2C_MST_CTRL, 0x4D)
        // self.write(ICM20948_I2C_MST_DELAY_CTRL, 0x01)


    //disable all secondary I2C slaves
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_CTRL,  0, "Failed to disable SLV0")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV1_CTRL,  0, "Failed to disable SLV1")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV2_CTRL,  0, "Failed to disable SLV2")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV3_CTRL,  0, "Failed to disable SLV3")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV4_CTRL,  0, "Failed to disable SLV4")) return false;
    m_settings->delayMs(5);
	// Set up the secondary I2C bus on 20630. (inv_icm20948_set_secondary)
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_MST_CTRL,  ICM20948_BIT_I2C_MST_P_NSR, "Failed to setup I2C")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_MST_ODR_CONFIG,  ICM20948_MIN_MST_ODR_CONFIG, "Failed to setup I2C")) return false;
    // if (!HALWrite(m_slaveAddr,  ICM20948_REG_USER_CTRL,  ICM20948_BIT_I2C_MST_EN, "Failed to use SLV0")) return false;

    m_settings->delayMs(50);

    unsigned char curUsrCtrl;
    if (!HALRead(m_slaveAddr,  ICM20948_REG_USER_CTRL, 1, &curUsrCtrl ,"Failed to use SLV0")) return false;
while (true)
{
    /* code */

    unsigned char akId = 0xff;
    //inv_icm20948_execute_read_secondary
    // inv_icm20948_read_secondary
    char len=1;
    unsigned char akAddr = AK09916_ADDRESS;
    unsigned char addr = ICM20948_INV_MPU_BIT_I2C_READ | akAddr;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_ADDR,  addr, "Failed to use SLV0")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_REG,  REG_AK09916_WIA1, "Failed to use SLV0")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_CTRL,  ICM20948_INV_MPU_BIT_SLV_EN | len, "Failed to use SLV0")) return false;
    // // inv_icm20948_secondary_enable_i2c
    // HAL_INFO1("USER_CTRL: %Xh\n",curUsrCtrl);
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_USER_CTRL,  curUsrCtrl | ICM20948_BIT_I2C_MST_EN, "Failed to use SLV0")) return false;
    m_settings->delayMs(20);
    // if (!HALWrite(m_slaveAddr,  ICM20948_REG_USER_CTRL,  curUsrCtrl & (~ICM20948_BIT_I2C_MST_EN), "Failed to use SLV0")) return false;
    if (!HALRead(m_slaveAddr,  ICM20948_REG_EXT_SLV_SENS_DATA_00,  1, &akId, "Failed to disable SLV0")) return false;
    HAL_INFO1("AK company ID: %Xh\n",akId);

    akId=0;
//    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_ADDR,  addr, "Failed to use SLV0")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_REG,  REG_AK09916_WIA2, "Failed to use SLV0")) return false;
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_I2C_SLV0_CTRL,  ICM20948_INV_MPU_BIT_SLV_EN | len, "Failed to use SLV0")) return false;
    // // inv_icm20948_secondary_enable_i2c
    // if (!HALWrite(m_slaveAddr,  ICM20948_REG_USER_CTRL,  curUsrCtrl | ICM20948_BIT_I2C_MST_EN, "Failed to use SLV0")) return false;
    //m_settings->delayMs(10);
    // if (!HALWrite(m_slaveAddr,  ICM20948_REG_USER_CTRL,  curUsrCtrl & (~ICM20948_BIT_I2C_MST_EN), "Failed to use SLV0")) return false;
     for (size_t i = 0; i < 10; i++)
     {
         if (!HALRead(m_slaveAddr,  ICM20948_REG_EXT_SLV_SENS_DATA_00,  1, &akId, "Failed to disable SLV0")) return false;
         HAL_INFO2("Reading incomming data, attempt: %d, value: %Xh\n",i+1,akId);
         if (akId==0 || akId==0x48)
             m_settings->delayMs(10);
         else
             break;
     }
    if (!HALRead(m_slaveAddr,  ICM20948_REG_EXT_SLV_SENS_DATA_00,  1, &akId, "Failed to disable SLV0")) return false;
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

    setSampleRate(m_settings->m_ICM20948GyroAccelSampleRate);
    setCompassRate(m_settings->m_ICM20948CompassSampleRate);
    setGyroLpf(m_settings->m_ICM20948GyroLpf);
    setAccelLpf(m_settings->m_ICM20948AccelLpf);
    setGyroFsr(m_settings->m_ICM20948GyroFsr);
    setAccelFsr(m_settings->m_ICM20948AccelFsr);

    setCalibrationData();


    //  enable the bus

    if (!HALOpen())
        return false;

    //  reset the ICM20948

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
    HAL_INFO1("PWR_MGMT_1 : %Xh\n",curPwrMgmt1);


    foo_debug();
    return false;
    //  now configure the various components

    if (!setGyroConfig())
        return false;

    if (!setAccelConfig())
        return false;

    if (!setSampleRate())
        return false;

    if(!compassSetup()) {
        return false;
    }

    if (!setCompassRate())
        return false;

    //  enable Gyro && Accel sensors
    if (!HALWrite(m_slaveAddr,  ICM20948_REG_PWR_MGMT_2,  0, "Failed to set REG_PWR_MGMT_2"))
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
    if ((rate < ICM20948_COMPASSRATE_MIN) || (rate > ICM20948_COMPASSRATE_MAX)) {
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

     if (!HALWrite(m_slaveAddr, ICM20948_REG_USER_CTRL, 0, "Disabling the fifo"))
        return false;
    if (!HALWrite(m_slaveAddr, ICM20948_REG_FIFO_EN_1, 0, "Writing fifo disable"))
        return false;
    if (!HALWrite(m_slaveAddr, ICM20948_REG_FIFO_EN_2, 0, "disabling fifo for Acc & Gyro"))
        return false;

    if (!HALWrite(m_slaveAddr, ICM20948_REG_FIFO_RST, 0x0F, "Resetting fifo"))
        return false;
    if (!HALWrite(m_slaveAddr, ICM20948_REG_FIFO_RST, 0x00, "Undo Resetting fifo"))
        return false;
    if (!HALWrite(m_slaveAddr, ICM20948_REG_FIFO_EN_2, ICM20948_BIT_ACCEL_FIFO_EN|ICM20948_BITS_GYRO_FIFO_EN, "enabling fifo for Acc & Gyro"))
        return false;

    if (!HALWrite(m_slaveAddr, ICM20948_REG_FIFO_MODE, 0x0F, "Set fifo as snapshot"))
        return false;
    if (!HALWrite(m_slaveAddr, ICM20948_REG_USER_CTRL, ICM20948_BIT_FIFO_EN, "Enabling the fifo"))
        return false;

    m_settings->delayMs(50);
    return true;
}

bool RTIMUICM20948::setGyroConfig()
{
    unsigned char gyroConfig = m_gyroFsr | m_gyroLpf;

    HAL_INFO1("Set GYRO_CONFIG_1 to 0x%x\n",gyroConfig)

    if (!HALWrite(m_slaveAddr, ICM20948_REG_GYRO_CONFIG_1, gyroConfig, "Failed to write gyro config"))
         return false;

    return true;
}

bool RTIMUICM20948::setAccelConfig()
{
    unsigned char accelConfig = m_accelFsr | m_accelLpf;
    HAL_INFO3("Set ACCEL_CONFIG to 0x%x (FS: 0x%x, LPF: 0x%x)\n",accelConfig,m_accelFsr,m_accelLpf);

    if (!HALWrite(m_slaveAddr, ICM20948_REG_ACCEL_CONFIG, accelConfig, "Failed to write accel config"))
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
  
    if (!HALWrite(m_slaveAddr, ICM20948_REG_GYRO_SMPLRT_DIV, gyroDiv, "Failed to set gyro sample rate"))
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
   if (!HALWrite(m_slaveAddr, ICM20948_REG_ACCEL_SMPLRT_DIV_1, (uint8_t) (accelDiv >> 8), "Failed to set MSB accel sample rate"))
        return false;
   if (!HALWrite(m_slaveAddr, ICM20948_REG_ACCEL_SMPLRT_DIV_2, (uint8_t) (accelDiv & 0xFF), "Failed to set LSB accel sample rate"))
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

            if (!HALRead(m_slaveAddr, ICM20948_REG_EXT_SLV_SENS_DATA_00, 8, m_cache[m_cacheIn].compass, "Failed to read compass data"))
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

bool RTIMUICM20948::compassSetup() {
    if (m_settings->m_busIsI2C) {
        // I2C mode

        bypassOn();

        // get fuse ROM data

        if (!m_settings->HALWrite(AK09916_ADDRESS, REG_AK09916_CNTL2, 0, "Failed to set compass in power down")) {
            bypassOff();
            return false;
        }

        if (!m_settings->HALWrite(AK09916_ADDRESS, REG_AK09916_CNTL2, 0x04, "Failed to set compass in mode 2")) {
            bypassOff();
            return false;
        }

        bypassOff();

    } else {
    //  SPI mode

        bypassOff();

//        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_MST_DELAY_CTRL, 0x80, "Failed to set I2C master mode"))
        // if (!HALWrite(m_slaveAddr,ICM20948_REG_LP_CONFIG, 0x40, "Failed to set I2C master mode"))
        //     return false;

        uint8_t len=1;
        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_ADDR, ICM20948_INV_MPU_BIT_I2C_READ | AK09916_ADDRESS, "Failed to set slave 0 address"))
            return false;

        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_REG, REG_AK09916_WIA2, "Failed to set slave 0 reg"))
            return false;

        if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_CTRL, ICM20948_INV_MPU_BIT_SLV_EN | len, "Failed to set slave 0 ctrl"))
            return false;
  
        m_settings->delayMs(20);
        unsigned char wia;
        if (!HALRead(m_slaveAddr, ICM20948_REG_EXT_SLV_SENS_DATA_00, len, &wia, "Failed to read WIA"))
            return false;

        if (wia != AK09916_WIA2_ID)
        {
            HAL_ERROR2("Invalid magnetometer ID, expected %Xh but was %Xh\n",AK09916_WIA2_ID,wia);
            return false;
        }

    }
    //  both interfaces

    m_compassAdjust[0] = 1.0f;
    m_compassAdjust[1] = 1.0f;
    m_compassAdjust[2] = 1.0f;

    if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_ADDR, ICM20948_INV_MPU_BIT_I2C_READ | AK09916_ADDRESS, "Failed to set slave 0 address"))
        return false;

    if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_REG, REG_AK09916_STATUS1, "Failed to set slave 0 reg"))
        return false;

    if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV0_CTRL, ICM20948_INV_MPU_BIT_SLV_EN | 9, "Failed to set slave 0 ctrl"))
        return false;
    
//    m_settings->delayMs(20);

    unsigned char compMeas[9];
    if (!HALRead(m_slaveAddr, ICM20948_REG_EXT_SLV_SENS_DATA_00, 9, compMeas, "Failed to set slave 0 ctrl"))
        return false;

    HAL_INFO2("Compass Status 1: %Xh - 2: %Xh\n",compMeas[0],compMeas[8]);
    HAL_INFO3("Compass Data X: %d Y: %Xh Z: %d\n", (compMeas[2]<<8) | compMeas[1]
                                                 , (compMeas[4]<<8) | compMeas[3]
                                                 , (compMeas[6]<<8) | compMeas[5]);


    return true;
}

bool RTIMUICM20948::setCompassRate()
{
    int rate;

    rate = m_sampleRate / m_compassRate - 1;

    if (rate > 31)
        rate = 31;
    if (!HALWrite(m_slaveAddr, ICM20948_REG_I2C_SLV4_CTRL, rate, "Failed to set slave ctrl 4"))
         return false;
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


