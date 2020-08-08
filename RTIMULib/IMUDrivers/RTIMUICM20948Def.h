/* AK09916 register definition - ICM20948 ondie magnetometer */
#define AK0991x_DEFAULT_I2C_ADDR	0x0C	/* The default I2C address for AK0991x Magnetometers */
#define AK0991x_SECONDARY_I2C_ADDR  0x0E	/* The secondary I2C address for AK0991x Magnetometers */

#define AK09916_ADDRESS     AK0991x_DEFAULT_I2C_ADDR

#define AK09916_WIA1_ID        0x48
#define AK09916_WIA2_ID        0x09

#define REG_AK09916_WIA1        0x0 
#define REG_AK09916_WIA2        0x1
#define REG_AK09916_DMP_READ    0x3
#define REG_AK09916_STATUS1     0x10
#define REG_AK09916_STATUS2     0x18
#define REG_AK09916_CNTL2       0x31
#define REG_AK09916_CNTL3       0x32
#define REG_AK09916_MEASURE_DATA     0x11
#define REG_AK09916_TEST        0x33



/* ICM20948 */

#define ICM20948_MIN_MST_ODR_CONFIG       4

#define  ICM20948_FIFO_DIVIDER 19

#define  ICM20948_REG_BANK_0 0x00
#define  ICM20948_REG_BANK_1 0x01

#define  ICM20948_DIAMOND_I2C_ADDRESS     0x68
#define  ICM20948_BANK_0                  (0 << 7)
#define  ICM20948_BANK_1                  (1 << 7)
#define  ICM20948_BANK_2                  (2 << 7)
#define  ICM20948_BANK_3                  (3 << 7)

/*register and associated bit definition*/
/* bank 0 register map */
#define  ICM20948_REG_WHO_AM_I            (ICM20948_BANK_0 | 0x00)
#define  ICM20948_DEVICE_ID                       0xea
#define  ICM20948_REG_LPF                 (ICM20948_BANK_0 | 0x01)

#define  ICM20948_REG_USER_CTRL           (ICM20948_BANK_0 | 0x03)
#define  ICM20948_BIT_DMP_EN                      0x80
#define  ICM20948_BIT_FIFO_EN                     0x40
#define  ICM20948_BIT_I2C_MST_EN                  0x20
#define  ICM20948_BIT_I2C_IF_DIS                  0x10
#define  ICM20948_BIT_DMP_RST                     0x08
#define  ICM20948_BIT_DIAMOND_DMP_RST			    0x04

#define  ICM20948_REG_LP_CONFIG           (ICM20948_BANK_0 | 0x05)
#define  ICM20948_BIT_I2C_MST_CYCLE               0x40
#define  ICM20948_BIT_ACCEL_CYCLE                 0x20
#define  ICM20948_BIT_GYRO_CYCLE                  0x10

#define  ICM20948_REG_PWR_MGMT_1          (ICM20948_BANK_0 | 0x06)
#define  ICM20948_BIT_H_RESET                     0x80
#define  ICM20948_BIT_SLEEP                       0x40
#define  ICM20948_BIT_LP_EN                       0x20
#define  ICM20948_BIT_CLK_PLL                     0x01

#define  ICM20948_REG_PWR_MGMT_2          (ICM20948_BANK_0 | 0x07)
#define  ICM20948_BIT_PWR_PRESSURE_STBY           0x40
#define  ICM20948_BIT_PWR_ACCEL_STBY              0x38
#define  ICM20948_BIT_PWR_GYRO_STBY               0x07
#define  ICM20948_BIT_PWR_ALL_OFF                 0x7f

#define  ICM20948_REG_INT_PIN_CFG         (ICM20948_BANK_0 | 0x0F)
#define  ICM20948_BIT_INT_LATCH_EN                0x20
#define  ICM20948_BIT_BYPASS_EN                   0x02

#define  ICM20948_REG_INT_ENABLE          (ICM20948_BANK_0 | 0x10)
#define  ICM20948_BIT_DMP_INT_EN                  0x02

#define  ICM20948_REG_INT_ENABLE_1        (ICM20948_BANK_0 | 0x11)
#define  ICM20948_BIT_DATA_RDY_3_EN               0x08
#define  ICM20948_BIT_DATA_RDY_2_EN               0x04
#define  ICM20948_BIT_DATA_RDY_1_EN               0x02
#define  ICM20948_BIT_DATA_RDY_0_EN               0x01

#define  ICM20948_REG_INT_ENABLE_2        (ICM20948_BANK_0 | 0x12)
#define  ICM20948_BIT_FIFO_OVERFLOW_EN_0          0x1

#define  ICM20948_REG_INT_ENABLE_3        (ICM20948_BANK_0 | 0x13)

#define  ICM20948_REG_DMP_INT_STATUS      (ICM20948_BANK_0 | 0x18)
#define  ICM20948_BIT_WAKE_ON_MOTION_INT          0x08
#define  ICM20948_BIT_MSG_DMP_INT                 0x0002
#define  ICM20948_BIT_MSG_DMP_INT_0               0x0100  // CI Command

#define  ICM20948_BIT_MSG_DMP_INT_2               0x0200  // CIM Command - SMD
#define  ICM20948_BIT_MSG_DMP_INT_3               0x0400  // CIM Command - Pedometer

#define  ICM20948_BIT_MSG_DMP_INT_4               0x1000  // CIM Command - Pedometer binning
#define  ICM20948_BIT_MSG_DMP_INT_5               0x2000  // CIM Command - Bring To See Gesture
#define  ICM20948_BIT_MSG_DMP_INT_6               0x4000  // CIM Command - Look To See Gesture

#define  ICM20948_REG_INT_STATUS          (ICM20948_BANK_0 | 0x19)
#define  ICM20948_BIT_DMP_INT                     0x02 

#define  ICM20948_REG_INT_STATUS_1        (ICM20948_BANK_0 | 0x1A)
#define  ICM20948_REG_INT_STATUS_2        (ICM20948_BANK_0 | 0x1B)

#define  ICM20948_REG_SINGLE_FIFO_PRIORITY_SEL        (ICM20948_BANK_0 | 0x26)	

#define  ICM20948_REG_GYRO_XOUT_H_SH      (ICM20948_BANK_0 | 0x33)

#define  ICM20948_REG_TEMPERATURE         (ICM20948_BANK_0 | 0x39)
#define  ICM20948_REG_TEMP_CONFIG         (ICM20948_BANK_0 | 0x53)

#define  ICM20948_REG_EXT_SLV_SENS_DATA_00 (ICM20948_BANK_0 | 0x3B)
#define  ICM20948_REG_EXT_SLV_SENS_DATA_08 (ICM20948_BANK_0 | 0x43)
#define  ICM20948_REG_EXT_SLV_SENS_DATA_09 (ICM20948_BANK_0 | 0x44)
#define  ICM20948_REG_EXT_SLV_SENS_DATA_10 (ICM20948_BANK_0 | 0x45)

#define  ICM20948_REG_FIFO_EN_1             (ICM20948_BANK_0 | 0x66)
#define  ICM20948_BIT_SLV_0_FIFO_EN               0x01

#define  ICM20948_REG_FIFO_EN_2           (ICM20948_BANK_0 | 0x67)
#define  ICM20948_BIT_PRS_FIFO_EN                 0x20
#define  ICM20948_BIT_ACCEL_FIFO_EN               0x10
#define  ICM20948_BITS_GYRO_FIFO_EN               0x0E

#define  ICM20948_REG_FIFO_RST            (ICM20948_BANK_0 | 0x68)
#define  ICM20948_REG_FIFO_MODE           (ICM20948_BANK_0 | 0x69)

#define  ICM20948_REG_FIFO_COUNT_H        (ICM20948_BANK_0 | 0x70)
#define  ICM20948_REG_FIFO_COUNT_L        (ICM20948_BANK_0 | 0x71)
#define  ICM20948_REG_FIFO_R_W            (ICM20948_BANK_0 | 0x72)

#define  ICM20948_REG_HW_FIX_DISABLE      (ICM20948_BANK_0 | 0x75)

#define  ICM20948_REG_FIFO_CFG            (ICM20948_BANK_0 | 0x76)
#define  ICM20948_BIT_MULTI_FIFO_CFG              0x01
#define  ICM20948_BIT_SINGLE_FIFO_CFG             0x00

#define  ICM20948_REG_ACCEL_XOUT_H_SH     (ICM20948_BANK_0 | 0x2D)
#define  ICM20948_REG_ACCEL_XOUT_L_SH     (ICM20948_BANK_0 | 0x2E)
#define  ICM20948_REG_ACCEL_YOUT_H_SH     (ICM20948_BANK_0 | 0x2F)
#define  ICM20948_REG_ACCEL_YOUT_L_SH     (ICM20948_BANK_0 | 0x30)
#define  ICM20948_REG_ACCEL_ZOUT_H_SH     (ICM20948_BANK_0 | 0x31)
#define  ICM20948_REG_ACCEL_ZOUT_L_SH     (ICM20948_BANK_0 | 0x32)

#define  ICM20948_REG_MEM_START_ADDR      (ICM20948_BANK_0 | 0x7C)
#define  ICM20948_REG_MEM_R_W             (ICM20948_BANK_0 | 0x7D)
#define  ICM20948_REG_MEM_BANK_SEL        (ICM20948_BANK_0 | 0x7E)

/* bank 1 register map */
#define  ICM20948_REG_TIMEBASE_CORRECTION_PLL   (ICM20948_BANK_1 | 0x28)
#define  ICM20948_REG_TIMEBASE_CORRECTION_RCOSC (ICM20948_BANK_1 | 0x29)
#define  ICM20948_REG_SELF_TEST1                (ICM20948_BANK_1 | 0x02)
#define  ICM20948_REG_SELF_TEST2                (ICM20948_BANK_1 | 0x03)
#define  ICM20948_REG_SELF_TEST3                (ICM20948_BANK_1 | 0x04)
#define  ICM20948_REG_SELF_TEST4                (ICM20948_BANK_1 | 0x0E)
#define  ICM20948_REG_SELF_TEST5                (ICM20948_BANK_1 | 0x0F)
#define  ICM20948_REG_SELF_TEST6                (ICM20948_BANK_1 | 0x10)

#define  ICM20948_REG_XA_OFFS_H                 (ICM20948_BANK_1 | 0x14)
#define  ICM20948_REG_XA_OFFS_L                 (ICM20948_BANK_1 | 0x15)
#define  ICM20948_REG_YA_OFFS_H                 (ICM20948_BANK_1 | 0x17)
#define  ICM20948_REG_YA_OFFS_L                 (ICM20948_BANK_1 | 0x18)
#define  ICM20948_REG_ZA_OFFS_H                 (ICM20948_BANK_1 | 0x1A)
#define  ICM20948_REG_ZA_OFFS_L                 (ICM20948_BANK_1 | 0x1B)

/* bank 2 register map */
#define  ICM20948_REG_GYRO_SMPLRT_DIV     (ICM20948_BANK_2 | 0x00)

#define  ICM20948_REG_GYRO_CONFIG_1       (ICM20948_BANK_2 | 0x01)
#define  ICM20948_SHIFT_GYRO_FS_SEL               1
#define  ICM20948_SHIFT_GYRO_DLPCFG               3

#define  ICM20948_REG_GYRO_CONFIG_2       (ICM20948_BANK_2 | 0x02)
#define  ICM20948_BIT_GYRO_CTEN                   0x38

#define  ICM20948_REG_XG_OFFS_USRH        (ICM20948_BANK_2 | 0x03)
#define  ICM20948_REG_XG_OFFS_USRL        (ICM20948_BANK_2 | 0x04)
#define  ICM20948_REG_YG_OFFS_USRH        (ICM20948_BANK_2 | 0x05)
#define  ICM20948_REG_YG_OFFS_USRL        (ICM20948_BANK_2 | 0x06)
#define  ICM20948_REG_ZG_OFFS_USRH        (ICM20948_BANK_2 | 0x07)
#define  ICM20948_REG_ZG_OFFS_USRL        (ICM20948_BANK_2 | 0x08)

#define  ICM20948_REG_ACCEL_SMPLRT_DIV_1  (ICM20948_BANK_2 | 0x10)
#define  ICM20948_REG_ACCEL_SMPLRT_DIV_2  (ICM20948_BANK_2 | 0x11)

#define  ICM20948_REG_ACCEL_CONFIG        (ICM20948_BANK_2 | 0x14)
#define  ICM20948_SHIFT_ACCEL_FS                  1

#define  ICM20948_REG_ACCEL_CONFIG_2      (ICM20948_BANK_2 | 0x15)
#define  ICM20948_BIT_ACCEL_CTEN                  0x1C

#define  ICM20948_REG_PRS_ODR_CONFIG      (ICM20948_BANK_2 | 0x20)
#define  ICM20948_REG_PRGM_START_ADDRH    (ICM20948_BANK_2 | 0x50)

#define  ICM20948_REG_MOD_CTRL_USR        (ICM20948_BANK_2 | 0x54)
#define  ICM20948_BIT_ODR_SYNC                    0x7

/* bank 3 register map */
#define  ICM20948_REG_I2C_MST_ODR_CONFIG  (ICM20948_BANK_3 | 0x0)

#define  ICM20948_REG_I2C_MST_CTRL        (ICM20948_BANK_3 | 0x01)
#define  ICM20948_BIT_I2C_MST_P_NSR               0x10

#define  ICM20948_REG_I2C_MST_DELAY_CTRL  (ICM20948_BANK_3 | 0x02)
#define  ICM20948_BIT_SLV0_DLY_EN                 0x01
#define  ICM20948_BIT_SLV1_DLY_EN                 0x02
#define  ICM20948_BIT_SLV2_DLY_EN                 0x04
#define  ICM20948_BIT_SLV3_DLY_EN                 0x08

#define  ICM20948_REG_I2C_SLV0_ADDR       (ICM20948_BANK_3 | 0x03)
#define  ICM20948_REG_I2C_SLV0_REG        (ICM20948_BANK_3 | 0x04)
#define  ICM20948_REG_I2C_SLV0_CTRL       (ICM20948_BANK_3 | 0x05)
#define  ICM20948_REG_I2C_SLV0_DO         (ICM20948_BANK_3 | 0x06)

#define  ICM20948_REG_I2C_SLV1_ADDR       (ICM20948_BANK_3 | 0x07)
#define  ICM20948_REG_I2C_SLV1_REG        (ICM20948_BANK_3 | 0x08)
#define  ICM20948_REG_I2C_SLV1_CTRL       (ICM20948_BANK_3 | 0x09)
#define  ICM20948_REG_I2C_SLV1_DO         (ICM20948_BANK_3 | 0x0A)

#define  ICM20948_REG_I2C_SLV2_ADDR       (ICM20948_BANK_3 | 0x0B)
#define  ICM20948_REG_I2C_SLV2_REG        (ICM20948_BANK_3 | 0x0C)
#define  ICM20948_REG_I2C_SLV2_CTRL       (ICM20948_BANK_3 | 0x0D)
#define  ICM20948_REG_I2C_SLV2_DO         (ICM20948_BANK_3 | 0x0E)

#define  ICM20948_REG_I2C_SLV3_ADDR       (ICM20948_BANK_3 | 0x0F)
#define  ICM20948_REG_I2C_SLV3_REG        (ICM20948_BANK_3 | 0x10)
#define  ICM20948_REG_I2C_SLV3_CTRL       (ICM20948_BANK_3 | 0x11)
#define  ICM20948_REG_I2C_SLV3_DO         (ICM20948_BANK_3 | 0x12)

#define  ICM20948_REG_I2C_SLV4_CTRL       (ICM20948_BANK_3 | 0x15)

#define  ICM20948_INV_MPU_BIT_SLV_EN      0x80
#define  ICM20948_INV_MPU_BIT_BYTE_SW     0x40
#define  ICM20948_INV_MPU_BIT_REG_DIS     0x20
#define  ICM20948_INV_MPU_BIT_GRP         0x10
#define  ICM20948_INV_MPU_BIT_I2C_READ    0x80

/* register for all banks */
#define  ICM20948_REG_BANK_SEL            0x7F
