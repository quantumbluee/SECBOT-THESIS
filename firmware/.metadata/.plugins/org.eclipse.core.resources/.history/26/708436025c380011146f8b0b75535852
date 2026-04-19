#include "imu.h"
#include <string.h>

// ======================= ICM20948 registers =======================

// ICM20948 uses banked registers.
// Bank select register:
#define REG_BANK_SEL        0x7F
#define BANK_0              0x00
#define BANK_2              0x20

// BANK 0 regs
#define WHO_AM_I            0x00
#define PWR_MGMT_1          0x06
#define PWR_MGMT_2          0x07
#define INT_PIN_CFG         0x0F

#define ACCEL_XOUT_H        0x2D
#define GYRO_XOUT_H         0x33
#define TEMP_OUT_H          0x39

// BANK 2 regs (gyro/accel config)
#define GYRO_SMPLRT_DIV     0x00
#define GYRO_CONFIG_1       0x01
#define ACCEL_SMPLRT_DIV_1  0x10
#define ACCEL_SMPLRT_DIV_2  0x11
#define ACCEL_CONFIG        0x14

// Expected WHO_AM_I value for ICM20948:
#define ICM20948_I2C_ADDR     0x69 //AD0 LOW
#define ICM20948_REG_WHO_AM_I 0x00
#define ICM20948_WHO_AM_I_VAL 0xEA

// ======================= Driver state =======================
static I2C_HandleTypeDef *g_imu_i2c = NULL;


// scale factors for chosen ranges
static float g_accel_lsb_to_g = 1.0f / 16384.0f;  // ±2g
static float g_gyro_lsb_to_dps = 1.0f / 131.0f;   // ±250 dps

// ======================= Low-level I2C =======================

static int i2c_write(uint8_t reg, uint8_t val) {
    if (HAL_I2C_Mem_Write(g_imu_i2c, ICM20948_I2C_ADDR << 1, reg,
                         I2C_MEMADD_SIZE_8BIT, &val, 1, 100) != HAL_OK)
        return -1;
    return 0;
}

static int i2c_read(uint8_t reg, uint8_t *buf, uint16_t len) {
    if (HAL_I2C_Mem_Read(g_imu_i2c, ICM20948_I2C_ADDR<<1, reg,
                        I2C_MEMADD_SIZE_8BIT, buf, len, 200) != HAL_OK)
        return -1;
    return 0;
}

static int select_bank(uint8_t bank) {
    return i2c_write(REG_BANK_SEL, bank);
}

// ======================= Public API =======================

int IMU_WhoAmI(uint8_t *whoami) {
    if (!whoami) return -1;
    if (select_bank(BANK_0) < 0) return -2;
    if (i2c_read(WHO_AM_I, whoami, 1) < 0) return -3;
    return 0;
}

int IMU_Init(I2C_HandleTypeDef *hi2c) {
    g_imu_i2c = hi2c;
    HAL_StatusTypeDef hrc;
    uint8_t who=0;

    //READ WHO AM I
    hrc = HAL_I2C_Mem_Read(g_imu_i2c, ICM20948_I2C_ADDR <<1,ICM20948_REG_WHO_AM_I, 1, &who, 1, 100);
    printf("IMU Init: WHAMI read hrc=%d, who=0x%02X\r\n",hrc,who);

    if(hrc != HAL_OK){
    	return -1; //i2c transaction failed
    }

    if(who!=ICM20948_WHO_AM_I_VAL){
    	return -2;
    }

    // If your AD0 strap makes address 0x69, set here:
    // g_addr7 = 0x69;

    // Bank 0
    if (select_bank(BANK_0) < 0) return -2;

    // Check WHO_AM_I
    if (i2c_read(ICM20948_REG_WHO_AM_I, &who, 1) < 0) return -3;
    if (who != ICM20948_WHO_AM_I_VAL) {
        // still allow continue but signal mismatch
        // return -4; // uncomment if you want hard fail
    }

    // Reset device
    if (i2c_write(PWR_MGMT_1, 0x80) < 0) return -5;
    HAL_Delay(50);

    // Wake up, set clock auto
    if (i2c_write(PWR_MGMT_1, 0x01) < 0) return -6; // clk best available
    HAL_Delay(10);

    // Enable accel+gyro
    if (i2c_write(PWR_MGMT_2, 0x00) < 0) return -7;

    // Bank 2 for config
    if (select_bank(BANK_2) < 0) return -8;

    // Gyro sample rate divider (smplrt = 1.1kHz/(1+div))
    if (i2c_write(GYRO_SMPLRT_DIV, 0x09) < 0) return -9; // ~100 Hz

    // Gyro config:
    // GYRO_CONFIG_1:
    // bits [2:1] FS_SEL = 00 => ±250 dps
    // bits [5:3] DLPFCFG = 001 => enable low-pass
    uint8_t gyro_cfg = (0 << 1) | (1 << 3); // FS=250dps, DLPF=1
    if (i2c_write(GYRO_CONFIG_1, gyro_cfg) < 0) return -10;

    // Accel sample rate divider (2 bytes)
    // accel rate = 1.125kHz/(1+div)
    if (i2c_write(ACCEL_SMPLRT_DIV_1, 0x00) < 0) return -11;
    if (i2c_write(ACCEL_SMPLRT_DIV_2, 0x09) < 0) return -12; // ~100 Hz

    // Accel config:
    // bits [2:1] FS_SEL = 00 => ±2g
    // bits [5:3] DLPFCFG = 001 => enable low-pass
    uint8_t acc_cfg = (0 << 1) | (1 << 3); // FS=2g, DLPF=1
    if (i2c_write(ACCEL_CONFIG, acc_cfg) < 0) return -13;

    // Back to bank 0 for reads
    if (select_bank(BANK_0) < 0) return -14;

    return 0;
}

int IMU_Read(imu_data_t *out) {
    if (!out) return -1;
    HAL_StatusTypeDef st;

    st = HAL_I2C_Mem_Write(g_imu_i2c, ICM20948_I2C_ADDR<<1, REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, (uint8_t[]){BANK_0}, 1, 100);
    if(st!=HAL_OK) return -101; //unique error code lol

    if (select_bank(BANK_0) < 0) return -2;

    uint8_t raw[14]; // accel(6) + gyro(6) + temp(2)
    st = HAL_I2C_Mem_Read(g_imu_i2c, ICM20948_I2C_ADDR<<1, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, raw, sizeof(raw), 200);
    if (st!=HAL_OK) return -102; //fucking unique
    // We’ll read starting at ACCEL_XOUT_H:
    // ACCEL_X/Y/Z, GYRO_X/Y/Z, TEMP
    if (i2c_read(ACCEL_XOUT_H, raw, sizeof(raw)) < 0) return -3;

    int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t az = (int16_t)((raw[4] << 8) | raw[5]);

    int16_t gx = (int16_t)((raw[6] << 8) | raw[7]);
    int16_t gy = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t gz = (int16_t)((raw[10] << 8) | raw[11]);

    int16_t tr = (int16_t)((raw[12] << 8) | raw[13]);

    out->accel_x = ax * g_accel_lsb_to_g;
    out->accel_y = ay * g_accel_lsb_to_g;
    out->accel_z = az * g_accel_lsb_to_g;

    out->gyro_x  = gx * g_gyro_lsb_to_dps;
    out->gyro_y  = gy * g_gyro_lsb_to_dps;
    out->gyro_z  = gz * g_gyro_lsb_to_dps;

    // Temp conversion per datasheet:
    // Temp(C) = ((TEMP_OUT - RoomTemp_Offset)/Temp_Sensitivity) + 21
    // Sensitivity ~333.87 LSB/°C, offset ~0 at 21C for ICM20948
    out->temp_c = (tr / 333.87f) + 21.0f;

    return 0;
}
