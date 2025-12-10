#include "drv2605.h"
#include "debug.h" // Delay_Us, printf ?? ?? ??

/* =================================================================================
 * Low-Level I2C Functions (CH32V30x SPL)
 * ================================================================================= */

void DRV2605_WriteByte(uint8_t reg, uint8_t data) {
    uint32_t timeout = 100000;

    while(I2C_GetFlagStatus(DRV_I2C_INST, I2C_FLAG_BUSY) && (timeout--));
    
    I2C_GenerateSTART(DRV_I2C_INST, ENABLE);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(DRV_I2C_INST, DRV2605_ADDR << 1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(DRV_I2C_INST, reg);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_SendData(DRV_I2C_INST, data);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTOP(DRV_I2C_INST, ENABLE);
}

void IMU_WriteByte(uint8_t dev_addr, uint8_t reg, uint8_t data) {
    uint32_t timeout = 100000;

    // 1. ?? Busy ??
    while(I2C_GetFlagStatus(DRV_I2C_INST, I2C_FLAG_BUSY) && (timeout--));
    
    // 2. Start Condition ??
    I2C_GenerateSTART(DRV_I2C_INST, ENABLE);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_MODE_SELECT));

    // 3. [??] ?? ??? ?? dev_addr? ?? (Write Mode: << 1)
    I2C_Send7bitAddress(DRV_I2C_INST, dev_addr << 1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    // 4. ???? ?? ??
    I2C_SendData(DRV_I2C_INST, reg);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // 5. ??? ??
    I2C_SendData(DRV_I2C_INST, data);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // 6. Stop Condition ??
    I2C_GenerateSTOP(DRV_I2C_INST, ENABLE);
}

uint8_t DRV2605_ReadByte(uint8_t reg) {
    uint8_t val = 0;
    uint32_t timeout = 100000;

    while(I2C_GetFlagStatus(DRV_I2C_INST, I2C_FLAG_BUSY) && (timeout--));

    // 1. Register Address Write
    I2C_GenerateSTART(DRV_I2C_INST, ENABLE);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(DRV_I2C_INST, DRV2605_ADDR << 1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(DRV_I2C_INST, reg);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // 2. Data Read
    I2C_GenerateSTART(DRV_I2C_INST, ENABLE);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(DRV_I2C_INST, DRV2605_ADDR << 1, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    I2C_AcknowledgeConfig(DRV_I2C_INST, DISABLE); // NACK for single byte
    while(I2C_GetFlagStatus(DRV_I2C_INST, I2C_FLAG_RXNE) == RESET);
    val = I2C_ReceiveData(DRV_I2C_INST);
    
    I2C_GenerateSTOP(DRV_I2C_INST, ENABLE);
    return val;
}

void IMU_ReadBytes(uint8_t dev_addr, uint8_t reg, uint8_t* buffer, uint8_t length) {
    uint32_t timeout = 100000;

    while(I2C_GetFlagStatus(DRV_I2C_INST, I2C_FLAG_BUSY) && (timeout--));

    // 1. ???? ?? ?? (Write ??)
    I2C_GenerateSTART(DRV_I2C_INST, ENABLE);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(DRV_I2C_INST, dev_addr << 1, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(DRV_I2C_INST, reg);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    // 2. Restart & ??? ?? (Read ??)
    I2C_GenerateSTART(DRV_I2C_INST, ENABLE);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_MODE_SELECT));

    I2C_Send7bitAddress(DRV_I2C_INST, dev_addr << 1, I2C_Direction_Receiver);
    while(!I2C_CheckEvent(DRV_I2C_INST, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    // 3. ??? ?? ??
    while(length) {
        if(length == 1) {
            // ??? ???? NACK? ??? ?? ??? ??
            I2C_AcknowledgeConfig(DRV_I2C_INST, DISABLE);
            I2C_GenerateSTOP(DRV_I2C_INST, ENABLE);
        } else {
            I2C_AcknowledgeConfig(DRV_I2C_INST, ENABLE); // ?? ?? ?? ACK
        }

        while(I2C_GetFlagStatus(DRV_I2C_INST, I2C_FLAG_RXNE) == RESET);
        *buffer = I2C_ReceiveData(DRV_I2C_INST);
        buffer++;
        length--;
    }
    // ?? ??? ?? ACK ?? ???
    I2C_AcknowledgeConfig(DRV_I2C_INST, ENABLE);
}

/* =================================================================================
 * High-Level Functions
 * ================================================================================= */

/*********************************************************************
 * @fn      DRV2605_Init
 * @brief   Initialize I2C2 (PB10, PB11) and DRV2605L
 * Configures for External Edge Trigger on PB4 with Click Effect
 *********************************************************************/
void DRV2605_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    I2C_InitTypeDef I2C_InitTSturcture = {0};

    // 1. Clock Enable
    RCC_APB2PeriphClockCmd(DRV_I2C_GPIO_CLK | DRV_TRIG_CLK | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(DRV_I2C_CLK, ENABLE);

    // 2. I2C Pin Config (PB10=SCL, PB11=SDA) - Open Drain
    GPIO_InitStructure.GPIO_Pin = DRV_I2C_SCL_PIN | DRV_I2C_SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DRV_I2C_PORT, &GPIO_InitStructure);

    // 3. Trigger Pin Config (PB4) - Push-Pull Output
    GPIO_InitStructure.GPIO_Pin = DRV_TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DRV_TRIG_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(DRV_TRIG_PORT, DRV_TRIG_PIN); // Init Low

    // 4. I2C2 Peripheral Config
    I2C_InitTSturcture.I2C_ClockSpeed = 100000; // 100kHz
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitTSturcture.I2C_OwnAddress1 = 0x00;
    I2C_InitTSturcture.I2C_Ack = ENABLE;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(DRV_I2C_INST, &I2C_InitTSturcture);
    I2C_Cmd(DRV_I2C_INST, ENABLE);

    Delay_Ms(10); // Power-up wait

    /* --- DRV2605L Setup --- */
    
    // 1. Mode 0x00 (Internal Trigger)?? ???? Standby ??
    DRV2605_WriteByte(DRV2605_REG_MODE, MODE_INT_TRIG); 

    // 2. Motor Type ?? (LRA) ? Library ??
    uint8_t feedback = DRV2605_ReadByte(DRV2605_REG_FEEDBACK);
    DRV2605_WriteByte(DRV2605_REG_FEEDBACK, feedback | DRV2605_MODE_LRA); 
    DRV2605_WriteByte(DRV2605_REG_LIBRARY, LIB_LRA); 

    /* [???] VG1036001D ?? ?? ?? */
    // 170Hz ?? ??? ?? (0x13 -> 0x18)
    DRV2605_WriteByte(DRV2605_REG_CONTROL1, 0x18); 
    
    // ?? ?? 2.0Vrms ?? (0x60 = ? 2.2V, ??? ??)
    DRV2605_WriteByte(DRV2605_REG_RATED_VOLT, 0x60);
    
    // ?????? ?? ?? ?? (3.3V ?? ? ??)
    DRV2605_WriteByte(DRV2605_REG_OD_CLAMP, 0xFF);

    /* [???] ?? ?????? (Auto Calibration) ?? */
    // ? ??? ??? ?? ??? ?? ???? ?? ??? ??????.
    DRV2605_WriteByte(DRV2605_REG_MODE, MODE_AUTO_CAL); // ?????? ??
    DRV2605_WriteByte(DRV2605_REG_GO, 0x01);            // ??
    
    // ?? ?? (GO ??? 0? ? ???)
    uint8_t go = 1;
    while(go) {
        go = DRV2605_ReadByte(DRV2605_REG_GO) & 0x01;
        Delay_Ms(10);
    }

    // 3. Mode? External Edge Trigger (0x01)? ?? (?? ?? ??)
    DRV2605_WriteByte(DRV2605_REG_MODE, MODE_EXT_TRIG_EDGE);

    // 4. Effect #1 (Strongest Click) ??
    // ?? 17??? 1?? ? ??? ???? ???.
    DRV2605_SelectEffect(1); 
}

/*********************************************************************
 * @fn      DRV2605_SelectEffect
 * @brief   Waveform Sequencer? ?? ?? (Slot 1)
 *********************************************************************/
void DRV2605_SelectEffect(uint8_t effect_id) {
    DRV2605_WriteByte(DRV2605_REG_WAV_SEQ1, effect_id); // Slot 1: Effect ID
    DRV2605_WriteByte(DRV2605_REG_WAV_SEQ2, 0x00);      // Slot 2: End (0)
}

/*********************************************************************
 * @fn      DRV2605_Trigger_Pulse
 * @brief   PB4? High Pulse? ???? ?? ??
 *********************************************************************/
void DRV2605_Trigger_Pulse(void) {
    GPIO_SetBits(DRV_TRIG_PORT, DRV_TRIG_PIN);
    Delay_Us(10); // Short pulse (>50ns required)
    GPIO_ResetBits(DRV_TRIG_PORT, DRV_TRIG_PIN);
}


typedef struct {
    int16_t ax, ay, az; // ???
    int16_t gx, gy, gz; // ???
    int16_t mx, my, mz; // ???
} IMU_Data_t;

IMU_Data_t imu_data;

void IMU_Init_All(void) {
    // 1. ADXL345 ?? (?? ?? ??)
    IMU_WriteByte(ADXL345_ADDR, 0x2D, 0x08); // POWER_CTL -> Measure

    // 2. ITG3205 ?? (?? ?? ? ?? ??)
    IMU_WriteByte(ITG3205_ADDR, 0x3E, 0x01); // PLL with X Gyro ref
    IMU_WriteByte(ITG3205_ADDR, 0x16, 0x18 + 0x01); // DLPF_CFG = 1, Full Scale

    // 3. HP5883 (QMC5883L) ?? [???]
    // 0x09 (Control 1): OSR=512, RNG=8G, ODR=200Hz, Mode=Continuous -> 0x1D
    IMU_WriteByte(HP5883_ADDR, 0x09, 0x1D); 
    // 0x0B (SET/RESET): Period=1
    IMU_WriteByte(HP5883_ADDR, 0x0B, 0x01);
}

void IMU_Read_All(void) {
    uint8_t buf[6];

    // 1. ADXL345 ?? (0x32 ~ 0x37) - Little Endian
    IMU_ReadBytes(ADXL345_ADDR, 0x32, buf, 6);
    imu_data.ax = (int16_t)((buf[1] << 8) | buf[0]);
    imu_data.ay = (int16_t)((buf[3] << 8) | buf[2]);
    imu_data.az = (int16_t)((buf[5] << 8) | buf[4]);

    // 2. ITG3205 ?? (0x1D ~ 0x22) - Big Endian (??!)
    IMU_ReadBytes(ITG3205_ADDR, 0x1D, buf, 6);
    imu_data.gx = (int16_t)((buf[0] << 8) | buf[1]);
    imu_data.gy = (int16_t)((buf[2] << 8) | buf[3]);
    imu_data.gz = (int16_t)((buf[4] << 8) | buf[5]);

    // 3. HP5883 (QMC5883L) ?? (0x00 ~ 0x05) - Little Endian [???]
    IMU_ReadBytes(HP5883_ADDR, 0x00, buf, 6);
    imu_data.mx = (int16_t)((buf[1] << 8) | buf[0]);
    imu_data.my = (int16_t)((buf[3] << 8) | buf[2]);
    imu_data.mz = (int16_t)((buf[5] << 8) | buf[4]);
}