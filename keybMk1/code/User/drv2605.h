#ifndef __DRV2605_H
#define __DRV2605_H

#include "ch32v30x.h"

/* --- DRV2605L I2C Address --- */

#define ADXL345_ADDR   0x53
#define ITG3205_ADDR   0x68
#define HP5883_ADDR  0x0D  // I2C ??

#define DRV2605_ADDR             0x5A // 7-bit address

/* --- Register Map (Datasheet & Arduino Lib Reference) --- */
#define DRV2605_REG_STATUS       0x00
#define DRV2605_REG_MODE         0x01
#define DRV2605_REG_RTP_INPUT    0x02
#define DRV2605_REG_LIBRARY      0x03
#define DRV2605_REG_WAV_SEQ1     0x04
#define DRV2605_REG_WAV_SEQ2     0x05
#define DRV2605_REG_GO           0x0C
#define DRV2605_REG_OVERDRIVE    0x0D
#define DRV2605_REG_SUSTAIN_POS  0x0E
#define DRV2605_REG_SUSTAIN_NEG  0x0F
#define DRV2605_REG_BREAK        0x10
#define DRV2605_REG_AUDIO_CTRL   0x11
#define DRV2605_REG_RATED_VOLT   0x16
#define DRV2605_REG_OD_CLAMP     0x17
#define DRV2605_REG_AUTO_CAL_COMP 0x18
#define DRV2605_REG_AUTO_CAL_BEMF 0x19
#define DRV2605_REG_FEEDBACK     0x1A
#define DRV2605_REG_CONTROL1     0x1B
#define DRV2605_REG_CONTROL2     0x1C
#define DRV2605_REG_CONTROL3     0x1D
#define DRV2605_REG_CONTROL4     0x1E
#define DRV2605_REG_CONTROL5     0x1F
#define DRV2605_REG_LRA_LOOP     0x20
#define DRV2605_REG_VBAT         0x21
#define DRV2605_REG_LRA_PERIOD   0x22

/* --- Modes (Register 0x01) --- */
#define MODE_INT_TRIG            0x00
#define MODE_EXT_TRIG_EDGE       0x01 // [Selected] External Edge Trigger (Click?)
#define MODE_EXT_TRIG_LVL        0x02
#define MODE_PWM_ANALOG          0x03
#define MODE_AUDIO_VIBE          0x04
#define MODE_RTP                 0x05
#define MODE_DIAGNOSTICS         0x06
#define MODE_AUTO_CAL            0x07

/* --- Library Selection (Register 0x03) --- */
#define LIB_EMPTY                0x00
#define LIB_ERM_A                0x01
#define LIB_ERM_B                0x02
#define LIB_ERM_C                0x03
#define LIB_ERM_D                0x04
#define LIB_ERM_E                0x05
#define LIB_LRA                  0x06 // LRA ?? ?? ? ??
#define LIB_ERM_F                0x07

/* --- Feedback Control (Register 0x1A) --- */
#define DRV2605_MODE_ERM         0x00 
#define DRV2605_MODE_LRA         (1 << 7) 

/* --- GPIO & I2C Definitions (Hardware Mapping) --- */
// I2C2 used for PB10(SCL), PB11(SDA)
#define DRV_I2C_PORT             GPIOB
#define DRV_I2C_SCL_PIN          GPIO_Pin_10
#define DRV_I2C_SDA_PIN          GPIO_Pin_11
#define DRV_I2C_INST             I2C2
#define DRV_I2C_CLK              RCC_APB1Periph_I2C2
#define DRV_I2C_GPIO_CLK         RCC_APB2Periph_GPIOB

// Trigger Pin: PB4 (External Trigger)
#define DRV_TRIG_PORT            GPIOB
#define DRV_TRIG_PIN             GPIO_Pin_4
#define DRV_TRIG_CLK             RCC_APB2Periph_GPIOB

/* --- Function Prototypes --- */
void DRV2605_Init(void);
void DRV2605_SelectEffect(uint8_t effect_id); // ?? ?? (1~123)
void DRV2605_Trigger_Pulse(void);             // PB4 ?? ?? (?? ??)

// Low Level (?? ? ?? ??)
void DRV2605_WriteByte(uint8_t reg, uint8_t data);
uint8_t DRV2605_ReadByte(uint8_t reg);
void IMU_WriteByte(uint8_t dev_addr, uint8_t reg, uint8_t data);
void IMU_ReadBytes(uint8_t dev_addr, uint8_t reg, uint8_t* buffer, uint8_t length);
#endif /* __DRV2605_H */
