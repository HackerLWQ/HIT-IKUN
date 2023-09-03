
#ifndef __BSP_I2C_H__
#define __BSP_I2C_H__
#include "stm32f4xx_hal.h"

#define u16 uint16_t
#define u8 uint8_t

#define hsccb &hi2c2
#define DCMI_RST_GPIO_PORT GPIOC
#define DCMI_RST_GPIO_PIN GPIO_PIN_15
#define DCMI_PWDN_GPIO_PORT GPIOC
#define DCMI_PWDN_GPIO_PIN GPIO_PIN_14
#define SCCB_SCL_PORT GPIOB
#define SCCB_SCL_PIN GPIO_PIN_10
#define SCCB_SDA_PORT GPIOB
#define SCCB_SDA_PIN GPIO_PIN_11
#define ov7670_addr     0x60
//extern I2C_HandleTypeDef hi2c2;

#define delay_100us for(int i=0;i<4300;i++);
#define delay_50us  for(int i=0;i<2150;i++);
void OV7670_Window_Set(u16 sx,u16 sy,u16 width,u16 height);
//u8 SCCB_RD_Reg(u8 reg);
//u8 SCCB_WR_Reg(u8 reg,u8 data);
void ov7670_init(void);
void ov7670_sccb_init(void);
#endif