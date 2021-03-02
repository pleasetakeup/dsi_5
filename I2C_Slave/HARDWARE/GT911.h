#ifndef __GT911_H
#define __GT911_H
#include "softiic.h"
#define GT_CMD_WR        0xBA
#define GT_CMD_RD        0xBB

#define GT_CTRL_REG      0x8040      //GT911控制寄存器
#define GT_CFGS_REG      0x8047      //GT911配置起始地址寄存器
#define GT_CHECK_REG     0x80FF      //GT911校验和寄存器
#define GT_PID_REG       0x8140      //GT911产品ID寄存器
#define GT_GSTID_REG     0x814E      //GT911当前检测到的触摸情况

uint8_t GT911_WR_Reg(uint16_t reg, uint8_t *buf, uint8_t len);
void GT911_RD_Reg(uint16_t reg, uint8_t *buf, uint8_t len);
uint8_t GT911_Send_Cfg(uint8_t mode);
#endif
