#ifndef __GT911_H
#define __GT911_H
#include "softiic.h"
#define GT_CMD_WR        0xBA
#define GT_CMD_RD        0xBB

#define GT_CTRL_REG      0x8040      //GT911���ƼĴ���
#define GT_CFGS_REG      0x8047      //GT911������ʼ��ַ�Ĵ���
#define GT_CHECK_REG     0x80FF      //GT911У��ͼĴ���
#define GT_PID_REG       0x8140      //GT911��ƷID�Ĵ���
#define GT_GSTID_REG     0x814E      //GT911��ǰ��⵽�Ĵ������

uint8_t GT911_WR_Reg(uint16_t reg, uint8_t *buf, uint8_t len);
void GT911_RD_Reg(uint16_t reg, uint8_t *buf, uint8_t len);
uint8_t GT911_Send_Cfg(uint8_t mode);
#endif
