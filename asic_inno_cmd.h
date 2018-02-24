#ifndef _ASIC_INNO_CMD_
#define _ASIC_INNO_CMD_

#include <pthread.h>
#include <sys/time.h>

#include "asic_inno.h"

#define CMD_BIST_START      0x01
#define CMD_BIST_COLLECT    0x0b
#define CMD_BIST_FIX        0x03
#define CMD_RESET           0x04
#define CMD_RESETBC         0x05
#define CMD_WRITE_JOB       0x07
#define CMD_READ_RESULT     0x08
#define CMD_WRITE_REG       0x09
#define CMD_READ_REG        0x0a
#define CMD_READ_REG_RESP   0x1a
#define CMD_POWER_ON        0x02
#define CMD_POWER_OFF       0x06
#define CMD_POWER_RESET     0x0c
#define CMD_READ_SEC_REG    0x0d

#define CMD_TYPE_A7         0xb0
#define CMD_TYPE_A8         0x30
#define CMD_TYPE_A11        0x70
#define CMD_TYPE_A12        0xc0


#define ADDR_BROADCAST      0x00

#define LEN_BIST_START      4
#define LEN_BIST_COLLECT    4
#define LEN_BIST_FIX        4
#define LEN_RESET           6
#define LEN_WRITE_JOB       92
#define LEN_READ_RESULT     10
#define LEN_WRITE_REG       18
#define LEN_READ_REG        12


#define SPI_REC_DATA_LOOP   10
#define SPI_REC_DATA_DELAY  1

#define ASIC_RESULT_LEN     6
#define READ_RESULT_LEN     (ASIC_RESULT_LEN + 2)

struct Test_bench {
    uint32_t uiPll; 
    int uiVol;
    uint32_t uiScore;
    uint32_t uiCoreNum;
};

#define WORK_BUSY 0
#define WORK_FREE 1


uint16_t CRC16_2(unsigned char* pchMsg, unsigned short wDataLen);

bool im_cmd_resetall(uint8_t chain_id, uint8_t chip_id, uint8_t *result);
bool im_cmd_resetjob(uint8_t chain_id, uint8_t chip_id, uint8_t *result);
bool im_cmd_resetbist(uint8_t chain_id, uint8_t chip_id, uint8_t *result);

//void flush_spi(struct A1_chain *pChain);
void hexdump_error(char *prefix, uint8_t *buff, int len);
void hexdump(char *prefix, uint8_t *buff, int len);


#endif
