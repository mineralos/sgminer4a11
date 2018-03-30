#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>

#include "spi-context.h"
#include "logging.h"
#include "miner.h"
#include "util.h"

#include "dragonmint_a11.h"
#include "dragonmint_a11_cmd.h"
#include "dragonmint_a11_clock.h"

#define CMD0D_LEN            7
#define CMD0D_RD_DATA_LEN    (CMD0D_LEN-2)


const unsigned short wCRCTalbeAbs[] =
{
    0x0000, 0xCC01, 0xD801, 0x1400, 
    0xF001, 0x3C00, 0x2800, 0xE401, 
    0xA001, 0x6C00, 0x7800, 0xB401, 
    0x5000, 0x9C01, 0x8801, 0x4400,
};

inline unsigned short CRC16_2(unsigned char* pchMsg, unsigned short wDataLen)
{
    volatile unsigned short wCRC = 0xFFFF;
    unsigned short i;
    unsigned char chChar;

    for (i = 0; i < wDataLen; i++){
        chChar = *pchMsg++;
        wCRC = wCRCTalbeAbs[(chChar ^ wCRC) & 15] ^ (wCRC >> 4);
        wCRC = wCRCTalbeAbs[((chChar >> 4) ^ wCRC) & 15] ^ (wCRC >> 4);
    }

    return wCRC;
}

static void applog_hexdump(char *prefix, uint8_t *buff, int len, int level)
{
    static char line[512];
    char *pos = line;
    int i;
    if (len < 1)
    {
        return;
    }

    pos += sprintf(pos, "%s: %d bytes:", prefix, len);
    for (i = 0; i < len; i++) 
    {
        if (i > 0 && (i % 32) == 0) 
        {
            applog(LOG_INFO, "%s", line);
            pos = line;
            pos += sprintf(pos, "\t");
        }
        pos += sprintf(pos, "%.2X ", buff[i]);
    }
    applog(level, "%s", line);
}

void hexdump(char *prefix, uint8_t *buff, int len)
{
    applog_hexdump(prefix, buff, len, LOG_WARNING);
}

void hexdump_error(char *prefix, uint8_t *buff, int len)
{
    applog_hexdump(prefix, buff, len, LOG_ERR);
}

bool mcompat_cmd_resetall(uint8_t chain_id, uint8_t chip_id, uint8_t *result)
{
    uint8_t cmd[2] = {0x0, 0x0};

    return mcompat_cmd_reset(chain_id, chip_id, cmd, result);
}

bool mcompat_cmd_resetjob(uint8_t chain_id, uint8_t chip_id, uint8_t *result)
{
    uint8_t cmd[2] = {0xed, 0xed};
    
    return mcompat_cmd_reset(chain_id, chip_id, cmd, result);
}

bool mcompat_cmd_resetbist(uint8_t chain_id, uint8_t chip_id, uint8_t *result)
{
    uint8_t cmd[2] = {0x20, 0x20};
    
    return mcompat_cmd_reset(chain_id, chip_id, cmd, result);
}


