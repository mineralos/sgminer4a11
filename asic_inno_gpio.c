#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include "miner.h"
#include "logging.h"
#include "asic_inno.h"
#include "asic_inno_gpio.h"

#define IOCTL_SET_VAL_0 _IOR(MAGIC_NUM, 0, char *)
#define IOCTL_SET_VALUE_0 _IOR(MAGIC_NUM, 0, char *)
#define IOCTL_SET_CHAIN_0 _IOR(MAGIC_NUM, 1, char *)

extern hardware_version_e g_hwver;

void loop_blink_led(int chain_id, int cnt)
{
    uint32_t u_cnt = (uint32_t) cnt;

    while(u_cnt--)
    {
        usleep(500000);
        inno_set_led(chain_id, 1);
        usleep(500000);
        inno_set_led(chain_id, 0);
    }
}

