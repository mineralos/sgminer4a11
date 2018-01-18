#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>

#include "logging.h"
#include "miner.h"
#include "util.h"

#include "spi-context.h"
#include "asic_inno.h"
#include "asic_inno_cmd.h"
#include "asic_inno_clock.h"
#include "asic_inno_gpio.h"
#include "inno_fan.h"


//#define MAGIC_NUM  100 
#define MUL_COEF 1.23
extern struct spi_ctx *spi[ASIC_CHAIN_NUM];
extern struct A1_chain *chain[ASIC_CHAIN_NUM];


static const float inno_vsadc_table[] = {   
    0.54691,
    0.54382,
    0.54073,
    0.53764,
    0.53455,
    0.53145,
    0.52827,
    0.52518,
    0.52200,
    0.51882,
    0.51573,
    0.51264,
    0.50945,
    0.50636,
    0.50318,
    0.50009,
    0.49691,
    0.49373,
    0.49064,
    0.48755,
    0.48445,
    0.48118,
    0.47818,
    0.47500,
    0.47191,
    0.46891,
    0.46582,
    0.46264,
    0.45964,
    0.45645,
    0.45345,
    0.45027,
};



extern inno_reg_ctrl_t s_reg_ctrl;

int nReadVolTimes = 0;
int nVolTotal = 0;

static const uint8_t default_reg[119][12] = 
{
    {0x02, 0x50, 0x40, 0xc2, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //120MHz
    {0x02, 0x53, 0x40, 0xc2, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //125MHz
    {0x02, 0x56, 0x40, 0xc2, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //129MHz
    {0x02, 0x5d, 0x40, 0xc2, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //140MHz
    {0x02, 0x32, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //150MHz
    {0x02, 0x35, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //159MHz
    {0x02, 0x39, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //171MHz
    {0x02, 0x3c, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //180MHz
    {0x02, 0x3f, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //189MHz
    {0x02, 0x43, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //201MHz
    {0x02, 0x46, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //210MHz
    {0x02, 0x49, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //219MHz
    {0x02, 0x4d, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //231MHz
    {0x02, 0x50, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //240MHz
    {0x02, 0x53, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //249MHz
    {0x02, 0x57, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //261MHz
    {0x02, 0x5a, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //270MHz
    {0x02, 0x5d, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //279MHz
    {0x02, 0x61, 0x40, 0x82, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //291MHz
    {0x02, 0x32, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //300MHz
    {0x02, 0x34, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //312MHz
    {0x02, 0x35, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //318MHz
    {0x02, 0x37, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //330MHz
    {0x02, 0x39, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //342MHz
    {0x02, 0x3a, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //348MHz
    {0x02, 0x3c, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //360MHz
    {0x02, 0x3e, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //372MHz
    {0x02, 0x3f, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //378MHz
    {0x02, 0x41, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //390MHz
    {0x02, 0x43, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //402MHz
    {0x02, 0x44, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //408MHz
    {0x02, 0x46, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //420MHz
    {0x02, 0x48, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //432MHz
    {0x02, 0x49, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //438MHz
    {0x02, 0x4b, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //450MHz
    {0x02, 0x4d, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //462MHz
    {0x02, 0x4e, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //468MHz
    {0x02, 0x50, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //480MHz
    {0x02, 0x52, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //492MHz
    {0x02, 0x53, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //498MHz
    {0x02, 0x55, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //510MHz
    {0x02, 0x57, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //522MHz
    {0x02, 0x58, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //528MHz
    {0x02, 0x5a, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //540MHz
    {0x02, 0x5c, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //552MHz
    {0x02, 0x5d, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //558MHz
    {0x02, 0x5f, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //570MHz
    {0x02, 0x61, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //582MHz
    {0x02, 0x62, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x24, 0x00, 0x00}, //588MHz
    {0x02, 0x64, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //600MHz
    {0x02, 0x66, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //612MHz
    {0x02, 0x68, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //624MHz
    {0x02, 0x69, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //630MHz
    {0x02, 0x6a, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //636MHz
    {0x02, 0x6c, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //648MHz
    {0x02, 0x6e, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //660MHz
    {0x02, 0x70, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //672MHz
    {0x02, 0x72, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //684MHz
    {0x02, 0x73, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //690MHz
    {0x02, 0x74, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //696MHz
    {0x02, 0x76, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //708MHz
    {0x02, 0x78, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //720MHz
    {0x02, 0x7a, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //732MHz
    {0x02, 0x7c, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //744MHz
    {0x02, 0x7d, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //750MHz
    {0x02, 0x7e, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //756MHz
    {0x02, 0x80, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //768MHz
    {0x02, 0x82, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //780MHz
    {0x02, 0x84, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //792MHz
    {0x02, 0x86, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //804MHz
    {0x02, 0x87, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //810MHz
    {0x02, 0x88, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //816MHz
    {0x02, 0x8a, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //828MHz
    {0x02, 0x8c, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //840MHz
    {0x02, 0x8e, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //852MHz
    {0x02, 0x90, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //864MHz
    {0x02, 0x91, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //870MHz
    {0x02, 0x92, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //876MHz
    {0x02, 0x94, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //888MHz
    {0x02, 0x96, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //900MHz
    {0x02, 0x98, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //912MHz
    {0x02, 0x9a, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //924MHz
    {0x02, 0x9b, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //930MHz
    {0x02, 0x9c, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //936MHz
    {0x02, 0x9e, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //948MHz
    {0x02, 0xa0, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //960MHz
    {0x02, 0xa2, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //972MHz
    {0x02, 0xa4, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //984MHz
    {0x02, 0xa5, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //990MHz
    {0x02, 0xa6, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //996MHz
    {0x02, 0xa8, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1008MHz
    {0x02, 0xaa, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1020MHz
    {0x02, 0xac, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1032MHz
    {0x02, 0xae, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1044MHz
    {0x02, 0xaf, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1050MHz
    {0x02, 0xb0, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1056MHz
    {0x02, 0xb2, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1068MHz
    {0x02, 0xb4, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1080MHz
    {0x02, 0xb6, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1092MHz
    {0x02, 0xb8, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1104MHz
    {0x02, 0xb9, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1110MHz
    {0x02, 0xba, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1116MHz
    {0x02, 0xbc, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1128MHz
    {0x02, 0xbe, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1140MHz
    {0x02, 0xc0, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1152MHz
    {0x02, 0xc2, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1164MHz
    {0x02, 0xc3, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1170MHz
    {0x02, 0xc4, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1176MHz
    {0x02, 0xc6, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1188MHz
    {0x02, 0xc8, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1200MHz
    {0x02, 0xca, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1212MHz
    {0x02, 0xcc, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1224MHz
    {0x02, 0xcd, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1230MHz
    {0x02, 0xce, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1236MHz
    {0x02, 0xd0, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1248MHz
    {0x02, 0xd2, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1260MHz
    {0x02, 0xd4, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1272MHz
    {0x02, 0xd6, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1284MHz
    {0x02, 0xd8, 0x40, 0x42, 0x00, 0x00, 0x00, 0xa0, 0x00, 0x20, 0x00, 0x00}, //1296MHz
};

#if 0
static void rev(unsigned char *s, size_t l)
{
    size_t i, j;
    unsigned char t;

    for (i = 0, j = l - 1; i < j; i++, j--) {
        t = s[i];
        s[i] = s[j];
        s[j] = t;
    }
}
#endif

static uint8_t *create_job(uint8_t chip_id, uint8_t job_id, struct work *work)
{
        int i;
        //bool ret;
        //uint8_t job_id;
        uint16_t crc;
        uint8_t spi_tx[128];
        uint8_t spi_rx[128];
        uint8_t tmp_buf[128];
        
        #if 0
        printf("job cmd: \r\n");
        for(i = 0; i < 98; i++)
        {
            printf("0x%02x  ", work->data[i]);
            if((i % 8) == 7)
            {
                printf("\r\n");
            }
        }
        printf("\r\n");
#endif
#if 0
      static uint8_t job[JOB_LENGTH] = {
        /* command */
        0x17, 0x01,
        /* wdata  75 to 0 */
        0x20, 0x00, 0x00, 0x00, 0x5c, 0x3c, 
        0xf5, 0x94, 0xeb, 0x10, 0x71, 0x3b, 0x0e, 0x5b, 
        0xe7, 0x5c, 0xb9, 0x01, 0x09, 0xc2, 0x16, 0xea, 
        0xfd, 0x1c, 0xba, 0xba, 0x9c, 0x5e, 0x00, 0x00, 
        0x59, 0x16, 0x00, 0x00, 0x00, 0x00, 0xc7, 0x82, 
        0xa0, 0xa7, 0xc5, 0xa6, 0xf2, 0x08, 0x21, 0x6a, 
        0xe8, 0x49, 0x21, 0xbc, 0x8c, 0xa3, 0x7b, 0xee, 
        0x8c, 0x08, 0x5f, 0x93, 0x95, 0x06, 0x04, 0x02, 
        0xe9, 0x15, 0x70, 0x6b, 0x76, 0x19, 0x59, 0x28, 
        0xd7, 0x2c, 0x1b, 0x00, 0x81, 0xd5, 0xe4, 0x2e, 
        0x36, 0x40,
        /* difficulty */
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27,
        /* end nonce */
        0xe4, 0x2e, 0x36, 0x50,
        /* crc data */
        0x00, 0x00
    };
    #else
    static uint8_t job[JOB_LENGTH] = {
          /* command */
          0x00, 0x00,
          /* wdata  75 to 0 */
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          0x00, 0x00, 0x00, 0x00,
          /* start nonce */
          0x00, 0x00, 0x00, 0x00,
          /* difficulty */
          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
          /* end nonce */
          0x00, 0x00, 0x00, 0x00,
          /* crc data */
          0x00, 0x00
      };


    
    
        //job_id = 1;
        memset(spi_tx, 0, sizeof(spi_tx));
        memset(spi_rx, 0, sizeof(spi_rx));
    
        //printf("send command [writ_job] \r\n");
        
        // cmd
        spi_tx[0] = ((job_id & 0x0f) << 4) | CMD_WRITE_JOB;
        spi_tx[1] = chip_id;
        
        // data
        for(i = 0; i < 19; i++)
        {
#if 0
            spi_tx[2 + (i * 4) + 0] = (uint8_t)((work->data[i] >>  0) & 0xff);
            spi_tx[2 + (i * 4) + 1] = (uint8_t)((work->data[i] >>  8) & 0xff);
            spi_tx[2 + (i * 4) + 2] = (uint8_t)((work->data[i] >> 16) & 0xff);
            spi_tx[2 + (i * 4) + 3] = (uint8_t)((work->data[i] >> 24) & 0xff);
#else
            spi_tx[2 + (i * 4) + 0] = (uint8_t)((work->data[4*i+ 3] ) & 0xff);
            spi_tx[2 + (i * 4) + 1] = (uint8_t)((work->data[4*i+ 2] ) & 0xff);
            spi_tx[2 + (i * 4) + 2] = (uint8_t)((work->data[4*i+ 1] ) & 0xff);
            spi_tx[2 + (i * 4) + 3] = (uint8_t)((work->data[4*i+ 0] ) & 0xff);
#endif
        }
        
#if 0
        spi_tx[78] = (uint8_t)((work->data[19] >> 24) & 0xff);
        spi_tx[79] = (uint8_t)((work->data[19] >> 16) & 0xff);
        spi_tx[80] = (uint8_t)((work->data[19] >>  8) & 0xff);
        spi_tx[81] = (uint8_t)((work->data[19] >>  0) & 0xff);
#else
        spi_tx[78] = (uint8_t)((work->data[76]) & 0xff);
        spi_tx[79] = (uint8_t)((work->data[77]) & 0xff);
        spi_tx[80] = (uint8_t)((work->data[78]) & 0xff);
        spi_tx[81] = (uint8_t)((work->data[79]) & 0xff);
#endif
    
        
        // target
        for(i = 0; i < 2; i++)
        {
#if 0
            spi_tx[82 + (i * 4) + 0] = (uint8_t)((work->target[6 + i] >>  0) & 0xff);
            spi_tx[82 + (i * 4) + 1] = (uint8_t)((work->target[6 + i] >>  8) & 0xff);
            spi_tx[82 + (i * 4) + 2] = (uint8_t)((work->target[6 + i] >> 16) & 0xff);
            spi_tx[82 + (i * 4) + 3] = (uint8_t)((work->target[6 + i] >> 24) & 0xff);
#else
            spi_tx[82 + (i * 4) + 0] = (uint8_t)((work->target[27 + 4*i]) & 0xff);
            spi_tx[82 + (i * 4) + 1] = (uint8_t)((work->target[26 + 4*i]) & 0xff);
            spi_tx[82 + (i * 4) + 2] = (uint8_t)((work->target[25 + 4*i]) & 0xff);
            spi_tx[82 + (i * 4) + 3] = (uint8_t)((work->target[24 + 4*i]) & 0xff);
#endif      
        }
        
        // end nonce
        spi_tx[90] = 0xff;
        spi_tx[91] = 0xff;
        spi_tx[92] = 0xff;
        spi_tx[93] = 0xff;
#endif      
        // crc
        memset(tmp_buf, 0, sizeof(tmp_buf));
        
        for(i = 0; i < 47; i++)
        {
            tmp_buf[(2 * i) + 1] = spi_tx[(2 * i) + 0];
            tmp_buf[(2 * i) + 0] = spi_tx[(2 * i) + 1];
        }
        crc = CRC16_2(tmp_buf, 94);
        spi_tx[94] = (uint8_t)((crc >> 8) & 0xff);
        spi_tx[95] = (uint8_t)((crc >> 0) & 0xff);
        
#if 0
        printf("job cmd: \r\n");
        for(i = 0; i < 98; i++)
        {
            printf("0x%02x  ", spi_tx[i]);
            if((i % 8) == 7)
            {
                printf("\r\n");
            }
        }
        printf("\r\n");
#endif

    memcpy(job,spi_tx,98);
        return job;
}


#define COOLDOWN_MS         (30 * 1000)
#define DISABLE_CHIP_FAIL_THRESHOLD 3
#define LEAST_CORE_ONE_CHAIN    100
#define RESET_CHAIN_CNT 2
/********** disable / re-enable related section (temporary for testing) */
int get_current_ms(void)
{
    cgtimer_t ct;
    cgtimer_time(&ct);
    return cgtimer_to_ms(&ct);
}

bool is_chip_disabled(struct A1_chain *a1, uint8_t chip_id)
{
    struct A1_chip *chip = &a1->chips[chip_id - 1];
    return chip->disabled || chip->cooldown_begin != 0;
}

/* check and disable chip, remember time */
void disable_chip(struct A1_chain *a1, uint8_t chip_id)
{
    printf("disable chip\n");
    flush_spi(a1);
    struct A1_chip *chip = &a1->chips[chip_id - 1];
    int cid = a1->chain_id;
    if (is_chip_disabled(a1, chip_id)) {
        applog(LOG_NOTICE, "%d: chip %d already disabled",
               cid, chip_id);
        return;
    }
    applog(LOG_NOTICE, "%d: temporary disabling chip %d", cid, chip_id);
    chip->cooldown_begin = get_current_ms();
}

/* check if disabled chips can be re-enabled */
void check_disabled_chips(struct A1_chain *a1, int pllnum)
{
    int i;
    int cid = a1->chain_id;
    uint8_t reg[REG_LENGTH];
    struct spi_ctx *ctx = a1->spi_ctx;

    for (i = 0; i < a1->num_active_chips; i++) 
    {
        int chip_id = i + 1;
        struct A1_chip *chip = &a1->chips[i];
        if (!is_chip_disabled(a1, chip_id))
            continue;
        /* do not re-enable fully disabled chips */
        if (chip->disabled)
            continue;
        if (chip->cooldown_begin + COOLDOWN_MS > get_current_ms())
            continue;
        
        if (!inno_cmd_read_reg(a1, chip_id, reg)) 
        {
            chip->fail_count++;
            applog(LOG_NOTICE, "%d: chip %d not yet working - %d",
                   cid, chip_id, chip->fail_count);
            if (chip->fail_count > DISABLE_CHIP_FAIL_THRESHOLD) 
            {
                applog(LOG_NOTICE, "%d: completely disabling chip %d at %d",
                       cid, chip_id, chip->fail_count);
                chip->disabled = true;
                a1->num_cores -= chip->num_cores;
                continue;
            }
            /* restart cooldown period */
            chip->cooldown_begin = get_current_ms();
            continue;
        }
        applog(LOG_NOTICE, "%d: chip %d is working again", cid, chip_id);
        chip->cooldown_begin = 0;
        chip->fail_count = 0;
        chip->fail_reset = 0;
    }
#if 1

 //if the core in chain least than 100, reinit this chain
    if(asic_gpio_read(ctx->plug) == 0)
    {
        if(a1->num_cores <= LEAST_CORE_ONE_CHAIN)
        {
           // applog(LOG_WARNING, "****core:%d*start to reset the chain:%d******************", a1->num_cores, cid);
          //  applog(LOG_WARNING, "****core:%d*start to reset the chain:%d******************", a1->num_cores, cid);
            applog(LOG_WARNING, "****core:%d*start to reset the chain:%d******************", a1->num_cores, cid);

            asic_gpio_write(spi[i]->power_en, 0);
            sleep(1);
            asic_gpio_write(spi[i]->start_en, 0);
            asic_gpio_write(spi[i]->reset, 0);
            asic_gpio_write(spi[i]->led, 1);
           
#if 0
            asic_gpio_write(ctx->power_en, 0);
            sleep(5);
            asic_gpio_write(ctx->power_en, 1);
            sleep(5);
            asic_gpio_write(ctx->reset, 1);
            sleep(1);
            asic_gpio_write(ctx->start_en, 1);
            sleep(2);

           
            inno_preinit(opt_A1Pll1,120);

            a1->num_chips =  chain_detect_reload(a1);
            
            usleep(10000);

            if (a1->num_chips <= 0)
                goto failure;

            inno_cmd_bist_fix(a1, ADDR_BROADCAST);

            for (i = 0; i < a1->num_active_chips; i++)
            {
                check_chip(a1, i);

          }
#endif

        }
    }else{
         //applog(LOG_WARNING, "******there is no board insert******");
         applog(LOG_WARNING, "chain %d not insert,change all gpio to zero****", cid);
         asic_gpio_write(spi[i]->power_en, 0);
         sleep(1);
         asic_gpio_write(spi[i]->start_en, 0);
         asic_gpio_write(spi[i]->reset, 0);
         asic_gpio_write(spi[i]->led, 1);

    }
    
    return;

//failure:
 //    exit_A1_chain(a1);
  //   return;
 #endif             

}



bool set_work(struct A1_chain *a1, uint8_t chip_id, struct work *work, uint8_t queue_states)
{
    int cid = a1->chain_id;
    struct A1_chip *chip = &a1->chips[chip_id - 1];
    bool retval = false;

    int job_id = chip->last_queued_id + 1;
    //printf("set work 490\n");
    
    //printf("%d: queuing chip %d with job_id %d, state=0x%02x,last_queued:0x%x,addr:0x%x,0x%x\n", cid, chip_id, job_id, queue_states,chip->last_queued_id,chip->work[chip->last_queued_id-1],chip->work[chip->last_queued_id]);
    if (job_id == (queue_states & 0x0f) || job_id == (queue_states >> 4))
    {
        applog(LOG_NOTICE, "%d: job overlap: %d, 0x%02x", cid, job_id, queue_states);
    }

    if (chip->work[chip->last_queued_id] != NULL) 
    {
        //printf("set work 500\n");
        work_completed(a1->cgpu, chip->work[chip->last_queued_id]);
        chip->work[chip->last_queued_id] = NULL;    
        retval = true;
    }
    
    uint8_t *jobdata = create_job(chip_id, job_id, work);
    if (!inno_cmd_write_job(a1, chip_id, jobdata)) 
    {
        /* give back work */
        work_completed(a1->cgpu, work);
        applog(LOG_ERR, "%d: failed to set work for chip %d.%d", cid, chip_id, job_id);
        disable_chip(a1, chip_id);
    } 
    else 
    {
        chip->work[chip->last_queued_id] = work;
        chip->last_queued_id++;
        chip->last_queued_id &= 3;
    }
    //printf("retval:%d,last_queued_id %d\n",retval,chip->last_queued_id);
    return retval;
}


bool get_nonce(struct A1_chain *a1, uint8_t *nonce, uint8_t *chip_id, uint8_t *job_id)
{
    uint8_t buffer[64];
    //uint8_t tmp_nonce[4];
    memset(buffer, 0, sizeof(buffer));
    if(inno_cmd_read_result(a1, ADDR_BROADCAST, buffer))
    {
        *job_id = buffer[0] >> 4;
        *chip_id = buffer[1];

        memcpy(nonce, buffer+2, 4);
        //printf("buffer:0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5]);
        //printf("nonce:0x%02x,0x%02x,0x%02x,0x%02x\n",nonce[0],nonce[1],nonce[2],nonce[3]);

        applog(LOG_WARNING, "Got nonce for chip %d / job_id %d",*chip_id, *job_id);
        
        return true;
    }
    
    return false;
}

bool abort_work(struct A1_chain *a1)
{

    applog(LOG_INFO,"Start to reset ");

    return true;
}

bool check_chip(struct A1_chain *a1, int i)
{
    uint8_t buffer[64];
    int chip_id = i + 1;
    int cid = a1->chain_id;

    memset(buffer, 0, sizeof(buffer));
    if (!inno_cmd_read_reg(a1, chip_id, buffer)) {
        applog(LOG_NOTICE, "%d: Failed to read register for ""chip %d -> disabling", cid, chip_id);
        a1->chips[i].num_cores = 0;
        a1->chips[i].disabled = 1;
        return false;;
    }

    a1->chips[i].num_cores = buffer[11];
    a1->num_cores += a1->chips[i].num_cores;
    //applog(LOG_WARNING, "%d: Found chip %d with %d active cores",cid, chip_id, a1->chips[i].num_cores);

    //keep ASIC register value
    memcpy(a1->chips[i].reg, buffer, 12);
    a1->chips[i].temp= 0x000003ff & ((buffer[7]<<8) | buffer[8]);

    if (a1->chips[i].num_cores < BROKEN_CHIP_THRESHOLD){
        applog(LOG_NOTICE, "%d: broken chip %d with %d active ""cores (threshold = %d)",cid,chip_id,a1->chips[i].num_cores,BROKEN_CHIP_THRESHOLD);
        hexdump_error("new.PLL", a1->spi_rx, 8);
        a1->chips[i].disabled = true;
        a1->num_cores -= a1->chips[i].num_cores;
        
        return false;
    }

    if (a1->chips[i].num_cores < WEAK_CHIP_THRESHOLD) {
        applog(LOG_NOTICE, "%d: weak chip %d with %d active ""cores (threshold = %d)",cid,chip_id, a1->chips[i].num_cores, WEAK_CHIP_THRESHOLD);
        hexdump_error("new.PLL", a1->spi_rx, 8);    
        return false;
    }

    return true;
}

int prechain_detect(struct A1_chain *a1, int idxpll, int lastidx)
{
    //uint8_t buffer[64];
    int cid = a1->chain_id;
    uint8_t temp_reg[REG_LENGTH];
    int i,nCount = 0;
  
    usleep(500000);
    
    for(i=lastidx; i<idxpll+1; i++)
    {
        nCount = 0;
        memcpy(temp_reg, default_reg[i], REG_LENGTH);
        
         while(!inno_cmd_write_reg(a1, ADDR_BROADCAST, temp_reg))
         {
               usleep(200000);
               nCount++;
               if(nCount > 5) 
               {
                  applog(LOG_ERR, "set default PLL fail");
                  return -1;
                }
          }
       
           usleep(200000);
      }
        
        applog(LOG_WARNING, "chain %d set default %d PLL success",cid, i);

        usleep(500000);
    return 0;
}

bool zynq_spi_exit(void)
{
    int i;
    
    for(i = 0; i < ASIC_CHAIN_NUM; i++)
    {
        if(spi[i] != NULL)
        {
            spi_exit(spi[i]);
        }

        if(chain[i] != NULL)
        {
            free(chain[i]);
        }
    }

    return true;
}


int inno_chain_power_down(struct A1_chain *a1)
{
    asic_gpio_write(a1->spi_ctx->power_en, 0);
    sleep(1);
    asic_gpio_write(a1->spi_ctx->start_en, 0);
    asic_gpio_write(a1->spi_ctx->reset, 0);  

    return 0;
}



int power_down_all_chain(void)
{
    int i;

    for(i = 0; i < ASIC_CHAIN_NUM; i++)
    {
        if(chain[i] != NULL)
          inno_chain_power_down(chain[i]);
    }
    return 0;
}


/*
 * BIST_START works only once after HW reset, on subsequent calls it
 * returns 0 as number of chips.
 */
int chain_detect(struct A1_chain *a1)
{
    uint8_t buffer[64];
    int cid = a1->chain_id;
   // uint8_t temp_reg[REG_LENGTH];
   // int i;

    memset(buffer, 0, sizeof(buffer));
    if(!inno_cmd_bist_start(a1, ADDR_BROADCAST, buffer)){
        applog(LOG_ERR, "bist start fail");
        return -1;
    }

    a1->num_chips = buffer[3]; 
    applog(LOG_ERR, "%d: detected %d chips", cid, a1->num_chips);

    return a1->num_chips;
}

//add 0922
void inno_configure_tvsensor(struct A1_chain *a1, int chip_id,bool is_tsensor)
{
 //int i;
 unsigned char *tmp_reg = malloc(128);
 unsigned char *src_reg = malloc(128);
 unsigned char *reg = malloc(128);
 inno_cmd_read_reg(a1, 0x01, reg);
 
 //chip_id = 0;

  memset(tmp_reg, 0, 128);
  memcpy(src_reg,reg,REG_LENGTH-2);
  inno_cmd_write_reg(a1,chip_id,src_reg);
  usleep(200);

 if(is_tsensor)//configure for tsensor
 {
  //Step1: wait for clock stable
  //Step2: low the tsdac rst_n and release rst_n after 4 SysClk
   //hexdump("write reg", reg, REG_LENGTH);

#if DEBUG   
   printf("Write Reg:");
   for(i=0; i<20;i++)
    printf("%x, ",reg[i]);

    printf("\n\n");
#endif

   reg[7] = (src_reg[7]&0x7f);
   memcpy(tmp_reg,reg,REG_LENGTH-2);
   //hexdump("write reg", tmp_reg, REG_LENGTH);
   inno_cmd_write_reg(a1,chip_id,tmp_reg);
   usleep(200);
   reg[7] = (src_reg[7]|0x80);
   memcpy(tmp_reg,reg,REG_LENGTH-2);
   inno_cmd_write_reg(a1,chip_id,tmp_reg);
   usleep(200);
   
 #if DEBUG
   printf("Write Reg:");
      
      for(i=0; i<20;i++)
       printf("%x, ",reg[i]);

    printf("\n\n");
#endif
    //Step3: Config tsadc_clk(default match)
    //Step4: low tsadc_tsen_pd
    //Step5: high tsadc_ana_reg_2

    reg[6] = (src_reg[6]|0x04);
    memcpy(tmp_reg,reg,REG_LENGTH-2);
    inno_cmd_write_reg(a1,chip_id,tmp_reg);
    usleep(200);

    //Step6: high tsadc_en
    reg[7] = (src_reg[7]|0x20);
    memcpy(tmp_reg,reg,REG_LENGTH-2);
    inno_cmd_write_reg(a1,chip_id,tmp_reg);
    usleep(200);

    //Step7: tsadc_ana_reg_9 = 0;tsadc_ana_reg_8  = 0
    reg[5] = (src_reg[5]&0xfc);
    memcpy(tmp_reg,reg,REG_LENGTH-2);
    inno_cmd_write_reg(a1,chip_id,tmp_reg);
    usleep(200);
    
    //Step8: tsadc_ana_reg_7 = 1;tsadc_ana_reg_1 = 0
    reg[6] = (src_reg[6]&0x7d);
    memcpy(tmp_reg,reg,REG_LENGTH-2);
    inno_cmd_write_reg(a1,chip_id,tmp_reg);
    usleep(200);
  }else{//configure for vsensor
     //Step1: wait for clock stable
  //Step2: low the tsdac rst_n and release rst_n after 4 SysClk
  // hexdump("write reg", reg, REG_LENGTH);
   
#if DEBUG
   printf("Write Reg:");
   
   for(i=0; i<20;i++)
    printf("%x, ",reg[i]);

    printf("\n\n");
#endif

   reg[7] = (src_reg[7]&0x7f);
   memcpy(tmp_reg,reg,REG_LENGTH-2);
  // hexdump("write reg", tmp_reg, REG_LENGTH);
   inno_cmd_write_reg(a1,chip_id,tmp_reg);
   usleep(200);
   reg[7] = (src_reg[7]|0x80);
   memcpy(tmp_reg,reg,REG_LENGTH-2);
   inno_cmd_write_reg(a1,chip_id,tmp_reg);
   usleep(200);
#if DEBUG
   printf("Write Reg:");
      
      for(i=0; i<20;i++)
       printf("%x, ",reg[i]);

    printf("\n\n");
#endif
    //Step3: Config tsadc_clk(default match)
    //Step4: low tsadc_tsen_pd
    //Step5: high tsadc_ana_reg_2

    reg[6] = (src_reg[6]|0x04);
    memcpy(tmp_reg,reg,REG_LENGTH-2);
    inno_cmd_write_reg(a1,chip_id,tmp_reg);
    usleep(200);

    //Step6: high tsadc_en
    reg[7] = (src_reg[7]|0x20);
    memcpy(tmp_reg,reg,REG_LENGTH-2);
    inno_cmd_write_reg(a1,chip_id,tmp_reg);
    usleep(200);

    //Step7: tsadc_ana_reg_9 = 0;tsadc_ana_reg_8  = 0
    reg[5] = ((src_reg[5]|0x01)&0xfd);
    memcpy(tmp_reg,reg,REG_LENGTH-2);
    inno_cmd_write_reg(a1,chip_id,tmp_reg);
    usleep(200);
    
    //Step8: tsadc_ana_reg_7 = 1;tsadc_ana_reg_1 = 0
    reg[6] = ((src_reg[6]|0x02)&0x7f);
    memcpy(tmp_reg,reg,REG_LENGTH-2);
    inno_cmd_write_reg(a1,chip_id,tmp_reg);
    usleep(200);

  }
    free(tmp_reg);
    free(src_reg);
}

int inno_get_voltage_stats(struct A1_chain *a1, inno_reg_ctrl_t *s_reg_ctrl)
{
   int i = 0;
   int cid = a1->chain_id;
   s_reg_ctrl->highest_vol[cid] = s_reg_ctrl->stat_val[cid][0];
   s_reg_ctrl->lowest_vol[cid] = s_reg_ctrl->stat_val[cid][0];
   int total_vol = s_reg_ctrl->stat_val[cid][0];

   if((a1->num_active_chips < 1) || (a1 == NULL))
    return -1;

   for (i = 1; i < a1->num_active_chips; i++)
   {
    if(s_reg_ctrl->highest_vol[cid] < s_reg_ctrl->stat_val[cid][i])
        s_reg_ctrl->highest_vol[cid] = s_reg_ctrl->stat_val[cid][i];

    if(s_reg_ctrl->lowest_vol[cid] > s_reg_ctrl->stat_val[cid][i])
        s_reg_ctrl->lowest_vol[cid] = s_reg_ctrl->stat_val[cid][i];

    total_vol += s_reg_ctrl->stat_val[cid][i];
   }

  s_reg_ctrl->avarge_vol[cid] = total_vol / a1->num_active_chips;

  return 0;
}

bool inno_check_voltage(struct A1_chain *a1, int chip_id, inno_reg_ctrl_t *s_reg_ctrl)
{
  
    uint8_t reg[128];
    memset(reg, 0, 128);
  
    if (!inno_cmd_read_reg(a1, chip_id, reg)) {
        applog(LOG_NOTICE, "%d: Failed to read register for ""chip %d -> disabling", a1->chain_id, chip_id);
        a1->chips[chip_id].num_cores = 0;
        a1->chips[chip_id].disabled = 1;
        return false;
    }else{
          usleep(2000);

            /* update temp database */
            uint32_t rd_v = 0;
            rd_v = 0x000003ff & ((reg[7] << 8) | reg[8]);
            float tmp_v = (float)(rd_v * MUL_COEF)/1024;
            a1->chips[chip_id-1].nVol = tmp_v *1000; 
            s_reg_ctrl->stat_val[a1->chain_id][chip_id-1] = a1->chips[chip_id-1].nVol;
            //s_reg_ctrl->stat_cnt[a1->chain_id][chip_id-1]++;
            
            #if 0
            s_reg_ctrl->stat_cnt[a1->chain_id][chip_id-1]++;
            
           if(s_reg_ctrl->stat_cnt[a1->chain_id][chip_id-1] == 1)
           {
             s_reg_ctrl->highest_vol[a1->chain_id][chip_id-1] = tmp_v;
             s_reg_ctrl->lowest_vol[a1->chain_id][chip_id-1] = tmp_v;
             s_reg_ctrl->avarge_vol[a1->chain_id][chip_id-1] = tmp_v;
           }else{
             if(s_reg_ctrl->highest_vol[a1->chain_id][chip_id-1] < tmp_v){
                s_reg_ctrl->highest_vol[a1->chain_id][chip_id-1] = tmp_v;
               }
            if(s_reg_ctrl->lowest_vol[a1->chain_id][chip_id-1] > tmp_v){
                s_reg_ctrl->lowest_vol[a1->chain_id][chip_id-1] = tmp_v;
               }
            s_reg_ctrl->avarge_vol[a1->chain_id][chip_id-1] = (s_reg_ctrl->avarge_vol[a1->chain_id][chip_id-1]*(s_reg_ctrl->stat_cnt[a1->chain_id][chip_id-1]-1) + tmp_v)/s_reg_ctrl->stat_cnt[a1->chain_id][chip_id-1];
           }
            #endif
            
            //applog(LOG_WARNING,"read tmp %f/%d form chain %d,chip %d h:%f,l:%f,av:%f,cnt:%d\n",tmp_v,rd_v,a1->chain_id, chip_id,s_reg_ctrl->highest_vol[a1->chain_id][chip_id-1],s_reg_ctrl->lowest_vol[a1->chain_id][chip_id-1],s_reg_ctrl->avarge_vol[a1->chain_id][chip_id-1],s_reg_ctrl->stat_cnt[a1->chain_id][chip_id-1]);
            nReadVolTimes++;
                        
            if((tmp_v > 0.55) || (tmp_v < 0.45)){
                nVolTotal++;
            }
            //applog(LOG_ERR,"nReadVolTimes = %d,nVolTotal = %d",nReadVolTimes,nVolTotal);
            
            if(nVolTotal >= 3){
                applog(LOG_ERR,"Notice chain %d  chip %d maybe has some promble in voltage\n",a1->chain_id,chip_id);
                nVolTotal = 0;
                nReadVolTimes = 0;
                asic_gpio_write(a1->spi_ctx->power_en, 0);
                early_quit(1,"Notice chain %d maybe has some promble in voltage\n",a1->chain_id);
            }
            if(nReadVolTimes == 3){
                nReadVolTimes = 0;
                nVolTotal = 0;
            }
            
   }
    return true;
}


hardware_version_e inno_get_hwver(void)
{
    FILE* fd;
    char buffer[64] = {0};
    hardware_version_e version;
    
    fd = fopen(INNO_HARDWARE_VERSION_FILE, "r");    
    if(fd == NULL)
    {               
        applog(LOG_ERR, "Open hwver File Failed !");
        return -1;
    }

    fread(buffer, 8, 1, fd);
    fclose(fd);

    if(strstr(buffer, "G9") != NULL) {
        version = HARDWARE_VERSION_G9;
        applog(LOG_INFO, "hardware version is G9");
    }else if(strstr(buffer, "G19") != 0) {
        version = HARDWARE_VERSION_G19;
        applog(LOG_INFO, "hardware version is G19");
    }else {
        version = 0;
        applog(LOG_ERR, "unknown hardware version !!!");
    }

    return version;
}


inno_type_e inno_get_miner_type(void)
{
    FILE* fd;
    char buffer[64] = {0};
    inno_type_e miner_type;
    
    fd = fopen(INNO_MINER_TYPE_FILE, "r");  
    if(fd == NULL)
    {               
        applog(LOG_ERR, "Open type File Failed!");
        return -1;
    }

    fread(buffer, 8, 1, fd);
    fclose(fd);

    if(strstr(buffer, "T1") != NULL) {
        miner_type = INNO_TYPE_A5;
        applog(LOG_INFO, "miner type is T1");
    }else if(strstr(buffer, "T2") != NULL) {
        miner_type = INNO_TYPE_A6;
        applog(LOG_INFO, "miner type is T2");
    }else if(strstr(buffer, "T3") != NULL) {
        miner_type = INNO_TYPE_A7;
        applog(LOG_INFO, "miner type is T3");
    }else if(strstr(buffer, "T4") != NULL) {
        miner_type = INNO_TYPE_A8;
        applog(LOG_INFO, "miner type is T4");
    }else {
        miner_type = 0;
        applog(LOG_INFO, "unknown miner type !!!");
    }

    return miner_type;
}

