#include "config.h"

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>

#include "logging.h"
#include "miner.h"
#include "util.h"

#include "asic_inno_cmd.h"
#include "asic_inno_clock.h"

const struct PLL_Clock PLL_Clk_12Mhz[119]={
    {0,  120,   A4_PLL(1,   80, 3)}, //default
    {1,  125,   A4_PLL(1,   83, 3)}, //
    {2,  129,   A4_PLL(1,   86, 3)},
    {3,  140,   A4_PLL(1,   93, 3)},
    {4,  150,   A4_PLL(1,   50, 2)}, //lym open this
    {5,  159,   A4_PLL(1,   53, 2)},
    {6,  171,   A4_PLL(1,   57, 2)},
    {7,  180,   A4_PLL(1,   60, 2)},
    {8,  189,   A4_PLL(1,   63, 2)},
    {9,  201,   A4_PLL(1,   67, 2)},
    {10, 210,   A4_PLL(1,   70, 2)},
    {11, 219,   A4_PLL(1,   73, 2)},
    {12, 231,   A4_PLL(1,   77, 2)},
    {13, 240,   A4_PLL(1,   80, 2)},
    {14, 249,   A4_PLL(1,   83, 2)},
    {15, 261,   A4_PLL(1,   87, 2)},
    {16, 270,   A4_PLL(1,   90, 2)},
    {17, 279,   A4_PLL(1,   93, 2)},
    {18, 291,   A4_PLL(1,   97, 2)},
    {19, 300,   A4_PLL(1,   50, 1)},
    {20, 312,   A4_PLL(1,   52, 1)},
    {21, 318,   A4_PLL(1,   53, 1)},
    {22, 330,   A4_PLL(1,   55, 1)},
    {23, 342,   A4_PLL(1,   57, 1)},
    {24, 348,   A4_PLL(1,   58, 1)},
    {25, 360,   A4_PLL(1,   60, 1)},
    {26, 372,   A4_PLL(1,   62, 1)},
    {27, 378,   A4_PLL(1,   63, 1)},
    {28, 390,   A4_PLL(1,   65, 1)},
    {29, 402,   A4_PLL(1,   67, 1)},
    {30, 408,   A4_PLL(1,   68, 1)},
    {31, 420,   A4_PLL(1,   70, 1)},
    {32, 432,   A4_PLL(1,   72, 1)},
    {33, 438,   A4_PLL(1,   73, 1)},
    {34, 450,   A4_PLL(1,   75, 1)},
    {35, 462,   A4_PLL(1,   77, 1)},
    {36, 468,   A4_PLL(1,   78, 1)},
    {37, 480,   A4_PLL(1,   80, 1)},
    {38, 492,   A4_PLL(1,   82, 1)},
    {39, 498,   A4_PLL(1,   83, 1)},
    {40, 510,   A4_PLL(1,   85, 1)},
    {41, 522,   A4_PLL(1,   87, 1)},
    {42, 528,   A4_PLL(1,   88, 1)},
    {43, 540,   A4_PLL(1,   90, 1)},
    {44, 552,   A4_PLL(1,   92, 1)},
    {45, 558,   A4_PLL(1,   93, 1)},
    {46, 570,   A4_PLL(1,   95, 1)},
    {47, 582,   A4_PLL(1,   97, 1)},
    {48, 588,   A4_PLL(1,   98, 1)},
    {49, 600,   A4_PLL(1,   100, 1)},
    {50, 612,   A4_PLL(1,   102, 1)},
    {51, 624,   A4_PLL(1,   104, 1)},
    {52, 630,   A4_PLL(1,   105, 1)},
    {53, 636,   A4_PLL(1,   106, 1)},
    {54, 648,   A4_PLL(1,   108, 1)},
    {55, 660,   A4_PLL(1,   110, 1)},
    {56, 672,   A4_PLL(1,   112, 1)},
    {57, 684,   A4_PLL(1,   114, 1)},
    {58, 690,   A4_PLL(1,   115, 1)},
    {59, 696,   A4_PLL(1,   116, 1)},
    {60, 708,   A4_PLL(1,   118, 1)},
    {61, 720,   A4_PLL(1,   120, 1)},
    {62, 732,   A4_PLL(1,   122, 1)},
    {63, 744,   A4_PLL(1,   124, 1)},
    {64, 750,   A4_PLL(1,   125, 1)},
    {65, 756,   A4_PLL(1,   126, 1)},
    {66, 768,   A4_PLL(1,   128, 1)},
    {67, 780,   A4_PLL(1,   130, 1)},
    {68, 792,   A4_PLL(1,   132, 1)},
    {69, 804,   A4_PLL(1,   134, 1)},
    {70, 810,   A4_PLL(1,   135, 1)},
    {71, 816,   A4_PLL(1,   136, 1)},
    {72, 828,   A4_PLL(1,   138, 1)},
    {73, 840,   A4_PLL(1,   140, 1)},
    {74, 852,   A4_PLL(1,   142, 1)},
    {75, 864,   A4_PLL(1,   144, 1)},
    {76, 870,   A4_PLL(1,   145, 1)},
    {77, 876,   A4_PLL(1,   146, 1)},
    {78, 888,   A4_PLL(1,   148, 1)},
    {79, 900,   A4_PLL(1,   150, 1)},
    {80, 912,   A4_PLL(1,   152, 1)},
    {81, 924,   A4_PLL(1,   154, 1)},
    {82, 930,   A4_PLL(1,   155, 1)},
    {83, 936,   A4_PLL(1,   156, 1)},
    {84, 948,   A4_PLL(1,   158, 1)},
    {85, 960,   A4_PLL(1,   160, 1)},
    {86, 972,   A4_PLL(1,   162, 1)},
    {87, 984,   A4_PLL(1,   164, 1)},
    {88, 990,   A4_PLL(1,   165, 1)},
    {89, 996,   A4_PLL(1,   166, 1)},
    {90, 1008,  A4_PLL(1,   168, 1)},
    {91, 1020,  A4_PLL(1,   170, 1)},
    {92, 1032,  A4_PLL(1,   172, 1)},
    {93, 1044,  A4_PLL(1,   174, 1)},
    {94, 1050,  A4_PLL(1,   175, 1)},
    {95, 1056,  A4_PLL(1,   176, 1)},
    {96, 1068,  A4_PLL(1,   178, 1)},
    {97, 1080,  A4_PLL(1,   180, 1)},
    {98, 1092,  A4_PLL(1,   182, 1)},
    {99, 1104,  A4_PLL(1,   184, 1)},
    {100,1110,  A4_PLL(1,   185, 1)},
    {101,1116,  A4_PLL(1,   186, 1)},
    {102,1128,  A4_PLL(1,   188, 1)},
    {103,1140,  A4_PLL(1,   190, 1)},
    {104,1152,  A4_PLL(1,   192, 1)},
    {105,1164,  A4_PLL(1,   194, 1)},
    {106,1170,  A4_PLL(1,   195, 1)},
    {107,1176,  A4_PLL(1,   196, 1)},
    {108,1188,  A4_PLL(1,   198, 1)},
    {109,1200,  A4_PLL(1,   200, 1)},
    {110,1212,  A4_PLL(1,   202, 1)},
    {111,1224,  A4_PLL(1,   204, 1)},
    {112,1230,  A4_PLL(1,   205, 1)},
    {113,1236,  A4_PLL(1,   206, 1)},
    {114,1248,  A4_PLL(1,   208, 1)},
    {115,1260,  A4_PLL(1,   210, 1)},
    {116,1272,  A4_PLL(1,   212, 1)},
    {117,1284,  A4_PLL(1,   214, 1)},
    {118,1296,  A4_PLL(1,   216, 1)}
};



int A1_ConfigA1PLLClock(int optPll)
{
    int i;
    int A1Pll;

    if(optPll>0){
        A1Pll=0;
        if(optPll<=PLL_Clk_12Mhz[0].speedMHz) {
            A1Pll=0; //found
        }else{
            for(i=1;i<119;i++){
                if((optPll<PLL_Clk_12Mhz[i].speedMHz)&&(optPll>=PLL_Clk_12Mhz[i-1].speedMHz)){
                    A1Pll=i-1; //found
                    break;
                }
            }
        }
        applog(LOG_NOTICE, "A1 = %d,%d",optPll,A1Pll);
        applog(LOG_NOTICE, "A1 PLL Clock = %dMHz",PLL_Clk_12Mhz[A1Pll].speedMHz);
    }

    return A1Pll;
}


void A1_SetA1PLLClock(struct A1_chain *a1,int pllClkIdx)
{
    uint8_t i;
    struct A1_chip *chip;
    uint32_t regPll;
    uint8_t rxbuf[12];
    
    uint8_t fix_val[8] = {0x00, 0x00, 0x00, 0xA8, 0x00, 0x24, 0xFF, 0xFF}; 

    

    assert(a1->chips != NULL);
    assert((pllClkIdx > 0) && (pllClkIdx < A5_PLL_CLOCK_MAX));

    regPll = PLL_Clk_12Mhz[pllClkIdx].pll_reg;

    chip = &a1->chips[0];
    memcpy(chip->reg,     (uint8_t*)&regPll + 3 ,1);
    memcpy(chip->reg + 1, (uint8_t*)&regPll + 2 ,1);
    memcpy(chip->reg + 2, (uint8_t*)&regPll + 1 ,1);
    memcpy(chip->reg + 3, (uint8_t*)&regPll + 0 ,1);
    memcpy(chip->reg + 4, fix_val , 8);
    
    //chip->reg[6] = (asic_vol_set&0xff00)>>8;
    //chip->reg[7] = asic_vol_set&0x00ff;
    //chip->reg[8] = pllClkIdx;
    //chip->reg[9] = pllClkIdx;
    //applog(LOG_INFO,"pllClkIdx is %d %d", chip->reg[8],chip->reg[9]); 

    inno_cmd_write_reg(a1, ADDR_BROADCAST, chip->reg);
    usleep(100000);
    inno_cmd_read_reg(a1, ADDR_BROADCAST, rxbuf);
    hexdump("read value", rxbuf, 12);   
    
}


