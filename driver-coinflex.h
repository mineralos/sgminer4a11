
/*
 * Copyright 2012-2013 Andrew Smith
 * Copyright 2012 Luke Dashjr
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

//=====================================================================//
//***   driver-coinflex.h is for X11 algorithm mining by using Han-Lab's Pantheon-XXX series miner  ***//
//=====================================================================//

//=====================================================================//
//  DRIVER_COINFLEX DEFINITION FOR X11 ALGORITHM
//  Support Product:
//      1) Pantheon-A   : Altera Stratix V E9 FPGA Chip
//                      : 1 base b'd, 10 miner b'd, 1 miner b'd includes 4EA FPGA Chip
//      2) Pantheon-AFS4    : Altera Stratix IV 530 FPGA Chip
//                      : 2 miner b'd(operating independently), 1 miner b'd includes 10EA FPGA Chip
//      3) Pantheon-CMF1 : Altera Stratix V E9 FPGA Chip
//                      :  1 base b'd, 1 core b'd, 1 core b'd includes 1EA FPGA Chip
//=====================================================================//


#ifndef __DRIVER_COINFLEX_H__
#define __DRIVER_COINFLEX_H__


#include "miner.h"
#include "asic_inno_cmd.h"

#define COINFLEX_DRIVER_NAME                            ("Coinflex")
#define COINFLEX_DRIVER_VER                         ("1.0")
#define COINFLEX_DRIVER_DATE                            ("2017.09.09")

typedef enum
{
    COINFLEX_PRODUCT_TYPE_PANTHEON_A        = 0,
    COINFLEX_PRODUCT_TYPE_PANTHEON_AFS4 = 1,
    COINFLEX_PRODUCT_TYPE_PANTHEON_CMF1 = 2                     // Customized Core 1 of Altera Stratix V
} COINFLEX_PRODUCT_TYPE;


#define COINFLEX_MINER_MAX                              (10)
#define COINFLEX_MINER_ID_ALL                           (0xFF)

#define COINFLEX_SCANTIME_DEF                           (15)                // refer to static bool coinflex_cal_working_time(struct cgpu_info *coinflex)

#define COINFLEX_PRODUCT_NAME_SIZE                  8
#define COINFLEX_SERIAL_NUMBER_SIZE                 8

#define COINFLEX_TEMP_MIN                               (0)
#define COINFLEX_TEMP_MAX                               (100)
#define COINFLEX_TEMP_DEF                               (60)
#define COINFLEX_TEMP_DISP                              (0)
#define COINFLEX_TEMP_RD_CNT                            (5)


// Total: 8Bytes [2Bytes: HA=Han-Lab or CF=Coin-Flex, 2Bytes: FA=FPGA model or AC=ASIC model, 2Bytes: P4=Pantheon-AFS4 or R2=Raptor-II, 2Bytes: 10=Ver 1.0
#define COINFLEX_PRODUCT_NAME_PANTHEON_A            ("Pantheon-A")
#define COINFLEX_SERILA_NUMBER_PANTHEON_A           ("12345678")        // This is the initial stage value
#define COINFLEX_MINER_CORE_NUM_PANTHEON_A          (4)
#define COINFLEX_ALGORITHM_CLOCK_PANTHEON_A     (13)
#define COINFLEX_WORKING_TIME_OFFSET_PANTHEON_A (50)                // msec, - offset
#define COINFLEX_HASH_RATE_OFFSET_PANTHEON_A        (0)             // %, +/- offset
#define COINFLEX_SCANTIME_PANTHEON_A                    (14)                // sec, refer to static bool coinflex_cal_working_time(struct cgpu_info *coinflex) and set it to the scripta's scan-time option

#define COINFLEX_PRODUCT_NAME_PANTHEON_AFS4     ("Pantheon-AFS4")
#define COINFLEX_SERILA_NUMBER_PANTHEON_AFS4        ("HLFAP410")
#define COINFLEX_MINER_CORE_NUM_PANTHEON_AFS4       (10)
#define COINFLEX_ALGORITHM_CLOCK_PANTHEON_AFS4      (25)
#define COINFLEX_WORKING_TIME_OFFSET_PANTHEON_AFS4  (100)       // msec, - offset
#define COINFLEX_HASH_RATE_OFFSET_PANTHEON_AFS4 (0)             // %, +/- offset
#define COINFLEX_SCANTIME_PANTHEON_AFS4             (107)           // sec, refer to static bool coinflex_cal_working_time(struct cgpu_info *coinflex) and set it to the scripta's scan-time option

#define COINFLEX_PRODUCT_NAME_PANTHEON_CMF1     ("Pantheon-CMF1")
#define COINFLEX_SERILA_NUMBER_PANTHEON_CMF1        ("HLFAPC10")
#define COINFLEX_MINER_CORE_NUM_PANTHEON_CMF1       (1)
#define COINFLEX_ALGORITHM_CLOCK_PANTHEON_CMF1      (13)
#define COINFLEX_WORKING_TIME_OFFSET_PANTHEON_CMF1  (100)       // msec, - offset
#define COINFLEX_HASH_RATE_OFFSET_PANTHEON_CMF1 (4)             // %, +/- offset
#define COINFLEX_SCANTIME_PANTHEON_CMF1             (558)           // sec, refer to static bool coinflex_cal_working_time(struct cgpu_info *coinflex) and set it to the scripta's scan-time option

#define COINFLEX_CORE_CLOCK_SET_NUM                 (12)

#define COINFLEX_CORE_CLOCK_MIN                     (50)
#define COINFLEX_CORE_CLOCK_MAX                     (120)
#define COINFLEX_CORE_CLOCK_DEF                     (100)           // coinflex_core_clock[COINFLEX_CORE_CLOCK_SET_NUM] = {50, 60, 70, 80, 85, 90, 95, 98, 100, 105, 110, 120};

#define COINFLEX_CORE_HASHCORE_NUM                  (1)


#define COINFLEX_NONE_NONCE_TEST                        (0)             // 0: Nonce Test,   1: Not Nonce Test before nonce submit in submit_nonce()
#define COINFLEX_TEST_MODE                              (0)             // 0: Normal Mode,  1: Test Mode

/*
struct coinflex_info
{
    char        product[COINFLEX_PRODUCT_NAME_SIZE];    
    char        serial[COINFLEX_SERIAL_NUMBER_SIZE];

    uint16_t    fw_ver;
    uint16_t    hw_ver;

    uint16_t    core_ver;

    uint8_t core_total_cnt;
    uint8_t algo_type;

    uint16_t    miner_sta;
    uint8_t miner_cnt;
    uint8_t miner_core_num;

    char        product_name[20];       // For display

    int     clock;
    int     temp;
    int     temp_max;
    int     settemp;
    int     fan;

    COINFLEX_PRODUCT_TYPE product_type;

    int     algorithm_clock;

    uint32_t    working_time;           // msec

    uint32_t    working_time_offset;    // msec

    int     hashrate_offset;            // %

    struct work *cfxwork[COINFLEX_MINER_MAX];

    int         corenum[COINFLEX_MINER_MAX];
    unsigned char   target[32];

    struct timeval tv_workstart;
    struct timeval tv_temp;
    struct timeval tv_hashrate;
};
*/

extern const int16_t        coinflex_core_clock[COINFLEX_CORE_CLOCK_SET_NUM];

#endif /* __DRIVER_COINFLEX_H__ */
