
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
//***   driver-ltctech.h is for scrypt algorithm mining by using Han-Lab's Raptor-XXX series miner      ***//
//=====================================================================//

//=====================================================================//
//  DRIVER_LTCTECH DEFINITION FOR SCRYPT ALGORITHM
//  Support Product:
//      1) Raptor-I     : BAYSAND Scrypt ASIC Chip
//                      : 1 base b'd, 10 miner b'd, 1 miner b'd includes 4EA ASIC Chip
//      2) Raptor-II        : BAYSAND Scrypt ASIC Chip
//                      : 2 miner b'd(operating independently), 1 miner b'd includes 20EA ASIC Chip
//=====================================================================//


#ifndef __DRIVER_LTCTECH_H__
#define __DRIVER_LTCTECH_H__


#include "miner.h"

#define LTCTECH_DRIVER_NAME                 ("Ltctech")
#define LTCTECH_DRIVER_VER                      ("1.0")
#define LTCTECH_DRIVER_DATE                 ("2016.09.19")

typedef enum
{
    LTCTECH_PRODUCT_TYPE_RAPTOR_I   = 0,
    LTCTECH_PRODUCT_TYPE_RAPTOR_II  = 1
} LTCTECH_PRODUCT_TYPE;


#define LTCTECH_MINER_MAX                       (10)
#define LTCTECH_MINER_ID_ALL                    (0xFF)

#define LTCTECH_SCANTIME_DEF                    (60)

#define LTCTECH_DIFF_DEF                        (8192)

#define LTCTECH_PRODUCT_NAME_SIZE           8
#define LTCTECH_SERIAL_NUMBER_SIZE          8

#define LTCTECH_TEMP_MIN                        (0)
#define LTCTECH_TEMP_MAX                        (100)
#define LTCTECH_TEMP_DEF                        (60)
#define LTCTECH_TEMP_DISP                       (0)
#define LTCTECH_TEMP_RD_CNT                 (5)


// Total: 8Bytes [2Bytes: HA=Han-Lab or CF=Coin-Flex, 2Bytes: FA=FPGA model or AC=ASIC model, 2Bytes: P4=Pantheon-AFS4 or R2=Raptor-II, 2Bytes: 10=Ver 1.1
#define LTCTECH_PRODUCT_NAME_RAPTOR_I       ("Raptor-I")
#define LTCTECH_SERILA_NUMBER_RAPTOR_I      ("12345678")        // This is the initial stage value
#define LTCTECH_MINER_NUM_RAPTOR_I          (10)
#define LTCTECH_MINER_CORE_NUM_RAPTOR_I     (6)             // 6(Real core number is 4): For compatibility with Raptor-I
#define LTCTECH_ALGORITHM_CLOCK_RAPTOR_I        (130)           // It is estimated value: 466MHz / CLOCK = 3.6MH/s
#define LTCTECH_WORKING_TIME_OFFSET_RAPTOR_I    (100)       // msec, - offset
#define LTCTECH_HASH_RATE_OFFSET_RAPTOR_I   (0)             // %, +/- offset
#define LTCTECH_SCANTIME_RAPTOR_I               (30)                // sec and about value, refer to static bool ltctech_cal_working_time(struct cgpu_info *ltctech)
                                                                // Set suitable value to the scripta's scan-time option. For example 28S

#define LTCTECH_PRODUCT_NAME_RAPTOR_II      ("Raptor-II")
#define LTCTECH_SERILA_NUMBER_RAPTOR_II     ("HLACR210")
#define LTCTECH_MINER_NUM_RAPTOR_II         (1)
#define LTCTECH_MINER_CORE_NUM_RAPTOR_II    (20)
#define LTCTECH_ALGORITHM_CLOCK_RAPTOR_II   (130)           // It is estimated value: 466MHz / CLOCK = 3.6MH/s
#define LTCTECH_WORKING_TIME_OFFSET_RAPTOR_II   (100)       // msec, - offset
#define LTCTECH_HASH_RATE_OFFSET_RAPTOR_II  (0)             // %, +/- offset
#define LTCTECH_SCANTIME_RAPTOR_II          (60)                // sec and about value, refer to static bool ltctech_cal_working_time(struct cgpu_info *ltctech)
                                                                // Set suitable value to the scripta's scan-time option. For example, 57S

#define LTCTECH_CORE_CLOCK_SET_NUM          (11)                //ltctech_core_clock[LTCTECH_CORE_CLOCK_SET_NUM] = { 400, 412, 425, 433, 437, 450, 462, 466, 483, 487, 500 };

#define LTCTECH_CORE_CLOCK_MIN              (400)
#define LTCTECH_CORE_CLOCK_MAX              (500)
#define LTCTECH_CORE_CLOCK_DEF              (466)

#define LTCTECH_CORE_HASHCORE_NUM           (4)

#define LTCTECH_NONE_NONCE_TEST             (0)             // 0: Nonce Test,   1: Not Nonce Test before nonce submit in submit_nonce()
#define LTCTECH_TEST_MODE                       (0)             // 0: Normal Mode,  1: Test Mode

struct core_info
{
    uint8_t     hash_core[LTCTECH_CORE_HASHCORE_NUM]; 
    uint32_t    err_count;
    uint32_t    reset_count;
    uint32_t    total_nonce;
    uint8_t     clk;
    uint8_t     status;
}; 

struct miner_info_raptor_i
{
    uint16_t        version;
    uint8_t         core_cnt;
    int8_t          temperature;
    uint32_t        err_count;
    uint32_t        total_nonce;
    uint32_t        reset_count;
    uint32_t        retry;
    struct core_info cores[LTCTECH_MINER_CORE_NUM_RAPTOR_I]; 
    uint32_t        status;
};

struct core_info_raptor_ii
{
    uint8_t status;
    uint8_t clk;
    uint16_t    dummy;
    uint8_t hash_core[LTCTECH_CORE_HASHCORE_NUM]; 
}; 

struct miner_info_raptor_ii
{
    uint32_t    status;
    struct core_info_raptor_ii cores[LTCTECH_MINER_CORE_NUM_RAPTOR_II];
};

struct ltctech_info
{
    char        product[LTCTECH_PRODUCT_NAME_SIZE]; 
    char        serial[LTCTECH_SERIAL_NUMBER_SIZE];
    
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

    double working_diff;

    pthread_t *getwork_thr;

    LTCTECH_PRODUCT_TYPE product_type;

    int     algorithm_clock;

    uint32_t    working_time;           // msec

    uint32_t    working_time_offset;    // msec

    int     hashrate_offset;            // %

    struct cgpu_info *ltcgpu[LTCTECH_MINER_MAX];
    struct work *ltcwork[LTCTECH_MINER_MAX];
    int corenum[LTCTECH_MINER_MAX];
    
    struct timeval tv_workstart;
    struct timeval tv_update; 
    struct timeval tv_hashrate;
    
    struct miner_info_raptor_i  miner_raptor_i[LTCTECH_MINER_NUM_RAPTOR_I];

    struct miner_info_raptor_ii miner_raptor_ii[LTCTECH_MINER_NUM_RAPTOR_II];
};

extern const int16_t        ltctech_core_clock[LTCTECH_CORE_CLOCK_SET_NUM];

#endif /* __DRIVER_LTCTECH_H__ */
