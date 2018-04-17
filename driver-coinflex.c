
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
//***   driver-coinflex.c is for X11 algorithm mining by using Han-Lab's Pantheon-XXX series miner      ***//
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


#include "config.h"

#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <error.h>

#include "logging.h"
#include "miner.h"
//#include "usbutils.h"
#include "util.h"
#include "driver-coinflex.h"
#include "compat.h"


#include "spi-context.h"
#include "logging.h"
#include "miner.h"
#include "util.h"

#include "A1-board-selector.h"
#include "A1-trimpot-mcp4x.h"

#include "asic_b29.h"
#include "asic_b29_clock.h"
#include "asic_b29_cmd.h"
#include "asic_b29_gpio.h"


#define WORK_SIZE               (80)
#define DEVICE_TARGET_SIZE      (32)
#define TARGET_POS              (80)
#define TARGET_SIZE             (4)
#define MINER_ID_POS            (84)
#define MINER_ID_SIZE           (1)
#define WORK_ID_POS             (85)
#define WORK_ID_SIZE            (1)
#define FIND_NONCE_SIZE         (6)             // For receive value from miner: 4-Bytes nonce, 1-Byte miner_id, 1-Byte work_id

#define REPLY_SIZE              (2)
#define BUF_SIZE                (128)
//#define TEMP_UPDATE_INT_MS  10000
#define CHECK_DISABLE_TIME  0

static int ret_pll[ASIC_CHAIN_NUM] = {0};

struct Test_bench Test_bench_Array[5]={
    {1100,  0,  0,  0},
    {1100,  0,  0,  0},
    {1100,  0,  0,  0},
    {1100,  0,  0,  0},
    {1100,  0,  0,  0},
};


struct A1_chain *chain[ASIC_CHAIN_NUM];

uint8_t A1Pll1=A5_PLL_CLOCK_400MHz;
static uint8_t A1Pll2=A5_PLL_CLOCK_400MHz;
static uint8_t A1Pll3=A5_PLL_CLOCK_400MHz;
static uint8_t A1Pll4=A5_PLL_CLOCK_400MHz;
static uint8_t A1Pll5=A5_PLL_CLOCK_400MHz;
static uint8_t A1Pll6=A5_PLL_CLOCK_400MHz;

static uint32_t show_log[ASIC_CHAIN_NUM];
static uint32_t update_temp[ASIC_CHAIN_NUM];
static uint32_t check_disbale_flag[ASIC_CHAIN_NUM];

int spi_plug_status[ASIC_CHAIN_NUM] = {0};

char szShowLog[ASIC_CHAIN_NUM][ASIC_CHIP_NUM][256] = {0};
char volShowLog[ASIC_CHAIN_NUM][256] = {0};

hardware_version_e g_hwver;
b29_type_e g_type;
int g_reset_delay = 0xffff;
int g_miner_state = 0;
int chain_flag[ASIC_CHAIN_NUM] = {0};
b29_reg_ctrl_t s_reg_ctrl;

/* added by yex in 20170907 */
/*
 * if not cooled sufficiently, communication fails and chip is temporary
 * disabled. we let it inactive for 30 seconds to cool down
 *
 * TODO: to be removed after bring up / test phase
 */

/*
 * for now, we have one global config, defaulting values:
 * - ref_clk 16MHz / sys_clk 800MHz
 * - 2000 kHz SPI clock
 */
struct A1_config_options A1_config_options = {
    .ref_clk_khz = 16000, .sys_clk_khz = 800000, .spi_clk_khz = 2000,
};

/* override values with --bitmine-a1-options ref:sys:spi: - use 0 for default */
static struct A1_config_options *parsed_config_options;


#if COINFLEX_TEST_MODE
static void coinflex_set_testdata(struct work *work);
static void coinflex_print_hash(struct work *work, uint32_t nonce);
#endif

//static void coinflex_print_hw_error(char *drv_name, int device_id, struct work *work, uint32_t nonce);
//static bool coinflex_set_algorithm(struct cgpu_info *coinflex);

/********** work queue */
static bool wq_enqueue(struct work_queue *wq, struct work *work)
{
    if (work == NULL)
        return false;
    struct work_ent *we = malloc(sizeof(*we));
    assert(we != NULL);

    we->work = work;
    INIT_LIST_HEAD(&we->head);
    list_add_tail(&we->head, &wq->head);
    wq->num_elems++;
    return true;
}

static struct work *wq_dequeue(struct work_queue *wq)
{
    if (wq == NULL)
        return NULL;
    if (wq->num_elems == 0)
        return NULL;
    struct work_ent *we;
    we = list_entry(wq->head.next, struct work_ent, head);
    struct work *work = we->work;

    list_del(&we->head);
    free(we);
    wq->num_elems--;
    return work;
}


/* queue two work items per chip in chain */
static bool coinflex_queue_full(struct cgpu_info *cgpu)
{
    struct A1_chain *a1 = cgpu->device_data;
    int queue_full = false;

    mutex_lock(&a1->lock);
    //  applog(LOG_NOTICE, "%d, A1 running queue_full: %d/%d",
    //     a1->chain_id, a1->active_wq.num_elems, a1->num_active_chips);

    if (a1->active_wq.num_elems >= a1->num_active_chips * 2)
        queue_full = true;
    else
        wq_enqueue(&a1->active_wq, get_queued(cgpu));

    mutex_unlock(&a1->lock);

    return queue_full;
}

void exit_A1_chain(struct A1_chain *a1)
{
    if (a1 == NULL){
        return;
    }
    free(a1->chips);

    mcompat_chain_power_down(a1->chain_id);

    a1->chips = NULL;
    free(a1);
}

int  cfg_tsadc_divider(struct A1_chain *a1,uint32_t pll_clk)
{
    // uint8_t  cmd_return;
    uint32_t tsadc_divider_tmp;
    uint8_t  tsadc_divider;
    //cmd0d(0x0d00, 0x0250, 0xa006 | (BYPASS_AUXPLL<<6), 0x2800 | tsadc_divider, 0x0300, 0x0000, 0x0000, 0x0000)
    uint8_t  buffer[] = {0x02,0x50,0xa0,0x06,0x28,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
    uint8_t  readbuf[32] = {0};
#ifdef MPW
    tsadc_divider_tmp = (pll_clk/2)*1000/256/650;
#else
    tsadc_divider_tmp = (pll_clk/2)*1000/16/650;
#endif
    tsadc_divider = (uint8_t)(tsadc_divider_tmp & 0xff);

    buffer[5] = 0x00 | tsadc_divider;

    if(!mcompat_cmd_read_write_reg0d(a1->chain_id, ADDR_BROADCAST, buffer, REG_LENGTH, readbuf)){
        applog(LOG_WARNING, "#####Write t/v sensor Value Failed!\n");
    }else{
        applog(LOG_WARNING, "#####Write t/v sensor Value Success!\n");
    }
    return 0;
}

void chain_detect_reload(struct A1_chain *a1)
{
    int cid = a1->chain_id;

    int n_chips = mcompat_cmd_bist_start(cid, ADDR_BROADCAST);
    if(likely(n_chips > 0) && likely(n_chips != 0xff)){
        a1->num_chips = n_chips;
    }

    applog(LOG_INFO, "[reload]%d: detected %d chips", cid, a1->num_chips);

    usleep(10000);

    if(!mcompat_cmd_bist_collect(cid, ADDR_BROADCAST)){
        applog(LOG_NOTICE, "[reload]bist collect fail");
        return ;
    }

    applog(LOG_NOTICE, "collect core success");
    applog(LOG_NOTICE, "%d:  A1 chip-chain detected", cid);
}

bool init_A1_chain_reload(struct A1_chain *a1, int chain_id)
{
    int i;
    uint8_t src_reg[REG_LENGTH] = {0};
    uint8_t reg[REG_LENGTH] = {0};
    uint8_t buffer[4] = {0};
    bool result;

    if(a1 == NULL){
        applog(LOG_INFO, "%d: chain not plugged", chain_id);
        return false;
    }

    applog(LOG_INFO, "%d: A1 init chain reload", chain_id);

    //    result = mcompat_cmd_resetbist(a1->chain_id, ADDR_BROADCAST, buffer);
    //applog(LOG_INFO, "mcompat_cmd_resetbist(): %d - %02X", result, buffer[0]);
    //    sleep(1);

    //bist mask
    //    mcompat_cmd_read_register(a1->chain_id, 0x01, reg, REG_LENGTH);
    //    memcpy(src_reg, reg, REG_LENGTH);
    //    src_reg[7] = src_reg[7] | 0x10;
    //    mcompat_cmd_write_register(a1->chain_id, ADDR_BROADCAST, src_reg, REG_LENGTH);
    //    usleep(200);

    mcompat_set_spi_speed(a1->chain_id, 4);    // 4: 6250000
    usleep(100000);

    chain_detect_reload(a1);
    usleep(10000);

    if (a1->num_chips < 1){
        goto failure;
    }

    /* override max number of active chips if requested */
    a1->num_active_chips = a1->num_chips;
    if ((A1_config_options.override_chip_num > 0) && a1->num_chips > A1_config_options.override_chip_num){
        a1->num_active_chips = A1_config_options.override_chip_num;
        applog(LOG_WARNING, "%d: limiting chain to %d chips",a1->chain_id, a1->num_active_chips);
    }

    a1->chips = calloc(a1->num_active_chips, sizeof(struct A1_chip));
    assert (a1->chips != NULL);

    if (!mcompat_cmd_bist_fix(a1->chain_id, ADDR_BROADCAST)){
        goto failure;
    }

    usleep(200);

    //configure for vsensor
    b29_configure_tvsensor(a1,ADDR_BROADCAST,0);
    for (i = 0; i < a1->num_active_chips; i++){
        b29_check_voltage(a1, i+1, &s_reg_ctrl);
    } 

    //configure for tsensor
    b29_configure_tvsensor(a1,ADDR_BROADCAST,1);
    b29_get_voltage_stats(a1, &s_reg_ctrl);
    sprintf(volShowLog[a1->chain_id], "+         %2d  |  %8f  |  %8f  |  %8f  |\n",a1->chain_id,   \
            s_reg_ctrl.highest_vol[a1->chain_id],s_reg_ctrl.average_vol[a1->chain_id],s_reg_ctrl.lowest_vol[a1->chain_id]);

    b29_log_record(a1->chain_id, volShowLog[a1->chain_id], strlen(volShowLog[0]));

    for (i = 0; i < a1->num_active_chips; i++){
        check_chip(a1, i);
    }


    applog(LOG_ERR, "[chain_ID:%d]: Found %d Chips With Total %d Active Cores",a1->chain_id, a1->num_active_chips, a1->num_cores);

    return true;

failure:
    exit_A1_chain(a1);
    return false;
}

struct A1_chain *init_A1_chain(int chain_id)
{
    //int i;
    struct A1_chain *a1 = malloc(sizeof(*a1)); 
    assert(a1 != NULL);
    memset(a1,0,sizeof(struct A1_chain));

    applog(LOG_INFO, "%d: A1 init chain", chain_id);

    memset(a1, 0, sizeof(*a1));
    a1->chain_id = chain_id;

    a1->num_chips = chain_detect(a1);
    if (a1->num_chips < 1){
        applog(LOG_ERR, "bist start fail");
        goto failure;
    }
    applog(LOG_INFO, "%d: detected %d chips", a1->chain_id, a1->num_chips);

    usleep(100000);
    //sleep(10);
    cfg_tsadc_divider(a1, CHIP_PLL_DEF);// PLL_Clk_12Mhz[A1Pll1].speedMHz);	

    /* override max number of active chips if requested */
    a1->num_active_chips = a1->num_chips;
    if ((A1_config_options.override_chip_num > 0) && a1->num_chips > A1_config_options.override_chip_num){
        a1->num_active_chips = A1_config_options.override_chip_num;
        applog(LOG_WARNING, "%d: limiting chain to %d chips",a1->chain_id, a1->num_active_chips);
    }

    a1->chips = calloc(a1->num_active_chips, sizeof(struct A1_chip));
    assert (a1->chips != NULL);

    usleep(200000);

    applog(LOG_WARNING, "[chain_ID:%d]: Found %d Chips",a1->chain_id, a1->num_active_chips);

    mutex_init(&a1->lock);
    INIT_LIST_HEAD(&a1->active_wq.head);

    return a1;

failure:
    exit_A1_chain(a1);
    return NULL;
}


int b29_preinit( uint32_t pll, uint32_t last_pll)
{ 
    int i;
    static int ret[ASIC_CHAIN_NUM]={0};
    int rep_cnt = 0;
    memset(ret,-1,sizeof(ret));

    for(i=0; i<ASIC_CHAIN_NUM; i++)
    {
        if((chain[i] == NULL)||(!chain_flag[i])){
            ret[i] = -2;
            continue;
        }

        while(prechain_detect(chain[i], A1_ConfigA1PLLClock(pll),A1_ConfigA1PLLClock(last_pll)))
        {

            if(( rep_cnt <= 5) )
            {
                rep_cnt++;
                sleep(5);

            }else{
                ret_pll[i] = -1;
                goto out_cfgpll;
            }
        }
    }

out_cfgpll:

    return 0;
}

static int inc_pll(void)
{
    uint32_t i = 0,j = 0; 
    static uint32_t last_pll;

    applog(LOG_ERR, "pre init_pll...");
    
    for(i = PLL_Clk_12Mhz[0].speedMHz; i < (opt_A1Pll1+200); i+=200)
    {      
        i = (i >= opt_A1Pll1 ? opt_A1Pll1 : i);

        b29_preinit(i,last_pll);
        last_pll = i;

        for(j=0; j<ASIC_CHAIN_NUM; j++)
        {
            if (-1 == ret_pll[j]){
                mcompat_chain_power_down(j);
            }
            else
            {
                applog(LOG_ERR,"chain %d pll %d finished\n", j, i);
            }
        }
    }

    return 0;
}

static void recfg_vid()
{
    int i, j;

    for(i = 0; i < ASIC_CHAIN_NUM; i++)
    {
        if((chain[i] == NULL) || (!chain_flag[i]))
        {
            continue;
        }
        // get chain vid
        chain[i]->vid = CHIP_VID_DEF;
        if(g_hwver == HARDWARE_VERSION_G9)
        {
            chain[i]->vid = opt_voltage1;
        }
        else if(g_hwver == HARDWARE_VERSION_G19)
        {
            switch(i)
            {
                case 0: chain[i]->vid = opt_voltage1; break;
                case 1: chain[i]->vid = opt_voltage2; break;
                case 2: chain[i]->vid = opt_voltage3; break;
                case 3: chain[i]->vid = opt_voltage4; break;
                case 4: chain[i]->vid = opt_voltage5; break;
                case 5: chain[i]->vid = opt_voltage6; break;
                case 6: chain[i]->vid = opt_voltage7; break;
                case 7: chain[i]->vid = opt_voltage8; break;
            }
        }
        // set vid step by step
        if(opt_voltage1 > CHIP_VID_DEF)
        {
            for(j = CHIP_VID_DEF + 1; j <= opt_voltage1; j++)
            {
                applog(LOG_ERR,"mcompat_set_vid(chain=%d, vid=%d)\n", i, j);
                mcompat_set_vid(i, j);
                usleep(500000);
            }
        }
        else if(opt_voltage1 < CHIP_VID_DEF)
        {
            for(j = CHIP_VID_DEF - 1; j >= opt_voltage1; j--)
            {
                applog(LOG_ERR,"mcompat_set_vid(chain=%d, vid=%d)\n", i, j);
                mcompat_set_vid(i, j);
                usleep(500000);
            }
        }
    }
}

static bool detect_A1_chain(void)
{
    int i,ret,res = 0;
    uint8_t buffer[4] = {0};

    for(i = 0; i < ASIC_CHAIN_NUM; i++){    
        if(mcompat_get_plug(i) != 0)
        {
            applog(LOG_ERR, "chain %d power on fail", i);
            chain[i] = NULL;
            chain_flag[i] = 0;
            continue;
        }

        mcompat_set_vid(i, CHIP_VID_DEF);   // init vid
        mcompat_set_spi_speed(i, 0);        // init spi speped 0: 400K
        usleep(10000);

        chain[i] = init_A1_chain(i);
        if (chain[i] == NULL){
            applog(LOG_ERR, "init chain %d fail", i);
            chain_flag[i] = 0;
            continue;
        } else {
            res++;
            chain_flag[i] = 1;
        }
        if(!mcompat_cmd_resetall(i, ADDR_BROADCAST, buffer))
        {
            applog(LOG_ERR, "failed to reset chain %d!", i);
        }
        applog(LOG_WARNING, "Detected the %d A1 chain with %d chips",i, chain[i]->num_active_chips);
    }

    usleep(200000);

    inc_pll();
    recfg_vid();

    for(i = 0; i < ASIC_CHAIN_NUM; i++){
        ret = init_A1_chain_reload(chain[i], i);
        if (false == ret){
            applog(LOG_ERR, "reload init a1 chain%d fail",i);
            continue;
        }

        struct cgpu_info *cgpu = malloc(sizeof(*cgpu));
        assert(cgpu != NULL);

        memset(cgpu, 0, sizeof(*cgpu));
        cgpu->drv = &coinflex_drv;
        cgpu->name = "BitmineA1.SingleChain";
        cgpu->threads = 1;
        cgpu->chainNum = i;
        
        cgpu->device_data = chain[i];
        if ((chain[i]->num_chips <= MAX_CHIP_NUM) && (chain[i]->num_cores <= MAX_CORES)){
                    cgpu->mhs_av = (double)(opt_A1Pll1 *  (chain[i]->num_cores) / 2);
        }else{
            cgpu->mhs_av = 0;
            chain_flag[i] = 0;
        }

        chain[i]->cgpu = cgpu;
        add_cgpu(cgpu);

        // led on
        mcompat_set_led(chain[i]->chain_id, 0);

        applog(LOG_WARNING, "Detected the %d A1 chain with %d chips / %d cores",i, chain[i]->num_active_chips, chain[i]->num_cores);
    }


    return (res == 0) ? false : true;
}

static void config_fan_module()
{
    /* val temp_f 
       652, //-40,
       645, //-35,
       638, //-30,
       631, //-25,
       623, //-20,
       616, //-15, 
       609, //-10, 
       601, // -5,
       594, //  0,
       587, //  5,
       579, // 10,
       572, // 15,  
       564, // 20, 
       557, // 25, 
       550, // 30,
       542, // 35,
       535, // 40,
       527, // 45,
       520, // 50,
       512, // 55,
       505, // 60,
       498, // 65,
       490, // 70,
       483, // 75,
       475, // 80,
       468, // 85,
       460, // 90,
       453, // 95,
       445, //100,
       438, //105,
       430, //110,
       423, //115,
       415, //120,
       408, //125,
       };
       */

mcompat_temp_config_s temp_config;
temp_config.temp_hi_thr = 408;
temp_config.temp_lo_thr = 652;
temp_config.temp_start_thr = 550;
temp_config.dangerous_stat_temp = 445;
temp_config.work_temp = 483;
temp_config.default_fan_speed = 100;
mcompat_fan_temp_init(0,temp_config);

}

static void coinflex_detect(bool __maybe_unused hotplug)
{
    if (hotplug){
        return;
    }

    struct timeval test_tv;
    int j = 0;

    /* parse bimine-a1-options */
    if ((opt_bitmine_a1_options != NULL) && (parsed_config_options == NULL)) {
        int ref_clk = 0;
        int sys_clk = 0;
        int spi_clk = 0;
        int override_chip_num = 0;
        int wiper = 0;

        sscanf(opt_bitmine_a1_options, "%d:%d:%d:%d:%d",&ref_clk, &sys_clk, &spi_clk, &override_chip_num,&wiper);
        if (ref_clk != 0){
            A1_config_options.ref_clk_khz = ref_clk;
        }
        if (sys_clk != 0) {
            if (sys_clk < 100000){
                quit(1, "system clock must be above 100MHz");
            }
            A1_config_options.sys_clk_khz = sys_clk;
        }
        if (spi_clk != 0){
            A1_config_options.spi_clk_khz = spi_clk;
        }
        if (override_chip_num != 0){
            A1_config_options.override_chip_num = override_chip_num;
        }
        if (wiper != 0){
            A1_config_options.wiper = wiper;
        }

        /* config options are global, scan them once */
        parsed_config_options = &A1_config_options;
    }
    applog(LOG_DEBUG, "A1 detect");

    g_hwver = b29_get_hwver();
    g_type = b29_get_miner_type();

    // TODO: 根据接口获取hwver和type
    sys_platform_init(PLATFORM_ZYNQ_HUB_G19, -1, ASIC_CHAIN_NUM, ASIC_CHIP_NUM);
    memset(&s_reg_ctrl,0,sizeof(s_reg_ctrl));
    sys_platform_debug_init(3);
    config_fan_module();


    // update time
    for(j = 0; j < 64; j++)
    {
        cgtime(&test_tv);
        if(test_tv.tv_sec > 1000000000)
        {
            break;
        }

        usleep(500000);
    }

    // chain poweron & reset
    mcompat_chain_power_down_all();
    sleep(5);
    mcompat_chain_power_on_all();

    if(detect_A1_chain()){
        return ;
    }

    applog(LOG_WARNING, "A1 dectect finish");

}


static void coinflex_get_statline_before(char *buf, size_t bufsiz, struct cgpu_info *coinflex)
{
    struct A1_chain *a1 = coinflex->device_data;
    char temp[10];
    if (a1->temp != 0)
        snprintf(temp, 9, "%2dC", a1->temp);
    tailsprintf(buf, bufsiz, " %2d:%2d/%3d %s",
            a1->chain_id, a1->num_active_chips, a1->num_cores,
            a1->temp == 0 ? "   " : temp);
}



static void coinflex_flush_work(struct cgpu_info *coinflex)
{
    struct A1_chain *a1 = coinflex->device_data;
    int cid = a1->chain_id;
    //board_selector->select(cid);
    int i;
    uint8_t buffer[4] = {0};

    mutex_lock(&a1->lock);
    /* stop chips hashing current work */
  //  if (!abort_work(a1)) 
   // {
  //      applog(LOG_ERR, "%d: failed to abort work in chip chain!", cid);
  //  }
    /* flush the work chips were currently hashing */
    for (i = 0; i < a1->num_active_chips; i++) 
    {
        int j;
        struct A1_chip *chip = &a1->chips[i];
        for (j = 0; j < 4; j++) 
        {
            struct work *work = chip->work[j];
            if (work == NULL)
                continue;
            //applog(LOG_DEBUG, "%d: flushing chip %d, work %d: 0x%p",
            //       cid, i, j + 1, work);
            work_completed(coinflex, work);
            chip->work[j] = NULL;
        }

        chip->last_queued_id = 0;

       // if(!mcompat_cmd_resetjob(a1->chain_id, i+1, buffer))
       // {
      //      applog(LOG_WARNING, "chip %d clear work failed", i);
      //      continue;
      //  }

        //applog(LOG_INFO, "chip :%d flushing queued work success", i);
    }
    /* flush queued work */
    //applog(LOG_DEBUG, "%d: flushing queued work...", cid);
    while (a1->active_wq.num_elems > 0) 
    {
        struct work *work = wq_dequeue(&a1->active_wq);
        assert(work != NULL);
        work_completed(coinflex, work);
    }
    mutex_unlock(&a1->lock);

}


#define VOLTAGE_UPDATE_INT  6000
//#define  LOG_FILE_PREFIX "/home/www/conf/analys"
#define  LOG_FILE_PREFIX "/tmp/log/analys"
#define  LOG_VOL_PREFIX "/tmp/log/volAnalys"


const char cLevelError1[3] = "!";
const char cLevelError2[3] = "#";
const char cLevelError3[3] = "$";
const char cLevelError4[3] = "%";
const char cLevelError5[3] = "*";
const char cLevelNormal[3] = "+";

void B29_Log_Save(struct A1_chip *chip,int nChip,int nChain)
{
    char szInNormal[8] = {0};
    memset(szInNormal,0, sizeof(szInNormal));
    if(chip->hw_errors > 0){
        strcat(szInNormal,cLevelError1);
    }
    if(chip->stales > 0){
        strcat(szInNormal,cLevelError2);
    }
    if((chip->temp > 564) || (chip->temp < 445)){
        strcat(szInNormal,cLevelError3);
    }
    if(chip->num_cores < 8){
        strcat(szInNormal,cLevelError4);
    }
    if((chip->nVol > 550) || (chip->nVol < 450)){
        strcat(szInNormal,cLevelError5);
    }

    if((chip->hw_errors == 0) && (chip->stales == 0) && ((chip->temp < 564) && (chip->temp > 445)) &&((chip->nVol < 550) && (chip->nVol > 450)) && (chip->num_cores == 8)){
        strcat(szInNormal,cLevelNormal);
    }

    sprintf(szShowLog[nChain][nChip], "\n%-8s|%32d|%8d|%8d|%8d|%8d|%8d|%8d|%8d",szInNormal,chip->nonces_found,
            chip->hw_errors, chip->stales,chip->temp,chip->nVol,chip->num_cores,nChip,nChain);
}

void b29_log_print(int cid, void* log, int len)
{
    FILE* fd;
    char fileName[128] = {0};

    sprintf(fileName, "%s%d.log", LOG_FILE_PREFIX, cid);

    fd = fopen(fileName, "w+"); 

    if(fd == NULL){             
        //applog(LOG_ERR, "Open log File%d Failed!%d", cid, errno);
        applog(LOG_ERR, "Open log File%d Failed!%s", cid, strerror(errno));
        return; 
    }

    fwrite(log, len, 1, fd);
    fflush(fd);
    fclose(fd);
}

void b29_log_record(int cid, void* log, int len)
{
    FILE* fd;
    char fileName[128] = {0};

    sprintf(fileName, "%s%d.log", LOG_VOL_PREFIX, cid);
    fd = fopen(fileName, "w+"); 
    if(fd == NULL){             
        //applog(LOG_ERR, "Open log File%d Failed!%d", cid, errno);
        applog(LOG_ERR, "Open log File%d Failed!%s", cid, strerror(errno));
        return; 
    }

    fwrite(log, len, 1, fd);
    fflush(fd);
    fclose(fd);
}

volatile int g_nonce_read_err = 0;

static int64_t coinflex_scanwork(struct thr_info *thr)
{
    int i;
    uint8_t reg[128];
    struct cgpu_info *cgpu = thr->cgpu;
    struct A1_chain *a1 = cgpu->device_data;
    int32_t nonce_ranges_processed = 0;

    uint32_t nonce;
    uint8_t chip_id;
    uint8_t job_id;
    bool work_updated = false;

    if (a1->num_cores == 0) {
        cgpu->deven = DEV_DISABLED;
        return 0;
    }

    mutex_lock(&a1->lock);
    int cid = a1->chain_id;

    if (a1->last_temp_time + TEMP_UPDATE_INT_MS < get_current_ms())
    {


        hub_cmd_get_temp(fan_temp_ctrl,cid);
        update_temp[cid]++;
        show_log[cid]++;
        check_disbale_flag[cid]++;

        if(fan_temp_ctrl->mcompat_temp[cid].final_temp_avg && fan_temp_ctrl->mcompat_temp[cid].final_temp_hi && fan_temp_ctrl->mcompat_temp[cid].final_temp_lo)
        {
            cgpu->temp = (double)((594 - fan_temp_ctrl->mcompat_temp[cid].final_temp_avg)* 5) / 7.5;
            cgpu->temp_max = (double)((594 - fan_temp_ctrl->mcompat_temp[cid].final_temp_hi)* 5) / 7.5;
            cgpu->temp_min = (double)((594 - fan_temp_ctrl->mcompat_temp[cid].final_temp_lo)* 5) / 7.5;
        }

        cgpu->fan_duty = fan_temp_ctrl->speed;
        cgpu->chip_num = a1->num_active_chips;
        cgpu->core_num = a1->num_cores;

        //printf("volShowLog[%d]=%s",cid,volShowLog[cid]);

        b29_log_print(cid, szShowLog[cid], sizeof(szShowLog[0]));

        a1->last_temp_time = get_current_ms();
    }

    /* poll queued results */
    while (true){
        if (!get_nonce(a1, (uint8_t*)&nonce, &chip_id, &job_id)){
            break;
        }

        work_updated = true;

        if (job_id < 1 || job_id > 4){
            applog(LOG_WARNING, "%d: chip %d: result has wrong ""job_id %d", cid, chip_id, job_id);
            continue;
        }

        struct A1_chip *chip = &a1->chips[chip_id - 1];
        struct work *work = chip->work[job_id - 1];
        if (work == NULL){
            /* already been flushed => stale */
            applog(LOG_WARNING, "%d: chip %d: stale nonce 0x%08x", cid, chip_id, nonce);
            chip->stales++;
            continue;
        }
        if (!submit_nonce(thr, work, nonce)){
            applog(LOG_WARNING, "%d: chip %d: invalid nonce 0x%08x", cid, chip_id, nonce);
            chip->hw_errors++;
            /* add a penalty of a full nonce range on HW errors */
            nonce_ranges_processed--;
            continue;
        }

        applog(LOG_INFO, "Got nonce for chain %d / chip %d / job_id %d", a1->chain_id, chip_id, job_id);

        chip->nonces_found++;
    }

#ifdef USE_AUTONONCE
    mcompat_cmd_auto_nonce(a1->chain_id, 0, REG_LENGTH);   // disable autononce
#endif

    /* check for completed works */
    if(a1->work_start_delay > 0)
    {
        applog(LOG_INFO, "wait for pll stable");
        a1->work_start_delay--;
    }
    else
    {
       // mcompat_cmd_reset_reg(cid);
        for (i = a1->num_active_chips; i > 0; i--)
        {
            if(mcompat_cmd_read_register(a1->chain_id, i, reg, REG_LENGTH))
            {
              struct A1_chip *chip = NULL;
              struct work *work = NULL;

              uint8_t qstate = reg[9] & 0x03;
              if (qstate != 0x03)
              {
                work_updated = true;
                if(qstate == 0x0){
                  chip = &a1->chips[i - 1];
                  work = wq_dequeue(&a1->active_wq);

                  if (work == NULL){
                      continue;
                  }

                  if (set_work(a1, i, work, 0))
                  {
                      nonce_ranges_processed++;
                      chip->nonce_ranges_done++;
                  }
                }

                  chip = &a1->chips[i - 1];
                  work = wq_dequeue(&a1->active_wq);

                  if (work == NULL){
                      continue;
                  }

                  if (set_work(a1, i, work, 0))
                  {
                      nonce_ranges_processed++;
                      chip->nonce_ranges_done++;
                  }
               }
            }
         } 
    }


#ifdef USE_AUTONONCE
    mcompat_cmd_auto_nonce(a1->chain_id, 1, REG_LENGTH);   // enable autononce
#endif
    mutex_unlock(&a1->lock);

    if (nonce_ranges_processed < 0){
        applog(LOG_INFO, "nonce_ranges_processed less than 0");
        nonce_ranges_processed = 0;
    }else{
        applog(LOG_DEBUG, "%d, nonces processed %d", cid, nonce_ranges_processed);
    }

    cgtime(&a1->tvScryptCurr);
    timersub(&a1->tvScryptCurr, &a1->tvScryptLast, &a1->tvScryptDiff);
    cgtime(&a1->tvScryptLast);

    /* in case of no progress, prevent busy looping */
    if (!work_updated) // after work updated, also delay 10ms
        cgsleep_ms(5);

    return ((double)opt_A1Pll1*a1->tvScryptDiff.tv_usec /2) * (a1->num_cores);
    }



static struct api_data *coinflex_api_stats(struct cgpu_info *cgpu)
{
    struct A1_chain *t1 = cgpu->device_data;
    unsigned long long int chipmap = 0;
    struct api_data *root = NULL;
    char s[32];
    int i;

    ROOT_ADD_API(int, "Chain ID", t1->chain_id, false);
    ROOT_ADD_API(int, "Num chips", t1->num_chips, false);
    ROOT_ADD_API(int, "Num cores", t1->num_cores, false);
    ROOT_ADD_API(int, "Num active chips", t1->num_active_chips, false);
    ROOT_ADD_API(int, "Chain skew", t1->chain_skew, false);
    ROOT_ADD_API(double, "Temp max", cgpu->temp_max, false);
    ROOT_ADD_API(double, "Temp min", cgpu->temp_min, false);
   
    ROOT_ADD_API(int, "Fan duty", cgpu->fan_duty, false);
//	ROOT_ADD_API(bool, "FanOptimal", g_fan_ctrl.optimal, false);
	ROOT_ADD_API(int, "iVid", t1->vid, false);
    ROOT_ADD_API(int, "PLL", t1->pll, false);
	ROOT_ADD_API(double, "Voltage Max", s_reg_ctrl.highest_vol[t1->chain_id], false);
	ROOT_ADD_API(double, "Voltage Min", s_reg_ctrl.lowest_vol[t1->chain_id], false);
	ROOT_ADD_API(double, "Voltage Avg", s_reg_ctrl.average_vol[t1->chain_id], false);
	ROOT_ADD_API(bool, "VidOptimal", t1->VidOptimal, false);
	ROOT_ADD_API(bool, "pllOptimal", t1->pllOptimal, false);
	ROOT_ADD_API(bool, "VoltageBalanced", t1->voltagebalanced, false);
	ROOT_ADD_API(int, "Chain num", cgpu->chainNum, false);
	ROOT_ADD_API(double, "MHS av", cgpu->mhs_av, false);
	ROOT_ADD_API(bool, "Disabled", t1->disabled, false);
	for (i = 0; i < t1->num_chips; i++) {
		if (!t1->chips[i].disabled)
			chipmap |= 1 << i;
	}
	sprintf(s, "%Lx", chipmap);
	ROOT_ADD_API(string, "Enabled chips", s[0], true);
	ROOT_ADD_API(double, "Temp", cgpu->temp, false);

	for (i = 0; i < t1->num_chips; i++) {
		sprintf(s, "%02d HW errors", i);
		ROOT_ADD_API(int, s, t1->chips[i].hw_errors, true);
		sprintf(s, "%02d Stales", i);
		ROOT_ADD_API(int, s, t1->chips[i].stales, true);
		sprintf(s, "%02d Nonces found", i);
		ROOT_ADD_API(int, s, t1->chips[i].nonces_found, true);
		sprintf(s, "%02d Nonce ranges", i);
		ROOT_ADD_API(int, s, t1->chips[i].nonce_ranges_done, true);
		sprintf(s, "%02d Cooldown", i);
		ROOT_ADD_API(int, s, t1->chips[i].cooldown_begin, true);
		sprintf(s, "%02d Fail count", i);
		ROOT_ADD_API(int, s, t1->chips[i].fail_count, true);
		sprintf(s, "%02d Fail reset", i);
		ROOT_ADD_API(int, s, t1->chips[i].fail_reset, true);
		sprintf(s, "%02d Temp", i);
		ROOT_ADD_API(int, s, t1->chips[i].temp, true);
		sprintf(s, "%02d nVol", i);
		ROOT_ADD_API(int, s, t1->chips[i].nVol, true);
		sprintf(s, "%02d PLL", i);
		ROOT_ADD_API(int, s, t1->chips[i].pll, true);
		sprintf(s, "%02d pllOptimal", i);
		ROOT_ADD_API(bool, s, t1->chips[i].pllOptimal, true);
	}
	return root;
}


    struct device_drv coinflex_drv = 
    {
        .drv_id                 = DRIVER_coinflex,
        .dname                  = "HLT_Coinflex",
        .name                   = "HLT",
        .drv_ver                = COINFLEX_DRIVER_VER,
        .drv_date               = COINFLEX_DRIVER_DATE,
        .drv_detect             = coinflex_detect,
        .get_statline_before    = coinflex_get_statline_before,
        .queue_full             = coinflex_queue_full,
        .get_api_stats          = coinflex_api_stats,
        .identify_device        = NULL,
        .set_device             = NULL,
        .thread_prepare         = NULL,
        .thread_shutdown        = NULL,
        .hw_reset               = NULL,
        .hash_work              = hash_queued_work,
        .update_work            = NULL,
        .flush_work             = coinflex_flush_work,          // new block detected or work restart 
        .scanwork               = coinflex_scanwork,            // scan hash
        .max_diff               = 65536
    };

