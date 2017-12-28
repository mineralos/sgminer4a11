
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

#include "asic_inno.h"
#include "asic_inno_clock.h"
#include "asic_inno_cmd.h"
#include "asic_inno_gpio.h"

#include "inno_fan.h"
//#include "inno_led.h"

#define WORK_SIZE               (80)
#define DEVICE_TARGET_SIZE      (32)
#define TARGET_POS              (80)
#define TARGET_SIZE             (4)
#define MINER_ID_POS            (84)
#define MINER_ID_SIZE           (1)
#define WORK_ID_POS         (85)
#define WORK_ID_SIZE            (1)
#define FIND_NONCE_SIZE     (6)             // For receive value from miner: 4-Bytes nonce, 1-Byte miner_id, 1-Byte work_id

#define REPLY_SIZE              (2)
#define BUF_SIZE                    (128)
#define TEMP_UPDATE_INT_MS  10000
#define CHECK_DISABLE_TIME  0


struct Test_bench Test_bench_Array[5]={
    {1100,  0,  0,  0},
    {1100,  0,  0,  0},
    {1100,  0,  0,  0},
    {1100,  0,  0,  0},
    {1100,  0,  0,  0},
};


struct spi_config cfg[ASIC_CHAIN_NUM];
struct spi_ctx *spi[ASIC_CHAIN_NUM];
struct A1_chain *chain[ASIC_CHAIN_NUM];

uint8_t A1Pll1=A5_PLL_CLOCK_400MHz;
static uint8_t A1Pll2=A5_PLL_CLOCK_400MHz;
static uint8_t A1Pll3=A5_PLL_CLOCK_400MHz;
static uint8_t A1Pll4=A5_PLL_CLOCK_400MHz;
static uint8_t A1Pll5=A5_PLL_CLOCK_400MHz;
static uint8_t A1Pll6=A5_PLL_CLOCK_400MHz;
static uint8_t A1Pll7=A5_PLL_CLOCK_400MHz;
static uint8_t A1Pll8=A5_PLL_CLOCK_400MHz;

/* FAN CTRL */
inno_fan_temp_s g_fan_ctrl;
inno_reg_ctrl_t s_reg_ctrl;

static uint32_t show_log[ASIC_CHAIN_NUM];
static uint32_t update_temp[ASIC_CHAIN_NUM];
static uint32_t check_disbale_flag[ASIC_CHAIN_NUM];

#define STD_V          0.84

int spi_plug_status[ASIC_CHAIN_NUM] = {0};

char szShowLog[ASIC_CHAIN_NUM][ASIC_CHIP_NUM][256] = {0};
int fan_level[8]={30,40,50,60,70,80,90,100};

hardware_version_e g_hwver;
inno_type_e g_type;
int g_reset_delay = 0xffff;
int g_miner_state = 0;
int chain_flag[ASIC_CHAIN_NUM] = {0};

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

static void coinflex_print_hw_error(char *drv_name, int device_id, struct work *work, uint32_t nonce);
static bool coinflex_set_algorithm(struct cgpu_info *coinflex);

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

    asic_gpio_write(a1->spi_ctx->led, 1);
    asic_gpio_write(a1->spi_ctx->power_en, 0);

    a1->chips = NULL;
    a1->spi_ctx = NULL;
    free(a1);
}

int  cfg_tsadc_divider(struct A1_chain *a1,uint32_t pll_clk)
{
    uint8_t  cmd_return;
    uint32_t tsadc_divider_tmp;
    uint8_t  tsadc_divider;
    //cmd0d(0x0d00, 0x0250, 0xa006 | (BYPASS_AUXPLL<<6), 0x2800 | tsadc_divider, 0x0300, 0x0000, 0x0000, 0x0000)
    uint8_t    buffer[64] = {0x02,0x50,0xa0,0x06,0x28,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
#ifdef MPW
    tsadc_divider_tmp = (pll_clk/2)*1000/256/650;
#else
    tsadc_divider_tmp = (pll_clk/2)*1000/16/650;
#endif
    tsadc_divider = (uint8_t)(tsadc_divider_tmp & 0xff);

    buffer[5] = 0x00 | tsadc_divider;

    if(!inno_cmd_write_sec_reg(a1,ADDR_BROADCAST,buffer)){
        applog(LOG_WARNING, "#####Write t/v sensor Value Failed!\n");
    }else{
        applog(LOG_WARNING, "#####Write t/v sensor Value Success!\n");
    }
}

int chain_detect_reload(struct A1_chain *a1)
{
    uint8_t buffer[64];
    int cid = a1->chain_id;
    uint8_t temp_reg[REG_LENGTH];
    int i;

    set_spi_speed(3250000);
    usleep(100000);

    memset(buffer, 0, sizeof(buffer));
    if(!inno_cmd_bist_start(a1, ADDR_BROADCAST, buffer)){
        applog(LOG_WARNING, "[reload]bist start fail");
        return -1;
    }

    if(buffer[3] != 0){
        a1->num_chips = buffer[3]; 
    }

    applog(LOG_WARNING, "[reload]%d: detected %d chips", cid, a1->num_chips);

    usleep(10000);

    if(!inno_cmd_bist_collect(a1, ADDR_BROADCAST)){
        applog(LOG_NOTICE, "[reload]bist collect fail");
        return -1;
    }

    applog(LOG_NOTICE, "collect core success");
    applog(LOG_NOTICE, "%d:  A1 chip-chain detected", cid);
    return a1->num_chips;
}

#define NET_PORT 53
#define NET_IP "8.8.8.8" //谷歌DNS

//获取联网状态
static int check_net(void)
{

    int fd; 
    int in_len=0;
    struct sockaddr_in servaddr;
    //char buf[128];

    in_len = sizeof(struct sockaddr_in);
    fd = socket(AF_INET,SOCK_STREAM,0);
    if(fd < 0)
    {   
        perror("socket");
        return -1; 
    }   

    /*设置默认服务器的信息*/
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(NET_PORT);
    servaddr.sin_addr.s_addr = inet_addr(NET_IP);
    memset(servaddr.sin_zero,0,sizeof(servaddr.sin_zero));

    /*connect 函数*/
    if(connect(fd,(struct sockaddr* )&servaddr,in_len) < 0 ) 
    {   

        //  printf("not connect to internet!\n ");
        close(fd);
        return -2; //没有联网成功
    }   
    else
    {   
        //   printf("=====connect ok!=====\n");
        close(fd);
        return 1;
    }   
}


bool init_ReadTemp(struct A1_chain *a1, int chain_id)
{
    int i,j;
    uint8_t reg[64];
    static int last_time = 0;

    //applog(LOG_ERR, "start read temp cid %d, a1 addr 0x%x\n", chain_id,a1);
    /* update temp database */
    uint32_t temp = 0;

    if(a1 == NULL)
    {
        return false;
    }

    int cid = a1->chain_id;
    //struct cgpu_info *cgpu = a1->cgpu;
    inno_fan_speed_set(&g_fan_ctrl,PREHEAT_SPEED);

    //while(s_fan_ctrl.temp_highest[cid] > 505)//FAN_FIRST_STAGE)
    do{
        for (i = a1->num_active_chips; i > 0; i -= 3)
        { 
            if (!inno_cmd_read_reg(a1, i, reg))
            {
                applog(LOG_ERR, "%d: Failed to read temperature sensor register for chip %d ", a1->chain_id, i);

                continue;
            }

            temp = 0x000003ff & ((reg[7] << 8) | reg[8]);
            //applog(LOG_ERR,"cid %d,chip %d,temp %d\n",cid, i, temp);
            inno_fan_temp_add(&g_fan_ctrl, cid, i, temp);
        } 

        asic_temp_sort(&g_fan_ctrl, chain_id);
        inno_fan_temp_highest(&g_fan_ctrl, chain_id,g_type);
        a1->pre_heat = 1;

        if((last_time + 3*TEMP_UPDATE_INT_MS) < get_current_ms())
        {
            applog(LOG_WARNING,"chain %d higtest temp %d\n",cid, g_fan_ctrl.temp_highest[cid]);
            last_time = get_current_ms();
        }

#if 0
        if(check_net()== -2)
        {
            cnt++;
            //printf("cnt = %d\n",cnt);
        }
        else
        {
            //printf("ping ok\n");
            cnt = 0;
        }

        if(cnt > 10)
        {
            printf("shutdown spi link\n");
            power_down_all_chain();

            for(j=0; j<ASIC_CHAIN_NUM; j++)
                loop_blink_led(spi[j]->led,10);

        }
#endif

        //applog(LOG_ERR,"higtest temp %d\n",g_fan_ctrl.temp_highest[cid]);
    }while(g_fan_ctrl.temp_highest[cid] > START_FAN_TH);
    a1->pre_heat = 0;
    applog(LOG_WARNING,"Now chain %d preheat is over\n",cid);
    return true;
}


bool init_A1_chain_reload(struct A1_chain *a1, int chain_id)
{
    int i;
    if(a1 == NULL){
        return false;
    }

    applog(LOG_DEBUG, "%d: A1 init chain reload", chain_id);

    a1->num_chips =  chain_detect_reload(a1);
    usleep(10000);

    if (a1->num_chips < 1){
        goto failure;
    }

    applog(LOG_WARNING, "spidev%d.%d: %d: Found %d A1 chips",a1->spi_ctx->config.bus, a1->spi_ctx->config.cs_line,a1->chain_id, a1->num_chips);

    /* override max number of active chips if requested */
    a1->num_active_chips = a1->num_chips;
    if ((A1_config_options.override_chip_num > 0) && a1->num_chips > A1_config_options.override_chip_num){
        a1->num_active_chips = A1_config_options.override_chip_num;
        applog(LOG_WARNING, "%d: limiting chain to %d chips",a1->chain_id, a1->num_active_chips);
    }

    a1->chips = calloc(a1->num_active_chips, sizeof(struct A1_chip));
    assert (a1->chips != NULL);

    if (!inno_cmd_bist_fix(a1, ADDR_BROADCAST)){
        goto failure;
    }

    usleep(200);

    //configure for vsensor
    inno_configure_tvsensor(a1,ADDR_BROADCAST,0);

    for (i = 0; i < a1->num_active_chips; i++){
        inno_check_voltage(a1, i+1, &s_reg_ctrl);
    }

    //configure for tsensor
    inno_configure_tvsensor(a1,ADDR_BROADCAST,1);

    for (i = 0; i < a1->num_active_chips; i++){
        check_chip(a1, i);
        inno_fan_temp_add(&g_fan_ctrl, chain_id, i+1, a1->chips[i].temp);
    }

    chain_temp_update(&g_fan_ctrl,chain_id, g_type);
    //inno_fan_speed_update(&g_fan_ctrl,fan_level);

    applog(LOG_WARNING, "[chain_ID:%d]: Found %d Chips With Total %d Active Cores",a1->chain_id, a1->num_active_chips, a1->num_cores);
    applog(LOG_WARNING, "[chain_ID]: Temp:%d\n",g_fan_ctrl.temp_highest[chain_id]);

#if 1
    if(g_fan_ctrl.temp_highest[chain_id] < DANGEROUS_TMP){
        //asic_gpio_write(spi[a1->chain_id]->power_en, 0);
        //loop_blink_led(spi[a1->chain_id]->led, 10);
        goto failure;
        //early_quit(1,"Notice Chain %d temp:%d Maybe Has Some Problem in Temperate\n",a1->chain_id,s_fan_ctrl.temp_highest[chain_id]);
    }
#endif

    return true;

failure:
    exit_A1_chain(a1);
    return false;
}

struct A1_chain *init_A1_chain(struct spi_ctx *ctx, int chain_id)
{
    int i;
    struct A1_chain *a1 = malloc(sizeof(*a1)); 
    assert(a1 != NULL);
    memset(a1,0,sizeof(struct A1_chain));

    applog(LOG_ERR, "%d: A1 init chain", chain_id);

    memset(a1, 0, sizeof(*a1));
    a1->spi_ctx = ctx;
    a1->chain_id = chain_id;
    
   // inno_cmd_reset(a1,ADDR_BROADCAST, &reg);
    usleep(200000);

    a1->num_chips =  chain_detect(a1);
    if (a1->num_chips < 1){
        goto failure;
    }
    usleep(100000);
    //sleep(10);
    cfg_tsadc_divider(a1,120);//PLL_Clk_12Mhz[A1Pll1].speedMHz);

    applog(LOG_WARNING, "spidev%d.%d: %d: Found %d A1 chips",a1->spi_ctx->config.bus, a1->spi_ctx->config.cs_line,a1->chain_id, a1->num_chips);

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

static  int prepll_chip_temp(struct A1_chain *a1)
{
    int i;
    uint8_t reg[64];
    int cid = a1->chain_id;
    int temp;
    memset(reg,0,sizeof(reg));

    //while(s_fan_ctrl.temp_highest[cid] > 505)//FAN_FIRST_STAGE)
    for (i = a1->num_active_chips; i > 0; i --)
    { 
        if (!inno_cmd_read_reg(a1, i, reg))
        {
            applog(LOG_ERR, "%d: Failed to read temperature sensor register for chip %d ", a1->chain_id, i);

            continue;
        }

        temp = 0x000003ff & ((reg[7] << 8) | reg[8]);
        //applog(LOG_ERR,"cid %d,chip %d,temp %d\n",cid, i, temp);
        inno_fan_temp_add(&g_fan_ctrl, cid, i, temp);
    } 

    chain_temp_update(&g_fan_ctrl,cid,g_type);

    return 0;

}

int* inno_preinit( uint32_t pll, uint32_t last_pll)
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

rept_cfgpll:
        ret[i] = prechain_detect(chain[i], A1_ConfigA1PLLClock(pll),A1_ConfigA1PLLClock(last_pll));
        usleep(200000);
        prepll_chip_temp(chain[i]);
        if(-1 == ret[i])
        {
            if((g_fan_ctrl.temp_highest[i] > DANGEROUS_TMP) && (g_fan_ctrl.temp_lowest[i] < START_FAN_TH))
            {
                rep_cnt++;
                if(rep_cnt < 5)
                    goto rept_cfgpll;
                else
                    goto out_cfgpll;
            }
        }
    }
out_cfgpll:
    inno_fan_speed_update(&g_fan_ctrl);


    return ret;
}


static int chain_spi_init()
{
    int i,j;

    for(i = 0; i < ASIC_CHAIN_NUM; i++)
    {
        cfg[i].bus     = i;
        cfg[i].cs_line = 0;
        cfg[i].mode    = SPI_MODE_1;
        cfg[i].speed   = DEFAULT_SPI_SPEED;
        cfg[i].bits    = DEFAULT_SPI_BITS_PER_WORD;
        cfg[i].delay   = DEFAULT_SPI_DELAY_USECS;

        spi[i] = spi_init(&cfg[i]);
        if(spi[i] == NULL){
            applog(LOG_ERR, "spi init fail");
            return false;
        }

        //mutex_init(&spi[i]->spi_lock);

        spi[i]->power_en = SPI_PIN_POWER_EN[i];     
        spi[i]->start_en = SPI_PIN_START_EN[i];     
        spi[i]->reset = SPI_PIN_RESET[i];
        spi[i]->led   = SPI_PIN_LED[i];
        spi[i]->plug  = SPI_PIN_PLUG[i];

        asic_gpio_init(spi[i]->power_en, 0);
        asic_gpio_init(spi[i]->start_en, 0);
        asic_gpio_init(spi[i]->reset, 0);
        asic_gpio_init(spi[i]->led, 0);
        asic_gpio_init(spi[i]->plug, 1);


        update_temp[i] = 0;
        show_log[i] = 0;

        asic_gpio_write(spi[i]->power_en, 0);
        sleep(1);
        asic_gpio_write(spi[i]->start_en, 0);
        asic_gpio_write(spi[i]->reset, 0);
        asic_gpio_write(spi[i]->led, 1);
    }

    sleep(5);
    applog(LOG_ERR,"spi init");

    for(i = 0; i < ASIC_CHAIN_NUM; i++){
       if(spi[i] == NULL)
           continue;
        
        asic_gpio_write(spi[i]->led, 0);
        asic_gpio_write(spi[i]->power_en, 1);
        sleep(5);
        asic_gpio_write(spi[i]->reset, 1);
        sleep(1);
        asic_gpio_write(spi[i]->start_en, 1);
        

        spi_plug_status[i] = asic_gpio_read(spi[i]->plug);
        g_fan_ctrl.valid_chain[i] = spi_plug_status[i];
        applog(LOG_ERR, "Plug Status[%d] = %d",i,spi_plug_status[i]);
    }

    return true;
}

static int inc_pll(void)
{
    uint32_t i = 0,j = 0; 
    int *ret;
    static uint32_t last_pll;

    applog(LOG_ERR, "pre init_pll...");
    for(i = PLL_Clk_12Mhz[0].speedMHz; i < opt_A1Pll1; i+=200)
    {
        ret = inno_preinit(i,last_pll);
        last_pll = i;

        for(j=0; j<ASIC_CHAIN_NUM; j++)
        {
            if (-1 == ret[j]){
                asic_gpio_write(chain[j]->spi_ctx->power_en, 0);
                sleep(1);
                asic_gpio_write(chain[j]->spi_ctx->start_en, 0);
                asic_gpio_write(chain[j]->spi_ctx->reset, 0);  
                
            }else{
                applog(LOG_ERR,"pll %d finished\n",i);
            }
        }
    }

    if((i > (opt_A1Pll1 - 200)) && (i < (opt_A1Pll1 + 200)))
    {
        ret = inno_preinit(opt_A1Pll1,last_pll);

        for(j=0; j<ASIC_CHAIN_NUM; j++)
        {
            if (-1 == ret[j]){
                asic_gpio_write(chain[j]->spi_ctx->power_en, 0);
                sleep(1);
                asic_gpio_write(chain[j]->spi_ctx->start_en, 0);
                asic_gpio_write(chain[j]->spi_ctx->reset, 0);  

            }else{
                applog(LOG_ERR,"pll %d finished\n",i);
            }
        }
    }
    return 0;
}

static bool detect_A1_chain(void)
{
    int i,j,ret,res = 0;
    applog(LOG_ERR, "A1: checking A1 chain %d,%d,%d,%d,%d,%d,%d,%d",opt_voltage1,opt_voltage2,opt_voltage3,opt_voltage4,opt_voltage5,opt_voltage6,opt_voltage7,opt_voltage8);

    ret = chain_spi_init();
    if(!ret)
        return false;

    set_vid_value(8);

   //add for A7
   asic_spi_init();
   set_spi_speed(1500000);


    for(i = 0; i < ASIC_CHAIN_NUM; i++){
        chain[i] = init_A1_chain(spi[i], i);
        if (chain[i] == NULL){
            applog(LOG_ERR, "init a1 chain fail");
            continue;
        }else{
            res++;
            chain_flag[i] = 1;
        }
        inno_cmd_reset(chain[i], ADDR_BROADCAST,NULL);
        applog(LOG_WARNING, "Detected the %d A1 chain with %d chips",i, chain[i]->num_active_chips);
    }

    usleep(200000);
    inc_pll();

    if(g_hwver == HARDWARE_VERSION_G9){

        //divide the init to break two part
        if(opt_voltage > 8){
            for(i=9; i<=opt_voltage; i++){
                set_vid_value(i);
                usleep(500000);
            }
        }

        if(opt_voltage < 8){
            for(i=7; i>=opt_voltage; i--){
                set_vid_value(i);
                usleep(500000);
            }
        }
    }else if(g_hwver == HARDWARE_VERSION_G19)
    {

        int j, vid;
        for(i = 0; i < ASIC_CHAIN_NUM; i++){

            if(chain[i] == NULL)
                continue;

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

            if(chain[i]->vid > 8){
                for(j=9; j<=chain[i]->vid; j++){
                    //applog(LOG_ERR,"set_vid_value_G19 vid > 8");
                    set_vid_value_G19(i, j);
                    usleep(500000);
                }
            }

            if(chain[i]->vid < 8){
                for(j=7; j>=chain[i]->vid; j--){
                    //applog(LOG_ERR,"set_vid_value_G19 vid < 8");
                    set_vid_value_G19(i, j);
                    usleep(500000);
                }
            }
        }       
    }

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

        cgpu->device_data = chain[i];

        chain[i]->cgpu = cgpu;
        add_cgpu(cgpu);

        asic_gpio_write(chain[i]->spi_ctx->led, 0);

        applog(LOG_WARNING, "Detected the %d A1 chain with %d chips / %d cores",i, chain[i]->num_active_chips, chain[i]->num_cores);
    }
    inno_fan_speed_update(&g_fan_ctrl);

#if 0
    Test_bench_Array[0].uiVol = opt_voltage;
    for(i = 0; i < ASIC_CHAIN_NUM; i++)
    {
        if(chain_flag[i] != 1)
        {
            continue;
        }
        Test_bench_Array[0].uiScore += inno_cmd_test_chip(chain[i]);
        Test_bench_Array[0].uiCoreNum += chain[i]->num_cores;
    }

    for(i = 1; i < 3; i++)
    {
        if(Test_bench_Array[0].uiVol - i < 1)
        {
            continue;
        }
        sleep(1);
        set_vid_value(Test_bench_Array[0].uiVol - i);
        Test_bench_Array[i].uiVol = Test_bench_Array[0].uiVol - i;
        sleep(1);
        for(j = 0; j < ASIC_CHAIN_NUM; j++)
        {   
            if(chain_flag[j] != 1)
            {
                continue;
            }
            Test_bench_Array[i].uiScore += inno_cmd_test_chip(chain[j]);
            Test_bench_Array[i].uiCoreNum += chain[j]->num_cores;
        }
    }

    for(i = 1; i >= 0; i--)
    {
        set_vid_value(Test_bench_Array[0].uiVol - i);
        sleep(1);
    }

    for(i = 3; i < 5; i++)
    {
        if(Test_bench_Array[0].uiVol + i - 2 > 31)
        {
            continue;
        }
        sleep(1);
        set_vid_value(Test_bench_Array[0].uiVol + i - 2);
        Test_bench_Array[i].uiVol = Test_bench_Array[0].uiVol + i - 2;
        sleep(1);
        for(j = 0; j < ASIC_CHAIN_NUM; j++)
        {
            if(chain_flag[j] != 1)
            {
                continue;
            }
            Test_bench_Array[i].uiScore += inno_cmd_test_chip(chain[j]);
            Test_bench_Array[i].uiCoreNum += chain[j]->num_cores;
        }
    }

    for(j = 0; j < 5; j++)
    {
        printf("after pll_vid_test_bench Test_bench_Array[%d].uiScore=%d,Test_bench_Array[%d].uiCoreNum=%d. \n", j, Test_bench_Array[j].uiScore, j, Test_bench_Array[j].uiCoreNum);
    }

    int index = 0;
    uint32_t cur= 0;
    for(j = 1; j < 5; j++)
    {
        cur = Test_bench_Array[j].uiScore + 5 * (Test_bench_Array[j].uiVol - Test_bench_Array[index].uiVol);

        if(cur > Test_bench_Array[index].uiScore)
        {
            index = j;
        }

        if((cur == Test_bench_Array[index].uiScore) && (Test_bench_Array[j].uiVol > Test_bench_Array[index].uiVol))
        {
            index = j;
        }
    }

    printf("The best group is %d. vid is %d! \t \n", index, Test_bench_Array[index].uiVol);

    for(i=Test_bench_Array[0].uiVol + 2; i>=Test_bench_Array[index].uiVol; i--){
        set_vid_value(i);
        usleep(500000);
    }
    opt_voltage = Test_bench_Array[index].uiVol;
#endif
    return (res == 0) ? false : true;
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

    g_hwver = inno_get_hwver();
    g_type = inno_get_miner_type();

    memset(&s_reg_ctrl,0,sizeof(s_reg_ctrl));
    memset(&g_fan_ctrl,0,sizeof(g_fan_ctrl));

    inno_fan_temp_init(&g_fan_ctrl,fan_level);

    // update time
    for(j = 0; j < 100; j++)
    {
        cgtime(&test_tv);
        if(test_tv.tv_sec > 1000000000)
        {
            break;
        }

        usleep(500000);
    }


    if(detect_A1_chain()){
        return ;
    }

    applog(LOG_WARNING, "A1 dectect finish");

    int i = 0;
    /* release SPI context if no A1 products found */
    for(i = 0; i < ASIC_CHAIN_NUM; i++){
        spi_exit(spi[i]);
    }   
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
#if 1
    struct A1_chain *a1 = coinflex->device_data;
    int cid = a1->chain_id;
    //board_selector->select(cid);
    int i;

    mutex_lock(&a1->lock);
    /* stop chips hashing current work */
    if (!abort_work(a1)) 
    {
        applog(LOG_ERR, "%d: failed to abort work in chip chain!", cid);
    }
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

        if(!inno_cmd_resetjob(a1, i+1))
        {
            applog(LOG_WARNING, "chip %d clear work false", i);\
                continue;
        }

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

#endif
}


#define VOLTAGE_UPDATE_INT  6000
//#define  LOG_FILE_PREFIX "/home/www/conf/analys"
#define  LOG_FILE_PREFIX "/tmp/log/analys"

uint8_t cLevelError1[3] = "!";
uint8_t cLevelError2[3] = "#";
uint8_t cLevelError3[3] = "$";
uint8_t cLevelError4[3] = "%";
uint8_t cLevelError5[3] = "*";
uint8_t cLevelNormal[3] = "+";

void Inno_Log_Save(struct A1_chip *chip,int nChip,int nChain)
{
    uint8_t szInNormal[8] = {0};
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

void inno_log_print(int cid, void* log, int len)
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

static int64_t coinflex_scanwork(struct thr_info *thr)
{
    //static uint8_t cnt = 0;
    //static uint8_t reset_buf[2] = {0x20,0x20};
    int i;
    int32_t A1Pll = 1000;
    uint8_t reg[128];
    struct cgpu_info *cgpu = thr->cgpu;
    struct A1_chain *a1 = cgpu->device_data;
    //struct A1_chip *a7 = &a1->chips[0];
    struct work local_work;
    int32_t nonce_ranges_processed = 0;

    uint32_t nonce;
    uint8_t chip_id;
    uint8_t job_id;
    bool work_updated = false;
    //  static uint8_t RD_result = 3;

    if (a1->num_cores == 0) {
        cgpu->deven = DEV_DISABLED;
        return 0;
    }

    mutex_lock(&a1->lock);
    int cid = a1->chain_id;

    if (a1->last_temp_time + TEMP_UPDATE_INT_MS < get_current_ms())
    {
        update_temp[cid]++;
        show_log[cid]++;
        check_disbale_flag[cid]++;

        inno_log_print(cid, szShowLog[cid], sizeof(szShowLog[0]));

        a1->last_temp_time = get_current_ms();
    }

    if(g_fan_ctrl.temp_highest[a1->chain_id] < DANGEROUS_TMP){
        asic_gpio_write(spi[a1->chain_id]->power_en, 0);
        loop_blink_led(spi[a1->chain_id]->led, 10);
        applog(LOG_ERR,"Notice Chain %d temp:%d Maybe Has Some Problem in Temperate\n",a1->chain_id,g_fan_ctrl.temp_highest[a1->chain_id]);
    }

    /* poll queued results */
    while (true){
        if (!get_nonce(a1, (uint8_t*)&nonce, &chip_id, &job_id)){
            break;
        }

        work_updated = true;
        if (chip_id < 1 || chip_id > a1->num_active_chips) {
            applog(LOG_WARNING, "%d: wrong chip_id %d", cid, chip_id);
            continue;
        }
        if (job_id < 1 || job_id > 4){
            applog(LOG_WARNING, "%d: chip %d: result has wrong ""job_id %d", cid, chip_id, job_id);
            flush_spi(a1);
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
        //applog(LOG_ERR, "YEAH: %d: chip %d / job_id %d: nonce 0x%08x,sdiff:0x%x", cid, chip_id, job_id, nonce,work->sdiff);
        chip->nonces_found++;
    }

    //uint8_t reg[REG_LENGTH];
    /* check for completed works */
    if(a1->work_start_delay > 0)
    {
        applog(LOG_INFO, "wait for pll stable");
        a1->work_start_delay--;
    }
    else
    {
        if(update_temp[cid] > 1)
        {
            for (i = a1->num_active_chips; i > 0; i--) 
            {
                //uint8_t c = i;
                if (is_chip_disabled(a1, i))
                {
                    printf("chip %d is disabled\n ",i);
                    continue;
                }
                if(!inno_cmd_read_reg(a1, i, reg))
                {
                    applog(LOG_ERR,"chip %d reg read failed\n ",i);
                    continue;
                }else{

                    /* update temp database */
                    uint32_t temp = 0;
                    //struct A1_chip *chip = &a1->chips[i - 1];

                    temp = 0x000003ff & ((reg[7] << 8) | reg[8]);
                    //chip->temp = temp;
                    inno_fan_temp_add(&g_fan_ctrl, cid, i, temp);

                    //cnt++;
                } 
            }

            chain_temp_update(&g_fan_ctrl, cid, g_type);
            //inno_fan_speed_update(&g_fan_ctrl,fan_level);

            applog(LOG_INFO,"cid %d,hi %d,lo:%d,av:%d\n",cid,g_fan_ctrl.temp_highest[cid],g_fan_ctrl.temp_lowest[cid],g_fan_ctrl.temp_arvarge[cid]);

            cgpu->temp = g_fan_ctrl.temp2float[cid][1];
            cgpu->temp_max = g_fan_ctrl.temp2float[cid][0];
            cgpu->temp_min = g_fan_ctrl.temp2float[cid][2];
            cgpu->fan_duty = g_fan_ctrl.speed;
            cgpu->pre_heat = a1->pre_heat;
            //printf("g_fan_ctrl: cid %d,chip %d, chip %d,hi %d\n",g_fan_ctrl.pre_warn[0],g_fan_ctrl.pre_warn[1],g_fan_ctrl.pre_warn[2],g_fan_ctrl.pre_warn[3]);
            memcpy(cgpu->temp_prewarn,g_fan_ctrl.pre_warn, 4*sizeof(int));
            //printf("cgpu: cid %d,chip %d, chip %d,hi %d\n",cgpu->temp_prewarn[0],cgpu->temp_prewarn[1],cgpu->temp_prewarn[2],cgpu->temp_prewarn[3]);

            cgpu->chip_num = a1->num_active_chips;
            cgpu->core_num = a1->num_cores; 


            update_temp[cid] = 0;

        }
        if( inno_cmd_read_reg(a1, 10, reg) ||  inno_cmd_read_reg(a1, 11, reg) ||  inno_cmd_read_reg(a1, 12, reg))
        {
            uint8_t qstate = reg[9] & 0x01;

            if (qstate != 0x01)
            {
                work_updated = true;
                for (i = a1->num_active_chips; i > 0; i--) 
                {
                    uint8_t c=i;
                    struct A1_chip *chip = &a1->chips[i - 1];
                    struct work *work = wq_dequeue(&a1->active_wq);
                    if(work == NULL)
                    {
                        applog(LOG_ERR, "Wait work queue...");
                        usleep(500);
                        continue;
                    }
                    //assert(work != NULL);

                    if (set_work(a1, c, work, 0))
                    {
                        nonce_ranges_processed++;
                        chip->nonce_ranges_done++;
                    }

                    if(show_log[cid] > 0)                   
                    {   
                        Inno_Log_Save(chip,c-1,cid);
                        if(i==1) show_log[cid] = 0; 
                    }
                }
            } 
        }
    }

    static int last_temp_time = 0;
    if (last_temp_time + 60*TEMP_UPDATE_INT_MS < get_current_ms())
    {
        //if(cnt > VOLTAGE_UPDATE_INT){
        //configure for vsensor
        inno_configure_tvsensor(a1,ADDR_BROADCAST,0);

        for (i = 0; i < a1->num_active_chips; i++){
            inno_check_voltage(a1, i+1, &s_reg_ctrl);
        }
        last_temp_time = get_current_ms();
        //configure for tsensor
        inno_configure_tvsensor(a1,ADDR_BROADCAST,1);
        usleep(200);
    }

    if(check_disbale_flag[cid] > CHECK_DISABLE_TIME)
    {
        //applog(LOG_INFO, "start to check disable chips");
        switch(cid){
            case 0:check_disabled_chips(a1, A1Pll1);;break;
            case 1:check_disabled_chips(a1, A1Pll2);;break;
            case 2:check_disabled_chips(a1, A1Pll3);;break;
            case 3:check_disabled_chips(a1, A1Pll4);;break;
            case 4:check_disabled_chips(a1, A1Pll5);;break;
            case 5:check_disabled_chips(a1, A1Pll6);;break;
            default:;
        }
        check_disbale_flag[cid] = 0;
    }
    mutex_unlock(&a1->lock);

    if (nonce_ranges_processed < 0){
        applog(LOG_INFO, "nonce_ranges_processed less than 0");
        nonce_ranges_processed = 0;
    }

    if (nonce_ranges_processed != 0) {
        applog(LOG_INFO, "%d, nonces processed %d", cid, nonce_ranges_processed);
    }
    /* in case of no progress, prevent busy looping */
    //if (!work_updated)
    //  cgsleep_ms(40);

    cgtime(&a1->tvScryptCurr);
    timersub(&a1->tvScryptCurr, &a1->tvScryptLast, &a1->tvScryptDiff);
    cgtime(&a1->tvScryptLast);
    /*
       switch(cgpu->device_id){
       case 0:A1Pll = PLL_Clk_12Mhz[A1Pll1].speedMHz;break;
       case 1:A1Pll = PLL_Clk_12Mhz[A1Pll2].speedMHz;break;
       case 2:A1Pll = PLL_Clk_12Mhz[A1Pll3].speedMHz;break;
       case 3:A1Pll = PLL_Clk_12Mhz[A1Pll4].speedMHz;break;
       case 4:A1Pll = PLL_Clk_12Mhz[A1Pll5].speedMHz;break;
       case 5:A1Pll = PLL_Clk_12Mhz[A1Pll6].speedMHz;break;
       case 6:A1Pll = PLL_Clk_12Mhz[A1Pll7].speedMHz;break;
       case 7:A1Pll = PLL_Clk_12Mhz[A1Pll8].speedMHz;break;
       default:break;
       }
       */
    //return (int64_t)(2011173.18 * A1Pll / 1000 * (a1->num_cores/9.0) * (a1->tvScryptDiff.tv_usec / 1000000.0));
    //applog(LOG_ERR,"speed:%d,cores:%d,time:%d",PLL_Clk_12Mhz[A1Pll1].speedMHz,a1->num_cores,a1->tvScryptDiff.tv_usec);

    return ((((double)opt_A1Pll1*a1->tvScryptDiff.tv_usec /2) * (a1->num_cores))/13);
    //	return ((((double)PLL_Clk_12Mhz[A1Pll1].speedMHz*a1->tvScryptDiff.tv_usec /2) * (a1->num_cores))/13);
    }


    static void coinflex_print_hw_error(char *drv_name, int device_id, struct work *work, uint32_t nonce)
    {
        char        *twork;
        char        twork_data[BUF_SIZE];
        int     twork_index;
        int     display_size = 16;      // 16 Bytes

        applog(LOG_ERR, "---------------------------------------------------------");
        applog(LOG_ERR, "[%s %d]:ERROR  - Nonce = 0x%X,Work_ID = %3d", drv_name, device_id, nonce, work->work_id);
        twork = bin2hex(work->data, WORK_SIZE);                     // Multiply 2 for making string in bin2hex()
        for(twork_index = 0; twork_index < (WORK_SIZE * 2); twork_index += (display_size * 2))
        {
            snprintf(twork_data, (display_size * 2) + 1, "%s", &twork[twork_index]);
            applog(LOG_ERR, "Work Data      = %s", twork_data);
        }
        free(twork);
        twork = bin2hex(work->device_target, DEVICE_TARGET_SIZE);       // Multiply 2 for making string in bin2hex()
        for(twork_index = 0; twork_index < (DEVICE_TARGET_SIZE * 2); twork_index += (display_size * 2))
        {
            snprintf(twork_data, (display_size * 2) + 1, "%s", &twork[twork_index]);
            applog(LOG_ERR, "Device Target  = %s", twork_data);
        }
        free(twork);
        applog(LOG_ERR, "---------------------------------------------------------");
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
        .get_api_stats          = NULL,
        .identify_device        = NULL,
        .set_device             = NULL,
        .thread_prepare         = NULL,
        .thread_shutdown        = NULL,
        .hw_reset               = NULL,
        .hash_work              = hash_queued_work,
        .update_work            = NULL,
        .flush_work             = coinflex_flush_work,          // new block detected or work restart 
        .scanwork               = coinflex_scanwork,                // scan hash
        .max_diff                   = 65536
    };

#if COINFLEX_TEST_MODE
#if 0
    char dataX11[80] =  { 0x00,0x00,0x00,0x02,0x75,0xd6,0x91,0x99,0x65,0x07,0x7e,0x96,0xb8,0x04,0xb1,0xbf,0x77,0x8a,0xe9,0x2a,0x6d,0xe7,0x5f,0xeb,0x56,0x91,0x11,0x00,0x00,0x04,0xbe,0x72,0x00,0x00,0x00,0x00,0x62,0x22,0x72,0xad,0xf6,0x99,0x66,0x75,0xf2,0xa0,0xf9,0xe5,0x54,0xb3,0x67,0x54,0xfb,0x40,0xf8,0x1f,0x2d,0xad,0x5e,0xd8,0x4a,0x34,0x56,0x09,0xd7,0x58,0x2c,0xe9,0x53,0x95,    0x16,0xe7,0x1b,0x14,0x76,0x0a,0x00,0x10,0x6b,0x0d };
    char targetX11[4] = { 0x00,0x00,0x00,0xff };
    uint32_t nonceX11 = 225120256;  // 0xd6b1000
#else
    char dataX11[82] = {0x00,0x00, 0x20,0x00,0x00, 0x00, 0x5c, 0x3c, 0xf5, 0x94, 0xeb, 0x10, 0x71, 0x3b, 0x0e, 0x5b, 0xe7, 0x5c, 0xb9, 0x01, 0x09, 0xc2, 0x16, 0xea, 0xfd,0x1c,0xba, 0xba, 0x9c, 0x5e, 0x00, 0x00, 0x59, 0x16, 0x00, 0x00, 0x00, 0x00, 0xc7, 0x82, 0xa0, 0xa7, 0xc5, 0xa6, 0xf2, 0x08, 0x21, 0x6a, 0xe8, 0x49, 0x21, 0xbc, 0x8c, 0xa3, 0x7b, 0xee, 0x8c, 0x08, 0x5f, 0x93, 0x95, 0x06, 0x04, 0x02, 0xe9, 0x15, 0x70, 0x6b, 0x76, 0x19, 0x59, 0x28, 0xd7, 0x2c, 0x1b, 0x00, 0x81, 0xd5, 0xe4, 0x2e, 0x36, 0x40};
    //char dataX11[82] =  {0x00,0x00, 0x00,0x00,0x00,0x20,0x94,0xf5,0x3c,0x5c,0x3b,0x71,0x10,0xeb,0x5c,0xe7,0x5b,0x0e,0xc2,0x09,0x01,0xb9,0x1c,0xfd,0xea,0x16,0x5e,0x9c,0xba,0xba,0x16,0x59,0x00,0x00,0x00,0x00,0x00,0x00,0xa7,0xa0,0x82,0xc7,0x08,0xf2,0xa6,0xc5,0x49,0xe8,0x6a,0x21,0xa3,0x8c,0xbc,0x21,0x08,0x8c,0xee,0x7b,0x06,0x95,0x93,0x5f,0x15,0xe9,0x02,0x04,0x19,0x76,0x6b,0x70,0x2c,0xd7,0x28,0x59,0xd5,0x81,0x00,0x1b,0xe4,0x2e,0x36,0x40 };//0x40,0x36,0x2e,0xe4
    //ffd80000_00000027
    char targetX11[4] = { 0x00,0x00,0x00,0x27 };
    uint32_t nonceX11 = 0x40362ee4;//e42e3650;  // 0xd6b1000
#endif

    char dataX13[80] =  { 0x00,0x00,0x00,0x02,0x5b,0x4a,0xbb,0x46,0x95,0x9d,0x93,0xd0,0x49,0x1a,0x8c,0x97,0xb0,0x02,0x37,0x29,0x5d,0x1e,0xf8,0xfd,0xe0,0x74,0x2c,0xf7,0x00,0xdd,0x5c,0xb2,0x00,0x00,0x00,0x00,0x56,0xc0,0x2f,0x12,0x82,0x24,0xd3,0xb8,0xe9,0x37,0x67,0x9f,0x9d,0x00,0x10,0x00,0x2e,0x32,0x02,0x6b,0xf2,0x9d,0x22,0xd5,0x30,0x68,0xcb,0x13,0xc0,0x14,0x4d,0xa5,0x53,0x95,    0x89,0xad,0x1c,0x02,0xac,0x3d,0x00,0x0a,0x1e,0xf1 };
    char targetX13[4] = { 0xff,0x00,0x00,0x00 };
    uint32_t nonceX13 = 4045277696; // 0xf11e0a00

    static void coinflex_set_testdata(struct work *work)
    {
        char        *data;
        char        *target;
        uint32_t    nonce;
        uint8_t i,j;

        switch(kernel)
        {
            case KL_SCRYPT:
                break;
            case KL_X11MOD:
                data = dataX11;
                target = targetX11;
                nonce = nonceX11;
                break;
            case KL_X13MOD:
                data = dataX13;
                target = targetX13;
                nonce = nonceX13;
                break;
            case KL_X15MOD:
            case KL_NONE:
            default:
                return;
        }

        work->data[0] = ((0x01 & 0x0f) << 4) | CMD_WRITE_JOB;
        work->data[1] = 0x01;



        memcpy(&work->data[2], data, 80);
        memcpy(&work->device_target[28], target, 4);
        work->nonce = nonce;


        for(i=0; i<10;i++)
        {
            for(j=0;j<8;j++)
                printf("  %02x ",work->data[i*10+j]);

            printf("\n");
        }

        printf("target\n");

        for(i=0;i<4; i++){
            printf("0x%08x",work->target[i+28]);
        }
    }


    static void coinflex_print_hash(struct work *work, uint32_t nonce)
    {
        uint32_t    *work_nonce = (uint32_t *)(work->data + 64 + 12);
        uint32_t    *ohash = (uint32_t *)(work->hash);
        unsigned char hash_swap[32], target_swap[32];
        char        *hash_str, *target_str;

        *work_nonce = htole32(nonce);

        switch(kernel)
        {
            case KL_SCRYPT:
                break;
            case KL_X11MOD:
                darkcoin_regenhash(work);
                break;
            case KL_X13MOD:
                marucoin_regenhash(work);
                break;
            case KL_X15MOD:
            case KL_NONE:
            default:
                //regen_hash(work);
                break;
        }

        swab256(hash_swap, work->hash);
        swab256(target_swap, work->target);
        hash_str = bin2hex(hash_swap, 32);
        target_str = bin2hex(target_swap, 32);

        applog(LOG_ERR, "nonce : %x(%u) => Hash : %08x : %08x : %08x : %08x : %08x : %08x : %08x : %08x",
                nonce, nonce, ohash[0], ohash[1], ohash[2], ohash[3], ohash[4], ohash[5], ohash[6], ohash[7]);
        applog(LOG_NOTICE, "COINFLEX Hash  : %s", hash_str);
        applog(LOG_NOTICE, "COINFLEX Target: %s", target_str);

        free(hash_str);
        free(target_str);
    }
#endif

