
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
//***	driver-coinflex.c is for X11 algorithm mining by using Han-Lab's Pantheon-XXX series miner		***//
//=====================================================================//

//=====================================================================//
//  DRIVER_COINFLEX DEFINITION FOR X11 ALGORITHM
//  Support Product:
//		1) Pantheon-A	: Altera Stratix V E9 FPGA Chip
//						: 1 base b'd, 10 miner b'd, 1 miner b'd includes 4EA FPGA Chip
//		2) Pantheon-AFS4	: Altera Stratix IV 530 FPGA Chip
//						: 2 miner b'd(operating independently), 1 miner b'd includes 10EA FPGA Chip
//		3) Pantheon-CMF1 : Altera Stratix V E9 FPGA Chip
//						:  1 base b'd, 1 core b'd, 1 core b'd includes 1EA FPGA Chip
//=====================================================================//


#include "config.h"

#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>

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



#define COINFLEX_TIMEOUT		(5)

#define COINFLEX_DEF_DIFF		(64)

#define COINFLEX_STX			(0x12)
#define COINFLEX_ETX			(0x13)

#define STX_POS					(0)
#define CMD_POS					(1)
#define LEN_POS					(2)
#define DATA_POS				(3)
#define ETX_POS					(DATA_POS + len)

#define WAIT_TIME				(10)
#define INFO_SIZE				(28)
#define ALGO_SIZE				(2)
#define CORE_SIZE				(16)
#define CLOCK_SIZE				(6)
#define MWORK_SIZE				(2)

#define WORK_SIZE				(80)
#define DEVICE_TARGET_SIZE		(32)
#define NONCE_POS				(76)
#define NONCE_SIZE				(4)
#define TARGET_POS				(80)
#define TARGET_SIZE				(4)
#define MINER_ID_POS			(84)
#define MINER_ID_SIZE			(1)
#define WORK_ID_POS			(85)
#define WORK_ID_SIZE			(1)
#define FIND_NONCE_SIZE		(6)				// For receive value from miner: 4-Bytes nonce, 1-Byte miner_id, 1-Byte work_id

#define REPLY_SIZE				(2)
#define BUF_SIZE					(128)
#define TEMP_UPDATE_TIME		(5 * 1000)		/* 30sec -> 5sec*/

#define COINFLEX_COM_TIMEOUT_MS		(999)
#define TEMP_UPDATE_INT_MS	2000

// Commands
enum coinflex_cmd
{
	COINFLEX_RESET			= 0x00,
	COINFLEX_GET_STATUS	    = 0x01,
	COINFLEX_GET_INFO		= 0x02,
	COINFLEX_SET_CLK		= 0x03,
	COINFLEX_GET_CLK		= 0x04,
	COINFLEX_SET_TEMP		= 0x05,
	COINFLEX_GET_TEMP		= 0x06,
	RESERVED07				= 0x07,
	RESERVED08				= 0x08,
	COINFLEX_SEND_WORK	    = 0x09,
	RESERVED0A				= 0x0A,
	RESERVED0B				= 0x0B,
	RESERVED0C				= 0x0C,
	RESERVED0D				= 0x0D,
	RESERVED0E				= 0x0E,
	RESERVED0F				= 0x0F,
	COINFLEX_GET_WORK	    = 0x10,
	COINFLEX_SET_LED		= 0x11,
	COINFLEX_SET_IDLE		= 0x12,
	COINFLEX_SET_ALGO		= 0x13,
	COINFLEX_GET_CORE		= 0x14,
	COINFLEX_SET_MUTI		= 0x15,
	COINFLEX_HW_RESET	= 0x16,
	COINFLEX_STOP_MINER	= 0x17,
	COINFLEX_START_MINER	= 0x18
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

/* FAN CTRL */
static INNO_FAN_CTRL_T s_fan_ctrl;
static inno_reg_ctrl_t s_reg_ctrl;

static uint32_t show_log[ASIC_CHAIN_NUM];
static uint32_t update_temp[ASIC_CHAIN_NUM];
#define  DANGEROUS_TMP   445
#define STD_V          0.84


extern int opt_voltage;

// Commands result
enum coinflex_result
{
	COINFLEX_RESULT_OK = 0,
	COINFLEX_SEND_FAIL = 1,
	COINFLEX_RECV_FAIL = 2,
	COINFLEX_CMD_FAIL  = 3
};

/* added by yex in 20170907 */
/*
 * if not cooled sufficiently, communication fails and chip is temporary
 * disabled. we let it inactive for 30 seconds to cool down
 *
 * TODO: to be removed after bring up / test phase
 */
#define COOLDOWN_MS (30 * 1000)
/* if after this number of retries a chip is still inaccessible, disable it */
#define DISABLE_CHIP_FAIL_THRESHOLD	3

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
//	applog(LOG_NOTICE, "%d, A1 running queue_full: %d/%d",
  //     a1->chain_id, a1->active_wq.num_elems, a1->num_active_chips);

	if (a1->active_wq.num_elems >= a1->num_active_chips * 2)
		queue_full = true;
	else
		wq_enqueue(&a1->active_wq, get_queued(cgpu));

	mutex_unlock(&a1->lock);

	return queue_full;
}


static int coinflex_encode(unsigned char *packet, unsigned char cmd, unsigned char *data, int len)
{
	

	return (ETX_POS + 1);
}


static bool coinflex_send(struct cgpu_info *coinflex, uint8_t cmd, char *data, int len)
{

	return (true);
}


static bool coinflex_recv(struct cgpu_info *coinflex, uint8_t cmd, char *data, int len)
{
	
	return (true);
}

static int coinflex_send_cmd(struct cgpu_info *coinflex, uint8_t cmd, uint8_t *data, uint32_t data_len, uint8_t *status)
{
	
	return (COINFLEX_RESULT_OK);
}


static void coinflex_info_clear(struct A1_chain *info)
{


}



static bool coinflex_reset(struct cgpu_info *coinflex)
{
	
}


static void coinflex_identify(struct cgpu_info *coinflex)
{
	
}



static bool coinflex_hw_reset(struct cgpu_info *coinflex)
{
	
	return (true);
}



#define TARGET_32BIT_LIMIT_E9				(0x00000002)
static bool coinflex_send_work(struct thr_info *thr)
{
	return (true);
}

void exit_A1_chain(struct A1_chain *a1)
{
	if (a1 == NULL){
		return;
	}
	free(a1->chips);
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
	tsadc_divider = tsadc_divider_tmp & 0xff;

	buffer[5] = 0x00 | tsadc_divider;

	if(!inno_cmd_write_sec_reg(a1,ADDR_BROADCAST,buffer)){
		applog(LOG_WARNING, "#####Write t/v sensor Value Failed!\n");
	}
	applog(LOG_WARNING, "#####Write t/v sensor Value Success!\n");
}

struct A1_chain *init_A1_chain(struct spi_ctx *ctx, int chain_id)
{
	int i;
	struct A1_chain *a1 = malloc(sizeof(*a1));
	assert(a1 != NULL);

	applog(LOG_DEBUG, "%d: A1 init chain", chain_id);
	
	memset(a1, 0, sizeof(*a1));
	a1->spi_ctx = ctx;
	a1->chain_id = chain_id;

	applog(LOG_NOTICE,"chain_id:%d", chain_id);
	switch(chain_id){
		case 0:a1->num_chips = chain_detect(a1, A1Pll1);break;
		case 1:a1->num_chips = chain_detect(a1, A1Pll2);break;
		case 2:a1->num_chips = chain_detect(a1, A1Pll3);break;
		case 3:a1->num_chips = chain_detect(a1, A1Pll4);break;
		case 4:a1->num_chips = chain_detect(a1, A1Pll5);break;
		case 5:a1->num_chips = chain_detect(a1, A1Pll6);break;
		default:;
	}
	usleep(10000);

	cfg_tsadc_divider(a1,PLL_Clk_12Mhz[A1Pll1].speedMHz);
	
	if (a1->num_chips == 0){
		goto failure;
	}

	applog(LOG_WARNING, "spidev%d.%d: %d: Found %d A1 chips",a1->spi_ctx->config.bus, a1->spi_ctx->config.cs_line,a1->chain_id, a1->num_chips);
	#if 0
	if (!set_pll_config(a1, 0, A1_config_options.ref_clk_khz,A1_config_options.sys_clk_khz)){
		goto failure;
	}
	#endif
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

	for (i = 0; i < a1->num_active_chips; i++){
		check_chip(a1, i);

       // inno_check_voltage(a1, i+1, &s_reg_ctrl);
        inno_fan_temp_add(&s_fan_ctrl, chain_id, a1->chips[i].temp, true);
    }

	//configure for vsensor
	inno_configure_tvsensor(a1,ADDR_BROADCAST,0);

	for (i = 0; i < a1->num_active_chips; i++){
        inno_check_voltage(a1, i+1, &s_reg_ctrl);
        //inno_fan_temp_add(&s_fan_ctrl, chain_id, a1->chips[i].temp, true);
    }
	
    //configure for tsensor
	inno_configure_tvsensor(a1,ADDR_BROADCAST,1);
	usleep(200);
	
    inno_fan_temp_init(&s_fan_ctrl, chain_id);

	applog(LOG_WARNING, "[chain_ID:%d]: Found %d Chips With Total %d Active Cores",a1->chain_id, a1->num_active_chips, a1->num_cores);

	//printf("[##########Current Temp]temp=%d\n",inno_fan_temp_get_highest(&s_fan_ctrl,chain_id));
	if(s_fan_ctrl.temp_highest[chain_id] < DANGEROUS_TMP){
		asic_gpio_write(spi[a1->chain_id]->power_en, 0);
		//asic_gpio_write(GPIO_RED, 1);
	  	early_quit(1,"Notice Chain %d temp:%d Maybe Has Some Problem in Temperate\n",a1->chain_id,s_fan_ctrl.temp_highest[chain_id]);
	}

	mutex_init(&a1->lock);
	INIT_LIST_HEAD(&a1->active_wq.head);

	return a1;

failure:
	exit_A1_chain(a1);
	return NULL;
}

static bool detect_A1_chain(void)
{
	int i,j;
	applog(LOG_WARNING, "A1: checking A1 chain");

	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		cfg[i].bus     = i;
		cfg[i].cs_line = 0;
		cfg[i].mode    = SPI_MODE_1;
		cfg[i].speed   = DEFAULT_SPI_SPEED;
		cfg[i].bits    = DEFAULT_SPI_BITS_PER_WORD;
		cfg[i].delay   = DEFAULT_SPI_DELAY_USECS;

		spi[i] = spi_init(&cfg[i]);
		if(spi[i] == NULL)
		{
			applog(LOG_ERR, "spi init fail");
			return false;
		}

        mutex_init(&spi[i]->spi_lock);
		spi[i]->power_en = SPI_PIN_POWER_EN[i];		
		spi[i]->start_en = SPI_PIN_START_EN[i];		
		spi[i]->reset = SPI_PIN_RESET[i];
		//spi[i]->plug  = SPI_PIN_PLUG[i];
		//spi[i]->led   = SPI_PIN_LED[i];
		

		asic_gpio_init(spi[i]->power_en, 0);
		asic_gpio_init(spi[i]->start_en, 0);
		asic_gpio_init(spi[i]->reset, 0);
		//asic_gpio_init(spi[i]->plug, 0);
		//asic_gpio_init(spi[i]->led, 0);

		update_temp[i] = 0;
		show_log[i] = 0;
	}

	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		asic_gpio_write(spi[i]->power_en, 1);
		asic_gpio_write(spi[i]->start_en, 1);
		asic_gpio_write(spi[i]->reset, 1);
		usleep(500000);
		asic_gpio_write(spi[i]->reset, 0);
		usleep(500000);
		asic_gpio_write(spi[i]->reset, 1);	
	}

	//divide the init to break two part
	if(opt_voltage > 8){
		for(i=9; i<=opt_voltage; i++){
			set_vid_value(i);
			usleep(200000);
		}
	}
			
	if(opt_voltage < 8){
		for(i=7; i>=opt_voltage; i--){
			set_vid_value(i);
			usleep(200000);
		}
	}


	for(i = 0; i < ASIC_CHAIN_NUM; i++)
	{
		chain[i] = init_A1_chain(spi[i], i);
		if (chain[i] == NULL)
		{
			applog(LOG_ERR, "init a1 chain fail");
			return false;
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

		applog(LOG_WARNING, "Detected the %d A1 chain with %d chips / %d cores",i, chain[i]->num_active_chips, chain[i]->num_cores);
	}

	return true;
}

static void coinflex_detect(bool __maybe_unused hotplug)
{
	if (hotplug){
		return;
	}
		
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
	memset(&s_reg_ctrl,0,sizeof(s_reg_ctrl));
	memset(&s_fan_ctrl,0,sizeof(s_fan_ctrl));
	
	inno_fan_init(&s_fan_ctrl);
	set_vid_value(opt_voltage);
		
	A1Pll1 = A1_ConfigA1PLLClock(opt_A1Pll1);
	A1Pll2 = A1_ConfigA1PLLClock(opt_A1Pll2);
	A1Pll3 = A1_ConfigA1PLLClock(opt_A1Pll3);
	A1Pll4 = A1_ConfigA1PLLClock(opt_A1Pll4);
	A1Pll5 = A1_ConfigA1PLLClock(opt_A1Pll5);
	A1Pll6 = A1_ConfigA1PLLClock(opt_A1Pll6);
		
	if(detect_A1_chain()){
		return ;
	}
	
	applog(LOG_WARNING, "A1 dectect finish");
	
	int i = 0;
	/* release SPI context if no A1 products found */
	for(i = 0; i < ASIC_CHAIN_NUM; i++){
		spi_exit(spi[i]);
	}	
	//usb_detect(&coinflex_drv, coinflex_detect_one);
}

static bool coinflex_prepare(struct thr_info *thr)
{
	return (true);
}

static char* coinflex_set_device(struct cgpu_info *coinflex, char *option, char *setting, char *replybuf)
{
	return (NULL);
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


static void coinflex_shutdown(struct thr_info *thr)
{
	
}

static void coinflex_update_work(struct cgpu_info *coinflex)
{
	
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
				//		 cid, i, j + 1, work);
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

static bool coinflex_get_result(struct cgpu_info *coinflex, char *data, int len)
{
	return (true);
}

#define VOLTAGE_UPDATE_INT  120

static int64_t coinflex_scanwork(struct thr_info *thr)
{
	static uint8_t cnt = 0; 
	int i;
	int32_t A1Pll = 1000;
	uint8_t reg[128];
	struct cgpu_info *cgpu = thr->cgpu;
	struct A1_chain *a1 = cgpu->device_data;
	//struct A1_chip *a7 = &a1->chips[0];
	struct work local_work;
	int32_t nonce_ranges_processed = 0;

	if (a1->num_cores == 0) {
		cgpu->deven = DEV_DISABLED;
		return 0;
	}

	uint32_t nonce;
	uint8_t chip_id;
	uint8_t job_id;
	bool work_updated = false;
	//uint8_t reg[REG_LENGTH];

	mutex_lock(&a1->lock);
    int cid = a1->chain_id;
	if (a1->last_temp_time + TEMP_UPDATE_INT_MS < get_current_ms())
	{
		update_temp[cid]++;
		show_log[cid]++;
		a1->last_temp_time = get_current_ms();
	}

	if(s_fan_ctrl.temp_highest[a1->chain_id] < DANGEROUS_TMP){
		asic_gpio_write(spi[a1->chain_id]->power_en, 0);
	   	early_quit(1,"Notice Chain %d temp:%d Maybe Has Some Problem in Temperate\n",a1->chain_id,s_fan_ctrl.temp_highest[a1->chain_id]);
	}

	/* poll queued results */
	while (true){
		if (!get_nonce(a1, (uint8_t*)&nonce, &chip_id, &job_id)){
			break;
		}
	
		//nonce = bswap_32(nonce);	 //modify for A4
	
		work_updated = true;
		if (chip_id < 1 || chip_id > a1->num_active_chips) {
			applog(LOG_WARNING, "%d: wrong chip_id %d", cid, chip_id);
			continue;
		}
		if (job_id < 1 && job_id > 4){
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
		applog(LOG_INFO, "YEAH: %d: chip %d / job_id %d: nonce 0x%08x", cid, chip_id, job_id, nonce);
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
		if(update_temp[cid] > 5)
		{
			for (i = a1->num_active_chips; i > 0; i--) 
			{
				uint8_t c = i;
				if (is_chip_disabled(a1, c))
					continue;
				if (!inno_cmd_read_reg(a1, c, reg)) 
				{
					disable_chip(a1, c);
					continue;
				}
	            else
	            {
	                /* update temp database */
	                uint32_t temp = 0;
	                float    temp_f = 0.0f;
					struct A1_chip *chip = &a1->chips[i - 1];

                	temp = 0x000003ff & ((reg[7] << 8) | reg[8]);
					chip->temp = temp;
                	inno_fan_temp_add(&s_fan_ctrl, cid, temp, false);
                
					//inno_fan_speed_update(&s_fan_ctrl, cid);
				//	a1->last_temp_time = get_current_ms();
					cnt++;
					//printf("cnt = %d\n",cnt);
				} 
			 }
        	inno_fan_speed_update(&s_fan_ctrl, cid, cgpu);
        	update_temp[cid] = 0;
        	
		}
		if (inno_cmd_read_reg(a1, 25, reg)) 
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
					assert(work != NULL);

					if (set_work(a1, c, work, 0))
					{
						nonce_ranges_processed++;
						chip->nonce_ranges_done++;
					}

					if(show_log[cid] > 0)					
					{						
						applog(LOG_INFO, "%d: chip %d: job done: %d/%d/%d/%d/%d/%5.2f",
                               cid, c, chip->nonce_ranges_done, chip->nonces_found, 
                               chip->hw_errors, chip->stales,chip->temp, inno_fan_temp_to_float(&s_fan_ctrl,chip->temp));
						
						if(i==1) show_log[cid] = 0;	
					}
			}
		} 
     }
  }
          int time_inter[2];
          time_inter[0] = get_current_ms();
	      if(cnt > VOLTAGE_UPDATE_INT){

	   //configure for vsensor
	inno_configure_tvsensor(a1,ADDR_BROADCAST,0);

	for (i = 0; i < a1->num_active_chips; i++){
        inno_check_voltage(a1, i+1, &s_reg_ctrl);
    }
	cnt = 0;
    //configure for tsensor
	inno_configure_tvsensor(a1,ADDR_BROADCAST,1);
	usleep(200);

			time_inter[1] = get_current_ms();
			printf("\nv waste time is %d\n",time_inter[1] - time_inter[0]);
	}
	
	switch(cid){
		case 0:check_disabled_chips(a1, A1Pll1);;break;
		case 1:check_disabled_chips(a1, A1Pll2);;break;
		case 2:check_disabled_chips(a1, A1Pll3);;break;
		case 3:check_disabled_chips(a1, A1Pll4);;break;
		case 4:check_disabled_chips(a1, A1Pll5);;break;
		case 5:check_disabled_chips(a1, A1Pll6);;break;
		default:;
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
	//	cgsleep_ms(40);
	
	cgtime(&a1->tvScryptCurr);
	timersub(&a1->tvScryptCurr, &a1->tvScryptLast, &a1->tvScryptDiff);
	cgtime(&a1->tvScryptLast);
	
	switch(cgpu->device_id){
		case 0:A1Pll = PLL_Clk_12Mhz[A1Pll1].speedMHz;break;
		case 1:A1Pll = PLL_Clk_12Mhz[A1Pll2].speedMHz;break;
		case 2:A1Pll = PLL_Clk_12Mhz[A1Pll3].speedMHz;break;
		case 3:A1Pll = PLL_Clk_12Mhz[A1Pll4].speedMHz;break;
		case 4:A1Pll = PLL_Clk_12Mhz[A1Pll5].speedMHz;break;
		case 5:A1Pll = PLL_Clk_12Mhz[A1Pll6].speedMHz;break;
		default:break;
	}
	
	//return (int64_t)(2011173.18 * A1Pll / 1000 * (a1->num_cores/9.0) * (a1->tvScryptDiff.tv_usec / 1000000.0));
	return ((((double)PLL_Clk_12Mhz[A1Pll1].speedMHz*a1->tvScryptDiff.tv_usec /2) * (a1->num_cores))/13);

}


static void coinflex_print_hw_error(char *drv_name, int device_id, struct work *work, uint32_t nonce)
{
	char		*twork;
	char		twork_data[BUF_SIZE];
	int		twork_index;
	int		display_size = 16;		// 16 Bytes

	applog(LOG_ERR, "---------------------------------------------------------");
	applog(LOG_ERR, "[%s %d]:ERROR  - Nonce = 0x%X,Work_ID = %3d", drv_name, device_id, nonce, work->work_id);
	twork = bin2hex(work->data, WORK_SIZE);						// Multiply 2 for making string in bin2hex()
	for(twork_index = 0; twork_index < (WORK_SIZE * 2); twork_index += (display_size * 2))
	{
		snprintf(twork_data, (display_size * 2) + 1, "%s", &twork[twork_index]);
		applog(LOG_ERR, "Work Data      = %s", twork_data);
	}
	free(twork);
	twork = bin2hex(work->device_target, DEVICE_TARGET_SIZE);		// Multiply 2 for making string in bin2hex()
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
	.drv_id					= DRIVER_coinflex,
	.dname					= "HLT_Coinflex",
	.name					= "HLT",
	.drv_ver					= COINFLEX_DRIVER_VER,
	.drv_date					= COINFLEX_DRIVER_DATE,
	.drv_detect				= coinflex_detect,
	.get_statline_before		= coinflex_get_statline_before,
	.queue_full             =   coinflex_queue_full,
	.get_api_stats				= NULL,
	.identify_device			= coinflex_identify,
	.set_device				= coinflex_set_device,
	.thread_prepare			= coinflex_prepare,
	.thread_shutdown			= coinflex_shutdown,
	.hw_reset				= coinflex_hw_reset,
	.hash_work				= hash_queued_work,
	.update_work				= coinflex_update_work,
	.flush_work				= coinflex_flush_work,			// new block detected or work restart 
	.scanwork				= coinflex_scanwork,				// scan hash
	.max_diff					= 65536
};

#if COINFLEX_TEST_MODE
#if 0
char dataX11[80] =  { 0x00,0x00,0x00,0x02,0x75,0xd6,0x91,0x99,0x65,0x07,0x7e,0x96,0xb8,0x04,0xb1,0xbf,0x77,0x8a,0xe9,0x2a,0x6d,0xe7,0x5f,0xeb,0x56,0x91,0x11,0x00,0x00,0x04,0xbe,0x72,0x00,0x00,0x00,0x00,0x62,0x22,0x72,0xad,0xf6,0x99,0x66,0x75,0xf2,0xa0,0xf9,0xe5,0x54,0xb3,0x67,0x54,0xfb,0x40,0xf8,0x1f,0x2d,0xad,0x5e,0xd8,0x4a,0x34,0x56,0x09,0xd7,0x58,0x2c,0xe9,0x53,0x95,	0x16,0xe7,0x1b,0x14,0x76,0x0a,0x00,0x10,0x6b,0x0d };
char targetX11[4] = { 0x00,0x00,0x00,0xff };
uint32_t nonceX11 = 225120256;	// 0xd6b1000
#else
char dataX11[82] = {0x00,0x00, 0x20,0x00,0x00, 0x00, 0x5c, 0x3c, 0xf5, 0x94, 0xeb, 0x10, 0x71, 0x3b, 0x0e, 0x5b, 0xe7, 0x5c, 0xb9, 0x01, 0x09, 0xc2, 0x16, 0xea, 0xfd,0x1c,0xba, 0xba, 0x9c, 0x5e, 0x00, 0x00, 0x59, 0x16, 0x00, 0x00, 0x00, 0x00, 0xc7, 0x82, 0xa0, 0xa7, 0xc5, 0xa6, 0xf2, 0x08, 0x21, 0x6a, 0xe8, 0x49, 0x21, 0xbc, 0x8c, 0xa3, 0x7b, 0xee, 0x8c, 0x08, 0x5f, 0x93, 0x95, 0x06, 0x04, 0x02, 0xe9, 0x15, 0x70, 0x6b, 0x76, 0x19, 0x59, 0x28, 0xd7, 0x2c, 0x1b, 0x00, 0x81, 0xd5, 0xe4, 0x2e, 0x36, 0x40};
//char dataX11[82] =  {0x00,0x00, 0x00,0x00,0x00,0x20,0x94,0xf5,0x3c,0x5c,0x3b,0x71,0x10,0xeb,0x5c,0xe7,0x5b,0x0e,0xc2,0x09,0x01,0xb9,0x1c,0xfd,0xea,0x16,0x5e,0x9c,0xba,0xba,0x16,0x59,0x00,0x00,0x00,0x00,0x00,0x00,0xa7,0xa0,0x82,0xc7,0x08,0xf2,0xa6,0xc5,0x49,0xe8,0x6a,0x21,0xa3,0x8c,0xbc,0x21,0x08,0x8c,0xee,0x7b,0x06,0x95,0x93,0x5f,0x15,0xe9,0x02,0x04,0x19,0x76,0x6b,0x70,0x2c,0xd7,0x28,0x59,0xd5,0x81,0x00,0x1b,0xe4,0x2e,0x36,0x40 };//0x40,0x36,0x2e,0xe4
//ffd80000_00000027
char targetX11[4] = { 0x00,0x00,0x00,0x27 };
uint32_t nonceX11 = 0x40362ee4;//e42e3650;	// 0xd6b1000
#endif

char dataX13[80] =  { 0x00,0x00,0x00,0x02,0x5b,0x4a,0xbb,0x46,0x95,0x9d,0x93,0xd0,0x49,0x1a,0x8c,0x97,0xb0,0x02,0x37,0x29,0x5d,0x1e,0xf8,0xfd,0xe0,0x74,0x2c,0xf7,0x00,0xdd,0x5c,0xb2,0x00,0x00,0x00,0x00,0x56,0xc0,0x2f,0x12,0x82,0x24,0xd3,0xb8,0xe9,0x37,0x67,0x9f,0x9d,0x00,0x10,0x00,0x2e,0x32,0x02,0x6b,0xf2,0x9d,0x22,0xd5,0x30,0x68,0xcb,0x13,0xc0,0x14,0x4d,0xa5,0x53,0x95,	0x89,0xad,0x1c,0x02,0xac,0x3d,0x00,0x0a,0x1e,0xf1 };
char targetX13[4] = { 0xff,0x00,0x00,0x00 };
uint32_t nonceX13 = 4045277696;	// 0xf11e0a00

static void coinflex_set_testdata(struct work *work)
{
	char		*data;
	char		*target;
	uint32_t	nonce;
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
	work->data[1] =	0x01;



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
	uint32_t	*work_nonce = (uint32_t *)(work->data + 64 + 12);
	uint32_t	*ohash = (uint32_t *)(work->hash);
	unsigned char hash_swap[32], target_swap[32];
	char		*hash_str, *target_str;

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

